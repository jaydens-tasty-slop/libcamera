/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2026, Jayden Milne
 *
 * SwStatsCpu focus statistics test
 */

#include <iostream>
#include <memory>
#include <stdint.h>
#include <sys/mman.h>
#include <vector>

#include <libcamera/camera_manager.h>
#include <libcamera/formats.h>
#include <libcamera/geometry.h>
#include <libcamera/stream.h>

#include "libcamera/internal/software_isp/swisp_stats.h"
#include "libcamera/internal/software_isp/swstats_cpu.h"

#include "test.h"

using namespace std;
using namespace libcamera;

namespace {

constexpr unsigned int kWidth = 256;
constexpr unsigned int kHeight = 192;
constexpr unsigned int kStride = kWidth * 2;

/* A 10-bit BGGR frame with a flat background. */
class Frame
{
public:
	Frame()
		: data_(kWidth * kHeight, 200)
	{
	}

	void set(unsigned int x, unsigned int y, uint16_t value)
	{
		data_[y * kWidth + x] = value;
	}

	const uint8_t *line(unsigned int y) const
	{
		return reinterpret_cast<const uint8_t *>(&data_[y * kWidth]);
	}

private:
	vector<uint16_t> data_;
};

} /* namespace */

class SwStatsCpuFocusTest : public Test
{
protected:
	int init() override
	{
		cm_ = make_unique<CameraManager>();
		stats_ = make_unique<SwStatsCpu>(*cm_);
		if (!stats_->isValid()) {
			cerr << "Failed to create SwStatsCpu" << endl;
			return TestFail;
		}

		StreamConfiguration cfg;
		cfg.pixelFormat = formats::SBGGR10;
		cfg.size = { kWidth, kHeight };
		cfg.stride = kStride;

		/* Use two buffers to exercise the per-thread merge. */
		if (stats_->configure(cfg, 2)) {
			cerr << "Failed to configure SwStatsCpu" << endl;
			return TestFail;
		}

		stats_->setWindow(Rectangle(cfg.size));

		void *mem = mmap(nullptr, sizeof(SwIspStats), PROT_READ,
				 MAP_SHARED, stats_->getStatsFD().get(), 0);
		if (mem == MAP_FAILED) {
			cerr << "Failed to map the statistics" << endl;
			return TestFail;
		}

		result_ = static_cast<const SwIspStats *>(mem);

		return TestPass;
	}

	void cleanup() override
	{
		if (result_)
			munmap(const_cast<SwIspStats *>(result_), sizeof(SwIspStats));
	}

	void process(const Frame &frame)
	{
		stats_->startFrame(0);

		for (unsigned int y = 0; y < kHeight; y += 2) {
			const uint8_t *lines[3] = {
				nullptr, frame.line(y), frame.line(y + 1)
			};
			stats_->processLine0(0, y, lines, (y / 4) % 2);
		}

		stats_->finishFrame(0, 0);
	}

	static uint64_t totalContrast(const SwIspStats &stats)
	{
		uint64_t total = 0;

		for (const SwIspStats::FocusRegion &region : stats.focus)
			total += region.contrast;

		return total;
	}

	int run() override
	{
		constexpr unsigned int kGridWidth = SwIspStats::kFocusGridWidth;
		constexpr unsigned int kGridHeight = SwIspStats::kFocusGridHeight;
		constexpr unsigned int kRegionWidth = kWidth / kGridWidth;
		constexpr unsigned int kRegionHeight = kHeight / kGridHeight;

		/* A flat frame has no contrast. */
		Frame flat;
		process(flat);

		if (!result_->valid) {
			cerr << "Statistics not valid" << endl;
			return TestFail;
		}

		/*
		 * Samples are taken every 4 pixels on every other pair of
		 * lines, the values are scaled to 8 bits.
		 */
		const uint32_t samples = (kRegionWidth / 4) * (kRegionHeight / 4);
		for (const SwIspStats::FocusRegion &region : result_->focus) {
			if (region.contrast != 0 || region.count != samples ||
			    region.sum != samples * (200 >> 2)) {
				cerr << "Unexpected flat region statistics "
				     << region.contrast << " " << region.sum
				     << " " << region.count << endl;
				return TestFail;
			}
		}

		/*
		 * Add vertical stripes with a period of 8 pixels to one region,
		 * so that adjacent samples alternate between 100 and 300.
		 */
		constexpr unsigned int col = 2;
		constexpr unsigned int row = 1;

		Frame stripes;
		for (unsigned int y = row * kRegionHeight; y < (row + 1) * kRegionHeight; y++) {
			for (unsigned int x = col * kRegionWidth; x < (col + 1) * kRegionWidth; x++)
				stripes.set(x, y, (x / 4) % 2 ? 300 : 100);
		}

		process(stripes);

		/*
		 * Each sampled line holds 8 samples in the region: one step of
		 * 100 from the background, then 7 steps of 200. The next region
		 * sees a step of 100 back to the background.
		 */
		const unsigned int lines = kRegionHeight / 4;
		const uint64_t expected = lines * (100 * 100 + 7 * 200 * 200) >> 4;
		const uint64_t edge = lines * (100 * 100) >> 4;

		for (unsigned int r = 0; r < kGridHeight; r++) {
			for (unsigned int c = 0; c < kGridWidth; c++) {
				const SwIspStats::FocusRegion &region =
					result_->focus[r * kGridWidth + c];
				uint64_t value = 0;

				if (r == row && c == col)
					value = expected;
				else if (r == row && c == col + 1)
					value = edge;

				if (region.contrast != value) {
					cerr << "Region (" << c << ", " << r
					     << ") contrast " << region.contrast
					     << ", expected " << value << endl;
					return TestFail;
				}
			}
		}

		/* Blurring the stripes must lower the contrast. */
		Frame blurred;
		for (unsigned int y = row * kRegionHeight; y < (row + 1) * kRegionHeight; y++) {
			for (unsigned int x = col * kRegionWidth; x < (col + 1) * kRegionWidth; x++)
				blurred.set(x, y, (x / 4) % 2 ? 250 : 150);
		}

		const uint64_t sharp = totalContrast(*result_);
		process(blurred);

		if (totalContrast(*result_) >= sharp) {
			cerr << "Blurred contrast " << totalContrast(*result_)
			     << " not below sharp contrast " << sharp << endl;
			return TestFail;
		}

		/* Frames that skip statistics must flag them as invalid. */
		stats_->startFrame(1);
		stats_->finishFrame(1, 0);
		if (result_->valid) {
			cerr << "Statistics valid on a skipped frame" << endl;
			return TestFail;
		}

		return TestPass;
	}

private:
	unique_ptr<CameraManager> cm_;
	unique_ptr<SwStatsCpu> stats_;
	const SwIspStats *result_ = nullptr;
};

TEST_REGISTER(SwStatsCpuFocusTest)
