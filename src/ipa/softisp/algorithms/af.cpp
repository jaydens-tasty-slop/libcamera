/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2026, Jayden Milne
 *
 * Autofocus
 */

#include "af.h"

#include <algorithm>
#include <optional>
#include <span>
#include <stdint.h>

#include <libcamera/base/log.h>

#include <libcamera/geometry.h>

namespace libcamera {

LOG_DEFINE_CATEGORY(IPASoftIspAf)

namespace ipa::softisp::algorithms {

/*
 * Mean level above black, on an 8-bit scale, below which the image is too
 * dark to measure focus.
 */
static constexpr double kMinLevel = 4.0;

/* Scale applied to the normalised contrast, so that FocusFoM is usable. */
static constexpr double kContrastScale = 1e6;

/**
 * \class Af
 * \brief Contrast detection autofocus
 *
 * The Af algorithm drives the focus lens, when the camera has one, using the
 * libipa AfContrast algorithm and the focus statistics of the software ISP.
 * Without a focus lens, it only reports the FocusFoM metadata.
 *
 * The software ISP computes statistics every SwStatsCpu::kStatPerNumFrames
 * frames, and can process a frame several frames after its capture when
 * debayering takes most of the frame time. Lens movements are timed against
 * the frames captured: prepare() runs when a captured frame is queued to the
 * software ISP.
 */

/**
 * \copydoc libcamera::ipa::Algorithm::init
 */
int Af::init(IPAContext &context, const ValueNode &tuningData)
{
	return af_.init(tuningData, context.lensControls, context.ctrlMap);
}

/**
 * \copydoc libcamera::ipa::Algorithm::configure
 */
int Af::configure(IPAContext &context, const IPAConfigInfo &configInfo)
{
	/*
	 * \todo The statistics cover the debayered area, which is centred in
	 * the sensor output and can be smaller than it. Account for it when
	 * mapping the AF windows, and use the analogue crop of the configured
	 * sensor mode instead of the one available at init time.
	 */
	return af_.configure(configInfo.lensControls,
			     Size(SwIspStats::kFocusGridWidth,
				  SwIspStats::kFocusGridHeight),
			     context.sensorInfo.analogCrop);
}

/**
 * \copydoc libcamera::ipa::Algorithm::queueRequest
 */
void Af::queueRequest([[maybe_unused]] IPAContext &context,
		      [[maybe_unused]] const uint32_t frame,
		      [[maybe_unused]] IPAFrameContext &frameContext,
		      const ControlList &controls)
{
	af_.queueRequest(controls);
}

/**
 * \copydoc libcamera::ipa::Algorithm::prepare
 */
void Af::prepare([[maybe_unused]] IPAContext &context, const uint32_t frame,
		 IPAFrameContext &frameContext,
		 [[maybe_unused]] DebayerParams *params)
{
	af_.prepare(frame, frameContext.af);
}

bool Af::computeRegions(const IPAContext &context, const SwIspStats *stats)
{
	const double blackLevel = context.activeState.blc.level;

	uint64_t sum = 0;
	uint64_t count = 0;
	for (const SwIspStats::FocusRegion &region : stats->focus) {
		sum += region.sum;
		count += region.count;
	}

	if (!count)
		return false;

	/*
	 * Normalise the contrast by the square of the mean level of the whole
	 * window rather than of each region, so that the measurement doesn't
	 * depend on the exposure while textured regions keep more weight than
	 * flat ones.
	 */
	const double level = static_cast<double>(sum) / count - blackLevel;
	if (level < kMinLevel) {
		LOG(IPASoftIspAf, Debug) << "Image too dark to measure focus";
		return false;
	}

	const double scale = kContrastScale / (level * level);

	for (unsigned int i = 0; i < regions_.size(); i++) {
		const SwIspStats::FocusRegion &region = stats->focus[i];

		if (!region.count) {
			regions_[i] = { 0.0, 0.0 };
			continue;
		}

		regions_[i].contrast = region.contrast * scale / region.count;
		regions_[i].average = std::max(static_cast<double>(region.sum) / region.count - blackLevel,
					       0.0);
	}

	return true;
}

/**
 * \copydoc libcamera::ipa::Algorithm::process
 */
void Af::process(IPAContext &context, const uint32_t frame,
		 IPAFrameContext &frameContext, const SwIspStats *stats,
		 ControlList &metadata)
{
	std::span<const AfContrast::RegionStats> regions;
	if (stats->valid && computeRegions(context, stats))
		regions = regions_;

	const double exposure = frameContext.sensor.exposure * frameContext.sensor.gain;
	af_.process(frame, frameContext.af, regions, exposure, metadata);

	if (std::optional<int32_t> setting = af_.lensSetting())
		context.activeState.lens.focusPosition = *setting;
}

REGISTER_IPA_ALGORITHM(Af, "Af")

} /* namespace ipa::softisp::algorithms */

} /* namespace libcamera */
