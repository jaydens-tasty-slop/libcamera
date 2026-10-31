/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2026 Red Hat Inc.
 *
 * Autofocus
 */

#include "af.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdint.h>

#include <linux/v4l2-controls.h>

#include <libcamera/base/log.h>

#include <libcamera/control_ids.h>

#include "libcamera/internal/software_isp/swisp_stats.h"

namespace libcamera {

namespace ipa::softisp::algorithms {

LOG_DEFINE_CATEGORY(IPASoftIspAf)

/*
 * Mean level above black, on an 8-bit scale, below which the image is too
 * dark to measure focus.
 */
static constexpr double kMinLevel = 4.0;

/* Scale applied to the normalised contrast, so that the values are readable. */
static constexpr double kContrastScale = 1e6;

int Af::configure(IPAContext &context, const IPAConfigInfo &configInfo)
{
	/*
	 * Without a focus lens, only report how sharp the frames are, which
	 * still helps to focus a lens by hand.
	 */
	context.afEnabled = configInfo.lensControls.count(V4L2_CID_FOCUS_ABSOLUTE);

	afAlgo_.configure(context.activeState.af, configInfo.lensControls);

	return 0;
}

/*
 * Measure the focus in the centre half of the width and third of the height
 * of the statistics window: the mean squared difference between adjacent
 * green values, normalised by the square of the mean level above black so
 * that the measurement doesn't depend on the exposure.
 */
std::optional<double> Af::measureFocus(const IPAContext &context,
				       const SwIspStats *stats)
{
	constexpr unsigned int kLeft = SwIspStats::kFocusGridWidth / 4;
	constexpr unsigned int kRight = SwIspStats::kFocusGridWidth - kLeft;
	constexpr unsigned int kTop = SwIspStats::kFocusGridHeight / 3;
	constexpr unsigned int kBottom = SwIspStats::kFocusGridHeight - kTop;

	uint64_t contrast = 0;
	uint64_t sum = 0;
	uint64_t count = 0;

	for (unsigned int y = kTop; y < kBottom; y++) {
		for (unsigned int x = kLeft; x < kRight; x++) {
			const SwIspStats::FocusRegion &region =
				stats->focus[y * SwIspStats::kFocusGridWidth + x];

			contrast += region.contrast;
			sum += region.sum;
			count += region.count;
		}
	}

	if (!count)
		return std::nullopt;

	const double level = static_cast<double>(sum) / count -
			     context.activeState.blc.level;
	if (level < kMinLevel)
		return std::nullopt;

	return kContrastScale * contrast / count / (level * level);
}

void Af::process(IPAContext &context,
		 [[maybe_unused]] const uint32_t frame,
		 [[maybe_unused]] IPAFrameContext &frameContext,
		 const SwIspStats *stats,
		 ControlList &metadata)
{
	/*
	 * The statistics are only computed for some frames, the others carry
	 * the values of an earlier frame.
	 */
	if (!stats->valid)
		return;

	std::optional<double> focus = measureFocus(context, stats);
	if (!focus) {
		LOG(IPASoftIspAf, Debug) << "Image too dark to measure focus";
		return;
	}

	LOG(IPASoftIspAf, Debug) << "Focus measure: " << *focus;

	constexpr double kMaxFoM = std::numeric_limits<int32_t>::max();
	metadata.set(controls::FocusFoM,
		     static_cast<int32_t>(std::min(std::round(*focus), kMaxFoM)));

	if (context.afEnabled)
		afAlgo_.process(context.activeState.af, *focus);
}

REGISTER_IPA_ALGORITHM(Af, "Af")

} /* namespace ipa::softisp::algorithms */

} /* namespace libcamera */
