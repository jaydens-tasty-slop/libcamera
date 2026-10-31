/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2021, Red Hat
 *
 * IPU3 auto focus algorithm
 */

#include "af.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include <linux/videodev2.h>

#include <libcamera/base/log.h>

#include <libcamera/ipa/core_ipa_interface.h>

/**
 * \file af.h
 */

/*
 * Static variables from ChromiumOS Intel Camera HAL and ia_imaging library:
 * - https://chromium.googlesource.com/chromiumos/platform/arc-camera/+/master/hal/intel/psl/ipu3/statsConverter/ipu3-stats.h
 * - https://chromium.googlesource.com/chromiumos/platform/camera/+/refs/heads/main/hal/intel/ipu3/include/ia_imaging/af_public.h
 */

/** The minimum horizontal grid dimension. */
static constexpr uint8_t kAfMinGridWidth = 16;
/** The minimum vertical grid dimension. */
static constexpr uint8_t kAfMinGridHeight = 16;
/** The maximum horizontal grid dimension. */
static constexpr uint8_t kAfMaxGridWidth = 32;
/** The maximum vertical grid dimension. */
static constexpr uint8_t kAfMaxGridHeight = 24;
/** The minimum value of Log2 of the width of the grid cell. */
static constexpr uint16_t kAfMinGridBlockWidth = 4;
/** The minimum value of Log2 of the height of the grid cell. */
static constexpr uint16_t kAfMinGridBlockHeight = 3;
/** The maximum value of Log2 of the width of the grid cell. */
static constexpr uint16_t kAfMaxGridBlockWidth = 6;
/** The maximum value of Log2 of the height of the grid cell. */
static constexpr uint16_t kAfMaxGridBlockHeight = 6;
/** The number of blocks in vertical axis per slice. */
static constexpr uint16_t kAfDefaultHeightPerSlice = 2;

namespace libcamera {

using namespace std::literals::chrono_literals;

namespace ipa::ipu3::algorithms {

LOG_DEFINE_CATEGORY(IPU3Af)

/* Settings for IPU3 AF filter */
static struct ipu3_uapi_af_filter_config afFilterConfigDefault = {
	.y1_coeff_0 = { 0, 1, 3, 7 },
	.y1_coeff_1 = { 11, 13, 1, 2 },
	.y1_coeff_2 = { 8, 19, 34, 242 },
	.y1_sign_vec = 0x7fdffbfe,
	.y2_coeff_0 = { 0, 1, 6, 6 },
	.y2_coeff_1 = { 13, 25, 3, 0 },
	.y2_coeff_2 = { 25, 3, 177, 254 },
	.y2_sign_vec = 0x4e53ca72,
	.y_calc = { 8, 8, 8, 8 },
	.nf = { 0, 9, 0, 9, 0 },
};

/**
 * \class Af
 * \brief An auto-focus algorithm based on IPU3 statistics
 *
 * This algorithm is used to determine the position of the lens to make a
 * focused image. The IPU3 AF processing block computes the statistics that
 * are composed by two types of filtered value and stores in a AF buffer.
 * Typically, for a clear image, it has a relatively higher contrast than a
 * blurred one. Therefore, if an image with the highest contrast can be
 * found through the scan, the position of the len indicates to a clearest
 * image.
 */
Af::Af()
{
}

/**
 * \brief Configure the Af given a configInfo
 * \param[in] context The shared IPA context
 * \param[in] configInfo The IPA configuration data
 * \return 0 on success, a negative error code otherwise
 */
int Af::configure(IPAContext &context, const IPAConfigInfo &configInfo)
{
	struct ipu3_uapi_grid_config &grid = context.configuration.af.afGrid;
	grid.width = kAfMinGridWidth;
	grid.height = kAfMinGridHeight;
	grid.block_width_log2 = kAfMinGridBlockWidth;
	grid.block_height_log2 = kAfMinGridBlockHeight;

	/*
	 * \todo - while this clamping code is effectively a no-op, it satisfies
	 * the compiler that the constant definitions of the hardware limits
	 * are used, and paves the way to support dynamic grid sizing in the
	 * future. While the block_{width,height}_log2 remain assigned to the
	 * minimum, this code should be optimized out by the compiler.
	 */
	grid.width = std::clamp(grid.width, kAfMinGridWidth, kAfMaxGridWidth);
	grid.height = std::clamp(grid.height, kAfMinGridHeight, kAfMaxGridHeight);

	grid.block_width_log2 = std::clamp(grid.block_width_log2,
					   kAfMinGridBlockWidth,
					   kAfMaxGridBlockWidth);

	grid.block_height_log2 = std::clamp(grid.block_height_log2,
					    kAfMinGridBlockHeight,
					    kAfMaxGridBlockHeight);

	grid.height_per_slice = kAfDefaultHeightPerSlice;

	/* Position the AF grid in the center of the BDS output. */
	Rectangle bds(configInfo.bdsOutputSize);
	Size gridSize(grid.width << grid.block_width_log2,
		      grid.height << grid.block_height_log2);

	/*
	 * \todo - Support request metadata
	 * - Set the ROI based on any input controls in the request
	 * - Return the AF ROI as metadata in the Request
	 */
	Rectangle roi = gridSize.centeredTo(bds.center());
	Point start = roi.topLeft();

	/* x_start and y_start should be even */
	grid.x_start = utils::alignDown(start.x, 2);
	grid.y_start = utils::alignDown(start.y, 2);
	grid.y_start |= IPU3_UAPI_GRID_Y_START_EN;

	afAlgo_.configure(context.activeState.af, configInfo.lensControls);

	return 0;
}

/**
 * \copydoc libcamera::ipa::Algorithm::prepare
 */
void Af::prepare(IPAContext &context,
		 [[maybe_unused]] const uint32_t frame,
		 [[maybe_unused]] IPAFrameContext &frameContext,
		 ipu3_uapi_params *params)
{
	const struct ipu3_uapi_grid_config &grid = context.configuration.af.afGrid;
	params->acc_param.af.grid_cfg = grid;
	params->acc_param.af.filter_config = afFilterConfigDefault;

	/* Enable AF processing block */
	params->use.acc_af = 1;
}

/**
 * \brief Estimate variance
 * \param[in] y_items The AF filter data set from the IPU3 statistics buffer
 * \param[in] isY1 Selects between filter Y1 or Y2 to calculate the variance
 *
 * Calculate the mean of the data set provided by \a y_item, and then calculate
 * the variance of that data set from the mean.
 *
 * The operation can work on one of two sets of values contained within the
 * y_item data set supplied by the IPU3. The two data sets are the results of
 * both the Y1 and Y2 filters which are used to support coarse (Y1) and fine
 * (Y2) calculations of the contrast.
 *
 * \return The variance of the values in the data set \a y_item selected by \a isY1
 */
double Af::afEstimateVariance(std::span<const y_table_item_t> y_items, bool isY1)
{
	uint32_t total = 0;
	double mean;
	double var_sum = 0;

	for (auto y : y_items)
		total += isY1 ? y.y1_avg : y.y2_avg;

	mean = total / y_items.size();

	for (auto y : y_items) {
		double avg = isY1 ? y.y1_avg : y.y2_avg;
		var_sum += pow(avg - mean, 2);
	}

	return var_sum / y_items.size();
}

/**
 * \brief Determine the max contrast image and lens position
 * \param[in] context The IPA context
 * \param[in] frame The frame context sequence number
 * \param[in] frameContext The current frame context
 * \param[in] stats The statistics buffer of IPU3
 * \param[out] metadata Metadata for the frame, to be filled by the algorithm
 *
 * Ideally, a clear image also has a relatively higher contrast. So, every
 * image for each focus step should be tested to find an optimal focus step.
 *
 * The Hill Climbing Algorithm[1] is used to find the maximum variance of the
 * AF statistics which is the AF output of IPU3. The focus step is increased
 * then the variance of the AF statistics are estimated. If it finds the
 * negative derivative we have just passed the peak, and we infer that the best
 * focus is found.
 *
 * [1] Hill Climbing Algorithm, https://en.wikipedia.org/wiki/Hill_climbing
 */
void Af::process(IPAContext &context, [[maybe_unused]] const uint32_t frame,
		 [[maybe_unused]] IPAFrameContext &frameContext,
		 const ipu3_uapi_stats_3a *stats,
		 [[maybe_unused]] ControlList &metadata)
{
	/* Evaluate the AF buffer length */
	uint32_t afRawBufferLen = context.configuration.af.afGrid.width *
				  context.configuration.af.afGrid.height;

	ASSERT(afRawBufferLen < IPU3_UAPI_AF_Y_TABLE_MAX_SIZE);

	std::span y_items(reinterpret_cast<const y_table_item_t *>(&stats->af_raw_buffer.y_table),
			  afRawBufferLen);

	/*
	 * Calculate the mean and the variance of AF statistics for a given grid.
	 * For coarse: y1 are used.
	 * For fine: y2 results are used.
	 */
	double variance = afEstimateVariance(y_items, !afAlgo_.coarseCompleted());
	afAlgo_.process(context.activeState.af, variance);
}

REGISTER_IPA_ALGORITHM(Af, "Af")

} /* namespace ipa::ipu3::algorithms */

} /* namespace libcamera */
