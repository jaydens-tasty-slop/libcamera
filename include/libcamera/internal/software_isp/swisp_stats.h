/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2023, Linaro Ltd
 *
 * Statistics data format used by the software ISP and software IPA
 */

#pragma once

#include <array>
#include <stdint.h>

#include "libcamera/internal/vector.h"

namespace libcamera {

/**
 * \brief Statistics for the Software ISP
 *
 * The software ISP computes two types of statistics:
 *
 * - The accumulator counts the number of non-saturated pixels and sums their
 *   R, G and B channels individually. Pixels that have any saturated channel
 *   are skipped as they don't provide accurate colour information due to
 *   clipping.
 *
 * - The histogram groups pixels in bins based on their luminance and counts
 *   the number of pixels in each bin. Saturated pixels are included.
 *
 * - The focus statistics divide the statistics window in a grid of regions
 *   and measure the high frequency content of the green channel in each of
 *   them, for use by contrast detection autofocus algorithms.
 *
 * The value types are large enough to not overflow. Should they still overflow
 * for some reason, no check is performed and they wrap around.
 */
struct SwIspStats {
	/**
	 * \brief True if the statistics buffer contains valid data, false if
	 *        no statistics were generated for this frame
	 */
	bool valid;
	/**
	 * \brief Sums of colour channels of all the unsaturated pixels
	 */
	RGB<uint64_t> sum_;
	/**
	 * \brief Number of unsaturated pixels accumulated in \a sum_
	 */
	uint64_t numPixels_;
	/**
	 * \brief Number of bins in the yHistogram
	 */
	static constexpr unsigned int kYHistogramSize = 64;
	/**
	 * \brief Type of the histogram.
	 */
	using Histogram = std::array<uint32_t, kYHistogramSize>;
	/**
	 * \brief A histogram of luminance values of all the sampled pixels
	 */
	Histogram yHistogram;
	/**
	 * \brief Number of columns in the focus statistics grid
	 */
	static constexpr unsigned int kFocusGridWidth = 8;
	/**
	 * \brief Number of rows in the focus statistics grid
	 */
	static constexpr unsigned int kFocusGridHeight = 6;
	/**
	 * \brief Focus statistics for one region of the grid
	 *
	 * The values are scaled to 8 bits per pixel regardless of the input
	 * format bit depth.
	 */
	struct FocusRegion {
		/**
		 * \brief Sum of the squared differences between the green
		 * values of horizontally adjacent samples
		 */
		uint64_t contrast;
		/**
		 * \brief Sum of the green values of the samples
		 */
		uint64_t sum;
		/**
		 * \brief Number of samples accumulated in the region
		 */
		uint32_t count;
	};
	/**
	 * \brief Focus statistics of the grid regions, in raster scan order
	 */
	std::array<FocusRegion, kFocusGridWidth * kFocusGridHeight> focus;
};

} /* namespace libcamera */
