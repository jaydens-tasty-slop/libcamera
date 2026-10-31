/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2021, Red Hat
 *
 * IPU3 Af algorithm
 */

#pragma once

#include <linux/intel-ipu3.h>

#include <libcamera/base/utils.h>

#include <libcamera/geometry.h>

#include "libipa/af.h"

#include "algorithm.h"

namespace libcamera {

namespace ipa::ipu3::algorithms {

class Af : public Algorithm
{
	/* The format of y_table. From ipu3-ipa repo */
	typedef struct __attribute__((packed)) y_table_item {
		uint16_t y1_avg;
		uint16_t y2_avg;
	} y_table_item_t;
public:
	Af();
	~Af() = default;

	int configure(IPAContext &context, const IPAConfigInfo &configInfo) override;
	void prepare(IPAContext &context, const uint32_t frame,
		     IPAFrameContext &frameContext,
		     ipu3_uapi_params *params) override;
	void process(IPAContext &context, const uint32_t frame,
		     IPAFrameContext &frameContext,
		     const ipu3_uapi_stats_3a *stats,
		     ControlList &metadata) override;

private:
	double afEstimateVariance(std::span<const y_table_item_t> y_items, bool isY1);

	AfAlgorithm afAlgo_;
};

} /* namespace ipa::ipu3::algorithms */

} /* namespace libcamera */
