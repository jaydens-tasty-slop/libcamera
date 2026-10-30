/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2026, Jayden Milne
 *
 * Autofocus
 */

#pragma once

#include <array>

#include <libipa/af_contrast.h>

#include "algorithm.h"

namespace libcamera {

namespace ipa::softisp::algorithms {

class Af : public Algorithm
{
public:
	int init(IPAContext &context, const ValueNode &tuningData) override;

	int configure(IPAContext &context, const IPAConfigInfo &configInfo) override;

	void queueRequest(IPAContext &context, const uint32_t frame,
			  IPAFrameContext &frameContext, const ControlList &controls) override;

	void prepare(IPAContext &context, const uint32_t frame,
		     IPAFrameContext &frameContext, DebayerParams *params) override;

	void process(IPAContext &context, const uint32_t frame,
		     IPAFrameContext &frameContext, const SwIspStats *stats,
		     ControlList &metadata) override;

private:
	static constexpr unsigned int kRegions =
		SwIspStats::kFocusGridWidth * SwIspStats::kFocusGridHeight;

	bool computeRegions(const IPAContext &context, const SwIspStats *stats);

	AfContrast af_;
	std::array<AfContrast::RegionStats, kRegions> regions_;
};

} /* namespace ipa::softisp::algorithms */

} /* namespace libcamera */
