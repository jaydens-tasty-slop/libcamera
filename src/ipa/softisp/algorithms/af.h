/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2026 Red Hat Inc.
 *
 * Autofocus
 */

#pragma once

#include "libipa/af.h"

#include "algorithm.h"
#include "ipa_context.h"

namespace libcamera {

namespace ipa::softisp::algorithms {

class Af : public Algorithm
{
public:
	Af() = default;
	~Af() = default;

	int init(IPAContext &context, const ValueNode &tuningData) override;
	int configure(IPAContext &context,
		      const IPAConfigInfo &configInfo) override;
	void process(IPAContext &context, const uint32_t frame,
		     IPAFrameContext &frameContext,
		     const SwIspStats *stats,
		     ControlList &metadata) override;

private:
	AfAlgorithm afAlgo_;
};

} /* namespace ipa::softisp::algorithms */

} /* namespace libcamera */
