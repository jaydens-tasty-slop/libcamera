/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2026 Red Hat Inc.
 *
 * Autofocus
 */

#include "af.h"

#include <libcamera/base/log.h>

#include <libcamera/control_ids.h>

#include "libcamera/internal/software_isp/swisp_stats.h"

namespace libcamera {

namespace ipa::softisp::algorithms {

LOG_DEFINE_CATEGORY(IPASoftIspAf)

int Af::init(IPAContext &context, [[maybe_unused]] const ValueNode &tuningData)
{
	/* Informs that lens controls can get focus data */
	context.afEnabled = true;

	return 0;
}

int Af::configure(IPAContext &context,
		  [[maybe_unused]] const IPAConfigInfo &configInfo)
{
	afAlgo_.configure(context.activeState.af);

	return 0;
}

void Af::process(IPAContext &context,
		 [[maybe_unused]] const uint32_t frame,
		 [[maybe_unused]] IPAFrameContext &frameContext,
		 const SwIspStats *stats,
		 [[maybe_unused]] ControlList &metadata)
{
	const double variance = stats->variance;
	LOG(IPASoftIspAf, Debug) << "Focus stats: " << variance;
	afAlgo_.process(context.activeState.af, variance);
}

REGISTER_IPA_ALGORITHM(Af, "Af")

} /* namespace ipa::softisp::algorithms */

} /* namespace libcamera */
