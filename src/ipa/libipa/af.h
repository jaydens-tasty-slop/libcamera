/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2021-2026 Red Hat
 *
 * Autofocus algorithm
 */

#pragma once

#include <linux/v4l2-controls.h>

#include <libcamera/base/log.h>

#include <libcamera/control_ids.h>

namespace libcamera {

namespace ipa {

LOG_DECLARE_CATEGORY(Af)

namespace af {

struct ActiveState {
	uint32_t focus;
	double maxVariance;
	bool stable;
};

inline void prepareControls(ControlList &controls, int32_t focus)
{
	controls.set(V4L2_CID_FOCUS_ABSOLUTE, focus);
}

} /* namespace af */

class AfAlgorithm
{
public:
	AfAlgorithm();

	void configure(af::ActiveState &state);
	void process(af::ActiveState &state, double variance);
	bool coarseCompleted() { return coarseCompleted_; }

private:
	void afCoarseScan(af::ActiveState &af);
	void afFineScan(af::ActiveState &af);
	bool afScan(af::ActiveState &af, int min_step);
	void afReset(af::ActiveState &af);
	bool afNeedIgnoreFrame();
	void afIgnoreFrameReset();

	bool afIsOutOfFocus(af::ActiveState &af);

	/* VCM step configuration. It is the current setting of the VCM step. */
	uint32_t focus_;
	/* The best VCM step. It is a local optimum VCM step during scanning. */
	uint32_t bestFocus_;
	/* Current AF statistic variance. */
	double currentVariance_;
	/* The frames are ignore before starting measuring. */
	uint32_t ignoreCounter_;
	/* It is used to determine the derivative during scanning */
	double previousVariance_;
	/* The designated maximum range of focus scanning. */
	uint32_t maxStep_;
	/* If the coarse scan completes, it is set to true. */
	bool coarseCompleted_;
	/* If the fine scan completes, it is set to true. */
	bool fineCompleted_;
};

} /* namespace ipa */

} /* namespace libcamera */
