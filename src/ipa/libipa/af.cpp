/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2021-2026 Red Hat
 *
 * Autofocus algorithm
 */

#include "af.h"

#include <linux/v4l2-controls.h>

namespace libcamera {

namespace ipa {

LOG_DEFINE_CATEGORY(Af)

namespace af {

/**
 * \struct ActiveState
 * \brief Autofocus algorithm state
 *
 * \var ActiveState::focus
 * \brief The focus value for the V4L2 control
 *
 * \var ActiveState::maxVariance
 * \brief Maximum variance of the AF statistics
 *
 * \var ActiveState::stable
 * \brief Flag determining whether focus is in a stable state
 */

} /* namespace af */

/**
 * \class AfAlgorithm
 * \brief Implementation of a common autofocus algorithm
 *
 * The algorithm is provided with a single variance value in
 * \a AfAlgorithm::process and adjusts \a ActiveState::focus accordingly.
 */

/**
 * Default maximum focus steps of the VCM control
 */
static constexpr uint32_t kMaxFocusSteps = 1023;

/* Minimum focus step for searching appropriate focus */
static constexpr uint32_t kCoarseSearchStep = 30;
static constexpr uint32_t kFineSearchStep = 1;

/* Max ratio of variance change, 0.0 < kMaxChange < 1.0 */
static constexpr double kMaxChange = 0.5;

/* The numbers of frame to be ignored, before performing focus scan. */
static constexpr uint32_t kIgnoreFrame = 10;

/* Fine scan range 0 < kFineRange < 1 */
static constexpr double kFineRange = 0.05;

AfAlgorithm::AfAlgorithm()
	: focus_(0), bestFocus_(0), currentVariance_(0.0), previousVariance_(0.0),
	  coarseCompleted_(false), fineCompleted_(false)
{
}

/**
 * \fn AfAlgorithm::coarseCompleted()
 * \brief Return whether the coarse autofocus scan is completed
 */

/**
 * \brief AF coarse scan
 * \param[in] state The autofocus state
 *
 * Find a near focused image using a coarse step. The step is determined by
 * kCoarseSearchStep.
 */
void AfAlgorithm::afCoarseScan(af::ActiveState &state)
{
	if (coarseCompleted_)
		return;

	if (afNeedIgnoreFrame())
		return;

	if (afScan(state, kCoarseSearchStep)) {
		coarseCompleted_ = true;
		state.maxVariance = 0;
		focus_ = state.focus -
			 (state.focus * kFineRange);
		state.focus = focus_;
		previousVariance_ = 0;
		maxStep_ = std::clamp(focus_ + static_cast<uint32_t>((focus_ * kFineRange)),
				      0U, kMaxFocusSteps);
	}
}

/**
 * \brief AF fine scan
 * \param[in] state The autofocus state
 *
 * Find an optimum lens position with moving 1 step for each search.
 */
void AfAlgorithm::afFineScan(af::ActiveState &state)
{
	if (!coarseCompleted_)
		return;

	if (afNeedIgnoreFrame())
		return;

	if (afScan(state, kFineSearchStep)) {
		state.stable = true;
		fineCompleted_ = true;
	}
}

/**
 * \brief AF reset
 * \param[in] state The autofocus state
 *
 * Reset all the parameters to start over the AF process.
 */
void AfAlgorithm::afReset(af::ActiveState &state)
{
	if (afNeedIgnoreFrame())
		return;

	state.maxVariance = 0;
	state.focus = 0;
	focus_ = 0;
	state.stable = false;
	ignoreCounter_ = kIgnoreFrame;
	previousVariance_ = 0.0;
	coarseCompleted_ = false;
	fineCompleted_ = false;
	maxStep_ = initMaxStep_;
}

/**
 * \brief AF variance comparison
 * \param[in] state The autofocus state
 * \param[in] min_step The VCM movement step
 *
 * We always pick the largest variance to replace the previous one. The image
 * with a larger variance also indicates it is a clearer image than previous
 * one. If we find a negative derivative, we return immediately.
 *
 * \return True, if it finds a AF value
 */
bool AfAlgorithm::afScan(af::ActiveState &state, int min_step)
{
	if (focus_ > maxStep_) {
		/* If reach the max step, move lens to the position. */
		state.focus = bestFocus_;
		return true;
	} else {
		/*
		 * Find the maximum of the variance by estimating its
		 * derivative. If the direction changes, it means we have
		 * passed a maximum one step before.
		 */
		if ((currentVariance_ - state.maxVariance) >=
		    -(state.maxVariance * 0.1)) {
			/*
			 * Positive and zero derivative:
			 * The variance is still increasing. The focus could be
			 * increased for the next comparison. Also, the max variance
			 * and previous focus value are updated.
			 */
			bestFocus_ = focus_;
			focus_ += min_step;
			state.focus = focus_;
			state.maxVariance = currentVariance_;
		} else {
			/*
			 * Negative derivative:
			 * The variance starts to decrease which means the maximum
			 * variance is found. Set focus step to previous good one
			 * then return immediately.
			 */
			state.focus = bestFocus_;
			return true;
		}
	}

	previousVariance_ = currentVariance_;
	LOG(Af, Debug) << " Previous step is "
		       << bestFocus_
		       << " Current step is "
		       << focus_;
	return false;
}

/**
 * \brief Determine the frame to be ignored
 * \return Return True if the frame should be ignored, false otherwise
 */
bool AfAlgorithm::afNeedIgnoreFrame()
{
	if (ignoreCounter_ == 0)
		return false;
	else
		ignoreCounter_--;
	return true;
}

/**
 * \brief Reset frame ignore counter
 */
void AfAlgorithm::afIgnoreFrameReset()
{
	ignoreCounter_ = kIgnoreFrame;
}

/**
 * \brief Determine out-of-focus situation
 * \param[in] state The autofocus state
 *
 * Out-of-focus means that the variance change rate for a focused and a new
 * variance is greater than a threshold.
 *
 * \return True if the variance threshold is crossed indicating lost focus,
 * false otherwise
 */
bool AfAlgorithm::afIsOutOfFocus(af::ActiveState &state)
{
	const uint32_t diff_var = std::abs(currentVariance_ -
					   state.maxVariance);
	const double var_ratio = diff_var / state.maxVariance;

	LOG(Af, Debug) << "Variance change rate: "
		       << var_ratio
		       << " Current VCM step: "
		       << state.focus;

	if (var_ratio > kMaxChange)
		return true;
	else
		return false;
}

/**
 * \brief Configure the algorithm and \a state
 * \param[in] state The autofocus state
 * \param[in] lensControls The lens controls
 *
 * The method just sets some initial variable values rather than performing real
 * configuration but it is supposed to be called from IPA's `configure`.
 */
void AfAlgorithm::configure(af::ActiveState &state, const ControlInfoMap &lensControls)
{
	/* Initial max focus step */
	initMaxStep_ = kMaxFocusSteps;

	/* Initial frame ignore counter */
	afIgnoreFrameReset();

	/* Initial focus value */
	state.focus = 0;
	/* Maximum variance of the AF statistics */
	state.maxVariance = 0;
	/* The stable AF value flag. if it is true, the AF should be in a stable state. */
	state.stable = false;

	const auto it = lensControls.find(V4L2_CID_FOCUS_ABSOLUTE);
	if (it != lensControls.end()) {
		const ControlInfo &focusInfo = it->second;
		if (!focusInfo.max().isNone())
			initMaxStep_ = static_cast<uint32_t>(focusInfo.max().get<int32_t>());
		if (!focusInfo.def().isNone())
			state.focus = static_cast<uint32_t>(focusInfo.def().get<int32_t>());
	}

	maxStep_ = initMaxStep_;

	LOG(Af, Debug) << "Configured focus parameters: max=" << initMaxStep_
		       << " focus=" << state.focus;
}

/**
 * \brief Determine the max contrast image and lens position
 * \param[in] state The autofocus state
 * \param[in] variance The variance of autofocus statistics
 *
 * Ideally, a clear image also has a relatively higher contrast. So, every
 * image for each focus step should be tested to find an optimal focus step.
 *
 * The Hill Climbing Algorithm[1] is used to find the maximum variance of the AF
 * statistics. The focus step is increased then the variance of the AF
 * statistics are estimated. If it finds the negative derivative we have just
 * passed the peak, and we infer that the best focus is found.
 *
 * [1] Hill Climbing Algorithm, https://en.wikipedia.org/wiki/Hill_climbing
 */
void AfAlgorithm::process(af::ActiveState &state, double variance)
{
	currentVariance_ = variance;

	if (!state.stable) {
		afCoarseScan(state);
		afFineScan(state);
	} else {
		if (afIsOutOfFocus(state))
			afReset(state);
		else
			afIgnoreFrameReset();
	}
}

} /* namespace ipa */

} /* namespace libcamera */
