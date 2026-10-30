/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2022-2023, Raspberry Pi Ltd
 * Copyright (C) 2026, Jayden Milne
 *
 * Contrast detection autofocus algorithm
 */

#include "af_contrast.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include <linux/v4l2-controls.h>

#include <libcamera/base/log.h>

/**
 * \file af_contrast.h
 * \brief Contrast detection autofocus algorithm
 */

namespace libcamera {

LOG_DEFINE_CATEGORY(AfContrast)

namespace ipa {

namespace {

/* Upper end of the lens map used when the tuning file doesn't provide one. */
constexpr double kDefaultMapMax = 15.0;

/* Number of positions sampled by the fine scan. */
constexpr unsigned int kFineSteps = 5;

/*
 * Relative change of the exposure between two measurements below which the
 * exposure is considered stable, to ignore rounding and small adjustments.
 */
constexpr double kExposureTolerance = 0.01;

} /* namespace */

namespace af {

/**
 * \struct FrameContext
 * \brief Per-frame autofocus state
 *
 * \var FrameContext::lensPosition
 * \brief Lens position in dioptres when the frame was captured
 */

} /* namespace af */

/**
 * \class AfContrast
 * \brief Contrast detection autofocus algorithm
 *
 * The AfContrast class implements the libcamera autofocus controls on top of
 * a contrast measurement provided by the IPA module for a grid of regions.
 * Its structure follows the contrast detection part of the Raspberry Pi
 * autofocus algorithm: lens positions are expressed in dioptres and
 * converted to lens driver units through a piecewise linear map, and the
 * tuning data describes the scan ranges and speeds in the same way.
 *
 * A scan steps the lens in coarse steps until the contrast drops below a
 * fraction of the maximum seen so far, or a range limit is reached. When the
 * scan starts in the middle of the range and the peak may lie behind the
 * starting position, the scan reverses. A fine scan around the interpolated
 * peak then refines the result before the lens settles on the final
 * position.
 *
 * In continuous mode, the algorithm watches the contrast and brightness of
 * the focus area once a scan completes. When either changes by more than a
 * configurable ratio and then stays stable for a number of frames, a new scan
 * is started. A change of exposure changes both, by up to the ratio of the
 * exposures, which is allowed for, so that the exposure control converging
 * after a scan doesn't look like a scene change.
 *
 * The statistics must be measured on frames captured after the lens has
 * settled. The algorithm ignores statistics until the configured number of
 * frames has elapsed after each lens movement, counted from the last frame
 * captured before the movement as recorded by prepare(). The statistics of a
 * frame can thus be processed any number of frames after its capture, as with
 * a software ISP that falls behind the sensor.
 *
 * The tuning data supports the following keys, all of them optional:
 *
 * - ranges: The normal, macro and full ranges, each with min, max and default
 *   lens positions in dioptres
 * - speeds: The normal and fast speeds, each with step_coarse and step_fine
 *   (in dioptres), contrast_ratio, retrigger_ratio, retrigger_delay (in
 *   frames), max_slew (in dioptres per frame) and step_frames (the number of
 *   frames to wait after a lens movement, counted from the last frame
 *   captured before the movement)
 * - skip_frames: The number of frames to skip at start
 * - map: A piecewise linear function from dioptres to lens driver units
 */

/**
 * \struct AfContrast::RegionStats
 * \brief Focus statistics for one region of the grid
 *
 * \var AfContrast::RegionStats::contrast
 * \brief The focus figure of merit of the region
 *
 * The value must not depend on the exposure time and gain, larger values
 * indicating sharper images. It is reported, rounded to an integer and
 * weighted over the focus area, through the FocusFoM control.
 *
 * \var AfContrast::RegionStats::average
 * \brief The average brightness of the region, used to detect scene changes
 */

void AfContrast::RangeParams::read(const ValueNode &params)
{
	focusMin = params["min"].get<double>(focusMin);
	focusMax = params["max"].get<double>(focusMax);
	focusDefault = params["default"].get<double>(focusDefault);
}

void AfContrast::SpeedParams::read(const ValueNode &params)
{
	stepCoarse = params["step_coarse"].get<double>(stepCoarse);
	stepFine = params["step_fine"].get<double>(stepFine);
	contrastRatio = params["contrast_ratio"].get<double>(contrastRatio);
	retriggerRatio = params["retrigger_ratio"].get<double>(retriggerRatio);
	retriggerDelay = params["retrigger_delay"].get<uint32_t>(retriggerDelay);
	maxSlew = params["max_slew"].get<double>(maxSlew);
	stepFrames = params["step_frames"].get<uint32_t>(stepFrames);
}

void AfContrast::parseTuning(const ValueNode &tuningData)
{
	const ValueNode &ranges = tuningData["ranges"];

	RangeParams &normal = ranges_[controls::AfRangeNormal];
	normal = { 0.0, 12.0, 1.0 };
	normal.read(ranges["normal"]);

	RangeParams &macro = ranges_[controls::AfRangeMacro];
	macro = normal;
	macro.read(ranges["macro"]);

	RangeParams &full = ranges_[controls::AfRangeFull];
	full = {
		std::min(normal.focusMin, macro.focusMin),
		std::max(normal.focusMax, macro.focusMax),
		normal.focusDefault,
	};
	full.read(ranges["full"]);

	const ValueNode &speeds = tuningData["speeds"];

	SpeedParams &normalSpeed = speeds_[controls::AfSpeedNormal];
	normalSpeed = { 1.0, 0.25, 0.75, 0.75, 10, 2.0, 4 };
	normalSpeed.read(speeds["normal"]);

	SpeedParams &fastSpeed = speeds_[controls::AfSpeedFast];
	fastSpeed = normalSpeed;
	fastSpeed.read(speeds["fast"]);

	skipFrames_ = tuningData["skip_frames"].get<uint32_t>(5);

	map_ = tuningData["map"].get<Pwl>(Pwl{});
	if (map_.size() < 2) {
		LOG(AfContrast, Warning)
			<< "No lens map in tuning data, lens positions are uncalibrated";
		map_ = Pwl({
			Pwl::Point({ 0.0, static_cast<double>(lensMin_) }),
			Pwl::Point({ kDefaultMapMax, static_cast<double>(lensMax_) }),
		});
	}

	/* Keep the scan ranges within the positions the lens can reach. */
	Pwl::Interval limits = map_.domain();
	for (RangeParams &range : ranges_) {
		range.focusMin = limits.clamp(range.focusMin);
		range.focusMax = limits.clamp(range.focusMax);
		range.focusDefault = std::clamp(range.focusDefault,
						range.focusMin, range.focusMax);
	}
}

/**
 * \brief Initialise the algorithm
 * \param[in] tuningData The tuning data for the algorithm
 * \param[in] lensControls The V4L2 controls of the focus lens
 * \param[inout] controls The control info map to register controls in
 *
 * The autofocus controls are only registered when \a lensControls contains a
 * V4L2_CID_FOCUS_ABSOLUTE control. Without a lens, the algorithm only reports
 * the FocusFoM metadata.
 *
 * \return 0 on success, a negative error code otherwise
 */
int AfContrast::init(const ValueNode &tuningData,
		     const ControlInfoMap &lensControls,
		     ControlInfoMap::Map &controls)
{
	auto focus = lensControls.find(V4L2_CID_FOCUS_ABSOLUTE);
	lensPresent_ = focus != lensControls.end();
	if (!lensPresent_) {
		LOG(AfContrast, Debug) << "No focus lens, autofocus disabled";
		return 0;
	}

	lensMin_ = focus->second.min().get<int32_t>();
	lensMax_ = focus->second.max().get<int32_t>();

	parseTuning(tuningData);

	Pwl::Interval limits = map_.domain();

	controls[&controls::AfMode] = ControlInfo(controls::AfModeValues);
	controls[&controls::AfRange] = ControlInfo(controls::AfRangeValues);
	controls[&controls::AfSpeed] = ControlInfo(controls::AfSpeedValues);
	controls[&controls::AfMetering] = ControlInfo(controls::AfMeteringValues);
	controls[&controls::AfWindows] =
		ControlInfo(Rectangle{}, Rectangle(65535, 65535, 65535, 65535),
			    std::span<const Rectangle, 1>{ { Rectangle{} } });
	controls[&controls::AfTrigger] = ControlInfo(controls::AfTriggerValues);
	controls[&controls::AfPause] = ControlInfo(controls::AfPauseValues);
	controls[&controls::LensPosition] =
		ControlInfo(static_cast<float>(limits.start),
			    static_cast<float>(limits.end),
			    static_cast<float>(ranges_[controls::AfRangeNormal].focusDefault));

	return 0;
}

/**
 * \brief Configure the algorithm for a streaming session
 * \param[in] lensControls The V4L2 controls of the focus lens
 * \param[in] grid The size of the focus statistics grid, in regions
 * \param[in] statsRegion The area covered by the statistics grid, in the
 * coordinates of the ScalerCropMaximum property
 *
 * The first time the camera is configured, the lens is driven to the default
 * position of the normal range.
 *
 * \return 0 on success, a negative error code otherwise
 */
int AfContrast::configure(const ControlInfoMap &lensControls, const Size &grid,
			  const Rectangle &statsRegion)
{
	grid_ = grid;
	statsRegion_ = statsRegion;
	computeWeights();

	if (!lensPresent_)
		return 0;

	auto focus = lensControls.find(V4L2_CID_FOCUS_ABSOLUTE);
	if (focus == lensControls.end()) {
		LOG(AfContrast, Error) << "Focus lens control not available";
		return -EINVAL;
	}

	lensMin_ = focus->second.min().get<int32_t>();
	lensMax_ = focus->second.max().get<int32_t>();

	if (!initialised_)
		target_ = ranges_[range_].focusDefault;

	skipCount_ = skipFrames_;
	lastFrame_.reset();
	lastCapturedFrame_.reset();
	lastMoveFrame_.reset();
	sceneContrast_.reset();
	sceneChangeFrame_.reset();

	/* A scan interrupted by a reconfiguration starts again. */
	if (scanning())
		scanState_ = ScanState::Trigger;

	return 0;
}

/**
 * \brief Handle the autofocus controls of a request
 * \param[in] controls The request controls
 */
void AfContrast::queueRequest(const ControlList &controls)
{
	if (!lensPresent_)
		return;

	/* The mode is handled first, the other controls depend on it. */
	if (const auto &mode = controls.get(controls::AfMode))
		setMode(static_cast<controls::AfModeEnum>(*mode));

	if (const auto &range = controls.get(controls::AfRange))
		range_ = static_cast<controls::AfRangeEnum>(*range);

	if (const auto &speed = controls.get(controls::AfSpeed))
		speed_ = static_cast<controls::AfSpeedEnum>(*speed);

	if (const auto &metering = controls.get(controls::AfMetering)) {
		useWindows_ = *metering == controls::AfMeteringWindows;
		computeWeights();
	}

	if (const auto &windows = controls.get(controls::AfWindows)) {
		windows_.assign(windows->begin(), windows->end());
		computeWeights();
	}

	if (const auto &pause = controls.get(controls::AfPause))
		setPause(static_cast<controls::AfPauseEnum>(*pause));

	if (const auto &trig = controls.get(controls::AfTrigger)) {
		if (*trig == controls::AfTriggerStart)
			trigger();
		else
			cancel();
	}

	if (const auto &position = controls.get(controls::LensPosition)) {
		if (mode_ == controls::AfModeManual)
			target_ = map_.domain().clamp(*position);
		else
			LOG(AfContrast, Debug)
				<< "LensPosition ignored outside of manual mode";
	}
}

/**
 * \brief Record the lens position for a frame
 * \param[in] frame The frame number
 * \param[out] frameContext The frame context
 *
 * This function shall be called when a frame is captured, in capture order, to
 * record the lens position reported in the frame metadata. The algorithm also
 * times lens movements against the frames captured: the statistics of the
 * frames captured until the lens has settled are ignored, whenever they are
 * processed.
 */
void AfContrast::prepare(uint32_t frame, af::FrameContext &frameContext)
{
	/* Frame numbers restart from zero when the camera is restarted. */
	if (lastCapturedFrame_ && frame < *lastCapturedFrame_)
		restart();
	lastCapturedFrame_ = frame;

	frameContext.lensPosition = position_;
}

/**
 * \brief Process the focus statistics of a frame
 * \param[in] frame The frame number
 * \param[in] frameContext The frame context
 * \param[in] stats The focus statistics, empty if none are available for
 * the frame
 * \param[in] exposure The exposure of the frame, the product of its exposure
 * time and gain, positive and in any unit as long as it is the same for all
 * frames
 * \param[out] metadata The frame metadata
 *
 * The \a stats regions are stored in raster scan order and their number must
 * match the grid size passed to configure(). This function shall be called
 * for every frame, with or without statistics, as lens movements are rate
 * limited on a per-frame basis.
 */
void AfContrast::process(uint32_t frame, const af::FrameContext &frameContext,
			 std::span<const RegionStats> stats, double exposure,
			 ControlList &metadata)
{
	double contrast = 0.0;
	double average = 0.0;
	bool valid = combineStats(stats, &contrast, &average);

	if (valid) {
		constexpr double kMaxFoM = std::numeric_limits<int32_t>::max();
		metadata.set(controls::FocusFoM,
			     static_cast<int32_t>(std::min(std::round(contrast), kMaxFoM)));
	}

	if (!lensPresent_)
		return;

	if (lastFrame_ && frame < *lastFrame_)
		restart();
	lastFrame_ = frame;

	if (skipCount_)
		skipCount_--;

	const bool settled = !lastMoveFrame_ ||
			     frame >= *lastMoveFrame_ + speeds_[speed_].stepFrames;

	if (scanState_ == ScanState::Trigger)
		startScan();
	else if (mode_ != controls::AfModeManual && valid && settled && !skipCount_)
		doAf(frame, contrast, average, exposure);

	updateLensPosition(frame);

	controls::AfStateEnum state = mode_ == controls::AfModeManual
					      ? controls::AfStateIdle
					      : reportState_;

	controls::AfPauseStateEnum pauseState = controls::AfPauseStateRunning;
	if (mode_ == controls::AfModeContinuous && pauseFlag_)
		pauseState = scanning() ? controls::AfPauseStatePausing
					: controls::AfPauseStatePaused;

	metadata.set(controls::AfState, state);
	metadata.set(controls::AfPauseState, pauseState);
	metadata.set(controls::LensPosition,
		     static_cast<float>(frameContext.lensPosition));
}

/**
 * \brief Retrieve the lens position to apply
 *
 * This function shall be called after process(), and the returned value sent
 * to the lens driver straight away.
 *
 * \return The lens position in lens driver units if the lens has to move,
 * std::nullopt otherwise
 */
std::optional<int32_t> AfContrast::lensSetting()
{
	if (!lensChanged_)
		return std::nullopt;

	lensChanged_ = false;

	double setting = map_.eval(map_.domain().clamp(position_));

	return std::clamp(static_cast<int32_t>(std::lround(setting)),
			  lensMin_, lensMax_);
}

void AfContrast::setMode(controls::AfModeEnum mode)
{
	if (mode == mode_)
		return;

	LOG(AfContrast, Debug) << "Mode " << mode_ << " -> " << mode;

	mode_ = mode;
	pauseFlag_ = false;

	if (mode == controls::AfModeContinuous) {
		scanState_ = ScanState::Trigger;
		reportState_ = controls::AfStateScanning;
	} else {
		goIdle();
	}
}

void AfContrast::setPause(controls::AfPauseEnum pause)
{
	if (mode_ != controls::AfModeContinuous)
		return;

	switch (pause) {
	case controls::AfPauseImmediate:
		pauseFlag_ = true;
		if (scanning()) {
			scanState_ = ScanState::Idle;
			scanData_.clear();
			target_ = position_;
		}
		break;

	case controls::AfPauseDeferred:
		pauseFlag_ = true;
		break;

	case controls::AfPauseResume:
		if (!pauseFlag_)
			break;

		pauseFlag_ = false;

		/* Restart a scan interrupted by an immediate pause. */
		if (scanState_ == ScanState::Idle &&
		    reportState_ == controls::AfStateScanning) {
			scanState_ = ScanState::Trigger;
		} else {
			sceneContrast_.reset();
			sceneChangeFrame_.reset();
		}
		break;
	}
}

void AfContrast::trigger()
{
	if (mode_ != controls::AfModeAuto || scanning())
		return;

	scanState_ = ScanState::Trigger;
	reportState_ = controls::AfStateScanning;
}

void AfContrast::cancel()
{
	if (mode_ != controls::AfModeAuto || !scanning())
		return;

	goIdle();
}

void AfContrast::goIdle()
{
	scanState_ = ScanState::Idle;
	reportState_ = controls::AfStateIdle;
	scanData_.clear();

	/* Leave the lens where it is. */
	target_ = position_;
}

void AfContrast::computeWeights()
{
	weights_.assign(grid_.width * grid_.height, 0.0);

	if (grid_.isNull() || !statsRegion_.width || !statsRegion_.height)
		return;

	std::vector<Rectangle> windows;
	if (useWindows_) {
		for (const Rectangle &window : windows_) {
			Rectangle area = window.boundedTo(statsRegion_);
			if (area.width && area.height)
				windows.push_back(area);
		}
	}

	/* Default to the centre half of the width and third of the height. */
	if (windows.empty())
		windows.emplace_back(statsRegion_.x + statsRegion_.width / 4,
				     statsRegion_.y + statsRegion_.height / 3,
				     statsRegion_.width / 2,
				     statsRegion_.height / 3);

	for (unsigned int y = 0; y < grid_.height; y++) {
		int top = statsRegion_.y + statsRegion_.height * y / grid_.height;
		int bottom = statsRegion_.y + statsRegion_.height * (y + 1) / grid_.height;

		for (unsigned int x = 0; x < grid_.width; x++) {
			int left = statsRegion_.x + statsRegion_.width * x / grid_.width;
			int right = statsRegion_.x + statsRegion_.width * (x + 1) / grid_.width;
			Rectangle cell(left, top, right - left, bottom - top);

			double cellArea = static_cast<double>(cell.width) * cell.height;
			if (!cellArea)
				continue;

			double &weight = weights_[y * grid_.width + x];
			for (const Rectangle &window : windows) {
				Rectangle overlap = window.boundedTo(cell);
				weight += static_cast<double>(overlap.width) *
					  overlap.height / cellArea;
			}
		}
	}
}

bool AfContrast::combineStats(std::span<const RegionStats> stats,
			      double *contrast, double *average) const
{
	if (stats.empty() || stats.size() != weights_.size())
		return false;

	double weightSum = 0.0;
	double contrastSum = 0.0;
	double averageSum = 0.0;

	for (unsigned int i = 0; i < stats.size(); i++) {
		weightSum += weights_[i];
		contrastSum += weights_[i] * stats[i].contrast;
		averageSum += weights_[i] * stats[i].average;
	}

	if (weightSum <= 0.0)
		return false;

	*contrast = contrastSum / weightSum;
	*average = averageSum / weightSum;

	return true;
}

void AfContrast::doAf(uint32_t frame, double contrast, double average,
		      double exposure)
{
	switch (scanState_) {
	case ScanState::Idle:
		if (mode_ == controls::AfModeContinuous && !pauseFlag_)
			checkSceneChange(frame, contrast, average, exposure);
		break;

	case ScanState::Coarse1:
	case ScanState::Coarse2:
	case ScanState::Fine:
		if (position_ == target_)
			doScan(contrast);
		break;

	case ScanState::Settle:
		if (position_ == target_)
			finishScan(contrast, average, exposure);
		break;

	case ScanState::Trigger:
		break;
	}
}

void AfContrast::startScan()
{
	const RangeParams &range = ranges_[range_];
	const SpeedParams &speed = speeds_[speed_];

	scanData_.clear();
	scanMaxContrast_ = 0.0;
	scanMinContrast_ = std::numeric_limits<double>::max();
	sceneChangeFrame_.reset();
	reportState_ = controls::AfStateScanning;

	/*
	 * A continuous autofocus scan starts from the current position when it
	 * is far enough from the range limits, towards infinity first. Other
	 * scans sweep the whole range from one end.
	 */
	const double margin = 2.0 * speed.stepCoarse;
	const bool continuous = mode_ == controls::AfModeContinuous && initialised_;

	if (continuous && position_ > range.focusMin + margin &&
	    position_ < range.focusMax - margin) {
		scanState_ = ScanState::Coarse1;
		scanStep_ = -speed.stepCoarse;
		target_ = position_;
	} else if (continuous && position_ >= range.focusMax - margin) {
		scanState_ = ScanState::Coarse2;
		scanStep_ = -speed.stepCoarse;
		target_ = range.focusMax;
	} else {
		scanState_ = ScanState::Coarse2;
		scanStep_ = speed.stepCoarse;
		target_ = range.focusMin;
	}

	LOG(AfContrast, Debug)
		<< "Start scan at " << target_ << ", step " << scanStep_;
}

void AfContrast::doScan(double contrast)
{
	const RangeParams &range = ranges_[range_];
	const SpeedParams &speed = speeds_[speed_];

	scanMaxContrast_ = std::max(scanMaxContrast_, contrast);
	scanMinContrast_ = std::min(scanMinContrast_, contrast);
	scanData_.push_back({ target_, contrast });

	LOG(AfContrast, Debug)
		<< "Scan position " << target_ << ", contrast " << contrast;

	double phaseMax = 0.0;
	for (const ScanRecord &record : scanData_)
		phaseMax = std::max(phaseMax, record.contrast);

	const bool limit = (scanStep_ > 0.0 && target_ >= range.focusMax) ||
			   (scanStep_ < 0.0 && target_ <= range.focusMin);
	const bool dropped = contrast < speed.contrastRatio * phaseMax;
	const bool complete = scanState_ == ScanState::Fine &&
			      scanData_.size() >= kFineSteps;

	if (!limit && !dropped && !complete) {
		target_ = std::clamp(target_ + scanStep_, range.focusMin,
				     range.focusMax);
		return;
	}

	/*
	 * If the first scan didn't see the contrast drop on the side of its
	 * starting position, the peak may lie beyond it. Scan in the other
	 * direction, starting from the next position past the start.
	 */
	if (scanState_ == ScanState::Coarse1 &&
	    scanData_.front().contrast >= speed.contrastRatio * phaseMax) {
		scanState_ = ScanState::Coarse2;
		scanStep_ = -scanStep_;
		target_ = std::clamp(scanData_.front().position + scanStep_,
				     range.focusMin, range.focusMax);
		return;
	}

	double peak = findPeak(scanData_);

	if (scanState_ != ScanState::Fine && speed.stepFine > 0.0) {
		/* Approach the peak from the side of the current position. */
		scanState_ = ScanState::Fine;
		scanStep_ = target_ < peak ? speed.stepFine : -speed.stepFine;
		target_ = std::clamp(peak - 2.0 * scanStep_, range.focusMin,
				     range.focusMax);
		scanData_.clear();
		return;
	}

	scanState_ = ScanState::Settle;
	target_ = peak;
}

void AfContrast::finishScan(double contrast, double average, double exposure)
{
	const double ratio = speeds_[speed_].contrastRatio;

	/*
	 * The scan succeeds if the contrast at the final position is close to
	 * the maximum, and if the scan saw a clear peak.
	 */
	const bool focused = contrast >= ratio * scanMaxContrast_ &&
			     scanMinContrast_ <= ratio * scanMaxContrast_;

	reportState_ = focused ? controls::AfStateFocused : controls::AfStateFailed;
	scanState_ = ScanState::Idle;
	scanData_.clear();

	sceneContrast_ = contrast;
	sceneAverage_ = average;
	sceneExposure_ = exposure;
	lastExposure_ = exposure;
	sceneChangeFrame_.reset();

	LOG(AfContrast, Debug)
		<< "Scan " << (focused ? "focused" : "failed") << " at " << position_
		<< ", contrast " << contrast << " (range " << scanMinContrast_
		<< " - " << scanMaxContrast_ << ")";
}

void AfContrast::checkSceneChange(uint32_t frame, double contrast,
				  double average, double exposure)
{
	const SpeedParams &speed = speeds_[speed_];
	const double previousExposure = lastExposure_;
	lastExposure_ = exposure;

	if (!sceneContrast_) {
		sceneContrast_ = contrast;
		sceneAverage_ = average;
		sceneExposure_ = exposure;
		return;
	}

	/*
	 * A change of exposure changes the brightness and, through noise and
	 * clipping, the contrast, by up to the ratio of the exposures.
	 */
	const double exposureRatio = std::max(exposure / sceneExposure_,
					      sceneExposure_ / exposure);
	const double ratio = speed.retriggerRatio / exposureRatio;
	const bool changed = contrast < ratio * *sceneContrast_ ||
			     *sceneContrast_ < ratio * contrast ||
			     average < ratio * sceneAverage_ ||
			     sceneAverage_ < ratio * average;

	/* Wait for the scene to be stable again after a change. */
	if (changed) {
		sceneContrast_ = contrast;
		sceneAverage_ = average;
		sceneExposure_ = exposure;
		sceneChangeFrame_ = frame;
		return;
	}

	/*
	 * Once the exposure has settled at a new value, compare the next frames
	 * with this one, without the allowance for the exposure change.
	 */
	const bool settled = std::abs(exposure / previousExposure - 1.0) < kExposureTolerance;
	if (settled && exposureRatio > 1.0 + kExposureTolerance) {
		sceneContrast_ = contrast;
		sceneAverage_ = average;
		sceneExposure_ = exposure;
	}

	if (sceneChangeFrame_ && frame >= *sceneChangeFrame_ + speed.retriggerDelay) {
		LOG(AfContrast, Debug) << "Scene changed, rescanning";
		startScan();
	}
}

/*
 * Find the position of the contrast peak by fitting a parabola to the
 * maximum and its neighbours.
 */
double AfContrast::findPeak(std::vector<ScanRecord> records)
{
	std::sort(records.begin(), records.end(),
		  [](const ScanRecord &a, const ScanRecord &b) {
			  return a.position < b.position;
		  });

	auto max = std::max_element(records.begin(), records.end(),
				    [](const ScanRecord &a, const ScanRecord &b) {
					    return a.contrast < b.contrast;
				    });

	if (max == records.begin() || max + 1 == records.end())
		return max->position;

	const double x0 = (max - 1)->position;
	const double x1 = max->position;
	const double x2 = (max + 1)->position;
	const double y0 = (max - 1)->contrast;
	const double y1 = max->contrast;
	const double y2 = (max + 1)->contrast;

	/* Coefficients of y = a * x^2 + b * x + c through the three points. */
	const double d0 = (y1 - y0) / (x1 - x0);
	const double d1 = (y2 - y1) / (x2 - x1);
	const double a = (d1 - d0) / (x2 - x0);
	if (a >= 0.0)
		return x1;

	const double b = d0 - a * (x0 + x1);

	return std::clamp(-b / (2.0 * a), x0, x2);
}

void AfContrast::updateLensPosition(uint32_t frame)
{
	const double previous = position_;
	const bool known = initialised_;

	/* The scan range only limits the lens movements while scanning. */
	target_ = map_.domain().clamp(target_);
	if (scanning())
		target_ = std::clamp(target_, ranges_[range_].focusMin,
				     ranges_[range_].focusMax);

	if (!initialised_) {
		/* The lens position is unknown, go straight to the target. */
		position_ = target_;
		initialised_ = true;
		skipCount_ = skipFrames_;
	} else {
		const double slew = speeds_[speed_].maxSlew;
		position_ = std::clamp(target_, position_ - slew, position_ + slew);
	}

	if (!known || position_ != previous) {
		lensChanged_ = true;
		/* The lens moves after the last frame captured so far. */
		lastMoveFrame_ = lastCapturedFrame_.value_or(frame);
	}
}

/* Frame numbers restarted from zero, forget the ones recorded. */
void AfContrast::restart()
{
	lastMoveFrame_.reset();
	sceneChangeFrame_.reset();
	skipCount_ = skipFrames_;
}

bool AfContrast::scanning() const
{
	return scanState_ != ScanState::Idle;
}

} /* namespace ipa */

} /* namespace libcamera */
