/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2022-2023, Raspberry Pi Ltd
 * Copyright (C) 2026, Jayden Milne
 *
 * Contrast detection autofocus algorithm
 */

#pragma once

#include <array>
#include <optional>
#include <span>
#include <stdint.h>
#include <vector>

#include <libcamera/control_ids.h>
#include <libcamera/controls.h>
#include <libcamera/geometry.h>

#include "libcamera/internal/value_node.h"

#include "pwl.h"

namespace libcamera {

namespace ipa {

namespace af {

struct FrameContext {
	double lensPosition;
};

} /* namespace af */

class AfContrast
{
public:
	struct RegionStats {
		double contrast;
		double average;
	};

	int init(const ValueNode &tuningData, const ControlInfoMap &lensControls,
		 ControlInfoMap::Map &controls);
	int configure(const ControlInfoMap &lensControls, const Size &grid,
		      const Rectangle &statsRegion);

	void queueRequest(const ControlList &controls);
	void prepare(uint32_t frame, af::FrameContext &frameContext);
	void process(uint32_t frame, const af::FrameContext &frameContext,
		     std::span<const RegionStats> stats, double exposure,
		     ControlList &metadata);

	std::optional<int32_t> lensSetting();

private:
	enum class ScanState {
		Idle,
		Trigger,
		Coarse1,
		Coarse2,
		Fine,
		Settle,
	};

	struct RangeParams {
		double focusMin;
		double focusMax;
		double focusDefault;

		void read(const ValueNode &params);
	};

	struct SpeedParams {
		double stepCoarse;
		double stepFine;
		double contrastRatio;
		double retriggerRatio;
		uint32_t retriggerDelay;
		double maxSlew;
		uint32_t stepFrames;

		void read(const ValueNode &params);
	};

	struct ScanRecord {
		double position;
		double contrast;
	};

	void parseTuning(const ValueNode &tuningData);

	void setMode(controls::AfModeEnum mode);
	void setPause(controls::AfPauseEnum pause);
	void trigger();
	void cancel();
	void goIdle();
	void restart();

	void computeWeights();
	bool combineStats(std::span<const RegionStats> stats,
			  double *contrast, double *average) const;

	void doAf(uint32_t frame, double contrast, double average,
		  double exposure);
	void startScan();
	void doScan(double contrast);
	void finishScan(double contrast, double average, double exposure);
	void checkSceneChange(uint32_t frame, double contrast, double average,
			      double exposure);
	static double findPeak(std::vector<ScanRecord> records);

	void updateLensPosition(uint32_t frame);
	bool scanning() const;

	bool lensPresent_ = false;
	int32_t lensMin_ = 0;
	int32_t lensMax_ = 0;

	std::array<RangeParams, 3> ranges_;
	std::array<SpeedParams, 2> speeds_;
	uint32_t skipFrames_;
	Pwl map_;

	Size grid_;
	Rectangle statsRegion_;
	std::vector<Rectangle> windows_;
	std::vector<double> weights_;
	bool useWindows_ = false;

	controls::AfModeEnum mode_ = controls::AfModeManual;
	controls::AfRangeEnum range_ = controls::AfRangeNormal;
	controls::AfSpeedEnum speed_ = controls::AfSpeedNormal;
	bool pauseFlag_ = false;

	ScanState scanState_ = ScanState::Idle;
	controls::AfStateEnum reportState_ = controls::AfStateIdle;

	bool initialised_ = false;
	double target_ = 0.0;
	double position_ = 0.0;
	bool lensChanged_ = false;
	std::optional<uint32_t> lastFrame_;
	std::optional<uint32_t> lastCapturedFrame_;
	std::optional<uint32_t> lastMoveFrame_;
	uint32_t skipCount_ = 0;

	double scanStep_ = 0.0;
	std::vector<ScanRecord> scanData_;
	double scanMaxContrast_ = 0.0;
	double scanMinContrast_ = 0.0;

	std::optional<double> sceneContrast_;
	double sceneAverage_ = 0.0;
	double sceneExposure_ = 0.0;
	double lastExposure_ = 0.0;
	std::optional<uint32_t> sceneChangeFrame_;
};

} /* namespace ipa */

} /* namespace libcamera */
