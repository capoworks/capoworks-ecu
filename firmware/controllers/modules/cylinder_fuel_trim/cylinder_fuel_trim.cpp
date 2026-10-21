/**
 * @file cylinder_fuel_trim.cpp
 * @author Giacomo di Ciocco, (C) 2026
 */

#include "pch.h"

#include "cylinder_fuel_trim.h"
#include "engine.h"
#include "table_helper.h"

#ifdef MODULE_CYLINDER_FUEL_TRIM

namespace {

constexpr float TRIM_LOAD_MIN = 0;
constexpr float TRIM_LOAD_MAX = 1000;
constexpr float TRIM_RPM_MIN = 0;
constexpr float TRIM_RPM_MAX = 18000;
constexpr float TRIM_CELL_MIN_PERCENT = -25;
constexpr float TRIM_CELL_MAX_PERCENT = 25;

template <typename T, size_t N>
bool axisStrictlyAscending(const T (&axis)[N]) {
	float previous = axis[0];
	for (size_t i = 1; i < N; i++) {
		const float current = axis[i];
		if (!(current > previous)) {
			return false;
		}
		previous = current;
	}
	return true;
}

template <typename T, size_t N>
bool axisInRange(const T (&axis)[N], float minimum, float maximum) {
	for (size_t i = 0; i < N; i++) {
		const float current = axis[i];
		if (!std::isfinite(current) || current < minimum || current > maximum) {
			return false;
		}
	}
	return true;
}

bool scanCylinderFuelTrimCalibration(size_t cylinderCount) {
	if (!axisStrictlyAscending(config->cylinderFuelTrimLoadBins) ||
		!axisStrictlyAscending(config->cylinderFuelTrimRpmBins) ||
		!axisInRange(config->cylinderFuelTrimLoadBins, TRIM_LOAD_MIN, TRIM_LOAD_MAX) ||
		!axisInRange(config->cylinderFuelTrimRpmBins, TRIM_RPM_MIN, TRIM_RPM_MAX)) {
		return false;
	}

	for (size_t cylinder = 0; cylinder < cylinderCount; cylinder++) {
		for (size_t row = 0; row < CYLINDER_FUEL_TRIM_SIZE; row++) {
			for (size_t column = 0; column < CYLINDER_FUEL_TRIM_SIZE; column++) {
				const float value = config->cylinderFuelTrims[cylinder].table[row][column];
				if (!std::isfinite(value) || value < TRIM_CELL_MIN_PERCENT || value > TRIM_CELL_MAX_PERCENT) {
					return false;
				}
			}
		}
	}
	return true;
}

void clearCylinderFuelTrimTelemetry(Engine& owner) {
	auto& output = owner.outputChannels;
	for (size_t i = 0; i < CYLINDER_FUEL_TRIM_COUNT; i++) {
		output.cylinderFuelTrim[i] = 0;
		output.cylinderFuelTrimLoad[i] = 0;
	}
}

bool isCylinderCountValid(float capturedCount, size_t& cylinderCount) {
	if (!std::isfinite(capturedCount) || capturedCount < 1 || capturedCount > MAX_CYLINDER_COUNT ||
		capturedCount != static_cast<float>(static_cast<size_t>(capturedCount))) {
		return false;
	}

	cylinderCount = static_cast<size_t>(capturedCount);
	return cylinderCount <= CYLINDER_FUEL_TRIM_COUNT;
}

bool consumerLoadsMatchInputs(const AirmassInputs& inputs, const AirmassConsumerLoadCandidate& loads) {
	const auto& identity = loads.Identity;
	return loads.Prepared && loads.Snapshot.Valid && identity.PublicationEpoch == inputs.PublicationEpoch &&
		   identity.ConfigurationVersion == inputs.ConfigurationVersion &&
		   identity.CalibrationGeneration == inputs.CalibrationGeneration &&
		   identity.Strategy == inputs.ActiveStrategy && identity.CylinderCount == inputs.CylinderCount &&
		   identity.Displacement == inputs.Displacement;
}

bool captureCalibrationValidity(Engine& owner, const AirmassInputs& inputs, size_t cylinderCount, bool& valid) {
	{
		chibios_rt::CriticalSectionLocker csl;
		auto& cache = owner.cylinderFuelTrimCalibration;
		if (cache.Generation != inputs.CalibrationGeneration ||
			cache.ConfigurationVersion != inputs.ConfigurationVersion || cache.CylinderCount != cylinderCount) {
			cache.Generation = inputs.CalibrationGeneration;
			cache.ConfigurationVersion = inputs.ConfigurationVersion;
			cache.CylinderCount = cylinderCount;
			cache.Known = false;
			cache.Valid = false;
		}
		if (owner.airmassCalibration.Generation != inputs.CalibrationGeneration ||
			owner.getGlobalConfigurationVersion() != inputs.ConfigurationVersion) {
			return false;
		}
		if (cache.Known) {
			valid = cache.Valid;
			return true;
		}
	}

	// Table reads remain outside the lock. A writer invalidates the generation
	// before changing calibration data, so an obsolete scan is never cached.
	valid = scanCylinderFuelTrimCalibration(cylinderCount);

	chibios_rt::CriticalSectionLocker csl;
	auto& cache = owner.cylinderFuelTrimCalibration;
	if (owner.airmassCalibration.Generation != inputs.CalibrationGeneration ||
		owner.getGlobalConfigurationVersion() != inputs.ConfigurationVersion ||
		cache.Generation != inputs.CalibrationGeneration || cache.ConfigurationVersion != inputs.ConfigurationVersion ||
		cache.CylinderCount != cylinderCount) {
		return false;
	}
#if EFI_UNIT_TEST
	++cache.Scans;
#endif
	cache.Valid = valid;
	cache.Known = true;
	return true;
}

} // namespace

void setDefaultCylinderFuelTrims() {
	// One shared 16x16 table per cylinder. All cells default neutral.
	setLinearCurve(config->cylinderFuelTrimLoadBins, 0, 200);
	setRpmTableBin(config->cylinderFuelTrimRpmBins);

	for (auto& source : config->cylinderFuelTrimLoadSource) {
		source = AFR_None;
	}
	for (size_t i = 0; i < efi::size(config->cylinderFuelTrims); i++) {
		setTable(config->cylinderFuelTrims[i].table, 0);
	}
}

CylinderFuelTrimCandidate::CylinderFuelTrimCandidate() {
	reset();
}

void CylinderFuelTrimCandidate::reset() {
	for (size_t i = 0; i < CYLINDER_FUEL_TRIM_COUNT; i++) {
		Factor[i] = 1;
		Load[i] = 0;
	}
	CalibrationGeneration = 0;
	ConfigurationVersion = 0;
	CylinderCount = 0;
	Valid = false;
}

CylinderFuelTrimCalibrationCache::CylinderFuelTrimCalibrationCache() {
	reset();
}

void CylinderFuelTrimCalibrationCache::reset() {
	Generation = 0xFFFFFFFFu;
	ConfigurationVersion = -1;
	CylinderCount = 0;
	Known = false;
	Valid = false;
#if EFI_UNIT_TEST
	Scans = 0;
#endif
}

bool prepareCylinderFuelTrim(
		Engine& owner,
		const AirmassInputs& inputs,
		const AirmassConsumerLoadCandidate& consumerLoads,
		CylinderFuelTrimCandidate& candidate) {
	candidate.reset();

	size_t cylinderCount;
	if (!isCylinderCountValid(inputs.CylinderCount, cylinderCount) || !std::isfinite(inputs.Rpm) ||
		inputs.Rpm < TRIM_RPM_MIN || !consumerLoadsMatchInputs(inputs, consumerLoads)) {
		return false;
	}

	AirmassConsumerLoadContext selectedLoads(consumerLoads);
	for (size_t cylinder = 0; cylinder < cylinderCount; cylinder++) {
		const auto source = consumerLoads.Sources[static_cast<size_t>(AirmassConsumer::CylinderFuelTrim)][cylinder];
		if (static_cast<uint8_t>(source) > AFR_EffectiveMAP) {
			return false;
		}

		const float load = selectedLoads.get(AirmassConsumer::CylinderFuelTrim, cylinder);
		if (!std::isfinite(load) || load < TRIM_LOAD_MIN || load > TRIM_LOAD_MAX) {
			return false;
		}
		candidate.Load[cylinder] = load;
	}

	bool calibrationValid = false;
	if (!captureCalibrationValidity(owner, inputs, cylinderCount, calibrationValid) || !calibrationValid) {
		return false;
	}

	for (size_t cylinder = 0; cylinder < cylinderCount; cylinder++) {
		PreparedTable3DInterpolation<CYLINDER_FUEL_TRIM_SIZE, CYLINDER_FUEL_TRIM_SIZE> interpolation(
				config->cylinderFuelTrimLoadBins,
				candidate.Load[cylinder],
				config->cylinderFuelTrimRpmBins,
				inputs.Rpm);
		const float trimPercent = interpolation.getValue(config->cylinderFuelTrims[cylinder].table);
		const float factor = 1 + trimPercent * 0.01f;
		if (!std::isfinite(trimPercent) || !std::isfinite(factor) || factor < 0) {
			candidate.reset();
			return false;
		}
		candidate.Factor[cylinder] = factor;
	}

	candidate.CalibrationGeneration = inputs.CalibrationGeneration;
	candidate.ConfigurationVersion = inputs.ConfigurationVersion;
	candidate.CylinderCount = static_cast<uint8_t>(cylinderCount);
	candidate.Valid = true;
	return true;
}

bool isCylinderFuelTrimCandidateValid(const CylinderFuelTrimCandidate& candidate, size_t cylinderCount) {
	if (!candidate.Valid || cylinderCount == 0 || cylinderCount > CYLINDER_FUEL_TRIM_COUNT ||
		candidate.CylinderCount != cylinderCount) {
		return false;
	}

	for (size_t i = 0; i < cylinderCount; i++) {
		if (!std::isfinite(candidate.Factor[i]) || candidate.Factor[i] < 0 || !std::isfinite(candidate.Load[i]) ||
			candidate.Load[i] < TRIM_LOAD_MIN || candidate.Load[i] > TRIM_LOAD_MAX) {
			return false;
		}
	}
	return true;
}

bool publishCylinderFuelTrim(Engine& owner, const CylinderFuelTrimCandidate& candidate) {
	if (!isCylinderFuelTrimCandidateValid(candidate, candidate.CylinderCount)) {
		resetCylinderFuelTrim(owner);
		return false;
	}

	auto& output = owner.outputChannels;
	for (size_t i = 0; i < CYLINDER_FUEL_TRIM_COUNT; i++) {
		const bool used = i < candidate.CylinderCount;
		const float factor = used ? candidate.Factor[i] : 1;
		output.cylinderFuelTrim[i] = (factor - 1) * 100;
		output.cylinderFuelTrimLoad[i] = used ? candidate.Load[i] : 0;
	}
	return true;
}

void resetCylinderFuelTrim(Engine& owner) {
	clearCylinderFuelTrimTelemetry(owner);
}

#endif // MODULE_CYLINDER_FUEL_TRIM
