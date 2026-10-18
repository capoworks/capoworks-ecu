#include "pch.h"

#include "cylinder_model_fuel_trim.h"
#include "engine.h"

#include <optional>

namespace {

constexpr uint16_t SD_TRIM_FAMILY_BIT = 0x01;
constexpr uint16_t AN_TRIM_FAMILY_BIT = 0x02;

constexpr float SD_TRIM_LOAD_MIN_KPA = 0;
constexpr float SD_TRIM_LOAD_MAX_KPA = 1000;
constexpr float AN_TRIM_TPS_MIN_PERCENT = 0;
constexpr float AN_TRIM_TPS_MAX_PERCENT = 100;
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

template <typename T>
bool trimCellsValid(const T& tables, size_t cylinderCount) {
	for (size_t cylinder = 0; cylinder < cylinderCount; cylinder++) {
		for (size_t row = 0; row < CYLINDER_FUEL_TRIM_SIZE; row++) {
			for (size_t column = 0; column < CYLINDER_FUEL_TRIM_SIZE; column++) {
				const float value = tables[cylinder].table[row][column];
				if (!std::isfinite(value) || value < TRIM_CELL_MIN_PERCENT || value > TRIM_CELL_MAX_PERCENT) {
					return false;
				}
			}
		}
	}
	return true;
}

bool scanSdTrimFamily(size_t cylinderCount) {
	return axisStrictlyAscending(config->sdCylinderFuelTrimLoadBins) &&
		   axisStrictlyAscending(config->sdCylinderFuelTrimRpmBins) &&
		   axisInRange(config->sdCylinderFuelTrimLoadBins, SD_TRIM_LOAD_MIN_KPA, SD_TRIM_LOAD_MAX_KPA) &&
		   axisInRange(config->sdCylinderFuelTrimRpmBins, TRIM_RPM_MIN, TRIM_RPM_MAX) &&
		   trimCellsValid(config->sdCylinderFuelTrims, cylinderCount);
}

bool scanAlphaNTrimFamily(size_t cylinderCount) {
	return axisStrictlyAscending(config->alphaNCylinderFuelTrimTpsBins) &&
		   axisStrictlyAscending(config->alphaNCylinderFuelTrimRpmBins) &&
		   axisInRange(config->alphaNCylinderFuelTrimTpsBins, AN_TRIM_TPS_MIN_PERCENT, AN_TRIM_TPS_MAX_PERCENT) &&
		   axisInRange(config->alphaNCylinderFuelTrimRpmBins, TRIM_RPM_MIN, TRIM_RPM_MAX) &&
		   trimCellsValid(config->alphaNCylinderFuelTrims, cylinderCount);
}

void clearTelemetry(Engine& owner) {
	auto& output = owner.outputChannels;
	output.sdCylinderFuelTrimLoad = 0;
	output.alphaNCylinderFuelTrimLoad = 0;
	output.cylinderModelFuelTrimBranches = 0;
	for (size_t i = 0; i < MAX_CYLINDER_COUNT; i++) {
		output.cylinderModelFuelTrim[i] = 0;
	}
}

bool isCylinderCountValid(float capturedCount, size_t& cylinderCount) {
	if (!std::isfinite(capturedCount) || capturedCount < 1 || capturedCount > MAX_CYLINDER_COUNT ||
		capturedCount != static_cast<float>(static_cast<size_t>(capturedCount))) {
		return false;
	}
	cylinderCount = static_cast<size_t>(capturedCount);
	return true;
}

bool captureFamilyValidity(Engine& owner, const AirmassInputs& inputs, size_t cylinderCount, bool alphaN, bool& valid) {
	const uint16_t bit = alphaN ? AN_TRIM_FAMILY_BIT : SD_TRIM_FAMILY_BIT;
	{
		chibios_rt::CriticalSectionLocker csl;
		auto& cache = owner.cylinderTrimCalibration;
		if (cache.Generation != inputs.CalibrationGeneration ||
			cache.ConfigurationVersion != inputs.ConfigurationVersion || cache.CylinderCount != cylinderCount) {
			cache.Generation = inputs.CalibrationGeneration;
			cache.ConfigurationVersion = inputs.ConfigurationVersion;
			cache.CylinderCount = cylinderCount;
			cache.Known = 0;
			cache.Valid = 0;
		}
		if (owner.airmassCalibration.Generation != inputs.CalibrationGeneration ||
			owner.getGlobalConfigurationVersion() != inputs.ConfigurationVersion) {
			return false;
		}
		if (cache.Known & bit) {
			valid = (cache.Valid & bit) != 0;
			return true;
		}
	}

	// The table walk stays outside the lock. Writers invalidate the calibration
	// generation before changing axes, so an obsolete scan is never cached.
	valid = alphaN ? scanAlphaNTrimFamily(cylinderCount) : scanSdTrimFamily(cylinderCount);

	chibios_rt::CriticalSectionLocker csl;
	auto& cache = owner.cylinderTrimCalibration;
	if (owner.airmassCalibration.Generation != inputs.CalibrationGeneration ||
		owner.getGlobalConfigurationVersion() != inputs.ConfigurationVersion ||
		cache.Generation != inputs.CalibrationGeneration || cache.ConfigurationVersion != inputs.ConfigurationVersion ||
		cache.CylinderCount != cylinderCount) {
		return false;
	}
#if EFI_UNIT_TEST
	++cache.Scans;
#endif
	cache.Valid = (cache.Valid & ~bit) | (valid ? bit : 0);
	cache.Known |= bit;
	return true;
}

} // namespace

CylinderModelFuelTrimCandidate::CylinderModelFuelTrimCandidate() {
	reset();
}

void CylinderModelFuelTrimCandidate::reset() {
	for (size_t i = 0; i < MAX_CYLINDER_COUNT; i++) {
		Factor[i] = 1;
	}
	SdLoadKpa = 0;
	AlphaNTps = 0;
	CalibrationGeneration = 0;
	ConfigurationVersion = 0;
	Branches = 0;
	CylinderCount = 0;
	Valid = false;
}

bool prepareCylinderModelFuelTrim(
		Engine& owner,
		const AirmassInputs& inputs,
		const AirmassEvaluation& evaluation,
		CylinderModelFuelTrimCandidate& candidate) {
	candidate.reset();

	size_t cylinderCount;
	if (!isCylinderCountValid(inputs.CylinderCount, cylinderCount)) {
		return false;
	}

	const bool sdActive = evaluation.TrimSdActive;
	const bool alphaNActive = evaluation.TrimAlphaNActive;
	const float sdMass = sdActive ? evaluation.TrimSdWeightedMass : 0;
	const float alphaNMass = alphaNActive ? evaluation.TrimAlphaNWeightedMass : 0;
	const float sdLoad = sdActive ? evaluation.TrimSdLoadKpa : 0;
	const float alphaNTps = alphaNActive ? evaluation.TrimAlphaNTps : 0;

	if ((sdActive || alphaNActive) && (!std::isfinite(inputs.Rpm) || inputs.Rpm < TRIM_RPM_MIN)) {
		return false;
	}
	if (sdActive && (!std::isfinite(sdMass) || sdMass < 0 || !std::isfinite(sdLoad) || sdLoad < SD_TRIM_LOAD_MIN_KPA ||
					 sdLoad > SD_TRIM_LOAD_MAX_KPA)) {
		return false;
	}
	if (alphaNActive && (!std::isfinite(alphaNMass) || alphaNMass < 0 || !std::isfinite(alphaNTps) ||
						 alphaNTps < AN_TRIM_TPS_MIN_PERCENT || alphaNTps > AN_TRIM_TPS_MAX_PERCENT)) {
		return false;
	}

	if (sdActive) {
		bool valid;
		if (!captureFamilyValidity(owner, inputs, cylinderCount, false, valid) || !valid) {
			return false;
		}
	}
	if (alphaNActive) {
		bool valid;
		if (!captureFamilyValidity(owner, inputs, cylinderCount, true, valid) || !valid) {
			return false;
		}
	}

	const float totalMass = sdMass + alphaNMass;
	if (!std::isfinite(totalMass)) {
		return false;
	}

	candidate.CalibrationGeneration = inputs.CalibrationGeneration;
	candidate.ConfigurationVersion = inputs.ConfigurationVersion;
	candidate.CylinderCount = static_cast<uint8_t>(cylinderCount);
	candidate.SdLoadKpa = sdLoad;
	candidate.AlphaNTps = alphaNTps;
	candidate.Branches = (sdActive ? CYLINDER_MODEL_FUEL_TRIM_BRANCH_SD : 0) |
						 (alphaNActive ? CYLINDER_MODEL_FUEL_TRIM_BRANCH_AN : 0);

	if (!(totalMass > 0)) {
		// True zero air remains neutral, but the active family has still been
		// validated above so additives cannot hide an invalid calibration.
		candidate.Valid = true;
		return true;
	}

	std::optional<PreparedTable3DInterpolation<CYLINDER_FUEL_TRIM_SIZE, CYLINDER_FUEL_TRIM_SIZE>> sdInterpolation;
	if (sdActive) {
		sdInterpolation.emplace(
				config->sdCylinderFuelTrimLoadBins, sdLoad, config->sdCylinderFuelTrimRpmBins, inputs.Rpm);
	}

	std::optional<PreparedTable3DInterpolation<CYLINDER_FUEL_TRIM_SIZE, CYLINDER_FUEL_TRIM_SIZE>> alphaNInterpolation;
	if (alphaNActive) {
		alphaNInterpolation.emplace(
				config->alphaNCylinderFuelTrimTpsBins, alphaNTps, config->alphaNCylinderFuelTrimRpmBins, inputs.Rpm);
	}

	for (size_t cylinder = 0; cylinder < cylinderCount; cylinder++) {
		const float sdTrim = sdActive ? sdInterpolation->getValue(config->sdCylinderFuelTrims[cylinder].table) : 0;
		const float alphaNTrim =
				alphaNActive ? alphaNInterpolation->getValue(config->alphaNCylinderFuelTrims[cylinder].table) : 0;
		const float factor = 1 + (sdMass * sdTrim + alphaNMass * alphaNTrim) / totalMass * 0.01f;
		if (!std::isfinite(factor) || factor < 0) {
			candidate.reset();
			return false;
		}
		candidate.Factor[cylinder] = factor;
	}

	candidate.Valid = true;
	return true;
}

bool isCylinderModelFuelTrimCandidateValid(const CylinderModelFuelTrimCandidate& candidate, size_t cylinderCount) {
	if (!candidate.Valid || cylinderCount == 0 || cylinderCount > MAX_CYLINDER_COUNT ||
		candidate.CylinderCount != cylinderCount) {
		return false;
	}

	for (size_t i = 0; i < cylinderCount; i++) {
		if (!std::isfinite(candidate.Factor[i]) || candidate.Factor[i] < 0) {
			return false;
		}
	}
	return true;
}

bool publishCylinderModelFuelTrim(Engine& owner, const CylinderModelFuelTrimCandidate& candidate) {
	if (!isCylinderModelFuelTrimCandidateValid(candidate, candidate.CylinderCount)) {
		resetCylinderModelFuelTrim(owner);
		return false;
	}

	auto& state = owner.cylinderModelFuelTrim;
	for (size_t i = 0; i < MAX_CYLINDER_COUNT; i++) {
		state.Factor[i] = i < candidate.CylinderCount ? candidate.Factor[i] : 1;
	}
	state.Branches = candidate.Branches;
	state.PublicationValid = true;

	auto& output = owner.outputChannels;
	output.sdCylinderFuelTrimLoad = (candidate.Branches & CYLINDER_MODEL_FUEL_TRIM_BRANCH_SD) ? candidate.SdLoadKpa : 0;
	output.alphaNCylinderFuelTrimLoad =
			(candidate.Branches & CYLINDER_MODEL_FUEL_TRIM_BRANCH_AN) ? candidate.AlphaNTps : 0;
	output.cylinderModelFuelTrimBranches = candidate.Branches;
	for (size_t i = 0; i < MAX_CYLINDER_COUNT; i++) {
		output.cylinderModelFuelTrim[i] = (state.Factor[i] - 1) * 100;
	}
	return true;
}

void resetCylinderModelFuelTrim(Engine& owner) {
	owner.cylinderModelFuelTrim.reset();
	clearTelemetry(owner);
}

float getCylinderModelFuelTrimFactor(const Engine& owner, size_t cylinderIndex) {
	return cylinderIndex < MAX_CYLINDER_COUNT ? owner.cylinderModelFuelTrim.Factor[cylinderIndex] : 1;
}

bool isCylinderModelFuelTrimPublicationValid(const Engine& owner) {
	return owner.cylinderModelFuelTrim.PublicationValid;
}
