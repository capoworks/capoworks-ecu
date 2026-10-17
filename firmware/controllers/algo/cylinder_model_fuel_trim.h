#pragma once

#include "airmass.h"
#include "rusefi_types.h"

class Engine;

#define CYLINDER_MODEL_FUEL_TRIM_BRANCH_SD 0x01
#define CYLINDER_MODEL_FUEL_TRIM_BRANCH_AN 0x02

// Self-contained result of calibration validation and factor calculation.
// Preparation never mutates live factors or telemetry. Unused cylinders remain
// neutral, and an invalid candidate is never partially publishable.
struct CylinderModelFuelTrimCandidate {
	float Factor[MAX_CYLINDER_COUNT];
	float SdLoadKpa;
	float AlphaNTps;
	uint32_t CalibrationGeneration;
	int ConfigurationVersion;
	uint8_t Branches;
	uint8_t CylinderCount;
	bool Valid;

	CylinderModelFuelTrimCandidate();
	void reset();
};

// Validate only the active trim families for the captured cylinder count and
// calculate every factor outside the publication lock. Cache scans are owned by
// the supplied Engine and keyed by the captured calibration identity.
bool prepareCylinderModelFuelTrim(
		Engine& owner,
		const AirmassInputs& inputs,
		const AirmassEvaluation& evaluation,
		CylinderModelFuelTrimCandidate& candidate);

// Copy a prepared candidate into the owner. Call only while holding the same
// critical section and after the caller has verified the fuel publication
// identity. Invalid candidates reset instead of publishing partial factors.
bool publishCylinderModelFuelTrim(Engine& owner, const CylinderModelFuelTrimCandidate& candidate);
bool isCylinderModelFuelTrimCandidateValid(const CylinderModelFuelTrimCandidate& candidate, size_t cylinderCount);

// Neutralize factors and clear both trim coordinate cursors on this owner only.
void resetCylinderModelFuelTrim(Engine& owner);

float getCylinderModelFuelTrimFactor(const Engine& owner, size_t cylinderIndex);
bool isCylinderModelFuelTrimPublicationValid(const Engine& owner);
