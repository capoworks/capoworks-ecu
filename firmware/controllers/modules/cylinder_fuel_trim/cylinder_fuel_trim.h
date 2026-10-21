/**
 * @file cylinder_fuel_trim.h
 * @author Giacomo di Ciocco, (C) 2026
 */

#pragma once

#include "airmass_loads.h"
#include "rusefi_types.h"

#ifdef MODULE_CYLINDER_FUEL_TRIM

static_assert(CYLINDER_FUEL_TRIM_COUNT > 0);
static_assert(CYLINDER_FUEL_TRIM_COUNT <= MAX_CYLINDER_COUNT);

class Engine;

// Prepared outside the publication lock. Only the factors and captured table
// coordinates are carried into the atomic publication.
struct CylinderFuelTrimCandidate {
	float Factor[CYLINDER_FUEL_TRIM_COUNT];
	float Load[CYLINDER_FUEL_TRIM_COUNT];
	uint32_t CalibrationGeneration;
	int ConfigurationVersion;
	uint8_t CylinderCount;
	bool Valid;

	CylinderFuelTrimCandidate();
	void reset();
};

// Table validation is keyed by the airmass calibration identity captured with
// the fuel calculation. The table walk itself is performed outside the lock.
struct CylinderFuelTrimCalibrationCache {
	uint32_t Generation;
	int ConfigurationVersion;
	size_t CylinderCount;
	bool Known;
	bool Valid;
#if EFI_UNIT_TEST
	uint32_t Scans;
#endif

	CylinderFuelTrimCalibrationCache();
	void reset();
};

bool prepareCylinderFuelTrim(
		Engine& owner,
		const AirmassInputs& inputs,
		const AirmassConsumerLoadCandidate& consumerLoads,
		CylinderFuelTrimCandidate& candidate);
// The caller must hold the atomic fuel publication lock and check the
// candidate identity against the captured publication before calling this.
bool publishCylinderFuelTrim(Engine& owner, const CylinderFuelTrimCandidate& candidate);
bool isCylinderFuelTrimCandidateValid(const CylinderFuelTrimCandidate& candidate, size_t cylinderCount);

void setDefaultCylinderFuelTrims();
void resetCylinderFuelTrim(Engine& owner);

#endif // MODULE_CYLINDER_FUEL_TRIM
