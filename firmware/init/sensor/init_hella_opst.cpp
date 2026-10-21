/**
 * @file init_hella_opst.cpp
 * @author Giacomo di Ciocco, (C) 2026
 */

#include "pch.h"

#include "init.h"
#include "hella_opst.h"

static HellaOpsTSensor hellaOpsT;

void initHellaOpsT() {
	hellaOpsT.init(engineConfiguration->hellaOpsTInput);
}

void deinitHellaOpsT() {
	hellaOpsT.deInit();
}
