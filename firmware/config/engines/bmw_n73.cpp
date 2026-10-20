/*
 * bmw_n73.cpp
 *
 * @date Oct 2, 2021
 * @author Andrey Belomutskiy, (c) 2012-2021
 */

#include "pch.h"

#include "bmw_n73.h"
#if EFI_LUA
#include "lua_lib.h"
#endif // EFI_LUA

/**
 * set engine_type 9
 */
void setEngineProteusGearboxManInTheMiddle() {
#if EFI_LUA
#include "bmw_6hp_mim.lua"
#endif // EFI_LUA
}
