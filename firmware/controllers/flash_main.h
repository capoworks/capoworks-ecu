/**
 * @file    flash_main.h
 * @brief
 *
 * @date Sep 19, 2013
 * @author Andrey Belomutskiy, (c) 2012-2020
 */

#pragma once

void readFromFlash();
void initFlash();

/**
 * Because of hardware-related issues, stm32f4 chip is totally
 * frozen while we are writing to internal flash. Writing the configuration takes
 * about 1-2 seconds, we cannot afford to do that while the engine is
 * running so we postpone the write until the engine is stopped. Direct writes
 * overlapping active flash I/O are queued, and reads are rejected while busy.
 */
void writeToFlashNow();
void setNeedToWriteConfiguration();
/**
 * @return true if a flash write is pending or the last attempt failed
 */
bool getNeedToWriteConfiguration();
/**
 * Consume one fresh request when the engine is stopped. A failed attempt stays
 * pending but is not retried until another request arrives.
 */
void writeToFlashIfPending();
