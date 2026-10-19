/**
 * @file	rusEfiFunctionalTest.cpp
 *
 * @date Mar 1, 2014
 * @author Andrey Belomutskiy, (c) 2012-2020
 */

#include "pch.h"

#include "boards.h"
#include "rusEfiFunctionalTest.h"
#include "console_io.h"
#include "eficonsole.h"
#include "trigger_central.h"
#include "datalogging.h"
#if EFI_ENGINE_SNIFFER
#include "engine_sniffer.h"
#endif // EFI_ENGINE_SNIFFER
#include "status_loop.h"
#include "trigger_emulator_algo.h"
#include "main_trigger_callback.h"
#include "bench_test.h"
#include "tunerstudio.h"
#include "sd_file_log.h"
#include "memstreams.h"
#include <chprintf.h>
#include "rusefi_lua.h"
#include "can_hw.h"
#include "flash_main.h"
#include "signal_executor_sleep.h"

#define DEFAULT_SIM_RPM 1200
#define DEFAULT_SNIFFER_THR 2500

static void assertString(const char* actual, const char* expected) {
	if (strcmp(actual, expected) != 0) {
		printf("assertString FAILED\n");
		firmwareError("chprintf test: got %s while %s", actual, expected);
	}
}

static void runChprintfTest() {
	static MemoryStream ts;
	static char testBuffer[200];
	msObjectInit(&ts, (uint8_t*)testBuffer, sizeof(testBuffer), 0);

	ts.eos = 0; // reset
	chprintf((BaseSequentialStream*)&ts, "%.2f - %.2f", NAN, NAN);
	ts.buffer[ts.eos] = 0;
	assertString(testBuffer, "NaN - NaN");

	// it's a very, very long and mostly forgotten story how this became our %.2f precision format
	ts.eos = 0; // reset
	chprintf((BaseSequentialStream*)&ts, "%.2f/%.4f/%.4f", 0.239f, 239.932, 0.1234);
	ts.buffer[ts.eos] = 0;

	assertString(testBuffer, "0.23/239.9320/0.1234");

	{
		LoggingWithStorage testLogging("test");
		testLogging.appendFloat(1.23, 5);
		testLogging.appendFloat(1.234, 2);
		assertString(testLogging.m_buffer, "1.230001.23");
	}

	{
		LoggingWithStorage testLogging("test");
		testLogging.appendFloat(-1.23, 5);
		assertString(testLogging.m_buffer, "-1.23000");
	}
	{
		LoggingWithStorage testLogging("test");
		testLogging.appendPrintf("a%.2fb%fc", -1.2, -3.4);
		assertString(testLogging.m_buffer, "a-1.20b-3.400000095c");
	}
}

#if EFI_SIGNAL_EXECUTOR_SLEEP
struct BatchOrderContext {
	semaphore_t completion;
	size_t callbackCount = 0;
	int order[MaxScheduleBatchSize] = {};
};

struct BatchOrderCallbackArg {
	BatchOrderContext* context;
	int value;
};

static void recordBatchOrder(BatchOrderCallbackArg* arg) {
	chibios_rt::CriticalSectionLocker csl;
	BatchOrderContext* context = arg->context;
	if (context->callbackCount < MaxScheduleBatchSize) {
		context->order[context->callbackCount] = arg->value;
	}
	context->callbackCount++;
	chSemSignalI(&context->completion);
}

static void prepareBatchOrderContext(BatchOrderContext* context) {
	chSemObjectInit(&context->completion, 0);
	context->callbackCount = 0;
	for (size_t i = 0; i < MaxScheduleBatchSize; i++) {
		context->order[i] = 0;
	}
}

static void assertBatchOrder(BatchOrderContext* context, const int* expected, size_t expectedCount,
		const char* testName) {
	for (size_t i = 0; i < expectedCount; i++) {
		if (chSemWaitTimeout(&context->completion, TIME_MS2I(1000)) != MSG_OK) {
			firmwareError("SleepExecutor batch %s: callback %d did not complete", testName,
					static_cast<int>(i + 1));
			return;
		}
	}

	if (context->callbackCount != expectedCount) {
		firmwareError("SleepExecutor batch %s: expected %zu callbacks, got %zu", testName,
				expectedCount, context->callbackCount);
		return;
	}

	for (size_t i = 0; i < expectedCount; i++) {
		if (context->order[i] != expected[i]) {
			firmwareError("SleepExecutor batch %s: callback %d was %d, expected %d", testName,
					static_cast<int>(i + 1), context->order[i], expected[i]);
			return;
		}
	}
}

static void runSleepExecutorBatchTest() {
	SleepExecutor executor;

	{
		BatchOrderContext context;
		prepareBatchOrderContext(&context);
		BatchOrderCallbackArg args[] = {
			{&context, 1},
			{&context, 2},
			{&context, 3},
		};
		efitick_t target = getTimeNowNt() + MS2NT(20);
		ScheduledAction events[] = {
			{target, {recordBatchOrder, &args[0]}},
			{target, {recordBatchOrder, &args[1]}},
			{target, {recordBatchOrder, &args[2]}},
		};

		if (!executor.scheduleBatch(events, efi::size(events))) {
			firmwareError("SleepExecutor batch future tie: scheduleBatch rejected valid events");
			return;
		}

		const int expected[] = {1, 2, 3};
		assertBatchOrder(&context, expected, efi::size(expected), "future tie");
	}

	{
		BatchOrderContext context;
		prepareBatchOrderContext(&context);
		BatchOrderCallbackArg args[] = {
			{&context, 4},
			{&context, 5},
			{&context, 6},
		};
		efitick_t now = getTimeNowNt();
		efitick_t target = now + MS2NT(20);
		ScheduledAction events[] = {
			{now - US2NT(100), {recordBatchOrder, &args[0]}},
			{target, {recordBatchOrder, &args[1]}},
			{target, {recordBatchOrder, &args[2]}},
		};

		if (!executor.scheduleBatch(events, efi::size(events))) {
			firmwareError("SleepExecutor batch mixed due/future: scheduleBatch rejected valid events");
			return;
		}

		const int expected[] = {4, 5, 6};
		assertBatchOrder(&context, expected, efi::size(expected), "mixed due/future");
	}

	{
		BatchOrderContext context;
		prepareBatchOrderContext(&context);
		BatchOrderCallbackArg args[] = {
			{&context, 7},
			{&context, 8},
		};
		efitick_t now = getTimeNowNt();
		ScheduledAction events[] = {
			{now - US2NT(100), {recordBatchOrder, &args[0]}},
			{now - US2NT(200), {recordBatchOrder, &args[1]}},
		};

		if (executor.scheduleBatch(events, efi::size(events))) {
			firmwareError("SleepExecutor batch invalid: unordered events were accepted");
			return;
		}

		if (context.callbackCount != 0) {
			firmwareError("SleepExecutor batch invalid: rejected events were published");
			return;
		}
	}

	printToConsole("SLEEP_EXECUTOR_BATCH_REGRESSION_PASS");
}
#endif // EFI_SIGNAL_EXECUTOR_SLEEP

void rusEfiFunctionalTest(void) {
	printToConsole("Running rusEfi simulator version:");
	static char versionBuffer[20];
	itoa10(versionBuffer, (int)getRusEfiVersion());
	printToConsole(versionBuffer);

	engine->setConfig();

	initializeConsole();

	initDataStructures();

	// todo: reduce code duplication with initEngineController

	initFlash();
	loadConfiguration();

#if EFI_SIGNAL_EXECUTOR_SLEEP
	runSleepExecutorBatchTest();
#endif // EFI_SIGNAL_EXECUTOR_SLEEP

	enableTriggerStimulator();

	commonInitEngineController();

	initTriggerCentral();

	initTriggerEmulator();

	startStatusThreads();

	startLoggingProcessor();

#if EFI_FILE_LOGGING
	initSdCardLogger();
#endif // EFI_FILE_LOGGING

	runChprintfTest();

	initMainLoop();

	setTriggerEmulatorRPM(DEFAULT_SIM_RPM);

	engineConfiguration->engineSnifferRpmThreshold = DEFAULT_SNIFFER_THR;

	void startSerialChannels();
	startSerialChannels();

	engineConfiguration->enableVerboseCanTx = true;

#if HAL_USE_CAN
	// Set CAN device name
	CAND1.deviceName = "can0";

	initCan();
#endif // HAL_USE_CAN

	startLua();

	extern bool main_loop_started;
	main_loop_started = true;
}

#if EFI_ENGINE_SNIFFER
extern WaveChart waveChart;
#endif // EFI_ENGINE_SNIFFER

void printPendingMessages(void) {
	updateDevConsoleState();

#if EFI_ENGINE_SNIFFER
	waveChart.publishIfFull();
#endif // EFI_ENGINE_SNIFFER
}

int isSerialOverTcpReady;

bool isCommandLineConsoleReady(void) {
	return isSerialOverTcpReady;
}

void onFatalError(const char* msg, const char* file, int line) {
	printf("onFatalError %s %s%d", msg, file, line);
	exit(-1);
}

void logMsg(const char* format, ...) {
	//	FILE * fp;
	//	fp = fopen ("simulator.log", "a");
	//
	//	va_list(args);
	//	va_start(args, format);
	//	vfprintf(fp, format, args);
	//
	//	fclose(fp);
}

#if HAL_USE_CAN
static bool didInitCan = false;
CANDriver* detectCanDevice(brain_pin_e pinRx, brain_pin_e pinTx) {
	if (didInitCan) {
		return nullptr;
	}

	didInitCan = true;
	return &CAND1;
}
#endif // HAL_USE_CAN

void setBoardConfigOverrides() {}

void initBoardSensors() {
	// Simulator gets battery voltage so it detects ignition-on
	Sensor::setMockValue(SensorType::BatteryVoltage, 14);
}
