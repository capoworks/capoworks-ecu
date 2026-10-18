#include "pch.h"

#include "event_queue.h"

namespace {
int callbackCount;

void countCallback(void*) {
	callbackCount++;
}

void fillQueue(EventQueue& queue, size_t count) {
	for (size_t i = 0; i < count; i++) {
		queue.insertTask(nullptr, 100 + i, countCallback);
		ASSERT_EQ(queue.size(), static_cast<int>(i + 1));
	}
}

struct RecordContext {
	std::vector<int>* order;
	int value;
};

void recordValue(void* argument) {
	auto* context = static_cast<RecordContext*>(argument);
	context->order->push_back(context->value);
}

class DrainingExecutor final : public TestExecutor {
public:
	bool scheduleBatch(const ScheduledAction* events, size_t count) override {
		if (!TestExecutor::scheduleBatch(events, count)) {
			return false;
		}

		if (!draining) {
			draining = true;
			executeAll(getTimeNowUs());
			draining = false;
		}
		return true;
	}

private:
	bool draining = false;
};

struct ReentrantContext {
	DrainingExecutor* executor;
	std::vector<int>* order;
};

void recordReentrant(void* argument) {
	auto* context = static_cast<ReentrantContext*>(argument);
	context->order->push_back(3);
}

void recordClose(void* argument) {
	auto* context = static_cast<ReentrantContext*>(argument);
	context->order->push_back(2);
}

void recordOpenAndScheduleAgain(void* argument) {
	auto* context = static_cast<ReentrantContext*>(argument);
	context->order->push_back(1);

	// The current callback has already been unlinked, but the other member of
	// the original batch must already be visible before this callback runs.
	EXPECT_EQ(context->executor->size(), 1);

	ScheduledAction followUp = {
			getTimeNowNt(),
			{recordReentrant, context},
	};
	EXPECT_TRUE(context->executor->scheduleBatch(&followUp, 1));
}
} // namespace

TEST(SchedulerBatch, PoolCapacityIsReservedAllOrNothing) {
	ScheduledAction pair[] = {
			{1000, countCallback},
			{1001, countCallback},
	};
	ScheduledAction triple[] = {
			{1000, countCallback},
			{1001, countCallback},
			{1002, countCallback},
	};

	{
		EventQueue queue;
		fillQueue(queue, 61);
		EXPECT_TRUE(queue.insertBatch(triple, efi::size(triple)));
		EXPECT_EQ(queue.size(), 64);
	}
	{
		EventQueue queue;
		fillQueue(queue, 62);
		EXPECT_TRUE(queue.insertBatch(pair, efi::size(pair)));
		EXPECT_EQ(queue.size(), 64);
	}
	{
		EventQueue queue;
		fillQueue(queue, 62);
		EXPECT_FALSE(queue.insertBatch(triple, efi::size(triple)));
		EXPECT_EQ(queue.size(), 62);

		callbackCount = 0;
		EXPECT_EQ(queue.executeAll(2000), 62);
		EXPECT_EQ(callbackCount, 62);
		EXPECT_EQ(queue.size(), 0);
		EXPECT_TRUE(queue.insertBatch(triple, efi::size(triple)));
		EXPECT_EQ(queue.size(), 3);
	}
	{
		EventQueue queue;
		fillQueue(queue, 63);
		EXPECT_FALSE(queue.insertBatch(pair, efi::size(pair)));
		EXPECT_EQ(queue.size(), 63);
	}
	{
		EventQueue queue;
		fillQueue(queue, 64);
		EXPECT_FALSE(queue.insertBatch(pair, 1));
		EXPECT_EQ(queue.size(), 64);
	}
}

TEST(SchedulerBatch, ThreeWayTiePreservesSuppliedOrder) {
	EventQueue queue;
	std::vector<int> order;
	RecordContext contexts[] = {
			{&order, 1},
			{&order, 2},
			{&order, 3},
	};
	ScheduledAction actions[] = {
			{0, {recordValue, &contexts[0]}},
			{0, {recordValue, &contexts[1]}},
			{0, {recordValue, &contexts[2]}},
	};

	ASSERT_TRUE(queue.insertBatch(actions, efi::size(actions)));
	ASSERT_EQ(queue.size(), 3);
	EXPECT_EQ(queue.executeAll(0), 3);
	EXPECT_EQ(order, (std::vector<int>{1, 2, 3}));
	EXPECT_EQ(queue.size(), 0);
}

TEST(SchedulerBatch, ExecutorPublishesWholeBatchBeforeReentrantCallbacks) {
	setTimeNowUs(0);
	DrainingExecutor executor;
	std::vector<int> order;
	ReentrantContext context{&executor, &order};
	ScheduledAction injection[] = {
			{getTimeNowNt(), {recordOpenAndScheduleAgain, &context}},
			{getTimeNowNt(), {recordClose, &context}},
	};

	ASSERT_TRUE(executor.scheduleBatch(injection, efi::size(injection)));
	EXPECT_EQ(order, (std::vector<int>{1, 2, 3}));
	EXPECT_EQ(executor.size(), 0);
}

TEST(SchedulerBatch, ExecutorRejectsInvalidRequestsWithoutPublishing) {
	setTimeNowUs(1000);
	TestExecutor executor;
	std::vector<int> order;
	RecordContext contexts[] = {
			{&order, 1},
			{&order, 2},
	};
	ScheduledAction valid[] = {
			{getTimeNowNt() + US2NT(100), {recordValue, &contexts[0]}},
			{getTimeNowNt() + US2NT(200), {recordValue, &contexts[1]}},
	};
	ASSERT_TRUE(executor.scheduleBatch(valid, efi::size(valid)));
	ASSERT_EQ(executor.size(), 2);

	ScheduledAction unordered[] = {
			{getTimeNowNt() + US2NT(300), {recordValue, &contexts[0]}},
			{getTimeNowNt() + US2NT(200), {recordValue, &contexts[1]}},
	};
	EXPECT_FALSE(executor.scheduleBatch(unordered, efi::size(unordered)));

	ScheduledAction nullAction[] = {{getTimeNowNt() + US2NT(300), {}}};
	EXPECT_FALSE(executor.scheduleBatch(nullAction, efi::size(nullAction)));

	ScheduledAction tooFar[] = {
			{getTimeNowNt() + US2NT(MaximumScheduleDelayUs), {recordValue, &contexts[0]}},
	};
	EXPECT_FALSE(executor.scheduleBatch(tooFar, efi::size(tooFar)));

	ScheduledAction tooMany[] = {
			{getTimeNowNt(), {recordValue, &contexts[0]}},
			{getTimeNowNt(), {recordValue, &contexts[1]}},
			{getTimeNowNt(), {recordValue, &contexts[0]}},
			{getTimeNowNt(), {recordValue, &contexts[1]}},
	};
	EXPECT_FALSE(executor.scheduleBatch(tooMany, efi::size(tooMany)));
	EXPECT_EQ(executor.size(), 2);

	EXPECT_EQ(executor.executeAll(1200), 2);
	EXPECT_EQ(order, (std::vector<int>{1, 2}));
	EXPECT_EQ(executor.size(), 0);
}

TEST(SchedulerBatch, ValidatorRejectsInvalidBoundsAndOverflowingHorizons) {
	const efitick_t now = 1000;
	ScheduledAction valid[] = {
			{now, countCallback},
			{now + 1, countCallback},
	};
	EXPECT_TRUE(isScheduleBatchValid(valid, efi::size(valid), now));
	EXPECT_FALSE(isScheduleBatchValid(nullptr, efi::size(valid), now));
	EXPECT_FALSE(isScheduleBatchValid(valid, 0, now));
	EXPECT_FALSE(isScheduleBatchValid(valid, MaxScheduleBatchSize + 1, now));

	ScheduledAction nullAction[] = {{now + 1, {}}};
	EXPECT_FALSE(isScheduleBatchValid(nullAction, efi::size(nullAction), now));

	ScheduledAction unordered[] = {
			{now + 2, countCallback},
			{now + 1, countCallback},
	};
	EXPECT_FALSE(isScheduleBatchValid(unordered, efi::size(unordered), now));

	int64_t horizon = US2NT(MaximumScheduleDelayUs).count();
	ScheduledAction atHorizon[] = {{now + horizon, countCallback}};
	EXPECT_FALSE(isScheduleBatchValid(atHorizon, efi::size(atHorizon), now));

	ScheduledAction beforeHorizon[] = {{now + horizon - 1, countCallback}};
	EXPECT_TRUE(isScheduleBatchValid(beforeHorizon, efi::size(beforeHorizon), now));

	ScheduledAction pastAtMaximum[] = {{INT64_MIN, countCallback}};
	EXPECT_TRUE(isScheduleBatchValid(pastAtMaximum, efi::size(pastAtMaximum), efitick_t{INT64_MAX}));

	ScheduledAction hugeForward[] = {{INT64_MAX, countCallback}};
	EXPECT_FALSE(isScheduleBatchValid(hugeForward, efi::size(hugeForward), efitick_t{INT64_MIN}));

	efitick_t nearMaximum = efitick_t{INT64_MAX - horizon};
	ScheduledAction maximumAtHorizon[] = {{INT64_MAX, countCallback}};
	EXPECT_FALSE(isScheduleBatchValid(maximumAtHorizon, efi::size(maximumAtHorizon), nearMaximum));
}
