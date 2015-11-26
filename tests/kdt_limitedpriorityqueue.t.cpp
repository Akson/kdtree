#include <gtest/gtest.h>

#include <kdt_limitedpriorityqueue.h>

#include <algorithm>

typedef kdt::LimitedDistanceQueue<int> LDQueue;

TEST(testLimitedPriorityQueue, itemsReturnsAllQueueElements)
{
    LDQueue queue(10);
    queue.Push(1, 1);
    queue.Push(2, 2);
    auto items = queue.Items();
    ASSERT_EQ(2, items.size());
    ASSERT_EQ(items[0], 1);
    ASSERT_EQ(items[1], 2);
}

TEST(testLimitedPriorityQueue, itemsReturnsEmptyVectorForEmptyQueue)
{
    LDQueue queue(10);
    auto items = queue.Items();
    ASSERT_TRUE(items.empty());
}

TEST(testLimitedPriorityQueue, itemsReturnsSortedItems)
{
    LDQueue queue(10);
    queue.Push(2, 2);
    queue.Push(1, 1);
    queue.Push(3, 3);
    auto items = queue.Items();
    ASSERT_TRUE(std::is_sorted(items.begin(), items.end()));
}

TEST(testLimitedPriorityQueue, pushRemovesLargestElementForFullQueue)
{
    LDQueue queue(2);
    queue.Push(1, 1);
    queue.Push(3, 3);
    queue.Push(2, 2);
    auto items = queue.Items();
    ASSERT_EQ(2, items.size());
    ASSERT_EQ(items[0], 1);
    ASSERT_EQ(items[1], 2);
}

TEST(testLimitedPriorityQueue, defaultCreatedQueueIsEmpty)
{
    LDQueue queue(1);
    ASSERT_TRUE(queue.Empty());
}

TEST(testLimitedPriorityQueue, queueWithAtLeastOneElementIsntEmpty)
{
    LDQueue queue(1);
    queue.Push(1, 1);
    ASSERT_FALSE(queue.Empty());
}

TEST(testLimitedPriorityQueue, queueWillLessThenMaxElementsIsNotEmpty)
{
    LDQueue queue(2);
    queue.Push(1, 1);
    ASSERT_FALSE(queue.Empty());
}

TEST(testLimitedPriorityQueue, emptyQueueIsNotFull)
{
    LDQueue queue(1);
    ASSERT_FALSE(queue.IsFull());
}

TEST(testLimitedPriorityQueue, queueWillLessThenMaxElementsIsNotFull)
{
    LDQueue queue(2);
    queue.Push(1, 1);
    ASSERT_FALSE(queue.IsFull());
}

TEST(testLimitedPriorityQueue, queueWithMaximumNumberOfElementsIsFull)
{
    LDQueue queue(2);
    queue.Push(1, 1);
    queue.Push(2, 2);
    ASSERT_TRUE(queue.IsFull());
}

TEST(testLimitedPriorityQueue, popRemovesOneElement)
{
    LDQueue queue(2);
    queue.Push(1, 1);
    queue.Push(2, 2);
    ASSERT_TRUE(queue.IsFull());
    ASSERT_FALSE(queue.Empty());

    queue.PopFront();

    ASSERT_FALSE(queue.IsFull());
    ASSERT_FALSE(queue.Empty());

    queue.PopFront();

    ASSERT_FALSE(queue.IsFull());
    ASSERT_TRUE(queue.Empty());
}

TEST(testLimitedPriorityQueue, popThrowsIfQueueIsEmpty)
{
    LDQueue queue(10);
    EXPECT_THROW(queue.PopFront(), std::out_of_range);
}

TEST(testLimitedPriorityQueue, popReturnsElementWithMinimumDistance)
{
    LDQueue queue(10);
    queue.Push(3, 3);
    queue.Push(1, 1);
    queue.Push(2, 2);
    int result = queue.PopFront();
    EXPECT_EQ(1, result);
}

TEST(testLimitedPriorityQueue, maxDistanceReturnsDistanceOfElementWithMaximum)
{
    LDQueue queue(10);
    queue.Push(3, 3);
    queue.Push(1, 1);
    queue.Push(4, 4);
    queue.Push(2, 2);
    double result = queue.MaxDistance();
    ASSERT_DOUBLE_EQ(4, result);
}

TEST(testLimitedPriorityQueue, maxDistanceReturnsDoubleMaxForEmptyQueue)
{
    LDQueue queue(10);
    double result = queue.MaxDistance();
    ASSERT_DOUBLE_EQ(std::numeric_limits<double>::max(), result);
}
