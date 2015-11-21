#ifndef KDT_LIMITEDPRIORITYQUEUE_H
#define KDT_LIMITEDPRIORITYQUEUE_H

#include <list>
#include <vector>

namespace kdt {
template <typename T> class LimitedDistanceQueue {
public:
    // Create an empty queue with the number of items specified by the
    // parameter 'maxNumElements'.
    LimitedDistanceQueue(unsigned int maxSize) 
        : d_maxSize(maxSize) {
    }

    // Insert the new item specified by the 'newItem' with the distance
    // specified by the 'newDistance'. If the queue size is larger than the
    // maximum queue size, remove the item with the highest distance.
    void Push(const T& newItem, double newDistance) {
        for (auto it = d_queue.begin(); it != d_queue.end(); it++) {
            double curDistance = it->first;
            if (newDistance < curDistance) {
                // Inserting new elements before the first element with higher
                // distance keeps the queue sorted.
                d_queue.insert(it, std::make_pair(newDistance, newItem));
                if (d_queue.size() > d_maxSize) {
                    // Since the queue is always sorted, the last element has
                    // the highest distance value.
                    d_queue.pop_back();
                }
                return;
            }
        }
        if (d_queue.size() < d_maxSize) {
            d_queue.push_back(std::make_pair(newDistance, newItem));
        }
    }

    // Return a vector of items in the queue sorted by distance values.
    std::vector<T> Items() {
        std::vector<T> result;
        result.reserve(d_queue.size());
        for (const auto& distItem : d_queue) {
            result.push_back(distItem.second);
        }
        return result;
    }

    double MaxDistance() {
        if (d_queue.empty()) {
            return std::numeric_limits<double>::max();
        }
        return d_queue.back().first;
    }

    T PopFront() {
        T front = d_queue.front().second;
        d_queue.pop_front();
        return front;
    }

    bool IsFull() {
        return d_queue.size() == d_maxSize;
    }

    bool Empty() {
        return d_queue.empty();
    }
private:
    typedef std::pair<double, T> DistItemPair;

    unsigned int d_maxSize;
    std::list<DistItemPair> d_queue;
};

} //namespace kdt

#endif