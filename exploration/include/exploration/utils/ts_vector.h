#ifndef EXPLORATION_TS_VECTOR_H
#define EXPLORATION_TS_VECTOR_H

#include <vector>
#include <mutex>
#include <cmath>
#include <iostream>

#include <geometry_msgs/PoseStamped.h>

namespace mre
{
    template <typename T>
    class TSVector
    {
    public:
        TSVector()
        {
            vector_.reserve(0);
        };

        const T &front()
        {
            std::lock_guard<std::mutex> lock(mutex_);
            return vector_.front();
        }

        const T &back()
        {
            std::lock_guard<std::mutex> lock(mutex_);
            return vector_.back();
        }

        void push_back(const T &item)
        {
            std::lock_guard<std::mutex> lock(mutex_);
            vector_.push_back(item);
        }

        void push_back(T &&item)
        {
            std::lock_guard<std::mutex> lock(mutex_);
            vector_.push_back(std::move(item));
        }

        void emplace_back(T &item)
        {
            std::lock_guard<std::mutex> lock(mutex_);
            vector_.emplace_back(std::move(item));
        }

        void pop_back()
        {
            std::lock_guard<std::mutex> lock(mutex_);
            vector_.pop_back();
        }

        bool empty()
        {
            std::lock_guard<std::mutex> lock(mutex_);
            return vector_.empty();
        }

        size_t size() const
        {
            std::lock_guard<std::mutex> lock(mutex_);
            return vector_.size();
        }

        void clear()
        {
            std::lock_guard<std::mutex> lock(mutex_);
            vector_.clear();
        }

        void erase(size_t index)
        {
            std::lock_guard<std::mutex> lock(mutex_);
            vector_.erase(vector_.begin() + index);
        }

        const T closest(const T &item)
        {
            std::lock_guard<std::mutex> lock(mutex_);
            T closest = vector_.front();
            int closest_index = 0;
            for (int i = 0; i < vector_.size(); i++)
            {
                if (std::hypot(vector_[i].pose.position.x - item.pose.position.x, vector_[i].pose.position.y - item.pose.position.y) <
                    std::hypot(closest.pose.position.x - item.pose.position.x, closest.pose.position.y - item.pose.position.y))
                {
                    closest = vector_[i];
                    closest_index = i;
                }
            }

            // reconstruct the vector without the closest item
            std::vector<T> new_vector;
            for (int i = 0; i < vector_.size(); i++)
            {
                if (i != closest_index)
                {
                    new_vector.push_back(vector_[i]);
                }
            }
            vector_ = new_vector;
            return closest;
        }

        void updateVector(const std::vector<T> vector)
        {
            std::lock_guard<std::mutex> lock(mutex_);
            vector_ = std::move(vector);
        }

    private:
        std::vector<T> vector_;
        std::mutex mutex_;
    };
}

#endif // EXPLORATION_TS_VECTOR_H