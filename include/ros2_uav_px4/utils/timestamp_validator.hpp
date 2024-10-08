// Copyright 2024 The Technology Innovation Institute (TII)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/**
 * @author Damien SIX (damien@robotsix.net)
 */

#pragma once

#include <chrono>
#include <cstdint>
#include <cmath>
#include "uav_cpp/utils/logger.hpp"

namespace uav_ros2
{
namespace utils
{

/**
 * @brief Class to validate timestamps based on monotonicity and tolerance
 */
class TimestampValidator : public uav_cpp::logger::LogTagHolder
{
public:
  /**
   * @brief Construct a new Timestamp Validator object
   *
   * @param tolerance_ns Acceptable difference in nanoseconds
   * @param max_outliers Max allowed consecutive outliers before reset
   */
  TimestampValidator(int64_t tolerance_ns = 1000000000, int max_outliers = 10)
  : uav_cpp::logger::LogTagHolder("Timestamp Validator"),
    previous_timestamp_ns_(0),
    tolerance_ns_(tolerance_ns),
    outlier_count_(0),
    max_outliers_(max_outliers),
    is_first_timestamp_(true)
  {
    previous_time_point_ = std::chrono::steady_clock::now();
  }

  /**
   * @brief Method to validate a timestamp
   *
   * @param timestamp_ns Timestamp in nanoseconds
   * @return true if timestamp is valid, false otherwise
   */
  bool isValidTimestamp(int64_t timestamp_ns)
  {
    auto now = std::chrono::steady_clock::now();

    if (is_first_timestamp_) {
      // Always accept the first timestamp
      previous_timestamp_ns_ = timestamp_ns;
      previous_time_point_ = now;
      is_first_timestamp_ = false;
      outlier_count_ = 0;
      return true;
    }

    // Check if the timestamp is monotonic
    if (timestamp_ns < previous_timestamp_ns_) {
      // Timestamp decreased, invalid
      outlier_count_++;
      if (outlier_count_ >= max_outliers_) {
        resetReference();
      }
      UAVCPP_WARN_TAG(this, "Timestamp decreased, invalid");
      return false;
    }

    // Compute expected timestamp based on previous timestamp and elapsed time
    int64_t elapsed_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
      now - previous_time_point_).count();
    int64_t expected_timestamp_ns = previous_timestamp_ns_ + elapsed_ns;

    int64_t diff_ns = timestamp_ns - expected_timestamp_ns;

    if (std::abs(diff_ns) > tolerance_ns_) {
      // Difference is larger than tolerance, invalid
      outlier_count_++;
      if (outlier_count_ >= max_outliers_) {
        resetReference();
      }
      UAVCPP_WARN_TAG(this, "Timestamp too far from expected, invalid");
      UAVCPP_WARN_TAG(this, "Expected: {}, Received: {}", expected_timestamp_ns,
                      timestamp_ns);
      return false;
    }

    // Timestamp is valid
    previous_timestamp_ns_ = timestamp_ns;
    previous_time_point_ = now;
    outlier_count_ = 0;      // Reset outlier count
    return true;
  }

  /**
   * @brief Method to reset the reference timestamp
   */
  void resetReference()
  {
    previous_timestamp_ns_ = 0;
    outlier_count_ = 0;
    previous_time_point_ = std::chrono::steady_clock::now();
    is_first_timestamp_ = true;
  }

private:
  int64_t previous_timestamp_ns_;    ///< Previous valid timestamp in nanoseconds
  int64_t tolerance_ns_;             ///< Acceptable difference in nanoseconds
  int outlier_count_;                ///< Number of consecutive outliers
  int max_outliers_;                 ///< Max allowed consecutive outliers before reset
  bool is_first_timestamp_;          ///< Flag to check if first timestamp
  std::chrono::steady_clock::time_point previous_time_point_;    ///< Time point of previous timestamp
};

}  // namespace utils
}  // namespace ros2_uav_px4
