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

#include <tf2/LinearMath/Vector3.h>
#include <vector>
#include <algorithm>
#include <cmath>

namespace uav_ros2::utils
{
class ButterworthFilter
{
public:
  ButterworthFilter(double cutoff_frequency, double sampling_frequency)
  : cutoff_(cutoff_frequency), fs_(sampling_frequency)
  {
    computeCoefficients();
  }

  std::vector<double> filtfilt(const std::vector<double> & input)
  {
    std::vector<double> filtered = filter(input);
    std::reverse(filtered.begin(), filtered.end());
    filtered = filter(filtered);
    std::reverse(filtered.begin(), filtered.end());
    return filtered;
  }

  std::vector<double> filter(const std::vector<double> & input)
  {
    std::vector<double> output(input.size(), 0.0);
    for (size_t i = 0; i < input.size(); i++) {
      output[i] = a0 * input[i];
      if (i > 0) {
        output[i] += a1 * input[i - 1] - b1 * output[i - 1];
      }
      if (i > 1) {
        output[i] += a2 * input[i - 2] - b2 * output[i - 2];
      }
    }
    return output;
  }

private:
  double cutoff_;
  double fs_;
  size_t clip_samples_;
  double a0, a1, a2, b1, b2;

  void computeCoefficients()
  {
    double wc = tan(M_PI * cutoff_ / fs_);
    double k1 = sqrt(2.0) * wc;
    double k2 = wc * wc;
    double norm = 1 / (1 + k1 + k2);

    a0 = k2 * norm;
    a1 = 2 * a0;
    a2 = a0;
    b1 = 2 * (k2 - 1) * norm;
    b2 = (1 - k1 + k2) * norm;
  }
};


class DerivativeFilter
{
public:
  DerivativeFilter(double cutoff_frequency, double sampling_frequency)
  : filter_(cutoff_frequency, sampling_frequency), fs_(sampling_frequency) {}

  void addData(double value)
  {
    data_.push_back(value);
  }

  std::vector<double> computeDerivative()
  {
    if (data_.size() < 3) {return {};}
    std::vector<double> derivatives(data_.size(), 0.0);
    double dt = 1.0 / fs_;
    for (size_t i = 1; i < data_.size() - 1; ++i) {
      derivatives[i] = (data_[i + 1] - data_[i - 1]) / (2 * dt);
    }
    return derivatives;
  }

  std::vector<double> process()
  {
    auto derivatives = computeDerivative();
    if (derivatives.empty()) {return {};}
    return filter_.filtfilt(derivatives);
  }

  void process(std::vector<double> & derivative, std::vector<double> & derivative_filtered)
  {
    derivative = computeDerivative();
    if (derivative.empty()) {return;}
    derivative_filtered = filter_.filtfilt(derivative);
  }

private:
  ButterworthFilter filter_;
  double fs_;    // Sampling frequency
  std::vector<double> data_;
};


class DerivativeFilter3D
{
public:
  DerivativeFilter3D(double cutoff_frequency, double sampling_frequency)
  : filter_x_(cutoff_frequency, sampling_frequency),
    filter_y_(cutoff_frequency, sampling_frequency),
    filter_z_(cutoff_frequency, sampling_frequency),
    fs_(sampling_frequency) {}

  void addData(const tf2::Vector3 & value)
  {
    filter_x_.addData(value.x());
    filter_y_.addData(value.y());
    filter_z_.addData(value.z());
  }

  std::vector<tf2::Vector3> process()
  {
    auto x = filter_x_.process();
    auto y = filter_y_.process();
    auto z = filter_z_.process();
    if (x.empty() || y.empty() || z.empty()) {return {};}
    std::vector<tf2::Vector3> derivatives;
    for (size_t i = 0; i < x.size(); ++i) {
      derivatives.emplace_back(x[i], y[i], z[i]);
    }
    return derivatives;
  }

  void process(
    std::vector<tf2::Vector3> & derivative,
    std::vector<tf2::Vector3> & derivative_filtered)
  {
    std::vector<double> x, y, z, xf, yf, zf;
    filter_x_.process(x, xf);
    filter_y_.process(y, yf);
    filter_z_.process(z, zf);
    if (x.empty() || y.empty() || z.empty() || xf.empty() || yf.empty() || zf.empty()) {return;}
    derivative.clear();
    for (size_t i = 0; i < x.size(); ++i) {
      derivative.emplace_back(x[i], y[i], z[i]);
    }
    derivative_filtered.clear();
    for (size_t i = 0; i < xf.size(); ++i) {
      derivative_filtered.emplace_back(xf[i], yf[i], zf[i]);
    }
  }

private:
  DerivativeFilter filter_x_;
  DerivativeFilter filter_y_;
  DerivativeFilter filter_z_;
  double fs_;    // Sampling frequency
};
}  // namespace uav_ros2::utils
