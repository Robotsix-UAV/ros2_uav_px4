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

#include <memory>
#include "uav_cpp/utils/logger.hpp"
#include "uav_cpp/utils/module_io_formatters.hpp"

namespace uav_ros2::debug
{

template<std::size_t I, typename PipelineT>
void moduleLoop(std::shared_ptr<PipelineT> pipeline)
{
  if constexpr (I < PipelineT::ModuleCount::value) {
    UAVCPP_DEBUG(
      "Module {} \nInput: {} \nOutput: {}", I,
      pipeline->template getModuleInput<I>(), pipeline->template getModuleOutput<I>());
    moduleLoop<I + 1, PipelineT>(pipeline);
  }
}
}  // namespace uav_ros2::debug
