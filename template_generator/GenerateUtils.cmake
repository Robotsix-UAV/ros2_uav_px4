# Copyright 2024 The Technology Innovation Institute (TII)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Author: Damien Six (damien@robotsix.net)

# Define the custom function for checking and generating files if needed
function(check_and_generate_if_needed config_file output_file template_file)
  set(PYTHON_GENERATOR "${CMAKE_CURRENT_SOURCE_DIR}/template_generator/generate_template.py")
  # Check if the output file exists
  if(NOT EXISTS ${output_file})
    set(GENERATE_FILE TRUE)
  else()
    # Compare the modification times
    file(TIMESTAMP ${config_file} CONFIG_TIMESTAMP)
    file(TIMESTAMP ${output_file} OUTPUT_TIMESTAMP)
    file(TIMESTAMP ${template_file} TEMPLATE_TIMESTAMP)

    # If the output file is older than the config file, mark for regeneration
    if(CONFIG_TIMESTAMP STRGREATER OUTPUT_TIMESTAMP OR TEMPLATE_TIMESTAMP STRGREATER OUTPUT_TIMESTAMP)
      set(GENERATE_FILE TRUE)
    else()
      set(GENERATE_FILE FALSE)
    endif()
  endif()

  # Generate the file if needed
  if(GENERATE_FILE)
    message(STATUS "Generating ${output_file} from ${config_file}...")

    # Run the provided command to generate the output file
    execute_process(
      COMMAND python3 ${PYTHON_GENERATOR} -c ${config_file} -o ${output_file} -t ${template_file}
      RESULT_VARIABLE result
    )

    # Check if the command was successful
    if(result EQUAL 0)
      message(STATUS "${output_file} generated successfully.")
    else()
      message(FATAL_ERROR "Failed to generate ${output_file}. Command exited with code ${result}")
    endif()
  else()
    message(STATUS "${output_file} is up to date, skipping generation.")
  endif()

endfunction()
