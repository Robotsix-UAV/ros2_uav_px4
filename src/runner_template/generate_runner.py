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

import argparse
import yaml
import re
from jinja2 import Environment, FileSystemLoader


def camel_to_snake(name):
    """
    Transform camel case to snake case.

    :param name: The camel case string.
    :return: The snake case string.
    """
    s1 = re.sub('(.)([A-Z][a-z]+)', r'\1_\2', name)
    return re.sub('([a-z0-9])([A-Z])', r'\1_\2', s1).lower()


def generate_cpp(config_file, output_file):
    """
    Generate C++ code from a configuration file.

    :param config_file: The path to the YAML configuration file.
    :param output_file: The path to the output C++ file.
    """
    # TODO(robotsix): Check the the configuration file architecture

    # Load configuration
    with open(config_file, 'r') as file:
        config = yaml.safe_load(file)

    # Setup Jinja2 environment
    env = Environment(loader=FileSystemLoader('.'), trim_blocks=True, lstrip_blocks=True)
    env.filters['camel_to_snake'] = camel_to_snake
    template = env.get_template('modes_runner.jinja')

    # Render the template with data
    cpp_code = template.render(modes=config['modes'])

    # Output to C++ file
    with open(output_file, 'w') as file:
        file.write(cpp_code)


def generate_service(config_file, output_file):
    """
    Generate C++ code from a configuration file.

    :param config_file: The path to the YAML configuration file.
    :param output_file: The path to the output C++ file.
    """
    with open(config_file, 'r') as file:
        config = yaml.safe_load(file)

    env = Environment(loader=FileSystemLoader('.'), trim_blocks=True, lstrip_blocks=True)
    template = env.get_template("modes_service.jinja")

    service_content = template.render(modes=config['modes'])

    with open(output_file, 'w') as file:
        file.write(service_content)


def main():
    parser = argparse.ArgumentParser(description="Generate C++ code from a configuration file.")
    parser.add_argument(
        "-c", "--config", type=str, required=True, help="Path to the YAML configuration file.")
    parser.add_argument(
        "-o", "--output", type=str, required=True, help="Path to the output C++ file.")
    parser.add_argument(
        "-s", "--service", type=str, required=True, help="Path to the output service file.")

    args = parser.parse_args()
    generate_cpp(args.config, args.output)
    generate_service(args.config, args.service)


if __name__ == "__main__":
    main()
