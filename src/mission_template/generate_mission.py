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


def replace_slash(name):
    """
    Replace slashes with underscores.

    :param name: The string with slashes.
    :return: The string with underscores.
    """
    return name.replace("/", "_")


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
    env.filters['replace_slash'] = replace_slash
    template = env.get_template('mission_executor.jinja')

    # Render the template with data
    cpp_code = template.render(steps=config['steps'])

    # Output to C++ file
    with open(output_file, 'w') as file:
        file.write(cpp_code)


def main():
    parser = argparse.ArgumentParser(
        description="Generate C++ code from a configuration file.")
    parser.add_argument(
        "-c", "--config", type=str, required=True, help="Path to the YAML configuration file.")
    parser.add_argument(
        "-o", "--output", type=str, required=True, help="Path to the output C++ file.")

    args = parser.parse_args()
    generate_cpp(args.config, args.output)


if __name__ == "__main__":
    main()
