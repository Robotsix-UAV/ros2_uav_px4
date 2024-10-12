import yaml
from jinja2 import FileSystemLoader, Environment

# Load the topics from a YAML file
with open('dds_topics.yaml', 'r') as f:
    topics_list = yaml.safe_load(f)

# Generate the publishers and subscribers lists from the topics
publishers = []
subscribers = []

# In the yaml file subscription/publications are defined relative to fcu
# In this python script, publishers/subscribers are defined relative to the
# companion computer, hence the inversion
for topic in topics_list['subscriptions']:
    publishers.append({
        'type': topic['type'],
        'topic': topic['topic'].replace('/', '', 1),
        'header': topic['topic'].replace('/fmu/in', 'px4_msgs/msg') + '.hpp'
    })
for topic in topics_list['publications']:
    subscribers.append({
        'type': topic['type'],
        'topic': topic['topic'].replace('/', '', 1),
        'callback': 'on' + topic['type'].replace('px4_msgs::msg::', ''),
        'header': topic['topic'].replace('/fmu/out', 'px4_msgs/msg') + '.hpp'
    })

# Combine the publishers and subscribers lists into a single topics list
topics = publishers + subscribers

# Render the Jinja2 template with the topics list
template_file = 'px4_comm_base.hpp.j2'
output_file = 'px4_comm_base.hpp'

templateLoader = FileSystemLoader(searchpath="./")
templateEnv = Environment(loader=templateLoader)
template = templateEnv.get_template(template_file)

output = template.render(publishers=publishers,
                         subscribers=subscribers, topics=topics)

# Write the rendered output to a file
with open(output_file, 'w') as f:
    f.write(output)

print(f"Generated {output_file}")