import sys

import roslib.message

from persistent_topics.persistent_topics_common import PersistentTopics

if len(sys.argv) < 2:
    print("Usage: %s FILENAME" % sys.argv[0])
    print("  FILENAME should be a persistent_topics persistence file.")
    print("  This script will print out the contents of the specified persistence file.")
    exit(1)

pt = PersistentTopics()
pt.file_name = sys.argv[1]
pt.readFromFile()

for topic in pt.latched_messages:
    type_name = pt.topic_type_names[topic]
    topic_type = roslib.message.get_message_class(type_name)
    msg = topic_type()
    msg.deserialize(pt.latched_messages[topic])
    print("Topic '%s':" % topic)
    print(msg)
