import struct
import sys
import os

import rospy
import roslib.message

from abc import ABCMeta

# http://stackoverflow.com/questions/35038999/write-a-single-byte-to-a-file-in-python-3-x
if sys.version_info.major >= 3:
    as_byte = lambda value: bytes([value])
else:
    as_byte = chr

# Reads a null-terminated string from stream and returns it
def readString(f):
    result = ''
    while True:
        c = f.read(1)
        if len(c) == 0:
            raise ValueError("Error reading file; encountered end of file instead of NULL string terminator")
        if c[0] == '\0':
            break
        result += c[0]
    return result

class PersistentTopics:
    __metaclass__ = ABCMeta

    def initFile(self):
        if not rospy.has_param("~file_name"):
           raise Exception("persistent_topics_node requires local parameter 'file_name'")
        self.file_name = rospy.get_param("~file_name")

        if os.path.isfile(self.file_name):
            rospy.logdebug("Loading persisted messages from file at %s", self.file_name)
            self.readFromFile()
        else:
            rospy.logdebug("Created new persistence file at %s", self.file_name)
            file_path = os.path.dirname(self.file_name)
            if not os.path.exists(file_path) and len(file_path) > 0:
                os.makedirs(file_path)

    def writeToFile(self):
        with open(self.file_name, "w") as f:
            f.write(struct.pack('I', len(self.latched_messages)))
            for topic in self.latched_messages:
                f.write(topic)
                f.write(as_byte(0))
                f.write(self.topic_type_names[topic])
                f.write(as_byte(0))
                content = self.latched_messages[topic]
                f.write(struct.pack('I', len(content)))
                f.write(content)

    def readFromFile(self):
        self.topic_type_names = dict()
        self.latched_messages = dict()
        with open(self.file_name, "r") as f:
            nTopics = struct.unpack('I', f.read(4))[0]
            for i in range(nTopics):
                topic = readString(f)
                self.topic_type_names[topic] = readString(f)
                content_length = struct.unpack('I', f.read(4))[0]
                self.latched_messages[topic] = f.read(content_length)
                if len(self.latched_messages[topic]) != content_length:
                    raise ValueError("Persistence file is corrupt; encountered end of file while reading content for topic %s from %s", topic, self.file_name)

    def getPublisher(self, topic):
        currentType = self.topic_type_names[topic]
        publisherMissing = not topic in self.publishers
        typeMismatch = False
        if not publisherMissing:
            typeMismatch = currentType != self.publishers[topic][1]

        if publisherMissing or typeMismatch:
            if typeMismatch:
                rospy.logwarn("Recreating publisher due to type mismatch (%s previously, %s now).  This will cause one 'Could not process inbound connection' warning below.",
                              self.publishers[topic][1], currentType)
                self.publishers[topic][0].unregister()
            topic_type = roslib.message.get_message_class(currentType)
            self.publishers[topic] = (rospy.Publisher(topic, topic_type, latch=True, queue_size=1), currentType)
            rospy.logdebug("Publisher of " + currentType + " on " + topic + " has been created")
        return self.publishers[topic][0]