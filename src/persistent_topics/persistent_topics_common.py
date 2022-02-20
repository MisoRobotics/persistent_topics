import six
import struct
import sys
import os

import rospy

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
        if not six.PY2:
            c = c.decode("utf-8")
        if len(c) == 0:
            raise ValueError("Error reading file; encountered end of file instead of NULL string terminator")
        if c[0] == '\0':
            break
        result += c[0]
    return result

class PersistentTopics:
    __metaclass__ = ABCMeta

    def __init__(self, file_name=None):
        self.topic_type_names = dict()
        self.latched_messages = dict()
        self.file_name = file_name

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

    def writeToFile(self, file_name=None):
        with open(file_name or self.file_name, "wb") as f:
            f.write(struct.pack('I', len(self.latched_messages)))
            def _write_utf8(msg):
                if not six.PY2:
                    msg = msg.encode("utf-8")
                f.write(msg)

            for topic in self.latched_messages:
                _write_utf8(topic)
                f.write(as_byte(0))
                _write_utf8(self.topic_type_names[topic])
                f.write(as_byte(0))
                content = self.latched_messages[topic]
                f.write(struct.pack('I', len(content)))
                f.write(content)

    def readFromFile(self):
        self.topic_type_names = dict()
        self.latched_messages = dict()
        with open(self.file_name, "rb") as f:
            nTopics = struct.unpack('I', f.read(4))[0]
            for i in range(nTopics):
                topic = readString(f)
                self.topic_type_names[topic] = readString(f)
                content_length = struct.unpack('I', f.read(4))[0]
                self.latched_messages[topic] = f.read(content_length)
                if len(self.latched_messages[topic]) != content_length:
                    raise ValueError("Persistence file is corrupt; encountered end of file while reading content for topic %s from %s", topic, self.file_name)


class LatchPublisher(rospy.Publisher, rospy.SubscribeListener):
    '''
    Correctly implements multiple Publisher latches in a single node.
    queue_size is shared between all publisher instances sharing a particular topic (because of underlying ROS infrastructure); be sure it's large enough
    See https://github.com/ros/ros_comm/issues/146
    '''
    def __init__(self, name, data_class, tcp_nodelay=False, headers=None, queue_size=None):
        super(LatchPublisher, self).__init__(name, data_class=data_class, tcp_nodelay=tcp_nodelay, headers=headers, queue_size=queue_size, subscriber_listener=self, latch=False)
        self.message = None

    def publish(self, msg):
        self.message = msg
        super(LatchPublisher, self).publish(msg)

    def peer_subscribe(self, resolved_name, publish, publish_single):
        if self.message is not None:
            publish_single(self.message)
