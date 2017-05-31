# persistent_topics
Persists the contents of specific topics between roscore invocations

## Introduction
It is often necessary for nodes to persist information between sessions.  A common approach to this problem appears to be to specify a file name as a parameter to the node or service and have the node serialize/deserialize to that location upon request (usually via ROS service).  To implement this pattern, a node author must essentially write a loader method, saver method, and translation layer depending on how the information will be consumed.

persistent_topics aims to simplify this process by abstracting the details of data persistence away from the persistence-using node via built-in ROS features (specifically, topics).

## Assumptions
In a single_channel_persistent_topics_node, persistent_topics operates under the assumption that that topic is a single-channel topic.  That is, it represents the state of a single thing and each successive message supersedes the previous message.  It does not make sense for single-channel topics to have multiple publishers latch messages on that topic.

In a multi_channel_persistent_topics_node, persistent_topics treats the main topic as the output topic and the source topic as the input topic.  persistent_topics listens for incoming messages on the source topic ONLY and then latch-echos them on the source topic in addition to publishing them on the main topic.

## Usage
In a launch file, declare a persistent_topics node for each distinct file you would like to use.  A single persistent_topics node can persist multiple topics.  Provide the persistent_topics node with two parameters: the path of the file to persist to/from, and the list of topic paths to persist (see launch/persistent_topics_test.launch for an example).  From a different node (or via rostopic), publish messages to one or more of the topics tracked by your persistent_topics node.  Shut down everything and restart it.  Observe that the tracked topics are populated with messages from the last session by the persistent_topics node.

## Parameters

```file_name```: The path to the file in which persistent topic messages for this node will be stored.  If a relative path is given (e.g., "persistence.ptb"), it will be relative to ~/.ros.

In all topic addresses below, if a topic starts with /, it is treated as absolute and only that exact topic will match.  Otherwise, the topic is treated as relative and will be resolved as the /NODENAME/TOPIC.  This allows relative topics to treated as "subfolders" of the node instance, and the node instance's base name can be easily changed.

### Single channel
```topics```: A list of strings, each containing a topic to persist with this node.

### Multi channel
```main_topics```: A list of strings, each containing a topic to publish persisted information on.

```source_topics```: A list of strings where each string specifies an input topic on which to listen for messages that will be persisted and republished on the corresponding topic in ```main_topics```.

## Details
* If a cached message exists for a topic, it will be published and latched to the main topic upon startup, regardless of other publishers on that topic.
* If a new message on a monitored topic is latched, this node will NOT republish that message on the topic it arrived on.  This is because it interprets the other publisher to be saying, "I am the primary source of this information now."
* If a new message on a monitored topic is not latched, this will republish and latch that message.  This is because it interprets the other publisher to be saying, "I'm not taking responsibility for making sure this information is persistently available."
* Only the most recent message on a particular incoming topic (regardless of publisher/origin) is retained.
* The file is entirely rewritten every time a new message is received, so this package is not well-suited to high-frequency topics.
* Changes in message type on a given topic are handled fine by this package, but are probably bad practice in general.
* Changes in the message format of a message persisted to file will render the entire file unusable.
* Deserialization is only performed once when publishing the persisted message for each topic.
* Serialization is never performed; instead, persistent_topics uses the already-serialized form of the message from the ROS infrastructure.

## Use cases
There are some types of information that are infrequently produced by a tool (ROS node) rather than manually entering the data (for instance, calibration of a camera's position and orientation), but consumed often by other parts of the system.  This node allows that information to be written to and read from a particular topic without needing to worry about how the information is persisted.

As a useful side effect, consumers can choose to be notified whenever their content of interest changes.

If a node follows the dynamic-parameters-via-topic pattern, simply including the topic of the dynamic parameters in this single-channel node's ```topics``` list will suffice to persist any changes in that information to file.

To persist information on a multi-channel topic (where multiple messages from multiple sources may need to be combined to form the current state of information on the topic, like /tf_static), specify an arbitrary ```source_topic``` and publish messages to that topic to have them both persisted and echoed on the output topic (e.g., /tf_static) by the persistent_topics node.
