# persistent_topics
Persists the contents of specific topics between roscore invocations

## Introduction
It is often necessary for nodes to persist information between sessions.  A common approach to this problem appears to be to specify a file name as a parameter to the node or service and have the node serialize/deserialize to that location upon request (usually via ROS service).  To implement this pattern, a node author must essentially write a loader method, saver method, and translation layer depending on how the information will be consumed.

persistent_topics aims to simplify this process by abstracting the details of data persistence away from the persistence-using node via built-in ROS features (specifically, topics).

## Usage
In a launch file, declare a persistent_topics node for each distinct file you would like to use.  A single persistent_topics node can persist multiple topics.  Provide the persistent_topics node with two parameters: the path of the file to persist to/from, and the list of topic paths to persist (see launch/persistent_topics_test.launch for an example).  From a different node (or via rostopic), publish messages to one or more of the topics tracked by your persistent_topics node.  Shut down everything and restart it.  Observe that the tracked topics are populated with messages from the last session by the persistent_topics node.

## Parameters
```file_name```: The path to the file in which persistent topic messages for this node will be stored.  If a relative path is given (e.g., "persistence.ptb"), it will be relative to ~/.ros.

```topics```: A list of strings, each containing a topic to persist with this node.  If a topic starts with /, it is treated as absolute and only that exact topic will match.  Otherwise, the topic is treated as relative and will be resolved as the /NODENAME/TOPIC.  This allows relative topics to treated as "subfolders" of the node instance, and the node instance's base name can be easily changed.

## Details
* The file is entirely rewritten every time a new message is received, so this package is not well-suited to high-frequency topics.
* Only the most recent message on a particular topic (regardless of publisher/origin) is retained.
* Changes in message type on a given topic are handled fine by this package, but are probably bad practice in general.
* Changes in the message format of a message persisted to file will render the entire file unusable.
* Deserialization is only performed once when publishing the persisted message for each topic.
* Serialization is never performed; instead, persistent_topics uses the already-serialized form of the message from the ROS infrastructure.
