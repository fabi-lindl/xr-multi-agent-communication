# xr-multi-agent-communication
Multi-agent communication infrastructure for ROS-enabled robots and Unity-operating XR headsets.

## Introduction
The Multi-Agent Communication System (MACS) is designed to enable communication between multiple ROS-enabled robots and Unity-operating XR headsets. At this stage of development, one headset can communicate with multiple robots, and one robot can communicate with multiple headsets. This allows for an n x m communication between headsets and robots. Further development could focus on extending the system to enable communication between headsets and between robots. The current communication protocol was designed with inter-robot and inter-headset communication in mind. Therefore, future development of communication among the same type of agents can focus on the interfaces present on the agents and use the already existing network protocol.

### MACS Components
The MACS provides functionality that enables multiple headsets and robots to communicate with each other. This communication takes place on the Multi-Agent Network (MAN). Headsets that come online on the MAN are called unityagents, whereas robots on the MAN are called rosagents. The component that enables communication between unityagents and rosagents is the rosrouter. This rosrouter is a server socket implementation in Python running on a ROS2 node. It is possible to run the rosrouter locally on a desktop computer, in a docker container, or in the cloud. This optionality is a prevalent to provide as much flexibility in operation as possible, since the ultimate goal of teleoperation is to operate a fleet of robots remotely.

#### Use of Unity Software Packages
The Unity side of the MACS uses software components from Unity’s ROS-TCP-Connector to serialize/deserialize ROS messages sent on the MAN. In order to make use of this functionality you must install the Robotics package from Unity (https://github.com/Unity-Technologies/ROS-TCP-Connector#installation). The details on how the serializer classes are used by the MACS are explained in the Messages section.

## Messages
Two different categories of messages can be exchanged between rosagents and unityagents. These two categories are born out of the need to exchange ROS messages between agents on the MAN on the one hand, and to provide functionality to exchange data for administrative tasks on the other hand. The first category, ROS message exchange, allows MAN agents to work with all types of ROS messages, ultimately enabling teleoperation. The second category, management message exchange, allows data to be exchanged to perform administrative tasks. For instance, the configuration of agents coming online on the MAN is achieved by exchanging such management messages.

### ROS Message Exchange
ROS messages can be exchanged between rosagents and unityagents. Inspecting and controlling the various states of a robot is a prevalent for teleoperation. Therefore, the MACS enables its agents to send ROS messages back and forth between them as plain bytes. Rosagents work internally with traditional ROS message datatypes, which are only serialized to a byte array before being sent to other agents on the MAN. ROS message handling on unityagents is achieved by leveraging functionality of the ROS-TCP-Connector package, which can be installed via the Unity Editor. This package was developed by Unity and provides functionality to serialize ROS messages in the form of byte arrays to C# classes. Every standard ROS message has a corresponding C# class on the Unity side. Customized ROS messages can also be converted to such C# classes using this Unity package (see “Create custom C# ROS message classes” below).

The C# ROS message class archive can be found here: https://github.com/Unity-Technologies/ROS-TCP-Connector/tree/main/com.unity.robotics.ros-tcp-connector/Runtime/Messages.

The code to create new C# classes from ROS message files (.msg and .srv) as well as the functionality to serialize ROS messages to such C# classes can be found here: https://github.com/Unity-Technologies/ROS-TCP-Connector/tree/main/com.unity.robotics.ros-tcp-connector/Runtime/MessageGeneration. The MACS uses the MessageSerializer and MessageDeserializer classes. The MessageDeserializer is used to convert the plain byte representation of ROS messages into C# classes, whereas the MessageSerializer is used to convert C# classes into byte arrays representing ROS messages. Such byte arrays can be deserialized by the built-in ROS deserialization method on rosagents. The following figure illustrates the message transfer and its underlying steps from a rosagent to a unityagent.

![alt text](https://github.com/fabi-lindl/xr-multi-agent-communication/blob/main/documentation/images/rosagent_to_unityagent_ros_message_transfer.png)

#### Create custom C# ROS message classes
Click on Robotics in the Unity Editor menu bar. Click on "Generate ROS Messages...". A new window will open allowing you to create new C# classes from your custom ROS message files. Click the Browse button and navigate to the directory where your .msg and .srv files are stored. The Unity Editor will suggest building all the ROS messages found in the directory you specified. Detailed instructions are also provided by Unity here: https://github.com/Unity-Technologies/Unity-Robotics-Hub/blob/main/tutorials/ros_unity_integration/setup.md.

#### ROS Interface Nodes
ROS Interface Nodes are traditional ROS nodes that are instantiated on rosagents following a request for instantiation from one or more unityagents on the MAN. While such nodes provide all the functionality of traditional ROS nodes, parts of this functionality are leveraged in a specific way to enable their host rosagent to communicate with other agents on the MAN. In particular, ROS Interface nodes can take on the roles of a publisher, subscriber, service or client. Each of these four roles is fulfilled by a ROS Interface Node of a specific type. Hence, four different types of ROS Interface Nodes exist.
ROS Interface Nodes are the fundamental building block that enables communication between different agents on the MAN. The following figure illustrates the concept. ROS Interface Nodes communicate locally with ROS elements such as topics and services on a rosagent and MAN-wide by sending data to and receiving data from the rosrouter.

![alt text](https://github.com/fabi-lindl/xr-multi-agent-communication/blob/main/documentation/images/ros_interface_nodes_illustration.png)

Every ROS Interface Node is assigned a unique name and data exchange type. The node name is based on the ROS topic or service name it communicates with. The data exchange type represents the message/service type of messages that are sent to the topic/service the interface node communicates with. This tuple of node type, node name, and data exchange type is part of every ROS message sent across the MAN to identify the node that processes the message on an agent (read the Metadata section in Message below). A ROS Interface Node is assigned its name and data exchange type for its entire lifetime, i.e. the topic/service the node talks to on a rosagent cannot be changed during its lifetime.
The figure below illustrates all the input arguments that a ROS interface node can take on its instantiation. The ROS Topic/Service Name identifies the ROS topic/service the instantiated interface node is supposed to communicate with on a rosagent. Whereas Destination Agent IDs identify separate rosagents on the MAN that are instructed to instantiate the desired interface node, the Destination Group IDs identify entire groups of rosagents that are instructed to instantiate the interface node. Callbacks provide the ROS agent interface node with its functionality on the unityagent. Multiple callbacks can be registered on an interface node, with each callback representing a different functionality. Hence, the callbacks of a ROS interface node represent its toolbox. Callbacks can be registered and deregistered during the lifetime of a node. It is important to note that each callback is applied on an incoming message. The RosAgent BufferSize and the UnityAgent Buffer Size represent the message buffer sizes on the rosagent and unityagent respectively. Message Latching can be enabled for publisher nodes if desired. The Message Drop Strategy allows you to decide whether to drop the newest incoming messages or the oldest stored messages in a message buffer (further details are provided in the README.md in the unity directory of the repository).

![alt text](https://github.com/fabi-lindl/xr-multi-agent-communication/blob/main/documentation/images/interface_node_instantiation_arguments.png)

Once an interface node has been instantiated on a rosagent, any further instantiation attempts by other agents on the MAN do not result in the instantiation of a new node. Instead, the agent type and agent id of the agent, which attempted to instantiate the node, are stored on the node. This helps the interface node on the rosagent to keep track of all agents that have permission to communicate with it. With respect to deregistration, an interface node remains alive as long as agents are still registered on it. Only when the last agent instructs the rosagent to deregister a particular node is that node deregistered from the rosagent.

![alt text](https://github.com/fabi-lindl/xr-multi-agent-communication/blob/main/documentation/images/interface_node_agent_registration.png)

##### InterfacePublisherNode
The InterfacePublisherNode enables other agents on the MAN to publish ROS messages to topics on its host rosagent. Messages of a specific type are published to a specific ROS topic.

##### InterfaceSubscriberNode
The InterfaceSubscriberNode enables other agents on the MAN to subscribe to ROS messages sent to ROS topics on its host rosagent. Subscriptions are defined by a specific ROS topic and ROS message type.

##### InterfaceServiceNode 
The InterfaceServiceNode enables its host rosagent to send ROS service requests to unityagents on the MAN. Unityagents can implement functionality to handle such requests and send the response back to the rosagent. This node acts as a proxy service between the ROS network client on the host rosagent and unityclients on the MAN, that is, it forwards requests received from a ROS client to unityagents, and forwards the response received from unityagents to the originating ROS client. It is important to note that only one unityagent can be associated with this node type. Any attempt by another unityagent to register the node is going to fail. Only when the first unityagent deregisters the node from the rosagent, is it possible for other unityagents to register the same node again.

##### InterfaceClientNode
The InterfaceClientNode enables unityagents on the MAN to make requests to running ROS services on its host rosagent. This node acts as a proxy service between unityagents and the ROS service. This node forwards ROS message requests received from unityagents to the ROS service and passes the response received from the ROS service on to unityagents. 

### Management Message Exchange
Any non-ROS messages that are exchanged between agents on the MAN are considered as management messages. Such messages are used to execute administrative tasks on agents. For instance, when an agent connects to the rosrouter, that is, it goes online on the MAN, the agent is sent a configuration message from the rosrouter. This message is a management message. Another example is the instantiation of ROS Interface Nodes on rosagents. If a unityagent wants to exchange ROS messages between a rosagent and itself, the unityagent first sends a management message to this particular rosagent instructing it to instantiate a ROS Interface Node. Management message data is sent in JSON format. The figure below illustrates the management message transfer from a unityagent to a rosagent.

![alt text](https://github.com/fabi-lindl/xr-multi-agent-communication/blob/main/documentation/images/unityagent_to_rosagent_management_message_transfer.png)

Every management message has a designated management message manager that is responsible for its handling, i.e. there exists one management message manager for every management message. Management message managers are implemented as C# classes on the Unity side and as Python classes on the ROS side. If a new management message is needed, its key-value pairs must be designed, a C# class representation of it must be implemented on the Unity side, and a new management manager class must be written on the agent where this management message is going to be handled. Existing management message managers can be found in a directory labeled management in both the ros_packages of the rosagent and rosrouter as well as in the unityagent directory on the Unity side.

In order to functionally integrate a new management message manager into the system, its class and module name must follow a specific naming convention. The class name of management message managers must be written in upper camel case notation. Module names of management message manager classes work differently for C# and Python code. In C# code, the module name must be exactly the same as the name of the class it contains. As far as Python code is concerned, the module name must be a snake case representation of the class name. This is best illustrated by an example. If the Python management message manager class is, for instance, named WorldClassManager, its module name must be named world_class_manager.py. The snake case module convention is enforced as it is recommended by the Python PEP 8 style guide.

## System Message
Messages exchanged between agents on the MAN consist of two parts. The first part is routing information, which instructs the rosrouter on how to handle a system message. The second part is the actual message data that is transmitted to other agents.

![alt text](https://github.com/fabi-lindl/xr-multi-agent-communication/blob/main/documentation/images/system_message_layout.png)

### Routing Information
Routing information is created for every message sent on the MAN. This information is used by the rosrouter to decide how to handle the system message. Agents only receive the message part of the system message.

#### Routing Data
The routing data is the first part of routing information and has a fixed size length during program runtime. Its length can be adjusted by changing the number of destination agent ids and destination group ids it can entail in the corresponding configuration files. The default size of the routing data is 39 bytes. The distinct parts of the routing data are as follows.
-	Agent Type (1 byte): This flag allows identifying the destination agent type the system message is sent to. When this value is set to 1, the message is solely forwarded to rosagents, while setting the agent type to 2 forwards the message exclusively to unityagents. This distinction is required as agent groups can be heterogeneous with respect to agent types, that is, they can have rosagent and unityagent group members. By setting the agent type, it is possible to forward a message only to agents of a specific type of a destination group. To broadcast the message to all agents on the MAN, the agent type must be set to zero.
-	Destination Agents (10 x 2 bytes): A byte array that identifies the agents the system message is sent to. Ten slots are available by default, each with a size of 2 bytes, since the data type used for agent ids in C# is ushort. The number of slots can be customized in routing_data.json on the ROS side and in RoutingDataConfiguration.cs on the Unity side.
-	Destination Groups (10 x 1 byte): A byte array that identifies the groups of agents the system message is sent to. All agents of a destination group that match the type set in the Agent Type field receive the message. Ten slots are available by default, each with a size of 1 byte, as the data type used for group ids is byte. The number of slots can be customized in routing_data.json on the ROS side and in RoutingDataConfiguration.cs on the unity side.
-	Management Data Size (4 bytes): Identifies the size of the management data that is part of routing information. If the management data size is zero, the message is directly sent to the destinations outlined in Destination Agents and Destination Groups. Otherwise, the message manager specified in the management data is instantiated and used to process the system message.
-	Message Size (1 byte): Identifies the message size.

![alt text](https://github.com/fabi-lindl/xr-multi-agent-communication/blob/main/documentation/images/routing_data_layout.png)

#### Management Data
Management data is of JSON format and allows instructing the rosrouter to carry out specific actions. This part of routing information is thus only populated if the rosrouter needs to execute specific processing other than forwarding the message to the destination agents. Management data is available to insert rosrouter processing steps before the message part of the system message is forwarded to its destinations or to instruct the rosrouter to perform actions that have nothing to do with message forwarding at all. For instance, the connection handshake when a new agent goes online on the MAN does not deal with message forwarding, but requires the rosrouter to execute specific actions for agent configuration. Management data, if used, must include the “messageManager” key. Its value is used to instantiate the corresponding management message manager class instance on the rosrouter that performs the specific processing. The management Data keys must be formatted in camel case.

### Message

#### Metadata
Message metadata is of JSON format and contains a standardized set of key-value pairs. These pairs instruct rosagents and unityagents on how to process the data part of the message. The key value pairs are listed below.
-	senderAgentType (byte): Identifies the sender agent type (e.g. rosagents have id 1).
-	senderAgentId (ushort): Identifies the sender agent by its id.
-	requestId (uint): Identifies requests, i.e. incoming messages that require a response message. The value zero stands for no response required.
-	messageManager (string): Identifies the message manager class that is used to process the message data. An empty string identifies the message as a ROS message.
-	nodeType (byte): Identifies the ROS Interface Node type (e.g. publisher).
-	nodeName (string): Identifies the ROS Interface Node that processes the message data.
-	dataExchangeType (string): Identifies the ROS message/service type.

![alt text](https://github.com/fabi-lindl/xr-multi-agent-communication/blob/main/documentation/images/message_metadata_layout.png)

As a message is either a management message or a ROS message, only a subset of the metadata values is populated for every message sent. If a management message is sent, the messageManager value is populated, otherwise it is an empty string. That is, if the messageManager value is a non-empty string, the message is identified as a management message, while an empty string identifies the message as a ROS message. For ROS messages the value for the nodeType, nodeName, and dataExchangeType are populated.

#### Data
The data part of the message is used to either send ROS messages or JSON data of management messages on the MAN. If JSON data is sent on the MAN, its keys must be formatted in camel case.