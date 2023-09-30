# Unity xr-multi-agent-communication implementation
*Note: Before you go ahead and read this README.md file, you should have read the README.md file in the root directory of this repository. It serves as a basic project overview.*

## Abstract
The code stored in this directory of the repo represents the Unity side of the ROS2/Unity tandem project. Since Unity is based on C#, this side of the project is implemented in C#. This application should serve as a basic building block for any further Unity applications that wish to use multi-agent communication between Unity and ROS2 applications. All details on how to use this application to add multi-agent communication capabilities to your code can be found in this README.md file, as well as in the README.md file stored in the ros directory of the same repo.

## Creating the project
To be able to create a Unity project from the code of this repo you must first:
- **Download and install the Unity Hub** (https://unity.com/download): The Unity Hub is a program designed to help developers manage their Unity projects effectively. It allows different versions of Unity to be installed so that a specific version can be selected for working on your projects. It also provides easy installation options if additional modules are needed for a project later in development, which is not possible when installing a Unity editor via platform-specific executables.
- **Install a Unity Editor**: The Unity Editor is software that allows you to create your Unity applications. After opening Unity Hub on your computer, click on "Installs" on the left menu bar and then click "Install Editor." Install one of the recommended LTS versions. If you decide to install an older version, visit theUnity download archive to find the version you need. It is essential that you select the download for Unity Hub option on this page. Using platform-specific installer executables prevents the downloaded Unity Editor from being extended with additional modules after installation.
- **Create a Unity project from the code in this repo**:
    - Clone this repo
    - Open the Unity Editor and click on Open
    - Navigate to the root directory of the cloned repo and open the unity directory
    - The Unity Editor opens and starts configuring the project
    - After the Unity Editor has configured the project successfully, you are ready to go
    - If you want to run the test or demo scene, double click on the respective .unity file in the directory unity/Assets/Scenes/

## Working on the project

### Local project files that are ignored by the repository
The following files and directories are only relevant for local development and must thus be ignored by any pushes to the repo:
- Library/
- Logs/
- Temp/
- UserSettings/
- obj/
- *.sln
- *.csproj

### How to use the code?
The following is a brief description of the classes that you will need to make use of as a user of the MACS.

#### Agent
This singleton class represents the unityagent. It establishes a connection with the rosrouter, configures this unityagent, and handles message sending and reception. Hence, it serves as the starting point for using the MACS and connects your application to the MAN.

To utilize the Agent class, instantiate it and call its Run method before any communication with other agents on the MAN. An approach showcased in the demo applications is to create and launch the Agent in the Awake or Start method. In doing so, the unityagent is connected to the MAN before the initial frame rendering in play mode. Additionally, the Run method enables a connection to a rosrouter on a designated IP address and port. It also allows the unityagent to be assigned a group id, if required.

```
Agent agent = Agent.Instance;
agent.Run();
```

#### RosAgentInterfaceNodesRegistryManager
This class manages registration and deregistration of rosagent interface nodes (publishers, subscribers, services, clients) to permit rosagents communication with unityagents.

The code snippet below demonstrates registering a rosagent interface subscriber node. The process is analogous for the other three types of rosagent interface nodes.

```
registryManager = RosAgentInterfaceNodesRegistryManager.Instance;
subscriberNode = registryManager.RegisterInterfaceSubscriberNode<RosColor>(ChangeCubeColor, topicName, agentIds);
```

#### RosAgentInterfaceNode
This class represents the base class of the 4 different rosagent interface node types. As this class is abstract, the interaction is done with its 4 subclasses representing the rosagent interface node types. The base functionality, e.g. registration of interface nodes on rosagents, is implemented in this class.

The following classes represent the rosagent interface nodes. For each node type exists one C# class.

#### RosAgentInterfacePublisherNode
Enables publication of ROS messages to rosagents from the unityagent.

#### RosAgentInterfaceSubscriberNode
Enables subscription to ROS messages that are published on rosagents.

#### RosAgentInterfaceServiceNode
Enables rosagents to make requests to service functionality on unityagents (a rosagent calls a service running on a unityagent).

#### RosAgentInterfaceClientNode
Enables unityagents to make requests to ROS services running on rosagents (a unityagent calls a service running on a rosagent).

## Demos and Tests
The project includes two Unity scenes that are available for running tests and demos.

### BasicTestScene
Tests the 4 different types of communication between unity- and rosagents. This scene is also used to get a better understanding of how unityagents and rosagents communicate with each other. Credit for the idea of these 4 basic tests goes to Unity. Their demo for testing network communication between the "ROS TCP Connector" and "ROS TCP Endpoint" applications works similarly (https://github.com/Unity-Technologies/Unity-Robotics-Hub/tree/main).

### DemoScene
This scene was developed specifically as a demo application to demonstrate the multi-agent functionality of the application. It also demonstrates how the multi-agent functionality of the MACS can be leveraged by applications built on top of it.

### Running the BasicTestScene
Before a test can be started in the Unity Editor, a rosagent and a rosrouter instance must be running. Follow the instructions in the README.md file in the *ros* directory of this repo on how to build and install the ROS2 workspace on your machine. Once the colcon workspace has been built and the ROS2 packages are installed, three terminals need to be opened. One terminal is used to run the rosrouter, another to run the rosagent, and the last terminal allows you to enter test commands.
- 1<sup>st</sup> terminal runs the rosrouter
  ```
  ros2 run rosrouter rosrouter_node
  ```
- 2<sup>nd</sup> terminal runs the rosagent
  ```
  ros2 run rosgent rosagent_node
  ```
- 3<sup>rd</sup> terminal runs commands for the respective test

#### The tests
Five different tests can be performed. Each of them must be run in isolation, i.e. no two or more tests can be run simultaneously. To run a test, activate the corresponding game object in the Unity BasicTestScene and click the play button (as mentioned above, the rosrouter and rosagent instances must already be running).
- **TestRosPublisher**: The unityagent sends pose data of the game object cube in the Unity scene on a continuous basis to the rosagent. To make the data visible on the rosagent, use the 3<sup>rd</sup> terminal you opened up and type in the following ROS2 command.
  ```
  ros2 topic echo pos_rot
  ```
- **TestRosSubscriber**: Change the color of the cube located in the Unity scene by issuing the following command from the 3<sup>rd</sup> terminal. This command sends a color message from the rosagent to the unityagent. A new color is randomly chosen on every run.
  ```
  ros2 run unity_robotics_demo color_publisher
  ```
- **TestRosService**: Throughout the test, the cube is moved around the Unity scene until it reaches a random destination. Once the target is reached, a service call is sent to the rosagent requesting a new target position. Start the service running on the rosagent using the command below (Note: this service must be started before the game is started in Unity).
  ```
  ros2 run unity_robotics_demo position_service
  ```
- **TestUnityService**: This test allows the rosagent to make requests to the unityagent. The command below sends a request to the unityagent, which responds with a pose message that is printed to the caller terminal.
  ```
  ros2 service call obj_pose_srv unity_robotics_demo_msgs/ObjectPoseService "{object_name: Cube}"
  ```
- **TestVideostreamSubscriber**: The underlying rosagent functionality (susbscriber interface node) for this test is the same as for "TestRosSubscriber." It was written to test the random image subscription for the XR headset demo (see below). To send randomly generated images from the rosagent to the unityagent, use the following command.
  ```
  ros2 run demo image_publisher
  ```

### Running the DemoScene on an XR headset
To run this demo, you will need to have an XR headset at hand and have your computer stream the Unity Editor scene to your headset.

Before starting the demo in the Unity Editor, the ROS side needs to be up and running to allow the unityagent to connect to the rosrouter. Docker-compose files are provided in subdirectories of the ros directory of this repo to pull up all the necessary instances needed for the demo on the ROS side. The fleet of docker containers created includes a rosrouter instance alon with one or more rosagent instances.

To pull up the docker container fleet, navigate to "*root/ros/docker/test/*" in this repo and execute one of the following commands:
- Single rosagent instance demo
  ```
  docker compose -f docker-compose.single-agent-test.yml up
  ```
- Multiple rosagent instances demo
  ```
  docker compose -f docker-compose.multi-agent-test.yml up
  ```

By reading the logs, you can easily identify the ongoing operations of the system. If there are no errors logged and the printing of logs has ceased, the docker container fleet has been created successfully. You may now proceed to click on "play" in the Unity Editor.

### DemoScene walk-through
Once the data stream between Unity and your headset is successfully set up, you can begin exploring the demo scene. To inspect the actions carried out by the unityagent, you may refer to the logs printed on the Unity Editor terminal at any given moment.

The DemoScene presents a connection menu, situated directly in front of you upon start-up. Simply use your headset controllers to click on the connect button and connect the unityagent to the rosrouter. A fresh menu will appear, displaying all connected rosagents and unityagents. To initiate communication with the rosagents online on the MAN, click on the Start button. One menu can be opened for each connected rosagent. This menu enables you to stream image data from a chosen rosagent to your headset. Moreover, you can publish the pose data of a cube positioned in front of you on the ground to the connected rosagents (modify its pose by grabbing the cube with one of the two controllers). It is essential to bear in mind that only one image publisher and one pose subscriber are functional on the ROS side. Every rosagent streams an identical image to the Unity scene and logs matching pose data to the terminal running the docker container fleet. To cease data transfer between the unityagent and a particular ROS agent, click the Stop button on the connected agents menu. To halt the entire demonstration, click the Disconnect button. Alternatively, you can terminate the demo from the Unity Editor by clicking the Stop game button.