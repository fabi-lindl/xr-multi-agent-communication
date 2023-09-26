# Unity xr-multi-agent-communication implementation
*Note: Before you go ahead and read this README.md file, you should have read the README.md file in the root directory of this repository. It serves as a basic project overview.*

## Abstract
The code stored in this directory of the repo represents the Unity side of the ROS2/Unity tandem project. As Unity is based on C#, this side of the project is implemented in C#. This application shoud serve as the basic building block for any further Unity applications that are intended to leverage multi-agent communication between Unity and ROS2 applications. Any details on how to use this application to enable your code with multi-agent communication capabilities can be found in this README.md file, as well as in the README.md file stored in the *ros* directory of this very same repo.

## Creating the project
To be able to create a Unity project from the code of this repo you must first:
- **Download and install the Unity Hub** (https://unity.com/download): The Unity Hub is a program that helps developers to effectively manage Unity projects. It allows to install different versions of Unity so that a specific version can be selected for working on any of your projects. Also, it provides easy install options if further modules are needed for a project later on during development time, which is not possible if a Unity Editor is installed via platform specific executables.
- **Install a Unity Editor**: The Unity Editor is software that allows you to create your Unity applications. After opening Unity Hub on your computer, click on "Installs" on the left menu bar and then click on "Install Editor." Install one of the LTS version that are recommended. If you decide to install an older version, visit Unity's download archive to find your desired version. It is vital to select the download for Unity Hub option on this page. Using platform dependent installer executables prevents any extension of the downloaded Unity Editor with additional modules after its installation.
- **Create a Unity project from the code in this repo**: *Try out how this can be done...*

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


## Demos and Tests
The project includes two Unity scenes that are available for running tests and demos.

### BasicTestScene
Tests the 4 different communication variants between unity- and rosagents. This scene also serves to get a better understanding of how unityagents and rosagents communicate with each other. Credit for the idea of these 4 basic tests is given to Unity. Their demo for testing network communication between the "ROS TCP Connector" and "ROS TCP Endpoint" applications works similarly (https://github.com/Unity-Technologies/Unity-Robotics-Hub/tree/main).

### DemoScene
This scene was specifically developed as a demo application to demonstrate the multi-agent functionality of the application. Moreover, it demonstrates how the multi-agent functionality of the MACS can be leveraged by applications that build on top of it.

### Running the BasicTestScene
Before a test can be started in the Unity Editor, a rosagent and a rosrouter instance must be running. Follow the instructions in the README.md file in the *ros* directory of this repo on how to build and install the ROS2 workspace on your computer. Once the colcon workspace is built and the ROS2 packages are installed, three terminals must be opened. One terminal is used to run the rosrouter, another one is used to run the rosagent, and the last terminal allows for entering test commands.
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
Five different tests can be run. Each of them must be run in isolation, i.e. no two or more tests can be executed simultaneously. To run a test, activate the concerend game object in the Unity BasicTestScene and hit the play button (as mentioned above, the rosrouter and rosagent instances must already be running).
- **TestRosPublisher**: The unityagent sends pose data of the game object cube in the Unity scene on a continuous basis to the rosagent. To make the data visible on the rosagent, use the 3<sup>rd</sup> terminal you opened up and type in the following ROS2 command.<br>
  ```
  ros2 topic echo pos_rot
  ```
- **TestRosSubscriber**: Change the color of the cube located in the Unity scene by executing the following command from the 3<sup>rd</sup> terminal. This command sends a color message from the rosagent to the unityagent. On every run, a new color is selected randomly.
  ```
  ros2 run unity_robotics_demo color_publisher
  ```
- **TestRosService**: Throughout this test, the cube in the Unity scene is constantly being moved around until it reaches a randomly set target destination. Once the target destination is reached, a service call is sent to the rosagent, requesting a new destination position. Start the service running on the rosagent via the command below (Note: This service must be started before starting the game in Unity).
  ```
  ros2 run unity_robotics_demo position_service
  ```
- **TestUnityService**: This test allows the rosagent to make requests to the unityagent. The command below sends a request to the unityagent, which responds with a pose message that is printed in the caller terminal.
  ```
  ros2 service call obj_pose_srv unity_robotics_demo_msgs/ObjectPoseService "{object_name: Cube}"
  ```
- **TestVideostreamSubscriber**: The underlying rosagent functionality (susbscriber interface node) for this test is the same as for "TestRosSubscriber." It was written to test the random image subscription for the actual XR headset demo (see below). To send randomly created images from the rosagent to the unityagent, use the following command.
  ```
  ros2 run demo image_publisher
  ```

### Running the DemoScene on an XR headset
To run this demo, you must have an XR headset at hand and make your computer stream the Unity Editor scene to your headset.

Before starting the demo in the Unity Editor, the ROS side must already be up and running to allow the unityagent to connect to the rosrouter. Docker-compose files are provided in subdirectories of the *ros* directory of this repo to pull up all necessary instances required for the demo on the ROS side. The fleet of pulled up docker containers includes a rosrouter instance alongside one or multiple rosagent instances.

To pull up the docker container fleet, navigate to "*root/ros/docker/test/*" in this repo and execute one of the following commands:
- Single rosagent instance demo
  ```
  docker compose -f docker-compose.single-agent-test.yml up
  ```
- Multiple rosagent instances demo
  ```
  docker compose -f docker-compose.multi-agent-test.yml up
  ```

In reading the logs, you should easily be able to identify the ongoings of the system. If no errors are logged and no more logs are printed, the docker container fleet was created successfully. You can now go ahead and click on play in the Unity Editor.

### DemoScene walk-through
Once you have successfully set up the data stream between Unity and your headset, you can start exploring the demo scene. You can check the logs printed to the Unity Editor terminal at any given moment to inspect the actions taken by the unityagent.

The DemoScene provides you with a connect menu that is located directly in front of you on start-up. Just click the connect button with the controllers of your headset to connect the unityagent to the rosrouter. A new menu with all connected rosagents and unityagents will open up. You can now start the communication with the rosagents online on the MAN simply by clicking the Start buttons. One menu can be opened for every connected rosagent. This menu allows you to stream random image data from a selected rosagent to your headset on the one hand. On the other hand, you can publish pose data of the cube that is located on the ground right in front of your feet to the connected rosagents (grab the cube with one of the two controllers to change its pose). It is important to note that there is only one image publisher and one pose subscriber running on the ROS side. Therefore, every rosagent streams the same image to the Unity scene and logs the same pose data to the terminal running the docker container fleet. To stop streaming or sending to a specific rosagent, click the Stop button on the connected agents menu in front of you. To stop the whole demo, click the Disconnect button. It is also possible to stop the demo from the Unity Editor by clicking the stop game button.