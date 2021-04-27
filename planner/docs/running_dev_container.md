<!-- ---------------------------------------------------------------------- -->
## **Running The Dev-Container**
 
If you have your [VSCode](https://code.visualstudio.com/) with the right extensions, and if you have Docker and Docker-compose installed in your system, when you open the project's main folder you'll see a window on the bottom right corner, click in "reopen in container" button, if you don't see anything press `Ctrl+Shift+P` and type `Remote-Containers: Rebuild and Reopen in container` or `Docker-images: Build Image` option. When the container is opened, and executed for the first time or when there are changes on it, you can go for a walk because the building image process will start and it'll take a while due to the installation of all packages and dependencies of the dev-environment as [ROS2](https://index.ros.org/doc/ros2/), [OpenCV](https://opencv.org/), [Python](https://www.python.org/), and more stuff related to, while the process is completed here are some videos of [puppies](https://www.youtube.com/watch?v=mRf3-JkwqfU). You can see at any time the logs of the building process clicking in `Starting with Dev Container` on the bottom right corner. When the process is done you'll see some messages of process succeed.
 
<img src="https://user-images.githubusercontent.com/43115782/87437367-d5806200-c5b3-11ea-9bf2-836e45f46ed8.gif" alt="building_dev-container" width="1200">
 
When the process is done you can open a terminal in the dev-container going to the menu bar `Terminal` and then `New Terminal`. Congratulations now you have everything that we use for our deployments.
 

<br />

<!-- ---------------------------------------------------------------------- -->
## **Architecture**
 
Find the distribution of final project in the next list:
 
- **[planner](https://github.com/kiwicampus/2D-Test-Track-Planner/tree/main/planner):** Main folder where most of the source code is located
  - **[configs](https://github.com/kiwicampus/2D-Test-Track-Planner/tree/main/planner/configs):** Path planner config files
     - [*startPlanner.sh:*](https://github.com/kiwicampus/2D-Test-Track-Planner/tree/main/planner/configs/startPlanner.sh) bash script to run stack of the project
     - [*env_vars.sh:*](https://github.com/kiwicampus/2D-Test-Track-Planner/blob/main/planner/configs/env_vars.sh) local environment variables
     - [*nodes_launch.yaml:*](https://github.com/kiwicampus/2D-Test-Track-Planner/tree/main/planner/configs/nodes_launch.yaml) file describing which nodes launch or not
     - [*key_points.csv:*](https://github.com/kiwicampus/2D-Test-Track-Planner/tree/main/planner/configs/key_points.csv) file describing the keypoints map
     - [*routines.yaml:*](https://github.com/kiwicampus/2D-Test-Track-Planner/tree/main/planner/configs/routines.yaml) file describing the routines that can be executed
  - **[ros2/src](https://github.com/kiwicampus/2D-Test-Track-Planner/tree/main/planner/ros2/src):** Development workspace & ROS 2 packages
  - **[media](https://github.com/kiwicampus/2D-Test-Track-Planner/tree/main/planner/media):** Media files for project such images, and audios 

Only some files are listed, and explained (most important).
 
<br />
 

<!-- ---------------------------------------------------------------------- -->
## **Running The Project Stack**
 
Find a brief explanation on how to run our stack in your IDE and the explanation of the launch file, and config files as the key to managing which nodes are going to be launched and how they're going to work.
 
In order to launch locally (Inside your IDE), please locate into the `configs/` folder in the dev-container terminal and execute the following prompt command:
 
     $ bash startPlanner.sh start

*Note:* if you've already launch and compile the whole stack and there's no a *hot* modification inside the stack, it's possible to avoid the entire compiling step running:

     $ bash startPlanner.sh start no-build

This bash performs the following steps to launch the *Planner* stack:
 
1. Sources the [`env_vars.sh`](https://github.com/kiwicampus/2D-Test-Track-Planner/blob/main/planner/configs/env_vars.sh) which contains the *Kiwibot* local environment variables.
2. Launch ros2 with the specified node in [*nodes_launch.yaml:*](https://github.com/kiwicampus/2D-Test-Track-Planner/tree/main/planner/configs/nodes_launch.yaml)

For ROS 2 development workspace
 
1. Sources ROS Foxy and clean the older development workspace (If enabled).
2. Builds the development workspace at [`planner/ros2/`](planner/ros2)
3. Sources the resulting setup in the install folder `. install/setup.bash`
4. Executes [`ros2 launch /configs/planner.launch.py`](https://github.com/kiwicampus/2D-Test-Track-Planner/blob/main/planner/configs/planner.launch.py)
 
You can compile and launch everything by your own if you already have a background and experience with ROS/ROS2, but for those who want everything easy, and fast the bash script will set up and run everything for you. With the [``startPlanner.sh``](https://github.com/kiwicampus/2D-Test-Track-Planner/blob/main/planner/configs/startPlanner.sh) bash script you can run the stack of the project, this file has all instruction to download third-party packages, other required dependencies if they're missing, and setup, source, and run the ros2 workspace, launching the nodes specified in the file ``nodes_launch.yaml`` (File created when you start or run the script for the first time).
 

When the script finishes the building, and the compilation process you'll see the planner window with default configs as shown next (in the SOLUTION VERSION):
 
 <p align="center">
     <img src="https://user-images.githubusercontent.com/43115782/114318886-99dbdf80-9ad4-11eb-947a-e7c6e417fec2.gif" alt="test_Track_map" width="400"/> 
</p>
  
You can press the keys-numbers to start a routine, to run a new one just close and reopen the program again, o just wait until the routine is finished.

*Note (Window is not displaying):* if you are having troubles or errors getting the user interface window, read about [Docker image with OpenCV with X11 forwarding for GUI](https://marcosnietoblog.wordpress.com/2017/04/30/docker-image-with-opencv-with-x11-forwarding-for-gui/) for explanations, and then run the  [``startXHost.sh``](planner/configs/startXHost.sh) script in the host terminal, or just run the prompt command (Do not this in the dev-container terminal):
 
     $ xhost +

the error is something like this: 

     [graphics-3] (planner_window:12789): Gdk-ERROR **: 20:17:14.340: The program 'planner_window' received an X Window System error.
     [graphics-3] This probably reflects a bug in the program.
     [graphics-3] The error was 'BadAccess (attempt to access private resource denied)'.
     [graphics-3]   (Details: serial 277 error_code 10 request_code 130 (MIT-SHM) minor_code 1)
     [graphics-3]   (Note to programmers: normally, X errors are reported asynchronously;
     [graphics-3]    that is, you will receive the error a while after causing it.
     [graphics-3]    To debug your program, run it with the GDK_SYNCHRONIZE environment
     [graphics-3]    variable to change this behavior. You can then get a meaningful
     [graphics-3]    backtrace from your debugger if you break on the gdk_x_error() function.)
     [ERROR] [graphics-3]: process has died [pid 12789, exit code -5, cmd '/workspace/planner/ros2/install/graphics/lib/graphics/graphics --ros-args'].

If the error remains, run the bash file until the window is shown, it could take even 10 times.

*Note (Audio is not reproducing):* if you are having troubles getting audio from the virtual environment, please create a issue with the error description. But also check the audio device is not busy, or try connecting adn disconnecting headsets in the audio port.

*Note (Launching specific nodes):* if you want to launch just a single node or some of them or the whole stack, go to the file ``nodes_launch.yaml`` and change the key *launch* for 1 to launch and 0 to don't launch. this could be useful when you are testing some nodes or just a single node.


<br />
