<!-- ---------------------------------------------------------------------- -->

---
# THE SUPER HYPER POKE DIGI MEGA MUCHI-FANTASTIC FINAL PROJECT (2D-BASIC-PLANNER)
NAME: **WRITE YOUR NAME HERE**

---
**INSTRUCTIONS**:
1. Fork this repository, and associate it with your GitHub account.
2. Create a new branch name as `develop` and work all your solutions there.
3. Every time that you complete a basic point create a commit, add the files and push with the next format description: `[FEAT]: description of what you did.
4. You can do your last commit and push until and before the deadline (date & time) if you do more after, we will make a reverse of your code until the last commit before the dead-line date&time (If the project is not complete there, it'll be rejected).
5. Remember partial solutions will be no accepted, you have to complete at least **BASIC POINTS**, **QUESTIONARY**, but we consider the real project the **EXTRA-HOMEWORK** session (Without it your chances will be less).

--------
**BASIC POINTS**:

Here are the basic points to accomplish the project, what we are evaluating you is the knowledge in python, C++,  ros2 in a basic level of concepts and implementation, and also a speed and turn profile implementation. This part of the project is more for the understanding of itself because we invite you to make the extra-homework, which is the real challenge, where you'll have to be more creative and go further. 

Well, the recommendation in this section is to start with the python nodes or points and then go with the C++ ones. You won't have to change the input or output attributes in any function or method, just implement functions contents, and operations, import a library and use it inside. 

Try first to understand the function of the node in the whole stack, then what is every function for, read the documentation and docs provided, and look for the TODO sections in these.

if you follow the instruction to launch all the stack you'll get a window like this:

<p align="center">
  <img height="300" src="https://user-images.githubusercontent.com/43115782/116264723-27a80380-a740-11eb-8e03-62039517b82a.png">
</p>

running the stack from the root, remember the command is:

        ada@vision1050:/workspace$ bash planner/configs/startPlanner.sh start

if you can not get this window it could be that the code your are making is broken, you are having packages building errors, or having the xhost display error. But before starting to code you solution make sure that you have this window.

### *Python*

if you complete python points you'll have the robot moving in the map without sound.

1. **Implement/Create the path planner status subscriber** - package: [`graphics`](https://github.com/kiwicampus/2D-Test-Track-Planner/tree/main/planner/ros2/src/graphics), Node: [`node_visual_gui.py`](https://github.com/kiwicampus/2D-Test-Track-Planner/tree/main/planner/ros2/src/graphics/graphics/node_visual_gui.py) -> Implement/Create in the node constructor a subscriber for the bot status, the topic name should be `/path_planner/msg` message type `planner_msg`, callback `cb_path_planner`.

2. **Implement/Create the Kiwibot status subscriber** - package: [`graphics`](https://github.com/kiwicampus/2D-Test-Track-Planner/tree/main/planner/ros2/src/graphics), Node: [`node_visual_gui.py`](https://github.com/kiwicampus/2D-Test-Track-Planner/tree/main/planner/ros2/src/graphics/graphics/node_visual_gui.py) -> Implement/Create in the node constructor a subscriber for the bot status, the topic name should be `/kiwibot/status` message type `kiwibot_msg`, callback `cb_kiwibot_status`.

3. **Drawing map descriptors** - package: [`graphics`](https://github.com/kiwicampus/2D-Test-Track-Planner/tree/main/tree/main/planner/ros2/src/graphics), Node: [`node_visual_gui.py`](https://github.com/kiwicampus/2D-Test-Track-Planner/tree/main/planner/tree/main/ros2/src/graphics/graphics/node_visual_gui.py) -> Implement the function `draw_descriptors` to draw every landmark in the map when a routine is loaded, please base on the red circles in the video example.**Note**: Do not add extra attributes o return.

4. **Draw the robot** - package: [`graphics`](https://github.com/kiwicampus/2D-Test-Track-Planner/tree/main/tree/main/planner/ros2/src/graphics), Node: [`node_visual_gui.py`](https://github.com/kiwicampus/2D-Test-Track-Planner/tree/main/tree/main/planner/ros2/src/graphics/graphics/node_visual_gui.py) -> Implement the function `draw_robot` to draw the robot over the map, you can use the function `overlay_image` located in the utils module in the `python_utils.py` file.**Note**: Do not add extra attributes o return.

5. **Implement/Trapezoidal speed profile**- package: [`graphics`](https://github.com/kiwicampus/2D-Test-Track-Planner/tree/main/planner/ros2/src/path_planner), Node: [`node_planner.py`](https://github.com/kiwicampus/2D-Test-Track-Planner/tree/main/planner/ros2/src/path_planner/path_planner/node_planner.py) -> Implement the function `get_profile_route` to get a list of waypoints for a trapezoidal speed profile to get the points and the times where and when the robot should be. you can not add more input or output arguments to the function. you can follow the method and operations explained in this link: https://www.linearmotiontips.com/how-to-generate-motion-profile-for-linear-system/ . And read carefully the instructions and the documentation in the function.

6. **Implement/Trapezoidal turn profile**- package: [`graphics`](https://github.com/kiwicampus/2D-Test-Track-Planner/tree/main/planner/ros2/src/path_planner), Node: [`node_planner.py`](https://github.com/kiwicampus/2D-Test-Track-Planner/tree/main/planner/ros2/src/path_planner/path_planner/node_planner.py) -> Implement the function `get_profile_turn` to get a list of waypoints for a trapezoidal turn profile to get the points and the times where and when the robot should be pointing. you can not add more input or output arguments to the function. you can follow the method and operations explained in this link: https://www.linearmotiontips.com/how-to-generate-motion-profile-for-linear-system/ . And read carefully the instructions and the documentation in the function.


### *C++* 

First of all, you need to modify `planner/configs/startPlanner.sh` and `planner/configs/nodes_launch.yaml` files to build the Interfaces node. 

        1. Look for the line:  
        ```.bash
        colcon build --symlink-install --packages-skip interfaces
        ```

        2. Change it for:  
        ```.bash
        colcon build --symlink-install
        ```

        3. Identify the `NODE_INTERFACES` object: 
        ```.yaml
        NODE_INTERFACES:
        launch: 0
        node_executable: interfaces_node
        output: screen
        package: interfaces
        ```

        4. Change launch parameter: 
        ```.yaml
        NODE_INTERFACES:
        launch: 1
        node_executable: interfaces_node
        output: screen
        package: interfaces
        ```

Don't worry this will fail, but that is part of the below exercises.

1.  **Create a subscriber inside the speaker node** - package: [`interfaces`](https://github.com/kiwicampus/2D-Test-Track-Planner/tree/main/planner/ros2/src/interfaces) file to modify: `interfaces/src/modules/speaker.cpp` -> Implement a subscriber `Amazing Speaker Subscriber` type: `std_msgs::msg::Int8` related to the CallBack Function `speakerCb`, the publisher come from `node_planner` and publish a Int8 message to indicated which track will play. **Note:** Use the `m_speaker_sub` defined in the .hpp file to create your subscriber.

2. **Create a publisher inside the speaker node** - package: [`interfaces`](https://github.com/kiwicampus/2D-Test-Track-Planner/tree/main/planner/ros2/src/interfaces) file to modify: `interfaces/src/modules/speaker.hpp` -> Define a publisher `Amazing Speaker Publisher` type: `std_msgs::msg::Bool` call it ```m_done_pub```. 

3. **Publish a Bool message using the previous publisher created** - package: [`interfaces`](https://github.com/kiwicampus/2D-Test-Track-Planner/tree/main/planner/ros2/src/interfaces) file to modify: `interfaces/src/modules/speaker.cpp` -> Use the documentation provided and your logic to discover how to publish a UniquePtr message in a simple publisher, which publish a True value each time a sound is ended and False each time a sound begin. Use the publisher called `m_done_pub`.

4. **Create a condition when a file isn't found** - package: [`interfaces`](https://github.com/kiwicampus/2D-Test-Track-Planner/tree/main/planner/ros2/src/interfaces) file to modify: `interfaces/src/modules/speaker.cpp` -> When a sound track isn't found, play the default sound called `track2` located in the same folder as the another ones. You could test this point using `ros2 topic pub -1 /device/speaker/command std_msgs/Int8 "data: 2"`.


If you finish successfully all the points, you must have a window like in this video: [solution of this amazing project](https://drive.google.com/file/d/19zo5GcFD73J9E9lJpVp6MlR2n9liaM2G/view?usp=sharing). Note: We had troubles capturing the audio, but if you listen carefully you can listen the audio effects.

 <p align="center">
     <img src="https://user-images.githubusercontent.com/43115782/114318886-99dbdf80-9ad4-11eb-947a-e7c6e417fec2.gif" alt="test_Track_map" width="300"/> 
</p>


---
**REMEMBER**: 
1. Once more again, No partial solution will be accepted, you have to complete all the basic points and the questionary to submit your solution to a review.
2. if you push just one minute after the time given by email we won't review your project solution (at least, that commit).
3. if you find an error in the code, bug, drawback, and there's no already an issue created in the main repository, please create it and you win extra points (+3%/5: **this is 3% more over 5.0, which is equal to 0.15 in the final project's grade**), we'll try to give a solution ASAP.
4. You have 3 questions that can be done as an issue in the main repository, so please check that what you are asking for is not already answered in the issues section (don't waste your questions).
5. It's forgiven talk with other project participants, if we think that you made tramp or you cheated, your project and the others (if apply) won't be reviewed and your application will be canceled.
6. We'll push the solution when the application job closes, we'll notify all participants of this by email.
7. What we are evaluating is:
    * **[5%/5]** Code style
    * **[5%/5]** Code Documentation
    * **[15%/5]** Questionary
    * **[25%/5]** Solution to basic home-work
    * **[50%/5]** Sustentation of your solution & answers
8. You can have a grade higher than 5.0.
9. We'll share the final grades with every candidate by email, with feedback included of his/her application (things to improve, to keep, to remove). The application process percentages are:
    * **[30%/5]** Cultural feed Interview
    * **[30%/5]** Pair programming Interview
    * **[40%/5]** Final Project
10. We'll select the 3 best candidates by their grades, and we'll make the final decision with the Ai&Team and administration, also they will get a call from our COO & CEO/Confounder.
11. We are evaluating your concepts and knowledge in ros2, Python, C++, Some basic control concepts, other Code stuff.
12. If no participant send the project before the dead line time, we will extend the deadline date and time, for everyone, with just person that complete the questionary and the basic-points the rest are doomed, but also if we considered that the solutions received are not enough or not well explained we also can extend the deadline, but we are pretty sure that this wont happened.

---
<!-- ---------------------------------------------------------------------- -->
## **QUESTIONARY**

Respond below every questions:

1. [Python] Why the robot's image gets distorted when is turning?

2. [Python] what is `ReentrantCallbackGroup()` in the python nodes with you own words, give an common life example, and what happen if is not used?

3. [Python] are Python packages compiled as C++ packages?

4. [Python] Why with some code errors in some nodes the logs are not printed?

5. [Control] What other turn or speed profile would you implement, why, and what are the benefits?

6. [C++] What is the mean of the number "15" used in the pthread_kill inside the destructor method?

7. [C++] Why are we using UniquePointer instead of SharedPointers to publish a ROS2 message?

8. [C++] Why are we using a m_multi_sound variable? Explain ...

9. [C++] Why are we freeing the memory allocated by raw pointer "buff" variable and not freeing the memory allocated by the Shared and Unique Pointers? (HARD)

10. [Docker] Explain with your own words what is the instructions `apt-get autoremove && apt-get clean -y for?`

11. [Docker] If you modify a layer what happen with the previous and the next ones?

12. [Docker] Can we change the basic image (`FROM ubuntu:20.04`) from the docker file to another?

Next questions is after you finish the project, it doesn't give points but we really appreciate you feedback:
* What do you think about this project? is it hard or enough? is it to complicated, is it well structure, explanations and instructions are clear?

---
<!-- ---------------------------------------------------------------------- -->
## **EXTRA-HOMEWORK**

For extra homework we recommend you create a new branch from the developed one when you finish the basic points of the homework, you can name this new branch as *feature/extra_homework*, don't forget to push it before the time that it was given.

1. **[+5%/5.0]**: Modify the docker file to source ros2 and have autocompleted commands like ```ros2 topic list```.
2. **[+5%/5.0]**: Make a video recorder, in that way if you start a routine a video file of it should be created as `routine_id_date.mp4` you can not overwrite if the file or video exits (at least it's corrupted or incomplete because the process was interrupted)
3. **[+10%/5.0]**: Implement a method or way to cancel routines and start another one, the could remain in the actual position or return to the origin, or to the closes landmark (with a key).
4. **[+15%/5.0]**: Implement a method or way to stop/pause the routine and then continue (with a key).
5. **[+15%/5.0]**: From point 3 start again the music.
5. **[+15%/5.0]**: From point 4 stop the music and then resume.
6. **[+5%/5.0]**: Make that the Kiwibot image doesn't get distorted when is turning.
7. **[+5%/5.0]**: Make that Kiwibot track2.wav don't get distorted.

Total possible Extra points: 100% -> 5.0. Maximum total grade: 10.0/5.0. Complete the point it doesn't mean you have 5.0, you have at least 3.0 but for the rest of the grade will evaluate the performance and the beauty of your solution. To complete these points, probably you will have to modify messages, services, or even create new ones, also topics subscribers and publishers, maybe services, who knows :)

---
Good luck, and God saves the Queen.

<p align="center">
  <img height="300" src="https://media.tenor.com/images/18a922837758f4d77b983dfa1f7acff2/tenor.gif">
</p>


*Davidson DO NOT forget erase the [solution branch](https://www.youtube.com/watch?v=dQw4w9WgXcQ)*