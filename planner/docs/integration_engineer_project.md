
---
# **2D-Planner-Structure**

Welcome again human, if you are here, we want to extend our greetings and congratulations for arriving here, you are awesome, your parents should be proud. Well, now we gonna explain what's the project about, and keep in mind that even if you are not the chosen one we guarantee that you will learn something useful for your professional/academic/personal life, so, you are not wasting your time, and we know that you'll give your best, competency is hard so better you do it.

First we'll talk about the project structure, then & 2nd about the challenge, 3rd the rules, 4th the extras.

---
## The Planner:

What you have here is a very basic 2D planner or it'll be if you finish all, we'll send to everyone the solution later, but without the extras (No worries for this yet). 

The [default map](https://github.com/kiwicampus/2D-Test-Track-Planner/planner/media/images/map.jpg) used is a replica of our Kiwibot test-track which is located in Medellin Colombia. Here are some pictures: 

 <p align="center">
    <img src="https://user-images.githubusercontent.com/43115782/114318930-c859ba80-9ad4-11eb-8c13-c6a0c6580bd1.jpeg" width="200">
    <img src="https://user-images.githubusercontent.com/43115782/114318946-d3ace600-9ad4-11eb-9184-0244b8ecbc4d.jpeg" width="200">
    <img src="https://user-images.githubusercontent.com/43115782/114318958-e45d5c00-9ad4-11eb-9307-25eb2cc18d14.jpeg" width="200">
</p>
  
In that map, we have defined landmarks as is shown in the picture below:

 <p align="center">
    <img src="https://user-images.githubusercontent.com/43115782/114318903-ac561900-9ad4-11eb-90d4-804f2a8331ba.jpg" alt="test_Track_map" width="600"/> 
</p>
  
These landmarks are defined in the images space [coordinates in pixels] with some descriptors and connected landmarks (neighbors). The landmarks are specified in the file ```key_points.csv```. If you like you can change the map just replacing the image and the land-marks descriptors.


| Origen | CoordOriX | CoordOriY | Destination | CoordDestX | CoordDestY |
| ------ | --------- | --------- | ----------- | ---------- | ---------- |
| Keypoint Number | (int) Coordinate X origin |(int) Coordinate Y origin |key_point destiny | (int) Coordinate X Destination | (int) Coordinate Y Destination |


If you want add a new landmark you must define the origin coordinates and the destination coordinates (X, Y) for each segment in the map, these coordinates are the number of pixels in the X axis and Y axis in the image. You need to define both ways for a segment, this means that if you define the start point from '1' to '2', it should be defined from '2' to '1' as well, otherwise the robot wont go from the unspecified coordinate, and Yes! We know there are more ways, but the implementation of this project is for academical reasons.

* **Origen**: Origin Landmark ID
* **CoordOriX**: Origin X axis coordinate
* **CoordOriY**: Origin Y axis coordinate
* **Destination**: Destination Landmark ID
* **CoordDestX**: Destination X axis coordinate
* **CoordDestY**: Destination Y axis coordinate

Additionally there's more information per segment in the csv file:

| Difficulty | Code | Description | Distance | Time |
| ---------- | ---- | ----------- | -------- | ---- |
| Segment Difficulty | Segment environment conditions code | Description of segment | Distance from origin to destination | Time to go from origin to destination |

* **Difficulty**: Number between 0 and 5 to stablish the difficulty of segment, this is a subjective value calculated by the user 
* **Code**: Code of environment conditions in the segment. One segment could be defined with different environment codes, example for rain, obstacles to avid, etc.
* **Description**: Description of segment, and describe what the current environment code is.
* **Distance**: Distance from origin to destination, but default this value is in meters
* **Time**: Time to go from origin to destination, but default this value is in seconds


---
## Defining Your Own Path

There're some pre-defined routines in  ```routines.yaml```, here you'll find 9 routines. Feel free to test everyone of them and change them. You have to be sure if the segment that you are typing are defined in the configuration csv file ```key_points.csv```.


For the ```path 1``` the robot will start in the Landmark `3` with the code `0` that is the `(3, 0)` in `key_points` variable. then the robot will move to the Landmark `2` and it will have the code `0` again which for the default configuration means normal environment conditions, then the robot will keep this value and it'll move to `10`, `9`, `24`, and so on until arrive to `3`.


You have examples of the robot moving around the map in the links below:

[<img src="https://img.youtube.com/vi/bzAzzKxlSRw/0.jpg" width="200">](https://studio.youtube.com/video/bzAzzKxlSRw/edit)
[<img src="https://img.youtube.com/vi/bVdHfSWY358/0.jpg" width="200">](https://studio.youtube.com/video/bVdHfSWY358/edit)
[<img src="https://img.youtube.com/vi/Z4jQc3-psy8/0.jpg" width="200">](https://studio.youtube.com/video/Z4jQc3-psy8/edit)
[<img src="https://img.youtube.com/vi/UDy5WzK9USE/0.jpg" width="200">](https://studio.youtube.com/video/UDy5WzK9USE/edit)
[<img src="https://img.youtube.com/vi/u2UJjx-YPHs/0.jpg" width="200">](https://studio.youtube.com/video/u2UJjx-YPHs/edit)
[<img src="https://img.youtube.com/vi/nYw4YRsqFTw/0.jpg" width="200">](https://studio.youtube.com/video/nYw4YRsqFTw/edit)
[<img src="https://img.youtube.com/vi/57b9bYL-zuw/0.jpg" width="200">](https://studio.youtube.com/video/57b9bYL-zuw/edit)
[<img src="https://img.youtube.com/vi/sZH0ucbVEsk/0.jpg" width="200">](https://studio.youtube.com/video/sZH0ucbVEsk/edit)
[<img src="https://img.youtube.com/vi/AYpYbxiBs5k/0.jpg" width="200">](https://studio.youtube.com/video/AYpYbxiBs5k/edit)

---
## Nodes Compositions:

- **node_planner**: this node loads read the land-marks file, load the routines, and calculate the waypoint for every segment in the routine (Patch planning process), calculate the speed profile (spline or trapezoidal) and the turn profile.

- **node_kiwibot**: this node is a virtual state of the robot, get the path planning references (waypoints) and uploads its position and heading references, and current state.

- **node_visual_gui**: this node print and draws all visual components as the map, the robot, landmarks and waypoint, it gets everything from the topics of other nodes.

- **interfaces**: this node is to reproduce audio tracks publishing through a topic the number of the track.