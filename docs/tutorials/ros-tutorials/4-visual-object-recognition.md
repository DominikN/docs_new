---
title: Visual object recognition
sidebar_label: 4. Visual object recognition
id: 4-visual-object-recognition
---

> You can run this tutorial on:
>
> - [ROSbot 2.0](https://store.husarion.com/products/rosbot)
> - [ROSbot 2.0 PRO](https://store.husarion.com/collections/dev-kits/products/rosbot-pro)
> - [ROSbot 2.0 simulation model (Gazebo)](https://github.com/husarion/rosbot_description)

We have prepared ready to go virtual environment with end effect of following this tutorial. It is available on ROSDS:

<div><center>
<a href="http://www.rosject.io/l/babdec5/">
<img alt="run-on-ROSDS" src="/img/ros/Run-on-ROSDS-button.png" width="250px"/></a>
</center></div>

## Introduction

Objects can be recognized by a robot with use of a vision system. It is
based on image characteristics like points, lines, edges colours and
their relative positions.

Processing of object recognition consists of two steps. First is
teaching and should be executed before main robot operation. During this
step object is presented to the vision system, image and extracted set of
features are saved as a pattern. Many objects can be presented to the
system.

Second step is actual recognition which is executed constantly during
robot operation. Every frame of camera is processed, image features are
extracted and compared to data set in the memory. If enough features matches
the pattern, then the object is recognized.

In this tutorial we will use `find_object_2d` node from `find_object_2d`
package for both teaching and recognition.

As an image source we will use nodes from `astra.launch` as in tutorial 1.

## Teaching objects

Anything could be an object to recognize, but remember, that the more edges
and contrast colours it has, the easier it will be recognized. A piece
of paper with something drawn on it would be enough for this tutorial.

First you should run `find_object_2d` and `astra.launch`. Node
`find_object_2d` by default subscribe to `image` topic, you should remap
it to topic `/camera/rgb/image_raw`.

You can use below `launch` file:

```xml
<launch>

    <arg name="rosbot_pro" default="false" />
    <arg name="use_gazebo" default="false" />

    <arg name="teach" default="true"/>
    <arg name="recognize" default="false"/>

    <arg if="$(arg teach)" name="chosen_world" value="rosbot_world_teaching"/>
    <arg if="$(arg recognize)" name="chosen_world" value="rosbot_world_recognition"/>

    <!-- Gazebo -->
    <group if="$(arg use_gazebo)">
        <include file="$(find rosbot_gazebo)/launch/$(arg chosen_world).launch"/>
        <include file="$(find rosbot_description)/launch/rosbot_gazebo.launch"/>
            <param name="use_sim_time" value="true" />
    </group>

    <!-- ROSbot 2.0 -->
    <group unless="$(arg use_gazebo)">
        <include file="$(find rosbot_ekf)/launch/all.launch">
            <arg name="rosbot_pro" value="$(arg rosbot_pro)" />
        </include>

        <include unless="$(arg use_gazebo)" file="$(find astra_launch)/launch/astra.launch"/>
    </group>

    <node unless="$(arg use_gazebo)" pkg="tf" type="static_transform_publisher" name="camera_publisher" args="-0.03 0 0.18 0 0 0 base_link camera_link 100" />

    <node name="teleop_twist_keyboard" pkg="teleop_twist_keyboard" type="teleop_twist_keyboard.py" output="screen"/>

    <node pkg="find_object_2d" type="find_object_2d" name="find_object_2d">
        <remap from="image" to="/camera/rgb/image_raw"/>
        <param name="gui" value="$(arg teach)"/>
        <param if="$(arg recognize)" name="objects_path" value="$(find tutorial_pkg)/image_rec/"/>
    </node>

    <node pkg="tutorial_pkg" type="action_controller_node" name="action_controller" output="screen"/>

</launch>
```

To start teaching objects on ROSbot:

```bash
roslaunch tutorial_pkg tutorial_4.launch use_gazebo:=false teach:=true recognize:=false
```

To start teaching objects in Gazebo:

```bash
roslaunch tutorial_pkg tutorial_4.launch use_gazebo:=true teach:=true recognize:=false
```

To place objects in front of camera using Gazebo, you can use buttons **Translation mode** and **Rotation mode** in top left corner and drag objects to desired position. Another option is to drive ROSbot to look at the selected object.

After launching the `find_object_2d` node with properly adjusted image
topic, new window should appear:

![image](/img/ros/man_4_1.jpg)

On the left side of the window there are thumbnails of saved images
(should be empty at first run). Application main window contains camera
view. Yellow circles on the image are marking extracted image features.

To begin teaching process choose from the main toolbar **`Edit`**
→ **`Add object from scene...`.** New window will appear:

![image](/img/ros/man_4_2.jpg)

Now move the object and camera in order to cover as many features of
the object as possible. While doing that try not to catch object surroundings. When it’s
done, click **`Take picture`**.

![image](/img/ros/man_4_3.jpg)

In next view click **`Select region`** and select only that part of
taken picture, that covers desired object and click **`Next`**.

![image](/img/ros/man_4_4.jpg)

You will get confirmation of features extracted from selected image
area. If presented image is in accordance with what you selected,
click **`End`**

![image](/img/ros/man_4_5.jpg)

You should see new thumbnail in the left panel. Notice the number outside of
parentheses on the left of the image, this is the object ID.

Now you can add some more objects to be recognized. Remember their IDs, you
will need them later:

![image](/img/ros/man_4_6.jpg)

![image](/img/ros/man_4_7.jpg)

When you have enough objects in the database choose from the main toolbar
**`File`** → **`Save objects...`** and choose a folder to
store recognized objects. Close the window and stop all running nodes.

## Recognizing objects

Objects will be recognized by the same node which was used for teaching but
it works in slightly different configuration. We will set two new parameters
for the node. For parameter `gui` we will set value `false`, this will run
node without window for learning objects as we no longer need it.
Another parameter will be `objects_path`, this should be a path to a folder
that you have just chosen to store recognized objects.

You can use the same launch file as for teaching, but with different parameter values.

On ROSbot:

```bash
roslaunch tutorial_pkg tutorial_4.launch use_gazebo:=false teach:=false recognize:=true
```

In Gazebo:

```bash
roslaunch tutorial_pkg tutorial_4.launch use_gazebo:=true teach:=false recognize:=true
```

Node is publishing to `/objects` topic with message type
`std_msgs/Float32MultiArray`. Data in this message is covering: object
ID, size of recognized object and its orientation. The most interesting
for us is object ID, this is first element of array.

Whenever object is recognized, it will be published in that topic. Place
different objects in front of camera and observe as their data is
published in the topic.

To watch messages published in the topic, you can use `rostopic` tool:

```bash
rostopic echo /objects
```

## Making decision with recognized object

To perform a robot action based on recognized object, we will make a new
node. It’s task will be to subscribe `/objects` topic and publish
message to `/cmd_vel` with speed and direction depending on the object.

Create a new file, name it `action_controller.cpp` and place it in `src`
folder under `tutorial_pkg`. Then open it in text editor and paste below code:

```cpp
#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Twist.h>

#define SMILE 4
#define ARROW_LEFT 3
#define ARROW_UP 5
#define ARROW_DOWN 6

int id = 0;
ros::Publisher action_pub;
geometry_msgs::Twist set_vel;

void objectCallback(const std_msgs::Float32MultiArrayPtr &object)
{
   if (object->data.size() > 0)
   {
      id = object->data[0];

      switch (id)
      {
      case ARROW_LEFT:
         set_vel.linear.x = 0;
         set_vel.angular.z = 1;
         break;
      case ARROW_UP:
         set_vel.linear.x = 1;
         set_vel.angular.z = 0;
         break;
      case ARROW_DOWN:
         set_vel.linear.x = -1;
         set_vel.angular.z = 0;
         break;
      default: // other object
         set_vel.linear.x = 0;
         set_vel.angular.z = 0;
      }
      action_pub.publish(set_vel);
   }
   else
   {
      // No object detected
      set_vel.linear.x = 0;
      set_vel.angular.z = 0;
      action_pub.publish(set_vel);
   }
}

int main(int argc, char **argv)
{

   ros::init(argc, argv, "action_controller");
   ros::NodeHandle n("~");
   ros::Rate loop_rate(50);
   ros::Subscriber sub = n.subscribe("/objects", 1, objectCallback);
   action_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
   set_vel.linear.x = 0;
   set_vel.linear.y = 0;
   set_vel.linear.z = 0;
   set_vel.angular.x = 0;
   set_vel.angular.y = 0;
   set_vel.angular.z = 0;
   while (ros::ok())
   {
      ros::spinOnce();
      loop_rate.sleep();
   }
}
```

Below is an explanation of the code line by line.

Including required headers:

```cpp
#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Twist.h>
```

Defining constants for recognized objects, adjusting values to IDs of objects
recognized by your system:

```cpp
#define SMILE 8
#define ARROW_LEFT 9
#define ARROW_UP 10
#define ARROW_DOWN 11
```

Integer for storing object identifier:

```cpp
int id = 0;
```

Publisher for velocity commands:

```cpp
ros::Publisher action_pub;
```

Velocity command message:

```cpp
geometry_msgs::Twist set_vel;
```

Callback function for handling incoming messages with recognized objects
data:

```cpp
void objectCallback(const std_msgs::Float32MultiArrayPtr &object) {
```

Checking if size of `data` field is non zero, if it is, then object is
recognized. When `data` field size is zero, then no object was
recognized.

```cpp
if (object->data.size() > 0) {
```

Reading id of recognized object:

```cpp
id = object->data[0];
```

Depending on recognized object, setting appropriate speed values:

```cpp
switch (id)
{
case ARROW_LEFT:
   set_vel.linear.x = 0;
   set_vel.angular.z = 1;
   break;
case ARROW_UP:
   set_vel.linear.x = 1;
   set_vel.angular.z = 0;
   break;
case ARROW_DOWN:
   set_vel.linear.x = -1;
   set_vel.angular.z = 0;
   break;
default: // other object
   set_vel.linear.x = 0;
   set_vel.angular.z = 0;
}
```

Publishing velocity command message:

```cpp
action_pub.publish(set_vel);
```

Stopping all motors when no object was detected:

```cpp
else
{
   // No object detected
   set_vel.linear.x = 0;
   set_vel.angular.z = 0;
   action_pub.publish(set_vel);
}
```

Main function, node initialization and setting main loop interval:

```cpp
int main(int argc, char **argv)
{
   ros::init(argc, argv, "action_controller");
   ros::NodeHandle n("~");
   ros::Rate loop_rate(50);
```

Subscribing to `/objects` topic:

```cpp
ros::Subscriber sub = n.subscribe("/objects", 1, objectCallback);
```

Preparing publisher for velocity commands:

```cpp
action_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
```

Setting zeros for initial speed values:

```cpp
set_vel.linear.x = 0;
set_vel.linear.y = 0;
set_vel.linear.z = 0;
set_vel.angular.x = 0;
set_vel.angular.y = 0;
set_vel.angular.z = 0;
```

Main loop, waiting for trigger messages:

```cpp
while (ros::ok())
{
   ros::spinOnce();
   loop_rate.sleep();
}
```

Last thing to do is editting the `CMakeLists.txt` file. Find line:

```
add_executable(tutorial_pkg_node src/tutorial_pkg_node.cpp)
```

and add below code after it:

```
add_executable(action_controller_node src/action_controller.cpp)
```
Find also:

```
target_link_libraries(tutorial_pkg_node
    ${catkin_LIBRARIES}
)
```

and add below code after it:

```
target_link_libraries(action_controller_node
    ${catkin_LIBRARIES}
)
```

Now you can build your node and test it.

**Task 1** Run your node along with `find_object_2d` and `astra.launch` or **Gazebo** simulator.
Then use `rosnode`, `rostopic` and `rqt_graph` tools to examine the
system. Place different objects in front of your robot one by one. Observe
how it drives and turns depending on differnt objects.

You can use below `launch` file:

```xml
<launch>

    <arg name="rosbot_pro" default="false" />
    <arg name="use_gazebo" default="false" />

    <arg name="teach" default="false"/>
    <arg name="recognize" default="true"/>

    <arg if="$(arg teach)" name="chosen_world" value="rosbot_world_teaching"/>
    <arg if="$(arg recognize)" name="chosen_world" value="rosbot_world_recognition"/>

    <!-- Gazebo -->
    <group if="$(arg use_gazebo)">
        <include file="$(find rosbot_gazebo)/launch/$(arg chosen_world).launch"/>
        <include file="$(find rosbot_description)/launch/rosbot_gazebo.launch"/>
            <param name="use_sim_time" value="true" />
    </group>

    <!-- ROSbot 2.0 -->
    <group unless="$(arg use_gazebo)">
        <include file="$(find rosbot_ekf)/launch/all.launch">
            <arg name="rosbot_pro" value="$(arg rosbot_pro)" />
        </include>

        <include unless="$(arg use_gazebo)" file="$(find astra_launch)/launch/astra.launch"/>
    </group>

    <node unless="$(arg use_gazebo)" pkg="tf" type="static_transform_publisher" name="camera_publisher" args="-0.03 0 0.18 0 0 0 base_link camera_link 100" />

    <node name="teleop_twist_keyboard" pkg="teleop_twist_keyboard" type="teleop_twist_keyboard.py" output="screen"/>

    <node pkg="find_object_2d" type="find_object_2d" name="find_object_2d">
        <remap from="image" to="/camera/rgb/image_raw"/>
        <param name="gui" value="$(arg teach)"/>
        <param if="$(arg recognize)" name="objects_path" value="$(find tutorial_pkg)/image_rec/"/>
    </node>

    <node pkg="tutorial_pkg" type="action_controller_node" name="action_controller" output="screen"/>

</launch>

```

## Getting position of recognized object

Besides the ID of recognized object, `find_object_2d` node is also
publishing a homography matrix of recognized object. In computer vision
homography is used to define position of object relative to the camera. We
will use it to obtain horizontal position of the object. Homography matrix
is published in the same topic as the ID, but in the next cells of array,
they are formatted as

`[objectId1, objectWidth, objectHeight, h11, h12, h13, h21, h22, h23, h31, h32, h33, object2]`.

We will modify node to rotate robot to the direction of recognized object.

Open `action_controller.cpp` file in text editor.

Begin with including of required header file:

```cpp
#include <opencv2/opencv.hpp>
```

Variable for storing camera centre- this should be half of your camera
horizontal resolution:

```cpp
int camera_center = 320; // left 0, right 640
```

Variables for defining rotation speed:

```cpp
float max_ang_vel = 0.6;
float min_ang_vel = 0.4;
float ang_vel = 0;
```

Variable for object width and height:

```cpp
float objectWidth = object->data[1];
float objectHeight = object->data[2];
```

Variable for storing calculated object centre:

```cpp
float x_pos;
```

Variable defining how much rotation speed should increase with every
pixel of object displacement:

```cpp
float speed_coefficient = (float) camera_center / max_ang_vel /4;
```

Object for homography matrix:

```cpp
cv::Mat cvHomography(3, 3, CV_32F);
```

Vectors for storing input and output planes:

```cpp
std::vector<cv::Point2f> inPts, outPts;
```

Adding new case in `switch` statement:

```cpp
case SMILE:
```

Extracting coefficients homography matrix:

```cpp
cvHomography.at<float>(0, 0) = object->data[3];
cvHomography.at<float>(1, 0) = object->data[4];
cvHomography.at<float>(2, 0) = object->data[5];
cvHomography.at<float>(0, 1) = object->data[6];
cvHomography.at<float>(1, 1) = object->data[7];
cvHomography.at<float>(2, 1) = object->data[8];
cvHomography.at<float>(0, 2) = object->data[9];
cvHomography.at<float>(1, 2) = object->data[10];
cvHomography.at<float>(2, 2) = object->data[11];
```

Defining corners of input plane:

```cpp
inPts.push_back(cv::Point2f(0, 0));
inPts.push_back(cv::Point2f(objectWidth, 0));
inPts.push_back(cv::Point2f(0, objectHeight));
inPts.push_back(cv::Point2f(objectWidth, objectHeight));
```

Calculating perspective transformation:

```cpp
cv::perspectiveTransform(inPts, outPts, cvHomography);
```

Calculating centre of object from its corners:

```cpp
x_pos = (int) (outPts.at(0).x + outPts.at(1).x + outPts.at(2).x + outPts.at(3).x) / 4;
```

Calculating angular speed value proportional to position of object and putting
it into velocity message:

```cpp
ang_vel = -(x_pos - camera_center) / speed_coefficient;

if (ang_vel >= -(min_ang_vel / 2) && ang_vel <= (min_ang_vel / 2))
{
   set_vel.angular.z = 0;
}
else if (ang_vel >= max_ang_vel)
{
   set_vel.angular.z = max_ang_vel;
}
else if (ang_vel <= -max_ang_vel)
{
   set_vel.angular.z = -max_ang_vel;
}
else
{
   set_vel.angular.z = ang_vel;
}
```

Your final file should look like this:

```cpp
#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <opencv2/opencv.hpp>

#define SMILE 4
#define ARROW_LEFT 3
#define ARROW_UP 5
#define ARROW_DOWN 6
int id = 0;
ros::Publisher action_pub;
geometry_msgs::Twist set_vel;
int camera_center = 320; // left 0, right 640
float max_ang_vel = 0.6;
float min_ang_vel = 0.4;
float ang_vel = 0;

void objectCallback(const std_msgs::Float32MultiArrayPtr &object)
{
   if (object->data.size() > 0)
   {
      id = object->data[0];

      float objectWidth = object->data[1];
      float objectHeight = object->data[2];
      float x_pos;
      float speed_coefficient = (float)camera_center / max_ang_vel / 4;

      // Find corners OpenCV
      cv::Mat cvHomography(3, 3, CV_32F);
      std::vector<cv::Point2f> inPts, outPts;
      switch (id)
      {
      case ARROW_LEFT:
         set_vel.linear.x = 0;
         set_vel.angular.z = 1.0;
         break;
      case ARROW_UP:
         set_vel.linear.x = 1;
         set_vel.angular.z = 0;
         break;
      case ARROW_DOWN:
         set_vel.linear.x = -1;
         set_vel.angular.z = 0;
         break;
      case SMILE:
         cvHomography.at<float>(0, 0) = object->data[3];
         cvHomography.at<float>(1, 0) = object->data[4];
         cvHomography.at<float>(2, 0) = object->data[5];
         cvHomography.at<float>(0, 1) = object->data[6];
         cvHomography.at<float>(1, 1) = object->data[7];
         cvHomography.at<float>(2, 1) = object->data[8];
         cvHomography.at<float>(0, 2) = object->data[9];
         cvHomography.at<float>(1, 2) = object->data[10];
         cvHomography.at<float>(2, 2) = object->data[11];

         inPts.push_back(cv::Point2f(0, 0));
         inPts.push_back(cv::Point2f(objectWidth, 0));
         inPts.push_back(cv::Point2f(0, objectHeight));
         inPts.push_back(cv::Point2f(objectWidth, objectHeight));
         cv::perspectiveTransform(inPts, outPts, cvHomography);

         x_pos = (int)(outPts.at(0).x + outPts.at(1).x + outPts.at(2).x +
                       outPts.at(3).x) /
                 4;
         ang_vel = -(x_pos - camera_center) / speed_coefficient;

         if (ang_vel >= -(min_ang_vel / 2) && ang_vel <= (min_ang_vel / 2))
         {
            set_vel.angular.z = 0;
         }
         else if (ang_vel >= max_ang_vel)
         {
            set_vel.angular.z = max_ang_vel;
         }
         else if (ang_vel <= -max_ang_vel)
         {
            set_vel.angular.z = -max_ang_vel;
         }
         else
         {
            set_vel.angular.z = ang_vel;
         }

         break;
      default: // other object
         set_vel.linear.x = 0;
         set_vel.angular.z = 0;
      }
      action_pub.publish(set_vel);
   }
   else
   {
      // No object detected
      set_vel.linear.x = 0;
      set_vel.angular.z = 0;
      action_pub.publish(set_vel);
   }
}

int main(int argc, char **argv)
{
   std_msgs::String s;
   std::string str;
   str.clear();
   str.append("");
   std::to_string(3);
   s.data = str;
   ros::init(argc, argv, "action_controller");
   ros::NodeHandle n("~");
   ros::Subscriber sub = n.subscribe("/objects", 1, objectCallback);
   ros::Rate loop_rate(50);
   action_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
   set_vel.linear.x = 0;
   set_vel.linear.y = 0;
   set_vel.linear.z = 0;
   set_vel.angular.x = 0;
   set_vel.angular.y = 0;
   set_vel.angular.z = 0;
   while (ros::ok())
   {
      ros::spinOnce();
      loop_rate.sleep();
   }
}
```

Last thing to do is to edit the `CMakeLists.txt` file. Find line:

```
find_package(catkin REQUIRED COMPONENTS roscpp )
```

and add below code after it:

```
find_package( OpenCV REQUIRED )
```

Find also:

```
include_directories(
    ${catkin_INCLUDE_DIRS}
)
```
and change it to:

```
include_directories(
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)
```

Find:

```
target_link_libraries(action_controller_node
    ${catkin_LIBRARIES}
)
```
and change it to:

```
target_link_libraries(action_controller_node
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
)
```

Now you can build your node and test it.

**Task 2** Run your node along with `find_object_2d` and `astra.launch` or **Gazebo** simulator.
Place the object with ID bonded to new case in switch statement in front of your robot. Observe how it turns towards the object.

## Following the object

Open `action_controller.cpp` file in text editor.

Begin with including required header file:

```cpp
#include <sensor_msgs/Range.h>
```

Add variables for measured object distance, average distance and desired
distance to obstacle:

```cpp
float distFL = 0;
float distFR = 0;
float average_dist = 0;
float desired_dist = 0.2;
```

Callback functions for incoming sensor messages, their task is only to
put values into appropriate variables:

```cpp
void distFL_callback(const sensor_msgs::Range &range)
{
   distFL = range.range;
}

void distFR_callback(const sensor_msgs::Range &range)
{
   distFR = range.range;
}
```

Then, in `switch` statement, calculate average distance and set velocity
proportional to it only if both sensors found an obstacle, else set zero
value for linear velocity:

```cpp
if (distFL > 0 && distFR > 0)
{
   average_dist = (distFL + distFR) / 2;
   set_vel.linear.x = (average_dist - desired_dist) / 4;
}
else
{
   set_vel.linear.x = 0;
}
```

In main function, subscribe to sensor topics:

```cpp
ros::Subscriber distFL_sub = n.subscribe("/range/fl", 1, distFL_callback);
ros::Subscriber distFR_sub = n.subscribe("/range/fr", 1, distFR_callback);
```

Final file should look like this:

```cpp
#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32MultiArray.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/Range.h>

#define SMILE 4
#define ARROW_LEFT 3
#define ARROW_UP 5
#define ARROW_DOWN 6
int id = 0;
ros::Publisher action_pub;
geometry_msgs::Twist set_vel;
int camera_center = 320; // left 0, right 640
float max_ang_vel = 0.6;
float min_ang_vel = 0.4;
float ang_vel = 0;

float distFL = 0;
float distFR = 0;
float average_dist = 0;
float desired_dist = 0.2;

void distFL_callback(const sensor_msgs::Range &range)
{
   distFL = range.range;
}

void distFR_callback(const sensor_msgs::Range &range)
{
   distFR = range.range;
}

void objectCallback(const std_msgs::Float32MultiArrayPtr &object)
{
   if (object->data.size() > 0)
   {
      id = object->data[0];

      float objectWidth = object->data[1];
      float objectHeight = object->data[2];
      float x_pos;
      float speed_coefficient = (float)camera_center / max_ang_vel;

      // Find corners OpenCV
      cv::Mat cvHomography(3, 3, CV_32F);
      std::vector<cv::Point2f> inPts, outPts;
      switch (id)
      {
      case ARROW_LEFT:
         set_vel.linear.x = 0;
         set_vel.angular.z = 1;
         break;
      case ARROW_UP:
         set_vel.linear.x = 1;
         set_vel.angular.z = 0;
         break;
      case ARROW_DOWN:
         set_vel.linear.x = -1;
         set_vel.angular.z = 0;
         break;
      case SMILE:
         cvHomography.at<float>(0, 0) = object->data[3];
         cvHomography.at<float>(1, 0) = object->data[4];
         cvHomography.at<float>(2, 0) = object->data[5];
         cvHomography.at<float>(0, 1) = object->data[6];
         cvHomography.at<float>(1, 1) = object->data[7];
         cvHomography.at<float>(2, 1) = object->data[8];
         cvHomography.at<float>(0, 2) = object->data[9];
         cvHomography.at<float>(1, 2) = object->data[10];
         cvHomography.at<float>(2, 2) = object->data[11];

         inPts.push_back(cv::Point2f(0, 0));
         inPts.push_back(cv::Point2f(objectWidth, 0));
         inPts.push_back(cv::Point2f(0, objectHeight));
         inPts.push_back(cv::Point2f(objectWidth, objectHeight));
         cv::perspectiveTransform(inPts, outPts, cvHomography);

         x_pos = (int)(outPts.at(0).x + outPts.at(1).x + outPts.at(2).x +
                       outPts.at(3).x) /
                 4;
         ang_vel = -(x_pos - camera_center) / speed_coefficient;

         if (ang_vel >= -(min_ang_vel / 2) && ang_vel <= (min_ang_vel / 2))
         {
            set_vel.angular.z = 0;
            if (distFL > 0 && distFR > 0)
            {
               average_dist = (distFL + distFR) / 2;
               set_vel.linear.x = (average_dist - desired_dist) / 4;
            }
            else
            {
               set_vel.linear.x = 0;
            }
         }
         else if (ang_vel >= max_ang_vel)
         {
            set_vel.angular.z = max_ang_vel;
         }
         else if (ang_vel <= -max_ang_vel)
         {
            set_vel.angular.z = -max_ang_vel;
         }
         else
         {
            set_vel.angular.z = ang_vel;
         }

         break;
      default: // other object
         set_vel.linear.x = 0;
         set_vel.angular.z = 0;
      }
      action_pub.publish(set_vel);
   }
   else
   {
      // No object detected
      set_vel.linear.x = 0;
      set_vel.angular.z = 0;
      action_pub.publish(set_vel);
   }
}

int main(int argc, char **argv)
{

   std_msgs::String s;
   std::string str;
   str.clear();
   str.append("");
   std::to_string(3);
   s.data = str;
   ros::init(argc, argv, "action_controller");
   ros::NodeHandle n("~");
   ros::Subscriber sub = n.subscribe("/objects", 1, objectCallback);

   ros::Subscriber distL_sub = n.subscribe("/range/fl", 1, distFL_callback);
   ros::Subscriber distR_sub = n.subscribe("/range/fr", 1, distFR_callback);
   ros::Rate loop_rate(50);
   action_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
   set_vel.linear.x = 0;
   set_vel.linear.y = 0;
   set_vel.linear.z = 0;
   set_vel.angular.x = 0;
   set_vel.angular.y = 0;
   set_vel.angular.z = 0;
   while (ros::ok())
   {
      ros::spinOnce();
      loop_rate.sleep();
   }
}
```

Now you can build your node and test it.

**Task 3** Run your node along with `find_object_2d` and `astra.launch` or **Gazebo** simulator.
Place the same object as in Task 2 in front of your robot.
Observe how it turns and drives towards the object.

## Summary

After completing this tutorial you should be able to configure your
CORE2 device with vision system to recognize objects. You should also be
able to determine the position of recognized object relative to camera and
create a node that perform specific action related to recognized objects.
You should also know how to handle proximity sensors with `sensor_msgs/Range`
message type.

---

_by Łukasz Mitka, Husarion_

_Do you need any support with completing this tutorial or have any difficulties with software or hardware? Feel free to describe your thoughts on our community forum: https://community.husarion.com/ or to contact with our support: support@husarion.com_
