---
title: Run ROS tutorials using AWS RoboMaker
sidebar_label: 2. Run ROS tutorials using AWS RoboMaker
id: run-ros-tutorials-using-aws-robomaker
---

## Introduction

**AWS RoboMaker** is the newest service by *Amazon* for robot developers community. It’s ROS based and provides web tools for robot development, simulation and deployment - https://aws.amazon.com/robomaker/ .

In this tutorial we will show you how to setup environment at AWS RoboMaker to learn ROS with our ROS tutorials:
1. [ROS introduction](https://husarion.com/tutorials/ros-tutorials/1-ros-introduction/)
2. [Creating nodes](https://husarion.com/tutorials/ros-tutorials/2-creating-nodes/)
3. [Simple kinematics for mobile robot](https://husarion.com/tutorials/ros-tutorials/3-simple-kinematics-for-mobile-robot/)
4. [Visual object recognition](https://husarion.com/tutorials/ros-tutorials/4-visual-object-recognition/)
5. [Running ROS on multiple machines](https://husarion.com/tutorials/ros-tutorials/5-running-ros-on-multiple-machines/ )
6. [SLAM navigation](https://husarion.com/tutorials/ros-tutorials/6-slam-navigation/ )
7. [Path planning](https://husarion.com/tutorials/ros-tutorials/7-path-planning/)
8. [Unknown environment exploration](https://husarion.com/tutorials/ros-tutorials/8-unknown-environment-exploration/ )
9. [Object search](https://husarion.com/tutorials/ros-tutorials/9-object-search/ )

*IMPORTANT: AWS RoboMaker is a paid service and you may be charged based on the usage of some of its functionalities. For pricing info visit https://aws.amazon.com/robomaker/pricing/* .

## Setting up AWS account

To begin, you need to have an active AWS account. Go to [signup panel](https://portal.aws.amazon.com/billing/signup?registration-confirmation#/start) and follow steps required to create account.

![](/docs/assets/img/aws-tutorials/aws_tutorial_img1.png)

## Configure AWS Environment

Before we use AWS RoboMaker to build and deploy the tutorial applications, we must first set up the AWS environment. To simplify the configuration, we will use AWS CloudFormation. CloudFormation enables us to use a template file to define the configuration of our environment. We will use CloudFormation to create a bucket in Amaazon S3, as well as to create the necessary permissions in AWS Identity and Access Manager (IAM) that AWS RoboMaker requires to simulate and deploy our robot applications.

To deploy the template, sign in to the [CloudFormation console](https://console.aws.amazon.com/cloudformation/). Following the following steps to deploy the template:

1.  Download the template file from [here](https://raw.githubusercontent.com/husarion/rosbot-robomaker/master/rosbot_tutorial_template.yaml).
2.  Click the **Create Stack** button.
3.  Under _Choose a template_, choose _Upload a template to Amazon S3_ and click **Choose File**.
4.  Browse to the rosbot_tutorial_template.yaml file you download in Step 1 above.
5.  Click **Next**.
6.  On the next screen, provide a _Stack name_. This should be something descriptive such as "ROSbot-setup".
7.  In the _S3BucketName_ field, provide a globally-unique name for the S3 bucket that will be created. This S3 bucket will be used to store your robot application bundles, as well as any logs that your robot may generate during simulation. Use a name unique to you, such as "&lt;user_id&gt;-rosbot-tutorial". Replace "&lt;user-id&gt;" with a unique string.
8.  Choose **Next**.
9.  On the Options page, leave all defaults and choose **Next**.
10. On the Review page, click the checkbox to acknowledge that CloudFormation will create IAM resources on your behalf.
11. Click **Create**.

After a few brief minutes, the stack will be created. When the status has changed to CREATE_COMPLETE, choose the stack you just created, and view its Outputs. You will see 3 key/value pairs. You will use these values later in this guide.

## Setup RoboMaker IDE
1. Open [RoboMaker module](https://console.aws.amazon.com/robomaker/home)

2. Open “Development environments” tab.

![Development environments](/docs/assets/img/aws-tutorials/aws_tutorial_img21.png)

3. Click “Create environment”
- In field **Name** type `robomaker_env` and as **instance type** choose `c3.2xlarge`. You can select different instances type to improve bundling performance.
- In **VPC** dropdown list choose the default value.
- In the **Subnets** dropdown list choose the first subnet. You can select different subnet if necessary.

![Create environment dialog](/docs/assets/img/aws-tutorials/aws_tutorial_img22.png)

4. Click "Create". You will be redirected to IDE.

![RoboMaker IDE](/docs/assets/img/aws-tutorials/aws_tutorial_img23.png)

- In the IDE, go to bash tab and clone the `rosbot-robomaker` repository in `~/environment/` directory:

```
cd ~/environment/
git clone --recurse-submodules https://github.com/husarion/rosbot-robomaker.git RoboMakerROSbotProject
```

5. Configure project

In this step you will need key/value pairs obtained in CloudForamtion module
- Replace `$BUCKET_NAME` with value of `S3BucketName` from CloudFormation
- Replace `$IAM_ROLE` eith value of `RoboMakerRole` from CloudFormation

```
cd ~/environment/RoboMakerROSbotProject/
python configure_project.py --bucket $BUCKET_NAME --iam $IAM_ROLE
```

## Launch sumilation job

1.  Choose menu **Run** -> **Add or Edit Configurations**.

![RoboMaker configuration dialog](/docs/assets/img/aws-tutorials/aws_tutorial_img24.png)

2. Click button **Switch config**.

3. Choose `RoboMakerSettings.json` from folder `RoboMakerROSbotProject` and click **OK** button and then **Save** button.

![Switch config dialog](/docs/assets/img/aws-tutorials/aws_tutorial_img25.png)

4. Choose menu **Run** -> **Workflow** -> **ROSbotTutorial - Build and Bundle All**.

![Workflow menu](/docs/assets/img/aws-tutorials/aws_tutorial_img26.png)

5. Package build process will start, when it is done, choose menu **Run** -> **Launch Simulation** -> **ROSbotTutorial9**. Simulation job will be sent to RoboMaker. Wait until **Your simulation job was created.** message appears in console.

![Simulation terminal](/docs/assets/img/aws-tutorials/aws_tutorial_img27.png)

6. Go to RoboMaker and open “Simulation jobs” menu.

![Simulation jobs menu](/docs/assets/img/aws-tutorials/aws_tutorial_img28.png)

7. Open simulation by clicking its name.

![Simulation view](/docs/assets/img/aws-tutorials/aws_tutorial_img29.png)

8. When it starts, you can open Gazebo view to watch as simulation proceeds.

![Gazebo view](/docs/assets/img/aws-tutorials/aws_tutorial_img30.png)

**Congratulations!**

You’ve just run Gazebo version of [Tutorial 9](https://husarion.com/tutorials/ros-tutorials/9-object-search/) on AWS RoboMaker cloud computing platform. Other tutorials can be launched by selecting appropriate entry in **Run** -> **Launch Simulation** menu.

## Summary

After completing this tutorial you should be familiar with AWS RoboMaker Service. You should be able to create your own environment and run various robotics simulations (in this case Husarion ROS Tutorial 9).

Possibility of outsourcing computation to powerful cloud servers opens doors for even more advanced robotics simulation for users that are not equipped with efficient work stations. We hope that this tutorial will help you understand basics of running ROS simulations in AWS RobotMaker by Amazon.

---------

*by Łukasz Mitka, Husarion*

*Do you need any support with completing this tutorial or have any difficulties with software or hardware? Feel free to describe your thoughts on our community forum: https://community.husarion.com/ or to contact with our support: support@husarion.com*
