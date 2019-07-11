---
title: ROSbot + AWS Robomaker - Run on multiple machines tutorial
sidebar_label: 6. Run ROS on multiple machines with AWS RoboMaker
id: rosbot-aws-robomaker-multiple-machines
---


AWS RoboMaker allows you to easily develop a software for robot, test it in the simulation environment and than deploy it to the whole robot fleet at once. 

However in many cases you need to run ROS on multiple machines, eg.

a) to give your robot more computing power for video processing or perform multi-robot SLAM  
b) to control your robots through the internet, for example using Rviz or web user interface  
c) to rapidly prototype robot software without being limited to compuing power of your robot

In this tutorial we will show you:
- how to setup ROS on Virutal Machine running on AWS cloud
- how to connect your physical robot to that VM, and create multi-machine distributed ROS system
- how to program your robot with RoboMaker
- how to use Rviz to control a physical robot

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

## Setup Husarnet account

For establishing secure and fast connection among devices, we will use [Husarnet](https://husarnet.com/). Log in or create account if you do not have it (free plan will be sufficient for this tutorial). Later we will explain, how to register devices to Husarnet. 

## ROSbot setup in RoboMaker

ROSbot need some system modifications before Greengrass will be able to run and deploy applications. To configure ROSbot:

- Sign in to the AWS RoboMaker [console](https://console.aws.amazon.com/robomaker/).
- In the left navigation pane, choose **Fleet Management** and then choose **Robots**.

![RoboMaker robots](/docs/assets/img/aws-tutorials/quick-start/aws_tutorial_robomaker_1.png)

- Choose \*_Create robot_.
- In the **Name** field, type `ROSbot`.
- From the **Architecture** dropdown menu choose **ARMHF**.
- From the **AWS Greengrass group** dropdown menu choose **Create new**.
- In the **AWS Greengrass prefix** field type `ROSbot`.
- In the **IAM role** select **ROSbot-deployment-role**.

![RoboMaker create robot](/docs/assets/img/aws-tutorials/quick-start/aws_tutorial_robomaker_2.png)

- Proceed with **Create**, you will be redirected to **Download your Core device** page.

![RoboMaker robot created](/docs/assets/img/aws-tutorials/quick-start/aws_tutorial_robomaker_3.png)

- Choose **Download** button next to **Download and store your Core's security resources**.

- You will get `ROSbot-setup.zip` file, it needs to be uploaded to ROSbot. The upload process will vary, depending on your host operating system.

### On Linux

Navigate to directory where the file is downloaded, by default it should be `~/Downloads`.

```
cd ~/Downloads
```

- Copy the file to your ROSbot, you will need to substitute `ROSBOT_IP` with device address you noted earlier:

```
scp ROSbot-setup.zip husarion@ROSBOT_IP:ROSbot-setup.zip
```

### On Windows

You will need an SCP client, download and install [WinSCP](https://winscp.net/eng/download.php).

Start WinSCP, you will see the login dialog:

![WinSCP login](/docs/assets/img/aws-tutorials/quick-start/winscp1.png)

- From `File protocol` dropdown menu choose: `SFTP`.
- In `Host name` field provide rosbot IP address that you noted earlier, it is the value which we described as `ROSBOT_IP`.
- In `Port number` field provide `22`.
- In `User name` field provide `husarion`.
- In `Password` field provide `husarion`.

When all fields are filled up, click `Login` button to connect, you will see file manager view.

![WinSCP file manager](/docs/assets/img/aws-tutorials/quick-start/winscp2.png)

In the left tab navigate to directory where you downloaded the ROSbot-setup.zip file. In the right tab navigate to `/home/husarion` directory.

Drag and drop the `ROSbot-setup.zip` to the right tab.

When the transfer is finished, close the window.

## ROSbot setup on device

You will need to make some system configurations on device. Depending on your host system, you can connect to your ROSbot with different methods:

### On Linux

Open terminal and start `ssh` connection, you will need to substitute `ROSBOT_IP` with device address you noted earlier:

```
ssh husarion@ROSBOT_IP
```

Proceed to **Device setup** section.

### On Windows

Press `WinKey` + `r` then type `mstsc`.

You will see a window appear:

![Windows RDP](/docs/assets/img/aws-tutorials/quick-start/win_rdp.png)

Type in your device IP address and click connect.

You will see the ROSbot desktop, from the top menu, choose the `Applications` -> `Terminal`.

## Device setup

Type the following lines in the terminal to update the package list and upgrade packages:

```
sudo apt update
sudo apt dist-upgrade
```

In the terminal execute bellow commands:

- Copy the `setup_ROSbot_for_gg.sh` file to your ROSbot and run it as root:

```
wget https://raw.githubusercontent.com/husarion/rosbot-robomaker/master/setup_ROSbot_for_gg.sh
chmod a+x setup_ROSbot_for_gg.sh
sudo ./setup_ROSbot_for_gg.sh
```

- Unzip ROSbot security resources:

```
cd ~
sudo unzip ROSbot-setup.zip -d /greengrass
```

- Register ROSbot within Husarnet network:

```
sudo husarnet websetup
```

As a response, you will get registration link, open it in your browser and follow instructions.
When asked for device name, provide `rosbot`.
From dropdown menu **Add to network** choose **Create new network** and provide name `tutorial`.
Check **Change device hostname...** checkbox.
Push **Add device to your account** button.

- Restart ROSbot to apply chnages.

At this moment, ROSbot is ready to start AWS GreenGrass and accept incoming deployments.

Connect with ROSbot again, open terminal and start the GreenGrass:

```
sudo /greengrass/ggc/core/greengrassd start
```

Leave the ROSbot turned on, it will wait for deployment.

## Creating a RoboMaker IDE

Application will be built using the RoboMaker environment. To create the IDE:

- Sign in to the AWS RoboMaker [console](https://console.aws.amazon.com/robomaker/home)

![RoboMaker new IDE](/docs/assets/img/aws-tutorials/quick-start/aws_tutorial_robomaker_4.png)

- On the left, expand **Development**, choose **Development environments**, and then choose **Create environment**.
- In the Create AWS RoboMaker development environment page, enter `rosbot_env` as the environment name.
- Accept the default Instance type (`m4.large`). You can select different instances type to improve bundling performance.
- In **VPC** dropdown list choose the default value.
- In the **Subnets** dropdown list choose the first subnet. You can select different subnet if necessary.

![RoboMaker create IDE](/docs/assets/img/aws-tutorials/quick-start/aws_tutorial_robomaker_5.png)

- Choose **Create** to create the AWS Cloud9 development environment.

![RoboMaker IDE ready](/docs/assets/img/aws-tutorials/quick-start/aws_tutorial_robomaker_6.png)

## Deploying the application

To deploy application, you will use RoboMaker environment created in previous step:

- Go to AWS RoboMaker home [console](https://console.aws.amazon.com/robomaker/home).

- On the left, expand **Development**, choose **Development environments**, and then choose `rosbot_env`.

- Open the development environment with **Open environment** button.

![RoboMaker open IDE](/docs/assets/img/aws-tutorials/quick-start/aws_tutorial_robomaker_8.png)

- In the IDE, go to bash tab and clone the `rosbot-robomaker` repository in `~/environment/` directory:

```
cd ~/environment/
git clone --recurse-submodules https://github.com/husarion/rosbot-robomaker.git RoboMakerROSbotProject
```

![RoboMaker open IDE](/docs/assets/img/aws-tutorials/quick-start/aws_tutorial_robomaker_10.png)

- Start the configuration script. You need to provide the S3 bucket name and the ARNs of the IAM roles that were created by CloudFormation earlier. The parameters to the script should be set to the corresponding values provided in the output of your CloudFormation stack:

```
cd ~/environment/RoboMakerROSbotProject/
./IDE_setup.bash <S3BucketName> <RoboMakerRole> <ROSbotDeploymentRole> --launch tutorial_robomaker_1_rosbot.launch
```
The script will configure EC2 Instance to serve as computation node.
During the script execution, you will be prompted to use registration links for robomaker IDE and EC2 instance, add both devices to `tutorial` network.

When both devices are added, go to [Husarnet](https://app.husarnet.com/) dashboard again and make sure that `rosbot` is set to be **ROS master**.

![RoboMaker master](/docs/assets/img/aws-tutorials/quick-start/aws_tutorial_robomaker_11.png)


The script will install all dependencies, configure project, build and set the deployment job.


When the script is done with its job, you can observe the deployment process:

- Sign in to the AWS RoboMaker [console](https://console.aws.amazon.com/robomaker/).
- In the left navigation pane, choose **Fleet Management** and then choose **Deployments**.
- When new deployment will appear, open it by clicking its name.
- Deployment job will start tasks both on ROSbot and EC2 instance.

![RoboMaker open IDE](/docs/assets/img/aws-tutorials/quick-start/aws_tutorial_robomaker_12.png)

## Controlling the robot

ROSbot is prepared to autonomously navigate towards destination selected by user.

You will use ROSbot remote desktop to set destination and observe progresses:

- Connect through remote desktop with ROSbot
- Open new terminal window
- Start rviz by typing:
    ```
    rviz -d $(rospack find tutorial_pkg)/rviz/tutorial_8.rviz
    ```

You will see the Rviz visualization tool with created map and ROSbot position.  Planned trajectory will appear as soon as destination is set.

To set a destination point on a map, use **`2D Nav Goal`** button in **Rviz**.

You can see an example map being created on the screenshot below.

![RoboMaker exploration screenshot](/docs/assets/img/aws-tutorials/quick-start/aws_tutorial_robomaker_13.png)

---

_by Łukasz Mitka, Husarion_

_Do you need any support with completing this tutorial or have any difficulties with software or hardware? Feel free to describe your thoughts on our community forum: https://community.husarion.com/ or to contact with our support: support@husarion.com_
