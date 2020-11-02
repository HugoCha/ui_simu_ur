# User Interface of simu_ur

The package goes with simu\_ur and correspond to the User Interface of simu\_ur, it contains an rqt plugin for UI to display with rqt, a ROS package installed by default. 

## Pre-requisites

### PyQt5

You need to install PyQt5 with (if your work with local environnement):
<pre><code> sudo apt-get install python-pyqt5 </code></pre>

If you work with virtual environnement, go check : http://www.niladicpodcast.com/blog/2017/8/install-pyqt5-inside-a-virtual-environment/


Please go check : https://github.com/HugoCha/simu_ur/blob/master/README.md to see the other instalation needed.

## Usage

**Run the plugin**

The first time you use it, try with a roscore running:
In a terminal :
<pre><code>roscore</code></pre>

In an other terminal:
<pre><code>rosrun ui_simu_ur rqt_ui</code></pre>

If it doesn't work run:
<pre><code>rqt --force-discover</code></pre>

Then you should see the plugin Ur_Cube in the Plugins list of the menu bar of rqt.

## Composition

**Labels**

<i> Remaining Time </i> : The countdown of the time.

<i> Score </i> : The current score of the player.

<i> Record </i> : The record is keep in memory during time.


<ins>Modify the values</ins>

In order to change value of <i>record</i>, you could go to ressource/record.txt and change the value inside the file.

In order to change the initial value of the <i>Remaining time</i>, you could change the variable MINUTE and SECOND, line 25 from src/my_module.py.

In order to change the value of the point scored in function of the position, you could change LOW_SCORE and HIGH_SCORE variable from simu_ur/src/pick_and_place.py

**Buttons**

<i> Play </i> : Activate the game, if it's the first click on a button, the robot will go to the secured position and then the game will start. 

<i> Pause </i> : Pause the game.

<i> Stop </i> : Reset the game and makes the robot goes to secured pose

<i> Home </i> :  Make the robot goes to secured position but the game must be paused first. If it's the first button clicked the robot will go to the secured position

**Image**

The image should be the output of the vision node from simu_ur package and it surrounds when a cube is detected. 

**Pop up window**

Dialog window will appear in function of the sequences.

## How it should look like


This is how it should look like when you use (with a different text size):
<pre><code>rosrun ui_simu_ur rqt_ui</code></pre>

![UI Alone Picture](https://gitlab.inria.fr/auctus/simuurui/-/raw/master/resource/UI_alone_picture.png)

This is how it should look like when you use :
<pre><code>roslaunch simu_ur Ur3.launch sim:=true protocol:=1</code></pre>

In simulation :

![UI simulation Picture](https://github.com/HugoCha/ui_simu_ur/raw/main/resource/UI_alone_picture.png)

In the reality :
![UI real Picture](https://github.com/HugoCha/ui_simu_ur/-/raw/master/resource/UI_real.png)


## Description of the Sequences

### The Main window event :

![Main Window](https://gitlab.inria.fr/auctus/simuurui/-/raw/master/resource/Main_process.png)


### The countdown event :

![Countdown](https://github.com/HugoCha/ui_simu_ur/-/raw/master/resource/Countdown.png)

### The Cube detection event :

![Cube detection](https://github.com/HugoCha/ui_simu_ur/-/raw/master/resource/cube_detect.png)
