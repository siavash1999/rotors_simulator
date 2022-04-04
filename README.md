RotorS - Modified
===============
NOTE: This repository is a [fork](https://github.com/ethz-asl/rotors_simulator)!

RotorS is a MAV gazebo simulator.
It provides some multirotor models such as the [AscTec Hummingbird](http://www.asctec.de/en/uav-uas-drone-products/asctec-hummingbird/), the [AscTec Pelican](http://www.asctec.de/en/uav-uas-drone-products/asctec-pelican/), or the [AscTec Firefly](http://www.asctec.de/en/uav-uas-drone-products/asctec-firefly/), but the simulator is not limited for the use with these multicopters.

There are simulated sensors coming with the simulator such as an IMU, a generic odometry sensor, and the [VI-Sensor](http://wiki.ros.org/vi_sensor), which can be mounted on the multirotor.

This package also contains some example controllers, basic worlds, and example launch files.

Below the ETHZ-asl group provide the instructions necessary for getting started. See RotorS' wiki for more instructions and examples (https://github.com/ethz-asl/rotors_simulator/wiki).

If you are using this simulator within the research for your publication, please cite:
```bibtex
@Inbook{Furrer2016,
author="Furrer, Fadri
and Burri, Michael
and Achtelik, Markus
and Siegwart, Roland",
editor="Koubaa, Anis",
chapter="RotorS---A Modular Gazebo MAV Simulator Framework",
title="Robot Operating System (ROS): The Complete Reference (Volume 1)",
year="2016",
publisher="Springer International Publishing",
address="Cham",
pages="595--625",
isbn="978-3-319-26054-9",
doi="10.1007/978-3-319-26054-9_23",
url="http://dx.doi.org/10.1007/978-3-319-26054-9_23"
}
```
Installation Instructions - Ubuntu 16.04 with ROS Kinetic or newer
---------------------------------------------------------
 1. Install and initialize ROS [distro] desktop full, additional ROS packages, catkin-tools, and wstool:

 ```
 $ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list'
 $ wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
 $ sudo apt-get update
 $ sudo apt-get install ros-[distro]-desktop-full ros-[distro]-octomap-ros ros-[distro]-mavlink python-wstool python-catkin-tools protobuf-compiler libgoogle-glog-dev ros-[distro]-control-toolbox ros-[distro]-mavros
 $ sudo rosdep init
 $ rosdep update
 $ source /opt/ros/[distro]/setup.bash
 ```

 2. If you don't have ROS workspace yet you can do so by

 ```
 $ mkdir -p ~/catkin_ws/src
 $ cd ~/catkin_ws/src
 $ catkin_init_workspace  # initialize your catkin workspace
 ```

 3. Get the simulator and additional dependencies

 ```
 $ cd ~/catkin_ws/src
 $ git clone git@github.com:ethz-asl/rotors_simulator.git
 $ git clone git@github.com:ethz-asl/mav_comm.git
 ```

 4. Build your workspace with `python_catkin_tools` (therefore you need `python_catkin_tools`)

   ```
   $ cd ~/catkin_ws/
   $ catkin build
   ```

 5. Add sourcing to your `.bashrc` file

   ```
   $ echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
   $ source ~/.bashrc
   ```

  > **Note** On OS X you need to install yaml-cpp using Homebrew `brew install yaml-cpp`.

Basic Usage
-----------

Launch the simulator with a hex-rotor helicopter model, in our case, the AscTec Firefly in a basic world.

```
$ roslaunch rotors_gazebo mav_hovering_example.launch mav_name:=firefly world_name:=basic
```

> **Note** The first run of gazebo might take considerably long, as it will download some models from an online database. Should you receive a timeout error, try running gazebo by itself (e.g. roslaunch gazebo_ros empty_world.launch ) so it has sufficient time to actually download all of the models.

The simulator starts by default in paused mode. To start it you can either
 - use the Gazebo GUI and press the play button
 - or you can send the following service call.

   ```
   $ rosservice call gazebo/unpause_physics
   ```

There are some basic launch files where you can load the different multicopters with additional sensors. They can all be found in `~/catkin_ws/src/rotors_simulator/rotors_gazebo/launch`.

The `world_name` argument looks for a .world file with a corresponding name in `~/catkin_ws/src/rotors_simulator/rotors_gazebo/worlds`. By default, all launch files, with the exception of those that have the world name explicitly included in the file name, use the empty world described in `basic.world`.

### Getting the multicopter to fly

To let the multicopter fly you need to generate thrust with the rotors, this is achieved by sending commands to the multicopter, which make the rotors spin.
There are currently a few ways to send commands to the multicopter, we will show one of them here.
The rest is documented [here](../../wiki) in our Wiki.
We will here also show how to write a stabilizing controller and how you can control the multicopter with a joystick.

#### Send direct motor commands

We will for now just send some constant motor velocities to the multicopter.

```
$ rostopic pub /firefly/command/motor_speed mav_msgs/Actuators '{angular_velocities: [100, 100, 100, 100, 100, 100]}'
```

> **Note** The size of the `motor_speed` array should be equal to the number of motors you have in your model of choice (e.g. 6 in the Firefly model).

You should see (if you unpaused the simulator and you have a multicopter in it), that the rotors start spinning. The thrust generated by these motor velocities is not enough though to let the multicopter take off.
> You can play with the numbers and will realize that the Firefly will take off with motor speeds of about 545 on each rotor. The multicopter is unstable though, since there is no controller running, if you just set the motor speeds.


#### Let the helicopter hover with ground truth odometry

You can let the helicopter hover with ground truth odometry (perfect state estimation), by launching:

```
$ roslaunch rotors_gazebo mav_hovering_example.launch mav_name:=firefly world_name:=basic
```

Gazebo Version
--------------

At a minimum, Gazebo `v2.x` is required (which is installed by default with ROS Indigo). However, it is **recommended to install at least Gazebo `v5.x`** for full functionlity, as there are the following limitations:

1. `iris.sdf` can only be generated with Gazebo >= `v3.0`, as it requires use of the `gz sdf ...` tool. If this requirement is not met, you will not be able to use the Iris MAV in any of the simulations.
2. The Gazebo plugins `GazeboGeotaggedImagesPlugin`, `LidarPlugin` and the `LiftDragPlugin` all require Gazebo >= `v5.0`, and will not be built if this requirement is not met.

Changes to the main branch
--------------------------

1. Changed the README.md file.
2. Made a LQR controller for stablizing position and orientation of pelican drone, with a launch file for simulation.
3. Made a PID controller for attitude and position control for pelican drone.
4. Deleted nodes and files that were unused.

**TODO:**

5. Make PID controller parameters dynamically reconfigurable.
6. Correct the model to be able to do yaw rotation.
7. Enhance the performance.
8. Import simple maps to world files.
9. Make path-planning nodes to send velocity messages.
