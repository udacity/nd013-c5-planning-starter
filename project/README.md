# Motion Planning and Decision Making for Autonomous Vehicles

In this project, you will implement two of the main components of a traditional hierarchical planner: The Behavior Planner and the Motion Planner. 

Both will work in unison to be able to:
* Avoid static objects (cars, bicycles and trucks) parked on the side of the road (but still invading the lane). The vehicle must avoid crashing with these vehicles by executing either a “nudge” or a “lane change” maneuver.
* Handle any type of intersection (3-way,  4-way intersections and roundabouts) by STOPPING in all of them (by default)
* Track the centerline on the traveling lane.

To accomplish this, you will implement:

* Behavioral planning logic using Finite State Machines - FSM
* Static objects Collision checking.
* Path and Trajectory generation using Cubic Spirals
* Best trajectory selection though a cost function evaluation. This cost function will mainly perform a collision check and a proximity check to bring cost higher as we get closer or collide with objects but maintaining a bias to stay closer to the lane center line.

To work on this project, the CARLA simulator along with the base project code needs to be installed.

> **NOTE: Any intersection, whether protected or unprotected will be considered as an “unprotected intersection and the car will always STOP for 5 seconds before continuing motion.**


## Getting Started

Instructions for how to get a copy of the project running on your local machine.

### Dependencies

```
eigen-3.3.7
```
but no worries, it's included in the project repo.

# Installation
 
## For Windows & Ubuntu 18.04 and Earlier
* Pre-requisites
   * python3
   * pip
* The deb installation is the easiest way to get the latest release in Linux.
```
sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-keys 1AF1527DE64CB8D9
sudo add-apt-repository "deb [arch=amd64] http://dist.carla.org/carla $(lsb_release -sc) main"
sudo apt-get update # Update the Debian package index
sudo apt-get install carla-simulator=0.9.10-2 
cd /opt/carla-simulator
# Linux
./CarlaUE4.sh

# Windows:
CarlaUE4.exe
```

More details can be found at [CARLA Quick Start Installation](https://carla.readthedocs.io/en/latest/start_quickstart/)  
> NOTE: CARLA does NOT work on Ubuntu 20.04 or Mac yet!!
 
## Download Udacity Project code-base    
* Once CARLA launches without problems you will clone the project code-base
   ```
   mkdir sdcnd_c5 # Or wherever you want to place
   cd sdcnd_c5
   git clone https://github.com/udacity/nd013-c5-planning-refresh.git
   ```
 
Make sure ALL is working well:
---------------
   * Open a terminal an launch CARLA (if you don't have it open already):
   ```
   cd /opt/carla-simulator
   # Linux
   ./CarlaUE4.sh
   ```   
   * Open another terminal and launch Udacity’s Decision making & Motion Planning application and Compile it
```
cd project/solution_cubic_spirals_integrated
cmake .
make
```
* Run it

```
cd project/solution_cubic_spirals_integrated
./spiral_planner
```

* Open new window

```
cd project
python3 simulatorAPI.py
```

Camera Control for spectator window from carla server script

key listener is from python script window, then use arrow keys to change camera angle
and use w/s to change camera zoom

IF rpclib is not a populated folder can get the source files with
git clone https://github.com/carla-simulator/rpclib.git

## Follow the TODO’s on the project plan and code all your changes.

Include all items used to build project.

## License
[License](../LICENSE.md)
