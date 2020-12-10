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
 
* Make sure ALL is working well:
   * Open a terminal an launch CARLA (if you don't have it open already):
   ```
   cd /opt/carla-simulator
   # Linux
   ./CarlaUE4.sh

   # Windows:
   CarlaUE4.exe
   ```   
   * Open another terminal and launch Udacity’s Decision making & Motion Planning application:
 
   ```
   cd sdcnd_c5/nd013-c5-planning-refresh/project/solution_cubic_spirals_STARTER
   make run
   ```
* Follow the TODO’s on the project plan and code all your changes.
 
> NOTE:
>
> To just compile use:
> ```
> make
> ```
> To compile and run use:
> ```
> make run
> ``` 
> To just run the latest compiled project use
> ```
> make run.only
> ```
> 

## Testing

Explain the steps needed to run any automated tests

### Break Down Tests

Explain what each test does and why

```
Examples here
```
## Project Instructions

This section should contain all the student deliverables for this project.

## Built With

* [Item1](www.item1.com) - Description of item
* [Item2](www.item2.com) - Description of item
* [Item3](www.item3.com) - Description of item

Include all items used to build project.

## License
[License](../LICENSE.md)
