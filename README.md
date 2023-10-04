# Purpose of This Repo

This repository contains the starter code to launch in the SDC Planning course workspace. 

## Instructions
Follow the series of commands below to launch the CARLA simulator: 


### Set Up Carla
Run the following commands in a new terminal window:
```bash
su - student
# Will say permission denied, ignore and continue 
cd /opt/carla-simulator/
SDL_VIDEODRIVER=offscreen ./CarlaUE4.sh -opengl
```

### Install the dependencies
Run the following commands in another terminal window:
```bash
git clone https://github.com/udacity/nd013-c5-planning-starter.git
cd nd013-c5-planning-starter/project
./install-ubuntu.sh
```

### Build the project and run the simulator
Run the following commands in the existing terminal window:
```bash
cd starter_files/
cmake .
make
cd nd013-c5-planning-starter/project
./run_main.sh
# This will silently fail 
# ctrl + C to stop 
```

Run the following command and go to desktop mode to see CARLA:
```bash
./run_main.sh
```



// If error bind is already in use, or address already being used
ps -aux | grep carla
kill id
```

