PID controller with Carla Simulator
========================================

Run Carla Simulator
-------------------
Open new window
'su - student'
// Will say permission denied, ignore and continue 
'cd /opt/carla-simulator/'
'SDL_VIDEODRIVER=offscreen ./CarlaUE4.sh -opengl'


Run
---
'cd nd013-c5-planning-starter/project'
'./install-ubuntu.sh'
'cd pid_controller/'
'cmake .'
'make'
'cd nd013-c5-planning-starter/project'
'./run_main_pid.sh'
// This will silently fail 
ctrl + C to stop 
'./run_main.sh' (again)
Go to desktop mode to see CARLA



Camera Control for spectator window from carla server script

key listener is from python script window, then use arrow keys to change camera angle
and use w/s to change camera zoom

IF rpclib is not a populated folder can get the source files with
git clone https://github.com/carla-simulator/rpclib.git
