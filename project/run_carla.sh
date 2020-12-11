#!/bin/bash

SDL_VIDEODRIVER=offscreen SDL_HINT_CUDA_DEVICE=0 /opt/carla-simulator/CarlaUE4.sh -opengl&
sleep 2
python3 simulatorAPI.py
trap "trap - SIGTERM && kill -- -$$" SIGINT SIGTERM EXIT
