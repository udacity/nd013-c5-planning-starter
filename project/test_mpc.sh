#!/bin/bash

g++ -o main_mpc.o main_mpc.cpp mpc_controller.cpp

./main_mpc.o

gnuplot -p "plot_mpc.plt"

