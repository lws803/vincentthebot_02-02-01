#!/bin/bash

#Compile and run the program
g++ -std=gnu++11 main.cpp serial.cpp serialize.cpp -pthread -o run
./run
