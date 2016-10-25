#! /bin/sh

g++ -g -w driverFunc.cpp graph.cpp maxflow.cpp -o driverFunc `pkg-config --cflags --libs opencv` -std=c++11