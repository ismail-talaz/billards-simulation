#include "program.h"
#include <bits/stdc++.h>


int main(){

    Ball b1;
    Ball b2;

    b1.x= 2;
    b1.y= 0;
    b1.vy= 0;
    b1.vx = 100;

    b2.x= 0;
    b2.y= 3;
    b2.vy = 5;
    b2.vx = 1;

    vector<Ball> balls = {b1,b2};
    vector<float> times = {50,20,5,3,2,1};


    Program program = Program(balls,times);

    program.startSimulation();

    return 0;
}