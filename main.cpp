#include "program.h"
#include <bits/stdc++.h>


int main(){

    Ball b1;
    Ball b2;
    Ball b3,b4,b5;


    b1.x= 100;
    b1.y= 10;
    b1.vy= 100;
    b1.vx = 0;

    b2.x= 0;
    b2.y= 6;
    b2.vy = 0;
    b2.vx = 6;

    b3.x = 55.5;
    b3.y = 10.9;
    b3.vx = 0;
    b3.vy = 0;

    b4.x = 32.5;
    b4.y = 0.9;
    b4.vx = 1;
    b4.vy = 0;

    b5.x = 70.5;
    b5.y = 14.9;
    b5.vx = 0;
    b5.vy = 1;


    vector<Ball> balls = {b1};
    vector<float> times = {200,5,3,2,1};


    Program program = Program(balls,times);

    program.startSimulation();

    return 0;
}