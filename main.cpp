#include "program.h"
#include <bits/stdc++.h>


int main(){

    Ball b1;
    Ball b2;
    Ball b3,b4,b5,b6;


    b1.x= 3;
    b1.y= 5;
    b1.vy= 0;
    b1.vx = -4;

    b2.x= 7;
    b2.y= 5;
    b2.vy = 0;
    b2.vx = 4;

    b3.x = 8;
    b3.y = 8;
    b3.vx = 0;
    b3.vy = 0;

    b4.x = 1;
    b4.y = 1;
    b4.vx = 3;
    b4.vy = 3;

    b5.x = 17;
    b5.y = 15;
    b5.vx = -2;
    b5.vy = 1;

    b6.x = 2;
    b6.y = 14;
    b6.vx = -2;
    b6.vy = 1;


    vector<Ball> balls = {b1,b2};
    vector<float> times = {200,5,3,2,1};


    Program program = Program(balls,times);

    program.startSimulation();

    return 0;
}