#include "program.h"
#include <bits/stdc++.h>


int main(){

    /*Ball b1;
    Ball b2;
    Ball b3,b4,b5,b6,b7,b8,b9,b10,b11,b12;


    b1.x = 9;
    b1.y = 1;
    b1.vy= 2;
    b1.vx = -2;

    b2.x = 1;
    b2.y = 9;
    b2.vy = -2;
    b2.vx = 2;

    b3.x = 3;
    b3.y = 4.8284271;
    b3.vx = 0;
    b3.vy = -1.4142135;

    b4.x = 9;
    b4.y = 9;
    b4.vx = -2;
    b4.vy = -2;

    b5.x = 9;
    b5.y = 1;
    b5.vx = -100;
    b5.vy = 0;

    b6.x = 1;
    b6.y = 1;
    b6.vx = 100;
    b6.vy = 0;


    b7.x = 5;
    b7.y = 5;
    b7.vx = 0;
    b7.vy = 0; //   b7 b8 b9 wrong

    b8.x = 5;
    b8.y = 1;
    b8.vx = 0;
    b8.vy = 2;

    b9.x = 5;
    b9.y = 9;
    b9.vx = 0;
    b9.vy = -2;




    b10.x = 5;
    b10.y = 7;
    b10.vx = 0;
    b10.vy = 0; //  b10 b11 b12 Newton's Cradle :)

    b11.x = 5;
    b11.y = 5;
    b11.vx = 0;
    b11.vy = 0;

    b12.x = 5;
    b12.y = 1;
    b12.vx = 0;
    b12.vy = 2;


    vector<Ball> balls = {b10,b11,b12};
    vector<float> times = {50,5,3,2,1};


    Program program = Program(balls,times);

    program.startSimulation();*/
    bool testmode = false;
    Program program = Program(testmode);
    program.startSimulation();

    return 0;
}