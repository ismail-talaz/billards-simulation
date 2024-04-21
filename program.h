#include <iostream>
#include <bits/stdc++.h>

#define FMAX 340282346638528859811704183484516925440.0000000000000000;


using namespace std;

struct Ball{
    float x;
    float y;
    float vx;
    float vy;
};


class Program{

    public:
        float fr=0.1; // deacceleration rate.    
        float current_time=0;
        int numberofballs=0;
        float radius=1.0;
        vector<Ball> balls;
        vector<float> times;

        Program(vector<Ball> input,vector<float> input_times){
            for(auto Ball : input){
                numberofballs++;
                balls.push_back(Ball);
            }

            for(auto time : input_times){
                times.push_back(time);
            }
        }

        float calculateAlphaX(Ball b1, Ball b2);
        float calculateAlphaY(Ball b1, Ball b2);
        float calculateBetaX(Ball b1, Ball b2);
        float calculateBetaY(Ball b1, Ball b2);
        float calculateTimeToCollide(Ball b1, Ball b2);
        float calculateTimeToCollideWithoutFR(Ball b1, Ball b2);
        float calculateFirstTwoBalls(){
            return calculateTimeToCollide(balls[0],balls[1]);
        };
        vector<Ball> findFirstCollision();
        void timeSkip(float time);
        void collide(Ball b1, Ball b2, float time);
        void startSimulation();
        void printSnapshot(float time);
        float calculateVelocity(Ball b1);
        float calculateTheta(Ball b1);
};


float Program::calculateAlphaX(Ball b1, Ball b2){

    float res=0;

    res += b1.x - b2.x;
    res += (b1.vx - b2.vx)/fr;

    return res;
}

float Program::calculateAlphaY(Ball b1, Ball b2){

    float res=0;

    res += b1.y - b2.y;
    res += (b1.vy - b2.vy)/fr;

    return res;
}

float Program::calculateBetaX(Ball b1, Ball b2){
    return (b2.vx - b1.vx)/fr;
}

float Program::calculateBetaY(Ball b1, Ball b2){
    return (b2.vy - b1.vy)/fr;
}

float Program::calculateTimeToCollide(Ball b1, Ball b2){
    float alphax = calculateAlphaX(b1,b2);
    float betax = calculateBetaX(b1,b2);

    float alphay = calculateAlphaY(b1,b2);
    float betay = calculateBetaY(b1,b2);

    float a = betax*betax+betay*betay;
    float b = 2*alphax*betax+2*alphay*betay;
    float c = alphax*alphax+alphay*alphay-4*radius*radius; // a,b,c değerleri hazır


    if(a==0){ // betax = 0 betay = 0 means that her iki düzlemde hızları eşit. eğer şu an çakışmıyorlarsa hiç çakışmayacaklar.
        float theta = (-c)/b;
        float t = (-1*log(abs(theta)))/fr;
        return t;
    }


    if ((b*b-4*a*c) < 0){ // they're parallel
        return FMAX;
    }


    float theta1 = (-b+sqrt(b*b-4*a*c))/(2*a);
    float theta2 = (-b-sqrt(b*b-4*a*c))/(2*a);

    float t1 = (-1*log(abs(theta1)))/fr;  // need to checked these values.
    float t2 = (-1*log(abs(theta2)))/fr;  //

    cout << "t1: "<< t1 << "  t2: " << t2 << "\n";

    float t = min(t1,t2)>=0?min(t1,t2):max(t1,t2)>=0?max(t1,t2):FMAX;

    return t;
}

vector<Ball> Program::findFirstCollision(){  // aynı anda olan çarpışmalar için ne yapacağımı tam bilmiyorum şu an
    vector<Ball> colliders;

    float min_time = times[numberofballs-1]-current_time;

    for(int i=0; i<numberofballs-1;i++){
        for(int j=1;j<numberofballs && j>i;j++){
            Ball b1 = balls[i];
            Ball b2 = balls[j];

            float time = calculateTimeToCollide(b1,b2);

            if (time >= 0 && time <= min_time){
                min_time = time;
                colliders.clear();
                colliders.push_back(b1);
                colliders.push_back(b2);     // there will be collision
            } 
        }
    }

    return colliders;
}


void Program::timeSkip(float time){

    if (time == 0) return;

    for(int i=0; i<numberofballs;i++){
        balls[i].x = balls[i].x + balls[i].vx*(1-exp(-fr*time))/fr;
        balls[i].y = balls[i].y + balls[i].vy*(1-exp(-fr*time))/fr;
        balls[i].vx = balls[i].vx/exp(fr*time);
        balls[i].vy = balls[i].vy/exp(fr*time);
    }

    current_time += time;
    if(current_time == times[times.size()-1]){
        times.pop_back();
        printSnapshot(current_time);
    }
}

void Program::collide(Ball b1, Ball b2, float time){ // 
    timeSkip(time);
}

void Program::startSimulation(){

    while(times.size()){
        vector<Ball> colliders = findFirstCollision();

        if(colliders.empty()){ // no collision before input TIME
            timeSkip(times[times.size()-1]-current_time);
        }
        else{
            float min_time = calculateTimeToCollide(colliders[0],colliders[1]);
            collide(colliders[0],colliders[1], min_time);
        }
    }
}

void Program::printSnapshot(float time){
    cout << "---------------"<< "\n";
    cout << "Time: "<< time << "\n";
    for(int b=0;b<balls.size();b++){
        cout << "  Ball" <<b << " x: "<< balls[b].x<< " y: "<<balls[b].y << " vx: "<< balls[b].vx<<" vy: "<< balls[b].vy<<"\n"; 
    }
}


float Program::calculateVelocity(Ball b1){
    float vx = b1.vx;
    float vy = b1.vy;

    return sqrt(vx*vx+vy*vy);
}

float Program::calculateTheta(Ball b1){ // MOVEMENT ANGLE THETA
    float v = calculateVelocity(b1);
    return acos(b1.vx/v);
}


