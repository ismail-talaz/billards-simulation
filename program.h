#include <iostream>
#include <bits/stdc++.h>
#include <cmath>

#define FMAX 340282346638528859811704183484516925440.0000000000000000


using namespace std;

struct Ball{
    float x;
    float y;
    float vx;
    float vy;
};

struct Table{
    float width;
    float height;
};


class Program{

    public:

        unordered_set<int> lastColliders;
        float fr=0.1; // deacceleration rate.    
        float current_time=0;
        int numberofballs=0;
        float radius=1.0;
        Table table;
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

            table.width=150;
            table.height=150;
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
        vector<int> findFirstCollision();
        void timeSkip(float time);
        void collide(Ball & b1, Ball & b2, float time);
        void startSimulation();
        void printSnapshot(float time);
        float calculateVelocity(Ball b1);
        float calculateTheta(Ball b1);
        float calculatePhi(Ball b1);
        void collideWithWall(Ball &b, float wall,float time);
        float calculateTimeToCollideWall(Ball b, float wall);
        vector<float> calculateWallCollide(Ball b);
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

    float t = min(t1,t2)>=0?min(t1,t2):max(t1,t2)>=0?max(t1,t2):FMAX;

    return t;
}

vector<int> Program::findFirstCollision(){  // aynı anda olan çarpışmalar için ne yapacağımı tam bilmiyorum şu an
    vector<int> colliders;
    float time;

    float min_time = times[times.size()-1]-current_time;

    for(int i=0; i<numberofballs;i++){

        Ball b1 = balls[i];
        
        vector<float> information = calculateWallCollide(b1);
        
        time = information[0];

        if(time <= min_time && (lastColliders.find(i)==lastColliders.end() || lastColliders.find(information[1])==lastColliders.end())){
            min_time = time;
            colliders.clear();
            colliders.push_back(i);
            colliders.push_back(information[1]);     // there will be collision

            lastColliders.clear();
            lastColliders.insert(i);
            lastColliders.insert(information[1]);
        }
        

        for(int j=i+1;j<numberofballs && j>i;j++){

            if(lastColliders.find(i)!=lastColliders.end() && lastColliders.find(j)!=lastColliders.end()){
                continue;
            }

            
            Ball b2 = balls[j];

            time = calculateTimeToCollide(b1,b2);

            if (time >= 0 && time <= min_time){
                min_time = time;
                colliders.clear();
                colliders.push_back(i);
                colliders.push_back(j);     // there will be collision

                lastColliders.clear();
                lastColliders.insert(i);
                lastColliders.insert(j);
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

void Program::collide(Ball & b1, Ball & b2, float time){ // 
    timeSkip(time);
    
    cout << "Çarpışmadan önce : ";
    printSnapshot(current_time);

    double theta1 = calculateTheta(b1);
    double theta2 = calculateTheta(b2);
    double phi = M_PI-atan( (b2.x-b1.x) / (b2.y-b1.y) );

    float v1 = calculateVelocity(b1);
    float v2 = calculateVelocity(b2);

    float v1fx = v2*cos(theta2-phi)*cos(phi)+v1*sin(theta1-phi)*cos(phi+M_PI/2);
    float v1fy = v2*cos(theta2-phi)*sin(phi)+v1*sin(theta1-phi)*sin(phi+M_PI/2);

    float v2fx = v1*cos(theta1-phi)*cos(phi)+v2*sin(theta2-phi)*cos(phi+M_PI/2);
    float v2fy = v1*cos(theta1-phi)*sin(phi)+v2*sin(theta2-phi)*sin(phi+M_PI/2);

    b1.vx = v2fx;
    b1.vy = v2fy;
    b2.vx = v1fx;
    b2.vy = v1fy;

    cout << "Çarpışmadan sonra : ";
    printSnapshot(current_time);
    
    timeSkip(0.00001);

}

void Program::startSimulation(){

    while(times.size()){
        vector<int> colliders = findFirstCollision();

        if(colliders.empty()){ // no collision before input TIME
            timeSkip(times[times.size()-1]-current_time);
        }
        else if (colliders[1] >= 0){
            float min_time = calculateTimeToCollide(balls[colliders[0]],balls[colliders[1]]);
            collide(balls[colliders[0]],balls[colliders[1]], min_time);
        }
        else{ // duvarla çarpışma
            float min_time = calculateTimeToCollideWall(balls[colliders[0]],colliders[1]);
            collideWithWall(balls[colliders[0]],colliders[1],min_time);
        }
    }
}

void Program::printSnapshot(float time){
    cout << "---------------"<< "\n";
    cout << "Time: "<< time << "\n";
    for(int b=0;b<balls.size();b++){
        std::cout << "  Ball" << b << " x: " << std::fixed << std::setprecision(2) << balls[b].x
                      << " y: " << std::fixed << std::setprecision(6) << balls[b].y
                      << " vx: " << std::fixed << std::setprecision(6) << balls[b].vx
                      << " vy: " << std::fixed << std::setprecision(6) << balls[b].vy << "\n";
    }
}


float Program::calculateVelocity(Ball b1){
    float vx = b1.vx;
    float vy = b1.vy;

    return sqrt(vx*vx+vy*vy);
}

float Program::calculateTheta(Ball b1){ // MOVEMENT ANGLE THETA
    float v = calculateVelocity(b1);

    if(v==0){
        return 0;
    }

    if(b1.vx != 0){
        return acos(b1.vx/v);
    }
    if(b1.vy != 0){
        return asin(b1.vy/v);
    }
}


vector<float> Program::calculateWallCollide(Ball b){
    float wall;
    float time = FMAX;
    
    float top = b.vy!=0?calculateTimeToCollideWall(b,-1):FMAX;
    float right = b.vx!=0?calculateTimeToCollideWall(b,-2):FMAX;
    float bottom = b.vy!=0?calculateTimeToCollideWall(b,-3):FMAX;
    float left = b.vx!=0?calculateTimeToCollideWall(b,-4):FMAX;

    cout << top <<" " <<right <<" "<< bottom <<" " <<left<<"\n";

    if(time > top && top >0) {time = top;wall = -1;} 
    if(time > right && right >0) {time = right;wall = -2;} 
    if(time > bottom && bottom >0 ) {time = bottom;wall = -3;} 
    if(time > left && left >0) {time = left;wall = -4;}

    vector<float> info;
    info.push_back(time);
    info.push_back(wall);

    return info;

}

float Program::calculateTimeToCollideWall(Ball b1, float wall){
    if(wall == -1){
        return -log(-fr*(table.height-radius-b1.y-b1.vy/fr)/b1.vy)/fr;
    }
    else if(wall == -2){
        return -log(-fr*(table.width-radius-b1.x-b1.vx/fr)/b1.vx)/fr;
    }
    else if(wall == -3){
        return -log(-fr*(radius-b1.y-b1.vy/fr)/b1.vy)/fr;
    }
    else {
        return -log(-fr*(radius-b1.x-b1.vx/fr)/b1.vx)/fr;
    }
}

void Program::collideWithWall(Ball &b, float wall, float time){

    timeSkip(time);
    
    cout << "Duvarla çarpışmadan önce : ";
    printSnapshot(current_time);

    if(wall == -1){
        b.vy = -b.vy;
    }
    else if(wall == -2){
        b.vx = -b.vx;
    }
    else if(wall == -3){
        b.vy = -b.vy;
    }
    else {
        b.vx = -b.vx;
    }

    cout << "Duvarla çarpışmadan sonra : ";
    printSnapshot(current_time);
    
    timeSkip(0.00001);
}



