#include <iostream>
#include <bits/stdc++.h>
#include <cmath>
#include "nlohmann/json.hpp"
#include <fstream>
#include <queue>
#include <stack>

const double THRESHOLD = 0.00001;
#define FMAX 340282346638528859811704183484516925440.0000000000000000

using json = nlohmann::json;
using namespace std;

struct Ball{
    double x;
    double y;
    double vx;
    double vy;
};

struct Table{
    double width;
    double height;
    double fr;
};

class Program{

    public:
        bool testmode = false;
        double current_time=0;
        int numberofballs=0;
        double radius;
        vector<vector<double>> collisionBtwnBallsTimes;
        vector<vector<double>> collisionWithWallTimes = vector<vector<double>>(4,vector<double>());
        Table table;
        vector<Ball> balls;
        queue<double> times;
        json out;

        vector<vector<int>> stackofcolliders;  
        double min_time_to_collide=FMAX;

        Program(bool test = false){
            ifstream f("initial-state.json");
            json inp = json::parse(f);

            testmode = test;

            table.fr = inp["table"]["deacceleration"];
            table.height = inp["table"]["height"];
            table.width = inp["table"]["width"];

            radius = inp["ball"]["radius"];

            numberofballs = inp["balls"].size();
            for(int i=0; i<numberofballs; i++){
                Ball temp;
                temp.x = inp["balls"][i]["position"]["x"];
                temp.y = inp["balls"][i]["position"]["y"];
                temp.vx = inp["balls"][i]["velocity"]["x"];
                temp.vy = inp["balls"][i]["velocity"]["y"];
                balls.push_back(temp);
            }

            collisionBtwnBallsTimes.resize(numberofballs,vector<double>());
            for(int i=0;i<numberofballs;i++){collisionBtwnBallsTimes[i].resize(numberofballs,-FMAX);}
            for(int i=0;i<4;i++){collisionWithWallTimes[i].resize(numberofballs,-FMAX);}

            ifstream t("snapshot-times.txt");

            string line;
            try{
                while(getline(t,line)){
                    double time = stoi(line);
                    times.push(time);
                }
            }catch(exception& e){
                cerr << "It must be given time input";
            }

            out["positions"] = vector<vector<json>>();
        }
        
        double calculateTimeToCollide(int i, int j);
        void findClosestCollision();
        void timeSkip(double time); // skips time until closest collision
        void collide(int index1, int index2, double time);
        void startSimulation();

        void printSnapshot();
        void exportSnapshot();
        void collideWithWall(int index, double wall,double time);
        double calculateTimeToCollideWall(int i, double wall);
        vector<double> calculateWallCollide(int i);

        void printStack(){
            cout << "--------------------------\n";
            for(int i=0;i<stackofcolliders.size();i++){
                for(int j=0;j<2;j++){
                    cout << stackofcolliders[i][j];
                }
                cout <<" ";
            }
            cout << "--------------------------\n";
        }
};

double Program::calculateTimeToCollide(int i, int j){
    double fr = table.fr;
    double alphax,alphay,betax,betay;
    double a,b,c;
    double theta1, theta2;
    double t1,t2,t;

    if(fr != 0.0){
        alphax = balls[i].x - balls[j].x + (balls[i].vx-balls[j].vx)/fr;
        betax = (balls[j].vx - balls[i].vx)/fr;
        alphay = balls[i].y - balls[j].y + (balls[i].vy-balls[j].vy)/fr;
        betay = (balls[j].vy - balls[i].vy)/fr;

        a = betax*betax+betay*betay;
        b = 2*alphax*betax+2*alphay*betay;
        c = alphax*alphax+alphay*alphay-4*radius*radius; 


        if(a==0){ 
            if(b==0){
                return FMAX;
            }
            else{ // ? This is probably unnecessary, it can be removed.
                double theta = (-c)/b;
                t = (-1*log(abs(theta)))/fr;
                return t;
            }
            
        }


        if ((b*b-4*a*c) < 0){ // ? no real solution
            return FMAX;
        }


        theta1 = (-b+sqrt(b*b-4*a*c))/(2*a);
        theta2 = (-b-sqrt(b*b-4*a*c))/(2*a);

        t1 = theta1>0?(-1*log(theta1))/fr+0.0:FMAX; 
        t2 = theta2>0?(-1*log(theta2))/fr+0.0:FMAX; 

        if (t1 == t2){ // ! they will cross each other -> no collision. tangent
            return FMAX;
        }

        if( abs(t1)<THRESHOLD || abs(t2)<THRESHOLD ){
            t = 0;
        }
        else{
            t = min(t1,t2)>=0?min(t1,t2):max(t1,t2)>=0?max(t1,t2):FMAX;
        }
    }
    else{
        alphax = balls[j].vx - balls[i].vx;
        alphay = balls[j].vy - balls[i].vy;
        betax = balls[j].x - balls[i].x;
        betay = balls[j].y - balls[i].y;

        a = alphax*alphax + alphay*alphay;
        b = 2*alphax*betax + 2*alphay*betay;
        c = betax*betax + betay*betay - 4*radius*radius;

        if (a == 0){
            if(b==0){
                return FMAX;
            }
            else{  // ? This is probably unnecessary, it can be removed.
                t = -c/b;
                return t;
            }
            
        }

        if ((b*b-4*a*c) < 0){ // ? they're parallel
            return FMAX;
        }

        t1 = (-b+sqrt(b*b-4*a*c))/(2*a);
        t2 = (-b-sqrt(b*b-4*a*c))/(2*a);

        

        if (t1 == t2){ // ? they will cross each other -> no collision. tangent
            return FMAX;
        }

        if( abs(t1)<THRESHOLD || abs(t2)<THRESHOLD ){
            t = 0;
        }
        else{
            t = min(t1,t2)>=0?min(t1,t2):max(t1,t2)>=0?max(t1,t2):FMAX;
        }
        
    }    
    return t;
}

void Program::findClosestCollision(){ 
    vector<int> colliders;
    double time;

    min_time_to_collide = times.front()-current_time;

    for(int i=0; i<numberofballs;i++){

        vector<double> information = calculateWallCollide(i);
        
        time = information[0];
        int wall = information[1];



        if(wall != -1 && (collisionWithWallTimes[wall][i] != current_time)){
            if(time < min_time_to_collide){
                min_time_to_collide = time;
                colliders.clear();
                colliders.push_back(i);
                colliders.push_back(information[1]+1000);
                stackofcolliders.clear();
                stackofcolliders.push_back(colliders);
            }
            else if(time == min_time_to_collide){
                colliders.clear();
                colliders.push_back(i);
                colliders.push_back(information[1]+1000);
                stackofcolliders.push_back(colliders);
            }
        }

        for(int j=i+1;j<numberofballs && j>i;j++){

            if(collisionBtwnBallsTimes[i][j] == current_time){ // ? if they've already collided, then continue.
                continue;
            }

            time = calculateTimeToCollide(i,j);

            if (time >= 0 && time < min_time_to_collide){
                min_time_to_collide = time;   // ? there will be collision

                colliders.clear();
                colliders.push_back(i);
                colliders.push_back(j);
                stackofcolliders.clear();
                stackofcolliders.push_back(colliders);
            }
            else if(time >= 0 && time == min_time_to_collide){
                colliders.clear();
                colliders.push_back(i);
                colliders.push_back(j);
                stackofcolliders.push_back(colliders);
            }
        }
    }
}


void Program::timeSkip(double time){
    double fr = table.fr;
    
    if (time+0.0 < THRESHOLD) return;

    if(fr != 0.0){ // ? if there is deacceleration, update velocities and positions.
        for(int i=0; i<numberofballs;i++){
            balls[i].x = balls[i].x + balls[i].vx*(1-exp(-fr*time))/fr;
            balls[i].y = balls[i].y + balls[i].vy*(1-exp(-fr*time))/fr;
            balls[i].vx = balls[i].vx/exp(fr*time);
            balls[i].vy = balls[i].vy/exp(fr*time);
        }
    }
    else{
        for(int i=0; i<numberofballs;i++){ // ? update positions only. velocities are constant
            balls[i].x = balls[i].x + balls[i].vx*time;
            balls[i].y = balls[i].y + balls[i].vy*time;
        }
    }

    current_time += time;
    if(current_time == times.front()){
        times.pop();
        if(testmode) printSnapshot();
        exportSnapshot();
    }
}

void Program::collide(int index1, int index2, double time){ //

    //timeSkip(time);

    Ball b1 = balls[index1];
    Ball b2 = balls[index2];
    
    if(testmode){
        cout << "Ball"<<index1<<" ve "<<"Ball"<<index2<<"çarpışmadan önce : ";
        printSnapshot();
    }

    collisionBtwnBallsTimes[index1][index2] = current_time;
    collisionBtwnBallsTimes[index2][index1] = current_time;



    double theta1 = atan2(b1.vy, b1.vx);
    double theta2 = atan2(b2.vy, b2.vx);
    double phi = M_PI-atan( (b2.x-b1.x) / (b2.y-b1.y) );

    double v1 = sqrt(b1.vx*b1.vx+b1.vy*b1.vy); // ? magnitude of velocities
    double v2 = sqrt(b2.vx*b2.vx+b2.vy*b2.vy); 

    double v1fx = v2*cos(theta2-phi)*cos(phi)+v1*sin(theta1-phi)*cos(phi+M_PI/2);
    double v1fy = v2*cos(theta2-phi)*sin(phi)+v1*sin(theta1-phi)*sin(phi+M_PI/2);

    double v2fx = v1*cos(theta1-phi)*cos(phi)+v2*sin(theta2-phi)*cos(phi+M_PI/2);
    double v2fy = v1*cos(theta1-phi)*sin(phi)+v2*sin(theta2-phi)*sin(phi+M_PI/2);

    balls[index1].vx = abs(v2fx-balls[index1].vx)>THRESHOLD?v2fx:balls[index1].vx;
    balls[index1].vy = abs(v2fy-balls[index1].vy)>THRESHOLD?v2fy:balls[index1].vy;
    balls[index2].vx = abs(v1fx-balls[index2].vx)>THRESHOLD?v1fx:balls[index2].vx;
    balls[index2].vy = abs(v1fy-balls[index2].vy)>THRESHOLD?v1fy:balls[index2].vy;

    if(testmode){
        cout << "Ball"<<index1<<" ve "<<"Ball"<<index2<<"çarpıştıktan sonra: ";
        printSnapshot();
    }

}

void Program::startSimulation(){
    ofstream o("output.json");

    while(!times.empty()){
        findClosestCollision();
        
        if(testmode)printStack();

        if(stackofcolliders.empty()){
            timeSkip(times.front()-current_time);
        }
        else{
            timeSkip(min_time_to_collide);
            while(stackofcolliders.empty() == false){
                vector<int> colliders = stackofcolliders[stackofcolliders.size()-1];
                if (colliders[1] < 1000){
                    collide(colliders[0],colliders[1], min_time_to_collide);
                }
                else{ // duvarla çarpışma
                    collideWithWall(colliders[0],colliders[1]-1000,min_time_to_collide);
                }
                stackofcolliders.pop_back();
            }
        }
        
    }

    o << std::setw(4) << out << std::endl;
}

void Program::printSnapshot(){
    cout << "---------------"<< "\n";
    cout << "Time: "<< current_time << "\n";
    for(int b=0;b<balls.size();b++){
        std::cout << "  Ball" << b << " x: " << std::fixed << std::setprecision(2) << balls[b].x
                      << " y: " << std::fixed << std::setprecision(6) << balls[b].y
                      << " vx: " << std::fixed << std::setprecision(6) << balls[b].vx
                      << " vy: " << std::fixed << std::setprecision(6) << balls[b].vy << "\n";
    }
}

void Program::exportSnapshot(){
    vector<json> datas;

    for(int i=0;i<numberofballs;i++){
        json data;
        data["x"] = balls[i].x;
        data["y"] = balls[i].y;
        datas.push_back(data);
    }

    out["positions"].push_back(datas);
}

vector<double> Program::calculateWallCollide(int i){ 
    double wall=-1; // * KEY
    double time = FMAX;
    
    double top = (balls[i].vy > 0)?calculateTimeToCollideWall(i,0):FMAX;
    double right = (balls[i].vx > 0)?calculateTimeToCollideWall(i,1):FMAX;
    double bottom = (balls[i].vy < 0)?calculateTimeToCollideWall(i,2):FMAX;
    double left = (balls[i].vx < 0)?calculateTimeToCollideWall(i,3):FMAX;

    if(time > top && (top+0.0) >=0) {time = top;wall = 0;} 
    if(time > right && (right+0.0) >=0) {time = right;wall = 1;} 
    if(time > bottom && (bottom+0.0) >=0 ) {time = bottom;wall = 2;} 
    if(time > left && (left+0.0) >=0) {time = left;wall = 3;}

    vector<double> info;
    info.push_back(time);
    info.push_back(wall);

    return info;

}

double Program::calculateTimeToCollideWall(int i, double wall){ 
    double value;
    Ball b1 = balls[i];
    double fr = table.fr;
    if(fr != 0){
        if(wall == 0){
            value = -fr*(table.height-radius-b1.y-b1.vy/fr)/b1.vy;
            return value>0?(-log(value)/fr):FMAX;
        }
        else if(wall == 1){
            value = -fr*(table.width-radius-b1.x-b1.vx/fr)/b1.vx;
            return value>0?-log(value)/fr:FMAX;
        }
        else if(wall == 2){
            value = -fr*(radius-b1.y-b1.vy/fr)/b1.vy;
            return value>0?-log(value)/fr:FMAX;
        }
        else {
            value = -fr*(radius-b1.x-b1.vx/fr)/b1.vx;
            return value>0?-log(value)/fr:FMAX;
        }
    }
    else{
        if(wall == 0){
            return (table.height-radius-b1.y)/b1.vy;
        }
        else if(wall == 1){
            return (table.width-radius-b1.x)/b1.vx;
        }
        else if(wall == 2){
            return (radius-b1.y)/b1.vy;
        }
        else {
            return (radius-b1.x)/b1.vx;
        }
    }
    
}

void Program::collideWithWall(int index, double wall, double time){

    collisionWithWallTimes[wall][index] = current_time;
    
    if(testmode){
        cout << "Duvarla çarpışmadan önce : ";
        printSnapshot();
    }

    if(wall == 0){
        balls[index].vy = -balls[index].vy;
    }
    else if(wall == 1){
        balls[index].vx = -balls[index].vx;
    }
    else if(wall == 2){
        balls[index].vy = -balls[index].vy;
    }
    else {
        balls[index].vx = -balls[index].vx;
    }

    if(testmode){
        cout << "Duvarla çarpışmadan sonra : ";
        printSnapshot();
    }
    
}



