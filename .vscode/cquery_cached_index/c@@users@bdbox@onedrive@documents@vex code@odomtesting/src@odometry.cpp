#ifndef ODOMETRY_CPP
#define ODOMETRY_CPP

#include "odometry.hpp"


////////////////////////////////////////////////////////////
////                  Constant Section                  ////
////      Edit these constants to fit your robot.       ////
////    They will be different for every bot and are    ////
////   VERY important to get right. Units are inches.   ////
////////////////////////////////////////////////////////////


float Sl = 5.125; //distance from tracking center to middle of left wheel
float Sr = 5.125; //distance from tracking center to middle of right wheel
float Ss = 7.75; //distance from tracking center to middle of the tracking wheel
float wheelDiameter = 4.125; //diameter of the side wheels being used for tracking
float trackingDiameter = 4.125; //diameter of the sideways tracking wheel




float x = 0;
float y = 0;
float angle = 0;;

float lastLeftPos = 0;
float lastRightPos = 0;
float lastSidePos = 0;

float deltaTheta = 0;
float thetaNew = 0;
float thetaM = 0;



float curLeft = 0;
float curRight = 0;
float curSide = 0;

float leftAtReset = 0;
float rightAtReset = 0;
float thetaReset = 0;

float deltaLeft = 0;
float deltaRight = 0;
float deltaSide = 0;

float deltaLr = 0;
float deltaRr = 0;

float deltaX = 0;
float deltaY = 0;


float theta = 0;
float radius = 0;

void updatePosition() {
  curLeft = LD.get_position();
  curRight = RD.get_position(); //step 1
  curSide = sideEnc.get_value();

  deltaLeft = (curLeft - lastLeftPos)*(M_PI/180)*(wheelDiameter/2);
  deltaRight = (curRight - lastRightPos)*(M_PI/180)*(wheelDiameter/2); //step 2
  deltaSide = (curSide - lastSidePos)*(M_PI/180)*(trackingDiameter/2);

  lastLeftPos = curLeft;
  lastRightPos = curRight; //step 3
  lastSidePos = curSide;

  deltaLr = (curLeft - leftAtReset)*(M_PI/180)*(wheelDiameter/2); //step 4
  deltaRr = (curRight - rightAtReset)*(M_PI/180)*(wheelDiameter/2);

  thetaNew = (thetaReset + (deltaLr - deltaRr)/(Sl + Sr)); //step 5
  // if (thetaNew >= M_PI) {
  //   thetaNew-=2*M_PI;
  // }
  // else if (thetaNew <= -M_PI) {
  //   thetaNew+=2*M_PI;
  // }



  deltaTheta = thetaNew - angle; //step 6

  deltaSide = deltaSide-Ss*deltaTheta;

  if (deltaTheta == 0) {
    deltaX = deltaSide; //step 7
    deltaY = deltaRight;
  }
  else {
    deltaX = (2*sin(deltaTheta/2))*(deltaSide/deltaTheta + Ss); //step 8
    deltaY = (2*sin(deltaTheta/2))*(deltaRight/deltaTheta +Sr);
  }

  thetaM = angle + deltaTheta/2; //step 9

  // if (deltaX == 0) {
  //   if (deltaY > 0) {
  //     theta = M_PI/2;
  //   }
  //   else if (deltaY < 0) {
  //     theta = 3*M_PI/2;
  //   }
  //   else {
  //     theta = 0;
  //   }
  // }
  // else {
  //   theta = atan(deltaY/deltaX);
  // }
  theta = atan2f(deltaY, deltaX);
  radius = sqrt(deltaX*deltaX + deltaY*deltaY);
  theta = theta-thetaM;                          //step 10
  deltaX = radius*cos(theta);
  deltaY = radius*sin(theta);

  thetaNew+=M_PI;
  while (thetaNew <= 0) {
    thetaNew+=2*M_PI;
  }
  thetaNew = modulo(thetaNew, 2*M_PI);
  thetaNew-=M_PI;

  angle = thetaNew;
  x = x - deltaX; //step 11
  y = y + deltaY;
}

float getX() {
  return x;
}

float getY() {
  return y;
}

float getAngleDegrees() {
  return angle*180/M_PI;
}
float getAngle() {
  return angle;
}

float modulo(float a, float b) {
  while (a>b) {
    a-=b;
  }
  return a;
}

#endif
