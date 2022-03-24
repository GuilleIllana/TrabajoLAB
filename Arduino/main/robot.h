#ifndef _ROBOT_CHULO_
#define _ROBOT_CHULO_


#include "I2Cdev.h"
#include "Wire.h"
#include <MPU6050_light.h>
#include <QTRSensors.h>

//Velocidades
#define rightMaxSpeed 80 // max speed of the robot (0-255)
#define leftMaxSpeed 80 // max speed of the robot (0-255)
#define rightBaseSpeed 50 // this is the speed at which the motors should spin when the robot is perfectly on the line (0-255)
#define leftBaseSpeed 50  // this is the speed at which the motors should spin when the robot is perfectly on the line (0-255)

//Sensor siguelineas
#define Kp 0.5 // experimentar (valores bajos)
#define Ki 0 // no se ni lo que estoy haciendo
#define Kd 30 // experimentar con valores bajos (Ki < Kp < Kd)

class Robot
{
  public:
    Robot(int en1, int en2, int in1, int in2, int in3, int in4);
    void adelante(int DPWM, int IPWM);
    void atras(int DPWM, int IPWM);
    void brake();
    void derecha(int IPWM);
    void izquierda(int DPWM);
    void siguelineas(int* integral, int* lastError);
    void siguelineasReverse(int* integral, int* lastError);
    bool checkIntersection();
    void QTRcalibration();
    int QTRreadLine();
    
  private:
    int _in1;
    int _in2;
    int _in3;
    int _in4;
    int _en1;
    int _en2;
    QTRSensors qtr;
    
};

#endif
