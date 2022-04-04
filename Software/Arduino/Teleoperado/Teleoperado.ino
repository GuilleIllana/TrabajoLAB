#include <PinChangeInt.h>
#include <PinChangeIntConfig.h>
#include <EEPROM.h>
#define _NAMIKI_MOTOR   //for Namiki 22CL-103501PG80:1
#include <fuzzy_table.h>
#include <PID_Beta6.h>
#include <MotorWheel.h>
#include <Omni4WD.h>

#include <fuzzy_table.h>
#include <PID_Beta6.h>


#include <ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Int32MultiArray.h>
/*

            \                    /
   wheel1   \                    /   wheel4
   Left     \                    /   Right
    
    
                              power switch
    
            /                    \
   wheel2   /                    \   wheel3
   Right    /                    \   Left

 */

irqISR(irq1,isr1);
MotorWheel wheel1(3,2,4,5,&irq1);

irqISR(irq2,isr2);
MotorWheel wheel2(11,12,14,15,&irq2);

irqISR(irq3,isr3);
MotorWheel wheel3(9,8,16,17,&irq3);

irqISR(irq4,isr4);
MotorWheel wheel4(10,7,18,19,&irq4);


Omni4WD Omni(&wheel1,&wheel2,&wheel3,&wheel4);


void joyCallback(const std_msgs::Int32MultiArray& joy)
{
  
  int a = joy.data[0];
  int b = joy.data[1];
  int x = joy.data[2];
  int y = joy.data[3];
  bool stopCar = false;
  
  float rad = 0;
  
  if (y==1) {
    rad = PI/2;
  }
  else if(b == 1) {
    rad = 0;
  }
  else if(a == 1) {
    rad = -PI/2;
  }
  else if(x == 1) {
    rad = PI;
  }
  else {
      stopCar = true;
      Omni.setCarStop();
      Omni.delayMS(500);
  }

  if (!stopCar) {
    Omni.setCarMove(1500,rad,0);
    Omni.delayMS(200);
  }
  
}


//ros::NodeHandle  nh;
ros::NodeHandle_<ArduinoHardware, 5, 5, 128, 128> nh;
ros::Subscriber<std_msgs::Int32MultiArray> sub("joyarduino", joyCallback);

void setup() {
  //TCCR0B=TCCR0B&0xf8|0x01;    // warning!! it will change millis()
  TCCR1B=TCCR1B&0xf8|0x01;    // Pin9,Pin10 PWM 31250Hz
  TCCR2B=TCCR2B&0xf8|0x01;    // Pin3,Pin11 PWM 31250Hz
    
  Omni.PIDEnable(0.31,0.01,0,10);

  // Inicialización del nodo
  nh.initNode();

  // Lectura
  nh.subscribe(sub);
}

void loop() {
  // Omni.demoActions(30,1500,500,false);
  //Omni.setCarMove(300,rad,0);
  //Omni.delayMS(5000,false);
  nh.spinOnce();
}
