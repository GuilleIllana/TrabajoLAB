#include "robot.h"
#include "Arduino.h"

Robot::Robot(int en1, int en2, int in1,int in2, int in3, int in4){
    pinMode(en1, OUTPUT);
    _en1 = en1;

    pinMode(en2, OUTPUT);
    _en2 = en2;

    pinMode(in1, OUTPUT);
    _in1 = in1;

    pinMode(in2, OUTPUT);
    _in2 = in2;

    pinMode(in3, OUTPUT);
    _in3 = in3;

    pinMode(in4, OUTPUT);
    _in4 = in4;
}


void Robot::brake() {
  // Direccion motor A
  digitalWrite (_in1, HIGH);
  digitalWrite (_in2, HIGH);
  analogWrite (_en1,0); //Velocidad motor A  Rueda izquierda Rueda debil

  // Direccion motor B
  digitalWrite (_in3, HIGH);
  digitalWrite (_in4, HIGH);
  analogWrite (_en2,0); //Velocidad motor B Rueda de la derecha
}


void Robot::adelante(int DPWM, int IPWM) {
  if (DPWM > rightMaxSpeed) DPWM = rightMaxSpeed; //Prevencion
  if (IPWM > leftMaxSpeed) IPWM = leftMaxSpeed; //Prevencion
  if (DPWM < 0) DPWM = 0; 
  if (IPWM < 0) IPWM = 0;

  // Direccion motor A
  digitalWrite (_in1, HIGH);
  digitalWrite (_in2, LOW);
  analogWrite (_en1,DPWM); //Velocidad motor A  Rueda izquierda Rueda debil

  // Direccion motor B
  digitalWrite (_in3, HIGH);
  digitalWrite (_in4, LOW);
  analogWrite (_en2,IPWM); //Velocidad motor B Rueda de la derecha
}

void Robot::atras(int DPWM, int IPWM) {
  if (DPWM > rightMaxSpeed) DPWM = rightMaxSpeed; //Prevencion
  if (IPWM > leftMaxSpeed) IPWM = leftMaxSpeed; //Prevencion
  if (DPWM < 0) DPWM = 0; 
  if (IPWM < 0) IPWM = 0;

  // Direccion motor A
  digitalWrite (_in1, LOW);
  digitalWrite (_in2, HIGH);
  analogWrite (_en1,DPWM); //Velocidad motor A  Rueda izquierda Rueda debil

  // Direccion motor B
  digitalWrite (_in3, LOW);
  digitalWrite (_in4, HIGH);
  analogWrite (_en2,IPWM); //Velocidad motor B Rueda de la derecha
}

void Robot::izquierda(int DPWM) {
  // Direccion motor A
  digitalWrite (_in1, HIGH);
  digitalWrite (_in2, HIGH);
  analogWrite (_en1,0); //Velocidad motor A  Rueda izquierda Rueda debil

  // Direccion motor B
  digitalWrite (_in3, HIGH);
  digitalWrite (_in4, LOW);
  analogWrite (_en2, DPWM); //Velocidad motor B Rueda de la derecha

}

void Robot::derecha(int IPWM) {
  // Direccion motor A
  digitalWrite (_in1, HIGH);
  digitalWrite (_in2, LOW);
  analogWrite (_en1,IPWM); //Velocidad motor A  Rueda izquierda Rueda debil

  // Direccion motor B
  digitalWrite (_in3, HIGH);
  digitalWrite (_in4, HIGH);
  analogWrite (_en2,0); //Velocidad motor B Rueda de la derecha
}


void Robot::QTRcalibration() {
  // Configure the QTR
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){23, 24, 25, 26, 27, 28, 29, 31}, 8);
  
  delay(500);

  // 2.5 ms RC read timeout (default) * 10 reads per calibrate() call
  // ~25 ms per calibrate() call.
  // Call calibrate() 200 times to make calibration take about 10 seconds.
  digitalWrite(LED_BUILTIN, HIGH); //Encender el led al empezar la calibración
  for (uint16_t i = 0; i < 200; i++)
  {
    qtr.calibrate();
  }
  digitalWrite(LED_BUILTIN, LOW); //Apagar el led al acabar la calibración
}


int Robot::QTRreadLine() {
  uint16_t sensorValues[8];
  return qtr.readLineBlack(sensorValues); //Lectura de la posición de la linea con respecto al robot
}


void Robot::siguelineas(int* integral, int* lastError) {
  int position = QTRreadLine(); //Lectura de la posición de la linea con respecto al robot
  int error =  position - 3500; //El error irá desde +3500 a -3500 si es >0 linea a izq del sensor, si es <0 linea a la dch del sensor
  *integral = *integral + error; 
  int derivado = error - *lastError; 
  
  int vel = Kp * error + Kd * derivado + Ki * *integral;
  *lastError = error; 

  // Asignación de velocidades
  int MotorDPWM = rightBaseSpeed - vel; //Base, modificar según parámetros PID
  int MotorIPWM = leftBaseSpeed + vel; //Base, modificar según parámetros PID
  
  adelante(MotorDPWM, MotorIPWM);
}


void Robot::siguelineasReverse(int* integral, int* lastError) {
  int position = QTRreadLine(); //Lectura de la posición de la linea con respecto al robot
  int error =  position - 3500; //El error irá desde +3500 a -3500 si es >0 linea a izq del sensor, si es <0 linea a la dch del sensor
  *integral = *integral + error; 
  int derivado = error - *lastError; 
  
  int vel = (Kp * error + Kd * derivado + Ki * *integral)*0.7;
  *lastError = error; 

  // Asignación de velocidades
  int MotorDPWM = rightBaseSpeed + vel; //Base, modificar según parámetros PID
  int MotorIPWM = leftBaseSpeed - vel; //Base, modificar según parámetros PID
  
  atras(MotorDPWM, MotorIPWM);
}


bool Robot::checkIntersection() {
  uint16_t sensorValues[8];
  int pos = qtr.readLineBlack(sensorValues);
  if (sensorValues[1] > 400 && sensorValues[7] > 400) return true;
  else return false;
}
