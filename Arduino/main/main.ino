#include "cuadricula.h"

int error;
int integral = 0;
int derivado = 0;
int lastError = 0;


// Ultrasonidos
const int Trigger = 5;   //Pin digital 69 para el Trigger del sensor
const int Echo = 4;   //Pin digital 4 para el echo del sensor
  
MPU6050 mpu(Wire);

// Robot
Robot robot(2, 3, 30, 32, 34, 36);

// Cuadricula
int robs[3] = {3,3,3};
int cobs[3] = {1,2,3};
int nobs = 0;
Cuadricula cuadricula(6, 6, robs, cobs, nobs);

// Planificador
int Recorrido[36];
int Movimientos[36];
int Orientacion[36];
int nRecorrido;
int count = 1;
int ro = 3;
int co = 0;
int rf = 0;
int cf = 2;

void IMUcalibration() {
  digitalWrite(LED_BUILTIN, HIGH); //Encender el led al empezar la calibración
  Wire.begin();
  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while (status != 0) { } // stop everything if could not connect to MPU6050
  Serial.println(F("Calculating offsets, do not move MPU6050"));
  delay(1000);
  mpu.calcOffsets(); // gyro and accelero
  Serial.println("Done!\n");
  digitalWrite(LED_BUILTIN, LOW); //Apagar el led al acabar la calibración
}


float giro_z() { // saca el ángulo de la IMU en z
  mpu.update();
  return mpu.getAngleZ();
}


void giro_imu(bool sentido, int valor_giro) { // false = giro izquierda, true = giro derecha
  float giro_inicial, dif;
  dif = 0;
  giro_inicial = giro_z();
  if (sentido) {
    while (dif < valor_giro)
    {
      dif = abs(giro_z() - giro_inicial);

      //Serial.println(dif);
      robot.derecha(80);
    }
    robot.brake();
    return;
  }
  else {
    while (dif < valor_giro )
    {
      dif = abs(giro_z() - giro_inicial);
      //Serial.println(dif);
      robot.izquierda(80);
    }
    robot.brake();
    return ;
  }
}


int ping(int TriggerPin, int EchoPin) {
   long duration, distanceCm;
   
   digitalWrite(TriggerPin, LOW);  //para generar un pulso limpio ponemos a LOW 4us
   delayMicroseconds(4);
   digitalWrite(TriggerPin, HIGH);  //generamos Trigger (disparo) de 10us
   delayMicroseconds(10);
   digitalWrite(TriggerPin, LOW);
   
   duration = pulseIn(EchoPin, HIGH);  //medimos el tiempo entre pulsos, en microsegundos
   
   distanceCm = duration * 10 / 292/ 2;   //convertimos a distancia, en cm
   if(distanceCm>50){distanceCm=50;}
   return distanceCm;
}


void giro(bool sentido, int tgiro) { // false = giro izquierda, true = giro derecha
  if (sentido) {
    robot.derecha(80);
    delay(tgiro);
    robot.brake();
    return;
  }
  else {
    robot.izquierda(80);
    delay(tgiro);
    robot.brake();
    return ;
  }
}


void setup() {
  Serial.begin(9600);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(Trigger, OUTPUT); //pin como salida
  pinMode(Echo, INPUT);  //pin como entrada
  digitalWrite(Trigger, LOW);//Inicializamos el pin con 0
  robot.QTRcalibration();
  cuadricula.printAvail();
  //IMUcalibration(); // La IMU ha muerto, oremos
  nRecorrido = cuadricula.Planner(ro, co, rf, cf, Recorrido);
  cuadricula.MovGenerator(nRecorrido, Recorrido, Movimientos, Orientacion);
  Serial.print("nRecorrido:");
  Serial.print(nRecorrido);
  Serial.print("\t Resultados:");
  Serial.print(Recorrido[0]);
  Serial.print('\t');
  Serial.print(Recorrido[1]);
  Serial.print('\t');
  Serial.print(Recorrido[2]);
  Serial.print('\t');
  Serial.print(Recorrido[3]);
  Serial.print('\t');
  Serial.print(Recorrido[4]);
  Serial.print('\t');
  Serial.print(Recorrido[5]);
  Serial.print('\t');
  Serial.print(Recorrido[6]);
  Serial.print('\t');
  Serial.print(Recorrido[7]);
  Serial.println();
  
  Serial.print("\t Movimientos:");
  Serial.print(Movimientos[0]);
  Serial.print('\t');
  Serial.print(Movimientos[1]);
  Serial.print('\t');
  Serial.print(Movimientos[2]);
  Serial.print('\t');
  Serial.print(Movimientos[3]);
  Serial.print('\t');
  Serial.print(Movimientos[4]);
  Serial.print('\t');
  Serial.print(Movimientos[5]);
  Serial.print('\t');
  Serial.print(Movimientos[6]);
  Serial.print('\t');
  Serial.print(Movimientos[7]);
  Serial.println();
}

void loop() {
  while (robot.checkIntersection()) {
    robot.siguelineas(&integral, &lastError);
  }
  while (!robot.checkIntersection()) {
    robot.siguelineas(&integral, &lastError);
    /*int dist = ping(Trigger, Echo);
    if (dist < 10) {
      int ori_inicio = Orientacion[count-1];
      cuadricula.Tablero[Recorrido[count]].setAvail(false);
      int posr = cuadricula.Tablero[Recorrido[count-1]].getRow();
      int posc = cuadricula.Tablero[Recorrido[count-1]].getCol();
      nRecorrido = cuadricula.Planner(posr, posc, rf, cf, Recorrido);
      cuadricula.MovGenerator(nRecorrido, Recorrido, Movimientos, Orientacion, ori_inicio);
      Serial.print("\t Movimientos:");
      Serial.print(Movimientos[0]);
      Serial.print('\t');
      Serial.print(Movimientos[1]);
      Serial.print('\t');
      Serial.print(Movimientos[2]);
      Serial.print('\t');
      Serial.print(Movimientos[3]);
      Serial.print('\t');
      Serial.print(Movimientos[4]);
      Serial.print('\t');
      Serial.print(Movimientos[5]);
      Serial.print('\t');
      Serial.print(Movimientos[6]);
      Serial.println();
      count = 0;
      Serial.print("ATRAS");
      while (!robot.checkIntersection()) {
        robot.siguelineasReverse(&integral, &lastError);
      }
    }*/
  }
  digitalWrite(LED_BUILTIN, HIGH); // Encender al detectar intersección
  robot.brake();
  delay(200);
  
  switch (Movimientos[count]){
    case 0:
      Serial.print("SEGUIR");
      robot.brake();
      break;
    case 1:
      Serial.print("IZQUIERDA");
      giro(false, 750);
      break;
    case 2:
      Serial.print("DERECHA");
      giro(true, 750);
      break;
    case 3:
      Serial.print("GIRO 180");
      giro(true, 2000);
      break;
    case 4:
      Serial.print("FINAL");
      robot.adelante(50,50);
      delay(200);
      robot.brake();
      delay(10000);
      break;
    default:
      break;
  }
  Serial.println();
  count++;
  delay(100);
  if (Movimientos[count] == 4) {  
     Serial.print("FINAL");  
      robot.adelante(50,50);
      delay(500);    
      robot.brake();
      delay(1000000);
  }
  else {
    delay(200);
    digitalWrite(LED_BUILTIN, LOW); //Apagar el led al acabar la intersección
  }
  
  
}
