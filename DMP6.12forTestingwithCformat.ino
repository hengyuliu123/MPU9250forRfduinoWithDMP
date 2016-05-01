#include <Wire.h>
#include "mpu.h"

static int ret;
static unsigned long preTimesMS = 0;
#define STATUS_LED  2 //LED status light
#define INT_PIN  3 // MPU interrupt pin
volatile bool mmaInterrupt;

void setup() {
  /* Initialize serial port */
  Serial.begin(115200);    //SET UP START BAUD RATE
  Wire.speed = 400;
  Wire.beginOnPins(4, 5);   //SCL PIN4,SDA PIN 5
 /*Initialize the DMP and configuration */
  ret = mympu_open();
  Serial.print("Debug code MPU init: "); Serial.println(ret);
  pinMode(STATUS_LED, OUTPUT);
  digitalWrite(STATUS_LED,HIGH);
}

void loop() {
 ret = mympu_update();
 if( ret == 0){
 // Serial.print(ret);
  
// Serial.println(ret);

unsigned long timeMoments = millis();
timeMoments = timeMoments- preTimesMS;
  //    Serial.print(" Gyro-x: "); 
  //    Serial.print(mympu.gyro[0]);Serial.print("  ");
  //    Serial.print(" Gyro-y: "); 
  //    Serial.print(mympu.gyro[1]);Serial.print("  ");
  //    Serial.print(" Gyro-z: "); 
  //   Serial.print(mympu.gyro[2]);Serial.println("  ");
  //  Serial.print(mympu.steps);Serial.println("  ");
  //   Serial.print(mympu.fifoRate);Serial.println("  ");
  //    Serial.print("\tAccel-x: "); 
   //   Serial.print(mympu.accel[0]);Serial.print("  ");
  //    Serial.print(" Accel-y: "); 
  //    Serial.print(mympu.accel[1]);Serial.print("  ");
  //    Serial.print(" Accel-z: "); 
  //   Serial.print(mympu.accel[2]);Serial.print("  ");
   //   Serial.print(" fifocount: "); Serial.println(mympu.fifocounter);
 //   Serial.print("TIME: "); Serial.println(timeMoments);
}
}

