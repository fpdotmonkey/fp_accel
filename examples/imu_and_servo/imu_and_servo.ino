#include <Servo.h>
#include <Wire.h>
#include <fp_accel.h>

fp compass;
Servo tracker;

int pos = 0;

int testReg(byte address, int reg) {
  Wire.beginTransmission(address);
  Wire.write((byte)reg);
  if (Wire.endTransmission() != 0) {
    return -1;
  }

  Wire.requestFrom(address, (byte)1);
  if (Wire.available()) {
    return Wire.read();
  }
  else {
    return -1;
  }
}

void setup() {
  Serial.begin(9600);
  Wire.begin();
  compass.init();
  tracker.attach(9);  
}

void loop() {
  compass.read();
  int heading = int(compass.heading(fp::vector_d{ 1, 0, 0 }));
  
  //Serial.println(heading);
  tracker.write(pos % 180);
  delay(100);
}
