#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Arduino.h>
#include <BleMouse.h>
#include <WiFi.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;
BleMouse mouse("Mole rat");

sensors_vec_t initial;

void setup() {
  Serial.begin(115200);

  mouse.begin();

  pinMode(D1, OUTPUT);
  
  Serial.println("Adafruit MPU6050 test!");

  bool found = false;
  while (!found) {
    // Try to initialize!
    if (mpu.begin()) {
      found = true;
    } else {
      Serial.println("Failed to find MPU6050 chip");
      delay(1000);
    }
  }
    
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);

  Serial.println("");
  delay(100);

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  initial = a.acceleration;
}

double vX, vY = 0;

sensors_vec_t rotateVector(sensors_vec_t base, sensors_vec_t rot) {
  // Do stuff here
}

void loop() {
  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  sensors_vec_t acc = a.acceleration;
  for (int i = 0; i <3; i++) {
    *(acc.v+i) -= *(initial.v+i);
  }

  /* Print out the values */
  Serial.print("Acceleration X: ");
  Serial.print(acc.x);
  Serial.print(", Y: ");
  Serial.print(acc.y);
  Serial.print(", Z: ");
  Serial.print(acc.z);
  Serial.println(" m/s^2");

  Serial.print("Gyro X: ");
  Serial.print(g.gyro.x);
  Serial.print(", Y: ");
  Serial.print(g.gyro.y);
  Serial.print(", Z: ");
  Serial.println(g.gyro.z);

  vX += acc.x * .008;
  vY += acc.y * .008;
  Serial.print("vX: ");
  Serial.print(vX);
  Serial.print(", vY: ");
  Serial.println(vY);

  // double m = (double)millis() / 500.0;
  delay(8);
  // digitalWrite(D1, (byte)(sin(m)* 255.0));
  if(mouse.isConnected()) {
    // mouse.move(cos(m) * 3, sin(m) * -3);
    mouse.move((byte)vX, (byte)vY);
  }
}
