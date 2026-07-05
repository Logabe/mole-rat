#include <Arduino.h>
#include <BleMouse.h>
#include <MPU6050_6Axis_MotionApps612.h>
#include <Wire.h>

#define CALIBRATE
#define INTERRUPT_PIN D3

MPU6050 mpu;
BleMouse mouse("Mole rat");

/* MPU6050 Stuff */
bool DMPReady = false; // Flag for loop
uint8_t MPUIntStatus;
uint8_t devStatus;
uint8_t packetSize;
uint8_t FIFOBuffer[64]; 

Quaternion q; // Quaternion rotation
VectorInt16 aa; // Acceleration
VectorInt16 gy; // Gyro
VectorInt16 aaReal; // Gravity free
VectorInt16 aaWorld; // World frame
VectorFloat gravity; // Gravity vector
float euler[3]; // Euler angles

/* Interrupt */
volatile bool MPUInterrupt = false;
void DMPDataReady() {
  MPUInterrupt = true;
}

void setup() {
  Wire.begin();
  Wire.setClock(400000);
  Serial.begin(115200);

  mouse.begin();
  pinMode(D1, OUTPUT);

  Serial.println("INITING I2C");
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);  
  
  Serial.println("INITING DMP");
  devStatus = mpu.dmpInitialize();
  Serial.println(devStatus);

  mpu.setXGyroOffset(0);
  mpu.setYGyroOffset(0);
  mpu.setZGyroOffset(0);
  mpu.setXAccelOffset(0);
  mpu.setYAccelOffset(0);
  mpu.setZAccelOffset(0);

  if (devStatus == 0) {
    mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_4);
    mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);

    #ifdef CALIBRATE
    mpu.CalibrateAccel(10);
    mpu.CalibrateGyro(10);
    mpu.PrintActiveOffsets();
    // Serial.print("Gyro Offset: "); Serial.print(mpu.getXGyroOffset()); Serial.print(", "); Serial.print(mpu.getYGyroOffset()); Serial.print(", "); Serial.println(mpu.getZGyroOffset());
    // Serial.print("Acc Offset"); Serial.print(mpu.getXAccelOffset()); Serial.print(", "); Serial.print(mpu.getYAccelOffset()); Serial.print(", "); Serial.println(mpu.getZAccelOffset());
    #endif
    

    Serial.println("ENABLING DMP");
    mpu.setDMPEnabled(true);

    Serial.println("ENABLING INT");
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), DMPDataReady, RISING);
    MPUIntStatus = mpu.getIntStatus();

    Serial.println("DMP Ready");
    DMPReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
    Serial.print("Packet size: "); Serial.println(packetSize);
  } else {
    Serial.println("DMP INITIALIZATION FAIL");
  }
}

double vX, vY = 0;

ulong lastTime = 0;

// sensors_vec_t rotateVector(sensors_vec_t base, sensors_vec_t rot) {
//   // Do stuff here
// }

void loop() {
  /* Get new sensor events with the readings */
  if (!DMPReady) return;

  ulong newTime = millis();
  float deltaTime = (float)(newTime - lastTime) / 1000;

  if (mpu.dmpGetCurrentFIFOPacket(FIFOBuffer)) {
    mpu.dmpGetQuaternion(&q, FIFOBuffer);
    mpu.dmpGetEuler(euler, &q);
    mpu.dmpGetAccel(&aa, FIFOBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
    mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
  }

  vX = ((float)aaWorld.x / (float)INT16_MAX) * deltaTime * 5000.0;
  vY = ((float)aaWorld.y / (float)INT16_MAX) * deltaTime * 5000.0;
  if (vX < 1) vX = 0;
  if (vY < 1) vY = 0;
  
  Serial.print("vX: ");
  Serial.print(vX);
  Serial.print(", vY: ");
  Serial.println(vY);

  // double m = (double)millis() / 500.0;
  // digitalWrite(D1, (byte)(sin(m)* 255.0));
  if(mouse.isConnected()) {
    // mouse.move(cos(m) * 3, sin(m) * -3);
    mouse.move((byte)vY, (byte)vX);
  }
  lastTime = newTime;
}
