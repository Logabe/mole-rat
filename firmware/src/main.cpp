#include <MPU6050_6Axis_MotionApps612.h>
#include <Arduino.h>

#define CALIBRATE
#define INTERRUPT_PIN D3

MPU6050 mpu;

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
  while(!Serial);

  delay(1000);

  Serial.println("INITING I2C");
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

  // Serial.println("TESTING MPU");
  // if (mpu.testConnection() == false) {
  //   Serial.println("MPU TEST FAIL");
  //   while(true);
  // } else {
  //   Serial.println("MPU TEST SUCCESS");
  // }

  Serial.println("INITING DMP");
  devStatus = mpu.dmpInitialize();

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
    mpu.CalibrateAccel(60);
    mpu.CalibrateGyro(60);
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

void loop() {
  // delay(10);
  if (!DMPReady) return;

  if (mpu.dmpGetCurrentFIFOPacket(FIFOBuffer)) {
    mpu.dmpGetQuaternion(&q, FIFOBuffer);
    mpu.dmpGetEuler(euler, &q);
    mpu.dmpGetAccel(&aa, FIFOBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
    mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
  }

  Serial.print("Ax:"); Serial.print((double)aaWorld.x / (double)UINT16_MAX); Serial.print(",");
  Serial.print("Ay:"); Serial.print((double)aaWorld.y / (double)UINT16_MAX); Serial.print(",");
  Serial.print("Az:"); Serial.print((double)aaWorld.z / (double)UINT16_MAX); Serial.print("\n");
}