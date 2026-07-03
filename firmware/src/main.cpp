#include <Arduino.h>
#include <BleMouse.h>
// #include <esp_now.h>
#include <WiFi.h>
#include <Wire.h>
#include "MPU6050_6Axis_MotionApps20.h"

MPU6050 mpu;
#define OUTPUT_READABLE_REALACCEL
VectorInt16 aaReal;
BleMouse mouse;

/*---MPU6050 Control/Status Variables---*/
bool DMPReady = false;  // Set true if DMP init was successful
uint8_t MPUIntStatus;   // Holds actual interrupt status byte from MPU
uint8_t devStatus;      // Return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // Expected DMP packet size (default is 42 bytes)
uint8_t FIFOBuffer[64]; // FIFO storage buffer

// Structure example to receive data
// Must match the sender structure
typedef struct struct_message {
    char a[32];
    int b;
    float c;
    bool d;
} struct_message;

// Create a struct_message called myData
struct_message myData;

// callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));
  Serial.print("Bytes received: ");
  Serial.println(len);
  Serial.print("Char: ");
  Serial.println(myData.a);
  Serial.print("Int: ");
  Serial.println(myData.b);
  Serial.print("Float: ");
  Serial.println(myData.c);
  Serial.print("Bool: ");
  Serial.println(myData.d);
  Serial.println();
}

void setup() {
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment on this line if having compilation difficulties
  Serial.begin(115200);

  mouse.begin();

  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  // pinMode(INTERRUPT_PIN, INPUT);

  /*Verify connection*/
  Serial.println(F("Testing MPU6050 connection..."));
  if(mpu.testConnection() == false){
    Serial.println("MPU6050 connection failed");
    while(true);
  }
  else {
    Serial.println("MPU6050 connection successful");
  }

  devStatus = mpu.dmpInitialize();

  if (devStatus == 0) {
    mpu.CalibrateAccel(6);  // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateGyro(6);
    Serial.println("These are the Active offsets: ");
    mpu.PrintActiveOffsets();
    Serial.println(F("Enabling DMP..."));   //Turning ON DMP
    mpu.setDMPEnabled(true);

    /*Enable Arduino interrupt detection*/
    // Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
    // Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
    // Serial.println(F(")..."));
    // attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), DMPDataReady, RISING);
    MPUIntStatus = mpu.getIntStatus();

    /* Set the DMP Ready flag so the main loop() function knows it is okay to use it */
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    DMPReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize(); //Get expected DMP packet size for later comparison
  } 
  else {
    Serial.print(F("DMP Initialization failed (code ")); //Print the error code
    Serial.print(devStatus);
    Serial.println(F(")"));
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
  }

  pinMode(D1, OUTPUT);
  
  // // Set device as a Wi-Fi Station
  // WiFi.mode(WIFI_STA);

  // // Init ESP-NOW
  // if (esp_now_init() != ESP_OK) {
  //   Serial.println("Error initializing ESP-NOW");
  //   return;
  // }
  
  // // Once ESPNow is successfully Init, we will register for recv CB to
  // // get recv packer info
  // esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);           // PWR_MGMT_1 register
  Wire.write(0x00);           // Wake up MPU6050 (set sleep = 0)
  Wire.endTransmission(true);

  // Set accelerometer range to ±2g (most stable)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1C);           // ACCEL_CONFIG register
  Wire.write(0x00);           // ±2g range
  Wire.endTransmission(true);

  // Set sample rate (optional)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x19);           // SMPLRT_DIV register
  Wire.write(0x07);           // Sample rate = 1kHz / (7+1) = 125Hz
  Wire.endTransmission(true);
}

int16_t AccX, AccY, AccZ;

void loop() {

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 6, true);

  // Read raw data (16-bit signed)
  AccX = (Wire.read() << 8) | Wire.read();
  AccY = (Wire.read() << 8) | Wire.read();
  AccZ = (Wire.read() << 8) | Wire.read();

  Serial.print("AccX: "); Serial.print(AccX);
  Serial.print(" | AccY: "); Serial.print(AccY);
  Serial.print(" | AccZ: "); Serial.println(AccZ);

  // double m = (double)millis() / 500.0;
  delay(8);
  // digitalWrite(D1, (byte)(sin(m)* 255.0));
  if(mouse.isConnected()) {
    Serial.println("Connected!");
    // mouse.move(cos(m) * 3, sin(m) * -3);
    mouse.move(AccX, AccY);
  }

}