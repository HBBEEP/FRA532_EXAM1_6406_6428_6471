#include "MPU9250.h"
#include <Arduino.h>

MPU9250 mpu;

#define LED 13
void setup() {
  Serial.begin(115200);
  Wire.begin();
  if (!mpu.setup(0x68)) {
    while (1) {
      Serial.println("MPU connection failed. Please check your connection with `connection_check` example.");
      delay(5000);
    }
  }
  pinMode(LED, OUTPUT);
}

void loop() {
  float acc_value[3] = {0, 0, 0};
  acc_read(acc_value);
  float x = acc_value[0];
  float y = acc_value[1];
  float z = acc_value[2];
  Serial.println();
  Serial.print("Acc X:");
  Serial.print(x);
  Serial.print("\t");
  Serial.print("Acc Y:");
  Serial.print(y);
  Serial.print("\t");
  Serial.print("Acc Z:");
  Serial.println(z);
  if (x > 0.20 || x < -0.20) {
    digitalWrite(LED, HIGH);
  }
  else if (y > 0.20 || y < -0.20) {
    digitalWrite(LED, HIGH);
  }
  else {
    digitalWrite(LED, LOW);
  }
  delay(100);
}

float acc_read(float * acc_in) {
  mpu.update();
  acc_in[0] = mpu.getAccX();
  acc_in[1] = mpu.getAccY();
  acc_in[2] = mpu.getAccZ();
}

void gyro_read(float * gyro_in) {
  mpu.update();
  gyro_in[0] = mpu.getGyroX();
  gyro_in[1] = mpu.getGyroY();
  gyro_in[2] = mpu.getGyroZ();
}

void mag_read(float * mag_in) {
  mpu.update();
  mag_in[0] = mpu.getMagX();
  mag_in[1] = mpu.getMagY();
  mag_in[2] = mpu.getMagZ();
}


