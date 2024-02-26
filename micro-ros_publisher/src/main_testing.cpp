// #include <Arduino.h>
// #include "MPU9250.h"

// MPU9250 mpu;

// void acc_read(float * acc_in);
// void gyro_read(float * gyro_in);

// void setup() {
//     Serial.begin(115200);
//     Wire.begin();
//     delay(2000);

//     if (!mpu.setup(0x68)) {  // change to your own address
//         while (1) {
//             Serial.println("MPU connection failed. Please check your connection with `connection_check` example.");
//             delay(5000);
//         }
//     }
// }

// void loop() {
//   float acc_value[3] = {0, 0, 0};
//   float gyro_value[3] = {0, 0, 0};

//   acc_read(acc_value);
//   gyro_read(gyro_value);

//   Serial.print("Acc X: ");
//   Serial.print(acc_value[0]);
//   Serial.print("\tAcc Y: ");
//   Serial.print(acc_value[1]);
//   Serial.print("\tAcc Z: ");
//   Serial.print(acc_value[2]);

//   Serial.print("\tGyro X: ");
//   Serial.print(gyro_value[0]);
//   Serial.print("\tGyro Y: ");
//   Serial.print(gyro_value[1]);
//   Serial.print("\tGyro Z: ");
//   Serial.println(gyro_value[2]);

//   delay(100);

// }

// void acc_read(float * acc_in) {
//   mpu.update();
//   acc_in[0] = mpu.getAccX();
//   acc_in[1] = mpu.getAccY();
//   acc_in[2] = mpu.getAccZ();
// }

// void gyro_read(float * gyro_in) {
//   mpu.update();
//   gyro_in[0] = mpu.getGyroX();
//   gyro_in[1] = mpu.getGyroY();
//   gyro_in[2] = mpu.getGyroZ();
// }