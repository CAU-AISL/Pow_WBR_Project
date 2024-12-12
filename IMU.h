#ifndef IMU_H
#define IMU_H

#include "Params.h"
#include <Wire.h>

class IMU {
public:
  // 센서의 측정값
  int16_t Tmp;
  Eigen::Matrix<int16_t, 3, 1> acc_raw_vec, gyr_raw_vec;
  Eigen::Matrix<float, 3, 1> acc_vec, gyr_vec;
  Eigen::Matrix<float, 3, 1> acc_vec_prev, gyr_vec_prev;
  float temperature;  // 온도 (섭씨)

  // 캘리브레이션 값
  Eigen::Vector3f gyro_bias{ -8.38f * M_PI / 180, 0.05f * M_PI / 180, 0.68f * M_PI / 180 };  // (rad/s)
  Eigen::Vector3f accel_bias{ 0.07f * 9.80665f, -0.02f * 9.80665f, -0.1f * 9.80665f };       // (m/s^2)

  // 생성자
  IMU() {}

  bool begin() {
    Wire.begin(SDA_PIN, SCL_PIN, 100000);  // SDA, SCL 핀과 클록 속도 설정
    Wire.beginTransmission(0x68);          // I2C 주소
    Wire.write(0x6B);                      // PWR_MGMT_1 레지스터
    Wire.write(0);                         // 슬립 모드 비활성화
    if (Wire.endTransmission(true) != 0) {
      Serial.println("[Error] Failed to initialize MPU6050. Check connections!");
      return false;
    }
    return true;
  }

  void setZero() {
    acc_raw_vec.setZero();
    gyr_raw_vec.setZero();
    acc_vec.setZero();
    gyr_vec.setZero();
    acc_vec_prev.setZero();
    gyr_vec_prev.setZero();
    temperature = 0;  // 온도 (섭씨)
  }

  void getIMUMeasurement(Eigen::Matrix<float, 8, 1>& z) {
    z.segment<3>(0) = acc_vec;
    z.segment<3>(3) = gyr_vec;
  }

  // 센서 데이터를 읽기
  bool readData() {
    Wire.beginTransmission(0x68);  // I2C 주소
    Wire.write(0x3B);              // 시작 레지스터 (ACCEL_XOUT_H)
    if (Wire.endTransmission(false) != 0) {
      return false;  // 데이터 요청 실패 시 함수 종료
    }

    if (Wire.requestFrom(0x68, 14, true) != 14) {
      return false;  // 데이터 수신 실패 시 함수 종료
    }

    // 데이터 읽기
    acc_raw_vec << (Wire.read() << 8 | Wire.read()),
      (Wire.read() << 8 | Wire.read()),
      (Wire.read() << 8 | Wire.read());
    Tmp = Wire.read() << 8 | Wire.read();  // 온도 데이터
    gyr_raw_vec << (Wire.read() << 8 | Wire.read()),
      (Wire.read() << 8 | Wire.read()),
      (Wire.read() << 8 | Wire.read());

    // 단위 변환 및 보정
    acc_vec = (acc_raw_vec.cast<float>() / 16384.0f) * 9.80665f - accel_bias;
    gyr_vec = (gyr_raw_vec.cast<float>() / 131.0f) * M_PI / 180 - gyro_bias;

    float alpha = cutoffFrequency(10);
    lowPassFilter(acc_vec, acc_vec_prev, alpha);
    lowPassFilter(gyr_vec, gyr_vec_prev, alpha);

    // 온도 변환
    temperature = Tmp / 340.0f + 36.53f;
    return true;
  }

  void lowPassFilter(Eigen::Vector3f& input, Eigen::Vector3f& prevOutput, const float alpha) {
    input = alpha * input + (1 - alpha) * prevOutput;
    prevOutput = input;
  }

  float cutoffFrequency(float f_c) {
    // Time constant τ 계산
    float tau = 1.0 / (2.0 * M_PI * f_c);

    // Alpha 값 계산
    float alpha = dt / (tau + dt);

    return alpha;
  }

  // 데이터를 Serial Plotter에 출력하는 함수
  void printData() {
    Serial.print("Accel_X:");
    Serial.print(acc_vec(0), 2);
    Serial.print(" ");
    Serial.print("Accel_Y:");
    Serial.print(acc_vec(1), 2);
    Serial.print(" ");
    Serial.print("Accel_Z:");
    Serial.print(acc_vec(2), 2);
    Serial.print(" ");
    Serial.print("Gyro_X:");
    Serial.print(gyr_vec(0), 2);
    Serial.print(" ");
    Serial.print("Gyro_Y:");
    Serial.print(gyr_vec(1), 2);
    Serial.print(" ");
    Serial.print("Gyro_Z:");
    Serial.print(gyr_vec(2), 2);
    Serial.print(" ");
    Serial.print("Temperature:");
    Serial.print(temperature, 2);  // 줄바꿈
  }
};

#endif  // IMU_H
