#ifndef VYB_CONTROLLER_H
#define VYB_CONTROLLER_H

#include <Arduino.h>
#include <ArduinoEigenDense.h>
#include "Params.h"
#include "MGServo.h"

class VYBController {
private:
  MGServo& ServoRW;
  MGServo& ServoLW;

  std::vector<Eigen::Matrix<float, 2, 4>> Ks;  // LQR 게인 vector
  Eigen::Matrix<float, 2, 4> K;
  Eigen::Matrix<float, 2, 1> u;  // input vector

  float iq_factor;        // (A/LSB)
  float torque_constant;  // (Nm/A)
  float saturation;
  const int RW_bias = 12;


public:
  // 생성자: 시스템 파라미터 초기화
  VYBController(MGServo& ServoRW_, MGServo& ServoLW_)
    : ServoRW(ServoRW_), ServoLW(ServoLW_) {
    iq_factor = 0.01611328f;  // (A/LSB) 33 / 2048
    torque_constant = 0.07f;  // (Nm/A)
    // torque_constant = 0.14f;  // (Nm/A)
    saturation = iq_factor * torque_constant * MAX_TORQUE_COMMAND;
    Eigen::Matrix<float, 2, 4> mat;

    //////// Gain Hard coding /////////////////
    mat << 1.10216287f, 0.12557887f, 0.19781040f, -0.02182668f,
      -1.09047379f, -0.12444375f, -0.19385010f, -0.02225212f;
    Ks.push_back(mat);

    mat << 1.15339511f, 0.13180833f, 0.19791767f, -0.02181192f,
      -1.14032743f, -0.13051870f, -0.19365305f, -0.02226797f;
    Ks.push_back(mat);

    mat << 1.20263348f, 0.13811207f, 0.19806660f, -0.02179740f,
      -1.18818190f, -0.13666266f, -0.19351291f, -0.02228256f;
    Ks.push_back(mat);

    mat << 1.24967814f, 0.14447839f, 0.19825486f, -0.02178328f,
      -1.23385205f, -0.14286490f, -0.19342807f, -0.02229597f;
    Ks.push_back(mat);

    mat << 1.29460209f, 0.15089994f, 0.19847595f, -0.02176953f,
      -1.27741492f, -0.14911827f, -0.19339053f, -0.02230830f;
    Ks.push_back(mat);

    mat << 1.33754624f, 0.15737051f, 0.19872319f, -0.02175604f,
      -1.31901196f, -0.15541656f, -0.19339167f, -0.02231968f;
    Ks.push_back(mat);

    mat << 1.37866614f, 0.16388479f, 0.19899060f, -0.02174276f,
      -1.35879793f, -0.16175438f, -0.19342375f, -0.02233021f;
    Ks.push_back(mat);

    mat << 1.41811357f, 0.17043841f, 0.19927307f, -0.02172964f,
      -1.39692366f, -0.16812732f, -0.19348022f, -0.02233997f;
    Ks.push_back(mat);

    mat << 1.45602974f, 0.17702821f, 0.19956640f, -0.02171664f,
      -1.43352972f, -0.17453220f, -0.19355582f, -0.02234906f;
    Ks.push_back(mat);

    mat << 1.49254316f, 0.18365243f, 0.19986717f, -0.02170378f,
      -1.46874470f, -0.18096738f, -0.19364645f, -0.02235755f;
    Ks.push_back(mat);

    mat << 1.52776991f, 0.19031125f, 0.20017273f, -0.02169107f,
      -1.50268560f, -0.18743316f, -0.19374912f, -0.02236550f;
    Ks.push_back(mat);

    mat << 1.56181630f, 0.19700757f, 0.20048107f, -0.02167859f,
      -1.53546079f, -0.19393272f, -0.19386189f, -0.02237295f;
    Ks.push_back(mat);

    mat << 1.59478585f, 0.20374880f, 0.20079068f, -0.02166645f,
      -1.56717806f, -0.20047399f, -0.19398394f, -0.02237994f;
    Ks.push_back(mat);

    mat << 1.62679901f, 0.21055092f, 0.20109995f, -0.02165495f,
      -1.59796869f, -0.20707413f, -0.19411617f, -0.02238651f;
    Ks.push_back(mat);
  }

  Eigen::Matrix<float, 2, 1> getInputVector() {
    return u;
  }

  void getMotorSpeedMeasurement(Eigen::Matrix<float, 8, 1>& z) {
    z(6) = ServoRW.getMotorSpeed() * M_PI / 180;
    z(7) = ServoLW.getMotorSpeed() * M_PI / 180;
  }

  void computeGainK(const float h) {
    float temp = (h - HEIGHT_MIN) / 0.01;  // 구간을 10mm당 하나씩 나눔
    int idx = static_cast<int>(temp);      // 구간의 정수 인덱스 계산

    if (idx >= 0 && idx < static_cast<int>(Ks.size()) - 1) {
      // 보간 비율 계산
      float ratio = temp - idx;  // 현재 위치가 구간 내에서 차지하는 비율

      // 보간 수행
      K = Ks.at(idx) * (1.0f - ratio) + Ks.at(idx + 1) * ratio;
    } else if (idx < 0) {
      // h가 HEIGHT_MIN 이하일 경우 최소값 사용
      K = Ks.front();
    } else {
      // h가 범위를 벗어날 경우 최대값 사용
      K = Ks.back();
    }
  }

  void computeInput(Eigen::Matrix<float, 4, 1>& x_d, Eigen::Matrix<float, 4, 1>& x) {
    u = K * (x_d - x) / 2;

    // Serial.print("x:");
    // Serial.print(x(0));
    // Serial.print(x(1));
    // Serial.print(x(2));
    // Serial.println(x(3));

    // Serial.print("x_d:");
    // Serial.print(x_d(0));
    // Serial.print(x_d(1));
    // Serial.print(x_d(2));
    // Serial.println(x_d(3));

    // Serial.println("K:");
    // for (int i = 0; i < K.rows(); ++i) {
    //   for (int j = 0; j < K.cols(); ++j) {
    //     Serial.print(K(i, j), 6);  // 소수점 6자리까지 출력
    //     Serial.print(" ");
    //   }
    //   Serial.println();
    // }
    // Serial.print("u:");
    // Serial.print(u(0));
    // Serial.print(u(1));


    // Input saturation
    for (int j = 0; j < 2; j++) {
      if (u(j) > saturation) {
        u(j) = saturation;
      } else if (u(j) < -saturation) {
        u(j) = -saturation;
      }
    }
    // Serial.print("Saturation : ");
    // Serial.print(saturation, 10);
    // Serial.print("u(after):");
    // Serial.print(u(0),10);
    // Serial.println(u(1),10);

    // Serial.print(" Constant:");
    // Serial.print(MAX_TORQUE_COMMAND, 10);
    // Serial.print(" ");
    // Serial.print(iq_factor,10);
    // Serial.print(" ");
    // Serial.println(torque_constant,10);
  }

  void sendControlCommand() {
    float u_RW = u(0) / (iq_factor * torque_constant);
    float u_LW = u(1) / (iq_factor * torque_constant);

    if (u_RW < 0) {
      u_RW -= RW_bias;
    } else if (u_RW > 0) {
      u_RW += RW_bias;
    }



    ServoRW.sendTorqueControlCommand(static_cast<int16_t>(u_RW));
    ServoLW.sendTorqueControlCommand(static_cast<int16_t>(u_LW));
  }
};

#endif  // VYB_CONTROLLER_H
