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
    torque_constant = 0.075f;  // (Nm/A)
    saturation = iq_factor * torque_constant * MAX_TORQUE_COMMAND;
    Eigen::Matrix<float, 2, 4> mat;

    //////// Gain Hard coding /////////////////

    mat << 3.04891881f, 0.53884039f, 1.12375728f, -0.45172982f,
      -3.17449093f, -0.56259745f, -1.23530815f, -0.45742827f;
    Ks.push_back(mat);

    mat << 3.23050636f, 0.56924210f, 1.16481091f, -0.45198842f,
      -3.36329188f, -0.59434102f, -1.27498468f, -0.45800049f;
    Ks.push_back(mat);

    mat << 3.41609047f, 0.60127433f, 1.20661342f, -0.45195375f,
      -3.55609258f, -0.62775068f, -1.31549715f, -0.45826476f;
    Ks.push_back(mat);

    mat << 3.60540237f, 0.63496898f, 1.24921138f, -0.45169661f,
      -3.75257065f, -0.66284952f, -1.35690000f, -0.45828962f;
    Ks.push_back(mat);

    mat << 3.79806181f, 0.67026409f, 1.29240808f, -0.45123519f,
      -3.95233410f, -0.69957290f, -1.39899369f, -0.45809450f;
    Ks.push_back(mat);

    mat << 3.99364405f, 0.70707144f, 1.33595375f, -0.45057827f,
      -4.15495992f, -0.73783207f, -1.44152350f, -0.45769011f;
    Ks.push_back(mat);

    mat << 4.19171899f, 0.74529810f, 1.37960048f, -0.44973555f,
      -4.36002502f, -0.77753447f, -1.48423663f, -0.45708808f;
    Ks.push_back(mat);

    mat << 4.39187699f, 0.78485589f, 1.42312341f, -0.44872085f,
      -4.56712828f, -0.81859266f, -1.52690375f, -0.45630383f;
    Ks.push_back(mat);

    mat << 4.59374897f, 0.82566698f, 1.46633052f, -0.44755287f,
      -4.77590846f, -0.86092955f, -1.56932868f, -0.45535728f;
    Ks.push_back(mat);

    mat << 4.79702605f, 0.86766860f, 1.50906840f, -0.44625477f,
      -4.98606249f, -0.90448290f, -1.61135400f, -0.45427233f;
    Ks.push_back(mat);

    mat << 5.00148397f, 0.91081865f, 1.55122789f, -0.44485341f,
      -5.19736913f, -0.94921073f, -1.65286649f, -0.45307619f;
    Ks.push_back(mat);

    mat << 5.20702083f, 0.95510455f, 1.59275174f, -0.44338030f,
      -5.40972573f, -0.99510010f, -1.69380463f, -0.45180023f;
    Ks.push_back(mat);

    mat << 5.41372531f, 1.00055938f, 1.63364654f, -0.44188076f,
      -5.62321270f, -1.04218246f, -1.73416967f, -0.45048884f;
    Ks.push_back(mat);

    mat << 5.62201629f, 1.04729492f, 1.67400021f, -0.44045556f,
      -5.83821583f, -1.09056328f, -1.77403900f, -0.44923966f;
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
    u = K * (x_d - x);

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
