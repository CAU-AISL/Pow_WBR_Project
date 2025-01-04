#ifndef VYB_CONTROLLER_H
#define VYB_CONTROLLER_H

#include <Arduino.h>
#include <ArduinoEigenDense.h>
#include "Params.h"
#include "MGServo.h"

/**
 * @class VYBController
 * @brief 제어 시스템의 LQR 기반 동작을 처리하는 클래스
 */
class VYBController {
private:
  MGServo& ServoRW;  ///< 우측 휠 서보
  MGServo& ServoLW;  ///< 좌측 휠 서보

  std::vector<Eigen::Matrix<float, 2, 4>> Ks;  ///< LQR 게인 행렬들의 벡터
  Eigen::Matrix<float, 2, 4> K;                ///< 현재 사용 중인 LQR 게인
  Eigen::Matrix<float, 2, 1> u;                ///< 제어 입력 벡터

  float iq_factor;        ///< 전류 변환 계수 (A/LSB)
  float torque_constant;  ///< 토크 상수 (Nm/A)
  float LW_factor = 0.001043224f;
  float RW_factor = 0.000857902f;

  float AngleFixRate = 0.1;

  float saturation;  ///< input saturation
  Eigen::Matrix<float, 2, 1> saturation_vec;
  const int RW_bias = 12;  ///< 우측 휠 모터의 바이어스 값

public:
  float theta_d;
  /**
   * @brief 생성자: VYBController 초기화
   * @param ServoRW_ 우측 휠 서보 객체 참조
   * @param ServoLW_ 좌측 휠 서보 객체 참조
   */
  VYBController(MGServo& ServoRW_, MGServo& ServoLW_)
    : ServoRW(ServoRW_), ServoLW(ServoLW_) {
    theta_d = 0.f;
    // 전류 및 토크 상수 초기화
    iq_factor = 0.001611328f;  // (A/LSB) 3.3 / 2048
    torque_constant = 0.7f;    // (Nm/A) * reduction ratio
    saturation = iq_factor * torque_constant * MAX_TORQUE_COMMAND;
    // saturation_vec << MAX_TORQUE_COMMAND * RW_factor, MAX_TORQUE_COMMAND * LW_factor;
    saturation_vec << MAX_TORQUE, MAX_TORQUE;


    // LQR 게인 초기화 (하드코딩된 데이터 삽입)
    Eigen::Matrix<float, 2, 4> mat;
    //////////////////////////////////////////////////////////
    mat << 0.79157211f, 0.08949879f, 0.13324021f, -0.04843267f,
      -0.80162498f, -0.09129509f, -0.13488934f, -0.04909636f;
    Ks.push_back(mat);

    mat << 0.82749528f, 0.09377660f, 0.13284322f, -0.04844744f,
      -0.83769858f, -0.09563814f, -0.13426330f, -0.04915773f;
    Ks.push_back(mat);

    mat << 0.86248546f, 0.09817186f, 0.13272420f, -0.04843592f,
      -0.87284050f, -0.10010141f, -0.13393901f, -0.04919017f;
    Ks.push_back(mat);

    mat << 0.89646446f, 0.10268749f, 0.13287970f, -0.04840456f,
      -0.90697205f, -0.10468726f, -0.13391117f, -0.04920005f;
    Ks.push_back(mat);

    mat << 0.92946708f, 0.10731846f, 0.13327102f, -0.04835495f,
      -0.94013083f, -0.10939071f, -0.13413915f, -0.04918921f;
    Ks.push_back(mat);

    mat << 0.96155309f, 0.11205740f, 0.13385600f, -0.04828783f,
      -0.97237973f, -0.11420460f, -0.13457902f, -0.04915871f;
    Ks.push_back(mat);

    mat << 0.99278639f, 0.11689660f, 0.13459647f, -0.04820401f,
      -1.00378535f, -0.11912147f, -0.13519096f, -0.04910964f;
    Ks.push_back(mat);

    mat << 1.02322897f, 0.12182895f, 0.13545998f, -0.04810470f,
      -1.03441171f, -0.12413443f, -0.13594105f, -0.04904345f;
    Ks.push_back(mat);

    mat << 1.05293953f, 0.12684856f, 0.13642001f, -0.04799157f,
      -1.06431876f, -0.12923772f, -0.13680135f, -0.04896198f;
    Ks.push_back(mat);

    mat << 1.08197396f, 0.13195122f, 0.13745562f, -0.04786668f,
      -1.09356295f, -0.13442718f, -0.13774963f, -0.04886743f;
    Ks.push_back(mat);

    mat << 1.11038720f, 0.13713502f, 0.13855115f, -0.04773247f,
      -1.12219903f, -0.13970087f, -0.13876898f, -0.04876231f;
    Ks.push_back(mat);

    mat << 1.13823694f, 0.14240134f, 0.13969602f, -0.04759179f,
      -1.15028372f, -0.14506002f, -0.13984764f, -0.04864950f;
    Ks.push_back(mat);

    mat << 1.16559157f, 0.14775666f, 0.14088482f, -0.04744880f,
      -1.17788281f, -0.15051066f, -0.14097876f, -0.04853311f;
    Ks.push_back(mat);

    mat << 1.19254970f, 0.15321643f, 0.14211740f, -0.04731278f,
      -1.20508696f, -0.15606684f, -0.14215981f, -0.04842216f;
    Ks.push_back(mat);
    /////////////////////////////////////////////////////////////////
  }

  /**
   * @brief 현재 제어 입력 벡터를 반환
   * @return Eigen::Matrix<float, 2, 1> u
   */
  Eigen::Matrix<float, 2, 1> getInputVector() {
    return u;
  }

  /**
   * @brief 모터 속도 측정을 수행하여 측정 벡터에 반영
   * @param z 모터 속도 측정값을 저장할 벡터
   */
  void getMotorSpeedMeasurement(Eigen::Matrix<float, 8, 1>& z) {
    z(6) = ServoRW.getMotorSpeed() * M_PI / 180;
    z(7) = ServoLW.getMotorSpeed() * M_PI / 180;
  }

  /**
  * @brief 모터 Current 측정값 update
  * @param iq_vec 모터 current 측정값을 저장할 벡터
  */
  void getMotorCurrentMeasurement(Eigen::Matrix<float, 2, 1>& iq_vec) {
    iq_vec << ServoRW.getMotorIq(), ServoLW.getMotorIq();
  }

  /**
   * @brief 현재 높이에 따라 LQR 게인 K를 계산
   * @param h 현재 높이 (m)
   */
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

  /**
   * @brief 상태 벡터를 기반으로 제어 입력 벡터를 계산
   * @param x_d 목표 상태 벡터
   * @param x 현재 상태 벡터
   */
  void computeInput(Eigen::Matrix<float, 4, 1>& x_d, Eigen::Matrix<float, 4, 1>& x) {

    // if (x_d(0) - x(0) < 0) {
    //   theta_d -= AngleFixRate * dt;
    // } else {
    //   theta_d += AngleFixRate * dt;
    // }
    // x_d(0) = theta_d;

    u = K * (x_d - x);

    // // Input saturation
    // for (int j = 0; j < 2; j++) {
    //   if (u(j) > saturation) {
    //     u(j) = saturation;
    //   } else if (u(j) < -saturation) {
    //     u(j) = -saturation;
    //   }
    // }
    // Input saturation
    for (int j = 0; j < 2; j++) {
      if (u(j) > saturation_vec(j)) {
        u(j) = saturation_vec(j);
      } else if (u(j) < -saturation_vec(j)) {
        u(j) = -saturation_vec(j);
      }
    }
  }

  /**
   * @brief 계산된 제어 명령을 서보에 전송
   */
  void sendControlCommand() {
    // float u_RW = u(0) / (iq_factor * torque_constant);
    // float u_LW = u(1) / (iq_factor * torque_constant);

    // // Right Wheel motor의 마찰로 인해 발생하는 torque 문제를 조정해줌
    // if (u_RW < 0) {
    //   u_RW -= RW_bias;
    // } else if (u_RW > 0) {
    //   u_RW += RW_bias;
    // }

    float u_RW = u(0) / RW_factor;
    float u_LW = u(1) / LW_factor;

    ServoRW.sendTorqueControlCommand(static_cast<int16_t>(u_RW));
    ServoLW.sendTorqueControlCommand(static_cast<int16_t>(u_LW));
  }

  void sendDirectControlCommand(Eigen::Matrix<float, 2, 1> u_) {
    float u_RW = u_(0) / (iq_factor * torque_constant);
    float u_LW = u_(1) / (iq_factor * torque_constant);

    // Right Wheel motor의 마찰로 인해 발생하는 torque 문제를 조정해줌
    if (u_RW < 0) {
      u_RW -= RW_bias;
    } else if (u_RW > 0) {
      u_RW += RW_bias;
    }

    ServoRW.sendTorqueControlCommand(static_cast<int16_t>(u_RW));
    ServoLW.sendTorqueControlCommand(static_cast<int16_t>(u_LW));
  }

  void sendReadStateCommand() {
    ServoRW.sendCommandReadMotorState2();
    ServoLW.sendCommandReadMotorState2();
  }
};

#endif  // VYB_CONTROLLER_H
