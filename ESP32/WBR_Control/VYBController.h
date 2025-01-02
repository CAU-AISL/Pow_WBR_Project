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
    mat << 1.37789573f, 0.17085039f, 0.43851903f, -0.06781964f,
      -1.37789285f, -0.17131392f, -0.43627616f, -0.06832617f;
    Ks.push_back(mat);

    mat << 1.44333034f, 0.17985207f, 0.43858525f, -0.06778619f,
      -1.44251525f, -0.18024059f, -0.43581413f, -0.06837072f;
    Ks.push_back(mat);

    mat << 1.50679478f, 0.18902976f, 0.43872679f, -0.06775152f,
      -1.50516620f, -0.18934098f, -0.43546947f, -0.06840824f;
    Ks.push_back(mat);

    mat << 1.56803452f, 0.19835838f, 0.43894428f, -0.06771658f,
      -1.56560117f, -0.19859056f, -0.43524185f, -0.06843971f;
    Ks.push_back(mat);

    mat << 1.62710201f, 0.20782203f, 0.43922656f, -0.06768135f,
      -1.62387789f, -0.20797379f, -0.43511653f, -0.06846563f;
    Ks.push_back(mat);

    mat << 1.68412793f, 0.21740806f, 0.43956091f, -0.06764569f,
      -1.68012999f, -0.21747827f, -0.43507684f, -0.06848643f;
    Ks.push_back(mat);

    mat << 1.73926251f, 0.22710615f, 0.43993557f, -0.06760944f,
      -1.73450929f, -0.22709389f, -0.43510731f, -0.06850252f;
    Ks.push_back(mat);

    mat << 1.79265535f, 0.23690815f, 0.44034042f, -0.06757257f,
      -1.78716622f, -0.23681264f, -0.43519456f, -0.06851433f;
    Ks.push_back(mat);

    mat << 1.84444806f, 0.24680814f, 0.44076715f, -0.06753511f,
      -1.83824273f, -0.24662872f, -0.43532748f, -0.06852229f;
    Ks.push_back(mat);

    mat << 1.89477202f, 0.25680261f, 0.44120919f, -0.06749722f,
      -1.88787037f, -0.25653871f, -0.43549723f, -0.06852687f;
    Ks.push_back(mat);

    mat << 1.94374903f, 0.26689099f, 0.44166162f, -0.06745913f,
      -1.93617105f, -0.26654212f, -0.43569702f, -0.06852855f;
    Ks.push_back(mat);

    mat << 1.99149513f, 0.27707675f, 0.44212110f, -0.06742119f,
      -1.98326103f, -0.27664249f, -0.43592213f, -0.06852784f;
    Ks.push_back(mat);

    mat << 2.03813150f, 0.28737015f, 0.44258580f, -0.06738399f,
      -2.02926171f, -0.28685008f, -0.43616979f, -0.06852537f;
    Ks.push_back(mat);

    mat << 2.08381544f, 0.29779506f, 0.44305520f, -0.06734897f,
      -2.07433014f, -0.29718848f, -0.43643915f, -0.06852249f;
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
