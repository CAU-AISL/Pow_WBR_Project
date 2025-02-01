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
    mat << 1.11706580f, 0.12269927f, 0.16707269f, -0.12171047f,
      -1.12839134f, -0.12501511f, -0.16733739f, -0.12193438f;
    Ks.push_back(mat);

    mat << 1.16616725f, 0.12854883f, 0.16709085f, -0.12169593f,
      -1.17784587f, -0.13099343f, -0.16729629f, -0.12197416f;
    Ks.push_back(mat);

    mat << 1.21296974f, 0.13443895f, 0.16713042f, -0.12167589f,
      -1.22501322f, -0.13701674f, -0.16728984f, -0.12199901f;
    Ks.push_back(mat);

    mat << 1.25737702f, 0.14036317f, 0.16719025f, -0.12165220f,
      -1.26979546f, -0.14307775f, -0.16731527f, -0.12201174f;
    Ks.push_back(mat);

    mat << 1.29950769f, 0.14631642f, 0.16726665f, -0.12162511f,
      -1.31231271f, -0.14917125f, -0.16736771f, -0.12201344f;
    Ks.push_back(mat);

    mat << 1.33952964f, 0.15229392f, 0.16735582f, -0.12159457f,
      -1.35273465f, -0.15529250f, -0.16744229f, -0.12200485f;
    Ks.push_back(mat);

    mat << 1.37761700f, 0.15829137f, 0.16745438f, -0.12156058f,
      -1.39123682f, -0.16143730f, -0.16753474f, -0.12198670f;
    Ks.push_back(mat);

    mat << 1.41393568f, 0.16430529f, 0.16755958f, -0.12152329f,
      -1.42798591f, -0.16760222f, -0.16764140f, -0.12195982f;
    Ks.push_back(mat);

    mat << 1.44863838f, 0.17033328f, 0.16766925f, -0.12148305f,
      -1.46313473f, -0.17378487f, -0.16775926f, -0.12192512f;
    Ks.push_back(mat);

    mat << 1.48186344f, 0.17637432f, 0.16778175f, -0.12144033f,
      -1.49682109f, -0.17998414f, -0.16788587f, -0.12188367f;
    Ks.push_back(mat);

    mat << 1.51373556f, 0.18242925f, 0.16789596f, -0.12139574f,
      -1.52916859f, -0.18620069f, -0.16801931f, -0.12183660f;
    Ks.push_back(mat);

    mat << 1.54436843f, 0.18850156f, 0.16801121f, -0.12135008f,
      -1.56028914f, -0.19243771f, -0.16815815f, -0.12178515f;
    Ks.push_back(mat);

    mat << 1.57387150f, 0.19459913f, 0.16812738f, -0.12130457f,
      -1.59028912f, -0.19870253f, -0.16830132f, -0.12173107f;
    Ks.push_back(mat);

    mat << 1.60236927f, 0.20073827f, 0.16824514f, -0.12126229f,
      -1.61928561f, -0.20501004f, -0.16844791f, -0.12167817f;
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
