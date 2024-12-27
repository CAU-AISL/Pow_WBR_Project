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

  float iq_factor;         ///< 전류 변환 계수 (A/LSB)
  float torque_constant;   ///< 토크 상수 (Nm/A)
  float saturation;        ///< input saturation
  const int RW_bias = 12;  ///< 우측 휠 모터의 바이어스 값

public:
  /**
   * @brief 생성자: VYBController 초기화
   * @param ServoRW_ 우측 휠 서보 객체 참조
   * @param ServoLW_ 좌측 휠 서보 객체 참조
   */
  VYBController(MGServo& ServoRW_, MGServo& ServoLW_)
    : ServoRW(ServoRW_), ServoLW(ServoLW_) {
    // 전류 및 토크 상수 초기화
    iq_factor = 0.01611328f;  // (A/LSB) 33 / 2048
    torque_constant = 0.7f;   // (Nm/A) * reduction ratio
    saturation = iq_factor * torque_constant * MAX_TORQUE_COMMAND;


    // LQR 게인 초기화 (하드코딩된 데이터 삽입)
    Eigen::Matrix<float, 2, 4> mat;
    //////////////////////////////////////////////////////////
    mat << 1.35605099f, 0.16778375f, 0.42689449f, -0.06729931f,
      -1.35643197f, -0.16830087f, -0.42482000f, -0.06781604f;
    Ks.push_back(mat);

    mat << 1.42034111f, 0.17660064f, 0.42692224f, -0.06726744f,
      -1.41994706f, -0.17704838f, -0.42433059f, -0.06786118f;
    Ks.push_back(mat);

    mat << 1.48271293f, 0.18559313f, 0.42704183f, -0.06723351f,
      -1.48154285f, -0.18596943f, -0.42397447f, -0.06789856f;
    Ks.push_back(mat);

    mat << 1.54292142f, 0.19473772f, 0.42725392f, -0.06719865f,
      -1.54098383f, -0.19504100f, -0.42375126f, -0.06792935f;
    Ks.push_back(mat);

    mat << 1.60101827f, 0.20401895f, 0.42754517f, -0.06716290f,
      -1.59832686f, -0.20424799f, -0.42364406f, -0.06795408f;
    Ks.push_back(mat);

    mat << 1.65713041f, 0.21342430f, 0.42790034f, -0.06712614f,
      -1.65370181f, -0.21357812f, -0.42363379f, -0.06797322f;
    Ks.push_back(mat);

    mat << 1.71140334f, 0.22294341f, 0.42830534f, -0.06708825f,
      -1.70725582f, -0.22302123f, -0.42370272f, -0.06798717f;
    Ks.push_back(mat);

    mat << 1.76398179f, 0.23256801f, 0.42874807f, -0.06704922f,
      -1.75913450f, -0.23256921f, -0.42383555f, -0.06799642f;
    Ks.push_back(mat);

    mat << 1.81500279f, 0.24229201f, 0.42921857f, -0.06700915f,
      -1.80947526f, -0.24221608f, -0.42401961f, -0.06800143f;
    Ks.push_back(mat);

    mat << 1.86459355f, 0.25211175f, 0.42970897f, -0.06696826f,
      -1.85840550f, -0.25195828f, -0.42424474f, -0.06800273f;
    Ks.push_back(mat);

    mat << 1.91287229f, 0.26202648f, 0.43021336f, -0.06692684f,
      -1.90604354f, -0.26179514f, -0.42450323f, -0.06800087f;
    Ks.push_back(mat);

    mat << 1.95995210f, 0.27203954f, 0.43072772f, -0.06688534f,
      -1.95250261f, -0.27173005f, -0.42478966f, -0.06799642f;
    Ks.push_back(mat);

    mat << 2.00595178f, 0.28216102f, 0.43124988f, -0.06684445f,
      -1.99790158f, -0.28177308f, -0.42510088f, -0.06799015f;
    Ks.push_back(mat);

    mat << 2.05102644f, 0.29241454f, 0.43177925f, -0.06680590f,
      -2.04239490f, -0.29194751f, -0.42543585f, -0.06798365f;
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
    u = K * (x_d - x);

    // Input saturation
    for (int j = 0; j < 2; j++) {
      if (u(j) > saturation) {
        u(j) = saturation;
      } else if (u(j) < -saturation) {
        u(j) = -saturation;
      }
    }
  }

  /**
   * @brief 계산된 제어 명령을 서보에 전송
   */
  void sendControlCommand() {
    float u_RW = u(0) / (iq_factor * torque_constant);
    float u_LW = u(1) / (iq_factor * torque_constant);

    // Right Wheel motor의 마찰로 인해 발생하는 torque 문제를 조정해줌
    if (u_RW < 0) {
      u_RW -= RW_bias;
    } else if (u_RW > 0) {
      u_RW += RW_bias;
    }

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
};

#endif  // VYB_CONTROLLER_H
