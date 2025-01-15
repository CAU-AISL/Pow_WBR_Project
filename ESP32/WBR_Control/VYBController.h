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
    mat << 1.07390100f, 0.12078304f, 0.16924351f, -0.01390550f,
      -1.05945946f, -0.11929005f, -0.16566709f, -0.01419625f;
    Ks.push_back(mat);

    mat << 1.12258739f, 0.12661536f, 0.16935664f, -0.01389502f,
      -1.10662649f, -0.12494900f, -0.16550773f, -0.01420689f;
    Ks.push_back(mat);

    mat << 1.16919988f, 0.13250201f, 0.16949161f, -0.01388488f,
      -1.15171541f, -0.13065647f, -0.16538255f, -0.01421691f;
    Ks.push_back(mat);

    mat << 1.21355886f, 0.13843316f, 0.16964678f, -0.01387513f,
      -1.19456222f, -0.13640367f, -0.16529068f, -0.01422631f;
    Ks.push_back(mat);

    mat << 1.25575254f, 0.14440292f, 0.16981867f, -0.01386574f,
      -1.23525856f, -0.14218485f, -0.16522728f, -0.01423516f;
    Ks.push_back(mat);

    mat << 1.29593427f, 0.15040636f, 0.17000376f, -0.01385663f,
      -1.27395734f, -0.14799497f, -0.16518714f, -0.01424352f;
    Ks.push_back(mat);

    mat << 1.33426976f, 0.15643923f, 0.17019894f, -0.01384776f,
      -1.31082259f, -0.15382963f, -0.16516555f, -0.01425146f;
    Ks.push_back(mat);

    mat << 1.37091899f, 0.16249808f, 0.17040153f, -0.01383908f,
      -1.34601262f, -0.15968526f, -0.16515859f, -0.01425904f;
    Ks.push_back(mat);

    mat << 1.40602962f, 0.16858047f, 0.17060930f, -0.01383056f,
      -1.37967398f, -0.16555938f, -0.16516312f, -0.01426630f;
    Ks.push_back(mat);

    mat << 1.43973503f, 0.17468522f, 0.17082040f, -0.01382219f,
      -1.41193988f, -0.17145086f, -0.16517674f, -0.01427327f;
    Ks.push_back(mat);

    mat << 1.47215467f, 0.18081287f, 0.17103333f, -0.01381398f,
      -1.44293076f, -0.17736042f, -0.16519776f, -0.01427997f;
    Ks.push_back(mat);

    mat << 1.50339652f, 0.18696641f, 0.17124689f, -0.01380596f,
      -1.47275699f, -0.18329134f, -0.16522514f, -0.01428642f;
    Ks.push_back(mat);

    mat << 1.53356347f, 0.19315291f, 0.17146001f, -0.01379816f,
      -1.50152671f, -0.18925134f, -0.16525864f, -0.01429259f;
    Ks.push_back(mat);

    mat << 1.56277047f, 0.19938706f, 0.17167114f, -0.01379077f,
      -1.52936885f, -0.19525674f, -0.16529941f, -0.01429844f;
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
