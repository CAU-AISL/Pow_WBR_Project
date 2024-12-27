#include "Params.h"
#include "Receiver.h"
#include "HRController.h"
#include "VYBController.h"
#include "MGServo.h"
#include "IMU.h"
#include "POL.h"
#include "EKF.h"
#include "Logger.h"


// Properties와 Receiver, Controller 초기화
const Properties properties = createDefaultProperties();
POL Pol(properties);
HardwareSerial RS485(1);
MGServo ServoLW(1, RS485);
MGServo ServoRW(2, RS485);
Receiver receiver(Serial2);
IMU MPU6050;

HRController HR_controller;
VYBController VYB_controller(ServoRW, ServoLW);
EKF Estimator(Pol);

Logger WIFI_Logger(ssid, password);

Eigen::Matrix<float, 4, 1> x = Eigen::Matrix<float, 4, 1>::Zero();
Eigen::Matrix<float, 4, 1> x_d = Eigen::Matrix<float, 4, 1>::Zero();
Eigen::Matrix<float, 2, 1> u = Eigen::Matrix<float, 2, 1>::Zero();
Eigen::Matrix<float, 8, 1> z = Eigen::Matrix<float, 8, 1>::Zero();
Eigen::Matrix<float, 2, 1> iq_vec = Eigen::Matrix<float, 2, 1>::Zero();

int i = 0;
unsigned long previousMillis = 0;
unsigned long time_ref = 0;
float h_d = HEIGHT_MAX, phi_d = 0;
float v_d = 0, dpsi_d = 0;

void serialPrintStates();

void setup() {
  Serial.begin(115200);
  receiver.begin();
  HR_controller.attachServos(LH_PIN, RH_PIN);

  if (!MPU6050.begin()) {
    Serial.println("[ERROR] Fail to initialize IMU.");
  }

  // RS485 핀 초기화
  pinMode(RS485_DE_RE, OUTPUT);
  digitalWrite(RS485_DE_RE, LOW);

  // RS485 및 Serial 통신 초기화
  RS485.begin(115200, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN);

  // WIFI 연결
  WIFI_Logger.begin();

  unsigned long receiver_timer_start = millis();
  const unsigned long timeout = 5000;  // 타임아웃 5초 설정

  // SBUS 데이터 수신 대기 (타임아웃 처리)
  while (!receiver.readData()) {
    if (millis() - receiver_timer_start > timeout) {
      Serial.println("Timeout: No data received from SBUS.");
      receiver_timer_start = millis();  // 타임아웃 초기화
    }
  }
  receiver.updateData();

  // 시간 측정 시작
  time_ref = millis();
}

void loop() {
  unsigned long currentMillis = millis();  // 현재 시간

  // sampling time이 경과했을 때만 실행
  if (currentMillis - previousMillis >= dt * 1000) {
    // 경과 시간 출력
    Serial.print("SamplingTime(ms):");
    Serial.print(currentMillis - previousMillis);
    Serial.print(" ");
    previousMillis = currentMillis;  // 마지막 실행 시간을 현재 시간으로 업데이트


    if (receiver.readData()) {
      receiver.updateData();
    }

    if (receiver.isRun()) {
      // Running Mode

      // measurement update
      MPU6050.readData();
      MPU6050.getIMUMeasurement(z);
      VYB_controller.getMotorSpeedMeasurement(z);
      VYB_controller.getMotorCurrentMeasurement(iq_vec);
      
      //// update Pol state and input for EKF ////
      Pol.setState(x);
      Pol.setInput(u);

      //// state estimation ////
      if (!Estimator.estimate_state(x, z)) {
        // Diverge Safe Gaurd
        ServoLW.sendTorqueControlCommand(0);
        ServoRW.sendTorqueControlCommand(0);
        while (true) {
          Serial.println("Mass matrix Singularity Error. Change mode to Off Mode...");
          x.setZero();
          u.setZero();
          Estimator.reset_estimator();
          if (receiver.readData()) {
            receiver.updateData();
          }
          delay(500);
          if (!receiver.isRun()) {
            return;
          }
        }
      }

      //// Get Desired States from receiver ////
      receiver.updateDesiredStates();
      h_d = receiver.getDesiredHeight();
      // phi_d = receiver.getDesiredRoll();
      phi_d = 0;  // roll control diable
      v_d = receiver.getDesiredVel();
      dpsi_d = receiver.getDesiredYawVel();
      x_d.segment<2>(2) << v_d, dpsi_d;

      //// calculate CoM and Inertia from CoM Calculator ////
      Pol.setHR(h_d, phi_d);
      Pol.calculate_com_and_inertia();  // 여기서 inverse kinematics로 theta_hips도 계산됨
      Pol.get_theta_eq(x_d(0));         // update desired pitch angle with equilibrium point

      //// compute VYB controller gain ////
      VYB_controller.computeGainK(h_d);
      VYB_controller.computeInput(x_d, x);

      //// Send control command of HR controller and VYB controller
      HR_controller.controlHipServos(Pol.get_theta_hips());
      VYB_controller.sendControlCommand();
      u = VYB_controller.getInputVector();     

      ///// Logging ////////////////////////////////////////////////////////////////////////////////
      WIFI_Logger.logTimeStamp(millis() - time_ref);  // 시간 기록
      // state 기록
      WIFI_Logger.logValue("h_d", h_d);
      WIFI_Logger.logValue("theta_eq", x_d(0));
      WIFI_Logger.logValue("v_d", x_d(2));
      WIFI_Logger.logValue("psi_dot_d", x_d(3));

      WIFI_Logger.logValue("theta_hat", x(0));
      WIFI_Logger.logValue("theta_dot_hat", x(1));  // 시간 기록
      WIFI_Logger.logValue("v_hat", x(2));          // 시간 기록
      WIFI_Logger.logValue("psi_dot_hat", x(3));    // 시간 기록

      WIFI_Logger.logValue("tau_RW", u(0));  // 시간 기록
      WIFI_Logger.logValue("tau_LW", u(1));  // 시간 기록

      WIFI_Logger.logValue("acc_x", z(0));
      WIFI_Logger.logValue("acc_y", z(1));  // 시간 기록
      WIFI_Logger.logValue("acc_z", z(2));  // 시간 기록
      WIFI_Logger.logValue("gyr_x", z(3));  // 시간 기록
      WIFI_Logger.logValue("gyr_y", z(4));
      WIFI_Logger.logValue("gyr_z", z(5));         // 시간 기록
      WIFI_Logger.logValue("theta_dot_RW", z(6));  // 시간 기록
      WIFI_Logger.logValue("theta_dot_LW", z(7));  // 시간 기록

      WIFI_Logger.logValue("current_RW", iq_vec(0));
      WIFI_Logger.logValue("current_LW", iq_vec(1));
      ///////////////////////////////////////////////////////////////////////////////////////////////
    } else if (receiver.isReset()) {
      // Estimator Reset
      x.setZero();
      u.setZero();
      Estimator.reset_estimator();
      WIFI_Logger.resetLogData();
      time_ref = millis();
    } else {
      // Off Mode
      ServoLW.sendTorqueControlCommand(0);
      ServoRW.sendTorqueControlCommand(0);
      u.setZero();

      WIFI_Logger.handleClientRequests();  // Log Data 전송

      Pol.setHR(h_d, phi_d);
      Pol.calculate_com_and_inertia();
      Pol.get_theta_eq(x_d(0));

      Pol.setState(x);
      Pol.setInput(u);

      // measurement update
      MPU6050.readData();
      MPU6050.getIMUMeasurement(z);
      VYB_controller.getMotorSpeedMeasurement(z);
      // state estimation
      if (!Estimator.estimate_state(x, z)) {
        // Diverge Safe Gaurd
        ServoLW.sendTorqueControlCommand(0);
        ServoRW.sendTorqueControlCommand(0);
        while (true) {
          Serial.println("Mass matrix Singularity Error. Change mode to Off Mode...");
          x.setZero();
          u.setZero();
          Estimator.reset_estimator();
          if (receiver.readData()) {
            receiver.updateData();
          }
          delay(500);
          if (!receiver.isRun()) {
            return;
          }
        }
      }

      ///// Logging /////
      // WIFI_Logger.logTimeStamp(millis() - time_ref); // 시간 기록

      // WIFI_Logger.logValue("h_d", h_d);
      // WIFI_Logger.logValue("theta_eq", x_d(0));
      // WIFI_Logger.logValue("v_d", x_d(2));
      // WIFI_Logger.logValue("psi_dot_d", x_d(3));

      // WIFI_Logger.logValue("theta_hat", x(0));
      // WIFI_Logger.logValue("theta_dot_hat", x(1));
      // WIFI_Logger.logValue("v_hat", x(2));
      // WIFI_Logger.logValue("psi_dot_hat", x(3));

      // WIFI_Logger.logValue("tau_RW", u(0));
      // WIFI_Logger.logValue("tau_LW", u(1));

      // WIFI_Logger.logValue("acc_x", z(0));
      // WIFI_Logger.logValue("acc_y", z(1));
      // WIFI_Logger.logValue("acc_z", z(2));
      // WIFI_Logger.logValue("gyr_x", z(3));
      // WIFI_Logger.logValue("gyr_y", z(4));
      // WIFI_Logger.logValue("gyr_z", z(5));
      // WIFI_Logger.logValue("theta_dot_RW", z(6));
      // WIFI_Logger.logValue("theta_dot_LW", z(7));
      ////////////////////
      //  Estimator.printAllEstimationData();
      serialPrintStates();
      Serial.println(" Off Mode");
    }
  }
}


  void serialPrintStates() {
    Serial.print("theta:");
    Serial.print(x(0) * 180 / M_PI, 6);
    Serial.print(" ");
    Serial.print("theta_dot:");
    Serial.print(x(1) * 180 / M_PI, 6);
    Serial.print(" ");
    Serial.print("v:");
    Serial.print(x(2), 6);
    Serial.print(" ");
    Serial.print("psi_dot:");
    Serial.print(x(3) * 180 / M_PI, 6);
    Serial.print(" ");
    Serial.print("u_RW:");
    Serial.print(u(0), 6);
    Serial.print(" ");
    Serial.print("u_LW:");
    Serial.print(u(1), 6);
    Serial.print(" ");
    Serial.print("acc_x:");
    Serial.print(z(0), 6);
    Serial.print(" ");
    Serial.print("acc_y:");
    Serial.print(z(1), 6);
    Serial.print(" ");
    Serial.print("acc_z:");
    Serial.print(z(2), 6);
    Serial.print(" ");
    Serial.print("gyro_x:");
    Serial.print(z(3), 6);
    Serial.print(" ");
    Serial.print("gyro_y:");
    Serial.print(z(4), 6);
    Serial.print(" ");
    Serial.print("gyro_z:");
    Serial.print(z(5), 6);
    Serial.print(" ");    
  }