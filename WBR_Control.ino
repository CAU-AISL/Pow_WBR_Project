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

int i = 0;
unsigned long previousMillis = 0;

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

  unsigned long startMicros = micros();
  unsigned long timeout = 5000 * 1000;  // 타임아웃 5초 설정
  bool dataReceived = false;

  // SBUS 데이터 수신 대기 (타임아웃 처리)
  while (!dataReceived) {
    dataReceived = receiver.readData();  // 데이터가 수신되면 true
    if (micros() - startMicros > timeout) {
      Serial.println("Timeout: No data received from SBUS.");
      startMicros = micros();  // 타임아웃 초기화
    }
  }
  receiver.updateData();
}

void loop() {
  unsigned long currentMillis = millis();  // 현재 시간

  // 12ms가 경과했을 때만 실행
  if (currentMillis - previousMillis >= dt*1000) {
    // 경과 시간 출력
    Serial.print("Sampling Time (ms): ");
    Serial.println(currentMillis - previousMillis);
    previousMillis = currentMillis;  // 마지막 실행 시간을 현재 시간으로 업데이트


    if (receiver.readData()) {
      receiver.updateData();
    }

    if (receiver.isRun()) {
      float h_d, phi_d;
      float v_d, dpsi_d;
      receiver.updateDesiredStates();
      h_d = receiver.getDesiredHeight();
      // phi_d = receiver.getDesiredRoll();
      phi_d = 0;  // roll control diable
      v_d = receiver.getDesiredVel();
      dpsi_d = receiver.getDesiredYawVel();


      Pol.setHR(h_d, phi_d);
      Pol.calculate_com_and_inertia();
      Pol.get_theta_eq(x_d(0));
      x_d.segment<2>(2) << v_d, dpsi_d;

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
      };
      // Estimator.printAllEstimationData();
      HR_controller.controlHipServos(Pol.get_theta_hips());

      VYB_controller.computeGainK(h_d);
      VYB_controller.computeInput(x_d, x);
      VYB_controller.sendControlCommand();
      u = VYB_controller.getInputVector();
      Pol.setState(x);
      Pol.setInput(u);
      // MPU6050.printData();
      Serial.print("theta:");Serial.print(x(0), 6);Serial.print(" ");
      Serial.print("theta_dot:");Serial.print(x(1), 6);Serial.print(" ");
      Serial.print("v:");Serial.print(x(2), 6);Serial.print(" ");
      Serial.print("psi_dot:");Serial.print(x(3), 6);Serial.print(" ");
      Serial.print("u_RW:");Serial.print(u(0), 6);Serial.print(" ");
      Serial.print("u_LW:");Serial.print(u(1), 6);Serial.print(" ");

    } else if (receiver.isReset()){
      x.setZero();
      u.setZero();
      Estimator.reset_estimator();
    } else {
      
      Serial.println("Off Mode");
      ServoLW.sendTorqueControlCommand(0);
      ServoRW.sendTorqueControlCommand(0);
      
      float h_d, phi_d;
      float v_d, dpsi_d;
      receiver.updateDesiredStates();
      h_d = receiver.getDesiredHeight();
      // phi_d = receiver.getDesiredRoll();
      phi_d = 0;  // roll control diable
      v_d = receiver.getDesiredVel();
      dpsi_d = receiver.getDesiredYawVel();


      Pol.setHR(h_d, phi_d);
      Pol.calculate_com_and_inertia();
      Pol.get_theta_eq(x_d(0));
      x_d.segment<2>(2) << v_d, dpsi_d;

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
      //  Estimator.printAllEstimationData();
      Serial.print("theta:");Serial.print(x(0)*180/M_PI, 6);Serial.print(" ");
      Serial.print("theta_dot:");Serial.print(x(1)*180/M_PI, 6);Serial.print(" ");
      Serial.print("v:");Serial.print(x(2), 6);Serial.print(" ");
      Serial.print("psi_dot:");Serial.print(x(3)*180/M_PI, 6);Serial.print(" ");
      Serial.print("u_RW:");Serial.print(u(0), 6);Serial.print(" ");
      Serial.print("u_LW:");Serial.print(u(1), 6);Serial.print(" ");
    }
  }
}
