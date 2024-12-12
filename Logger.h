#ifndef LOGGER_H
#define LOGGER_H

#include <WiFi.h>     // WiFi 관련
#include <Arduino.h>  // 기본 Arduino 함수들
#include "Params.h"
#include <map>

class Logger {
private:
  WiFiServer server;
  const char* ssid;
  const char* password;

  // 데이터 저장소: 필드명을 키로 사용
  std::map<String, std::vector<float>> dataStorage;
  std::vector<uint32_t> timeStamps;

public:
  Logger(const char* wifiSSID, const char* wifiPassword, int port = 80)
    : server(port), ssid(wifiSSID), password(wifiPassword) {}

  void begin() {
    WiFi.begin(ssid, password);

    while (WiFi.status() != WL_CONNECTED) {
      delay(1000);
      Serial.println("Connecting to WiFi...");
    }

    Serial.println("Connected to WiFi.");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
    server.begin();
  }

  void logTimeStamp(uint32_t timestamp) {
    timeStamps.push_back(timestamp);
  }

  void logValue(const String& fieldName, float value) {
    dataStorage[fieldName].push_back(value);
  }

  void sendCSVFile(WiFiClient& client) {
    size_t dataSize = timeStamps.size();
    if (dataSize == 0) return;

    client.println("HTTP/1.1 200 OK");
    client.println("Content-Type: text/csv");
    client.println("Connection: close");
    client.println();

    // 헤더 생성
    client.print("TimeStamp,");
    for (const auto& field : dataStorage) {
      client.print(field.first);
      client.print(",");
    }
    client.println();

    // 데이터 생성
    for (size_t i = 0; i < dataSize; i++) {
      client.print(timeStamps[i]);
      client.print(",");
      for (const auto& field : dataStorage) {
        // 데이터를 벡터 크기 내에서 출력, 없으면 빈 값
        if (i < field.second.size()) {
          client.print(field.second[i], 4);  // 소수점 4자리까지 출력
          client.print(",");
        } else {
          client.print(",");
        }
      }
      client.println();
    }

    Serial.println("CSV file sent to client.");
  }

  void handleClientRequests() {
    WiFiClient client = server.accept();
    if (client) {
      Serial.println("New Client Connected.");
      String request = client.readStringUntil('\r');
      client.clear();

      if (request.indexOf("/logdata") != -1) {
        sendCSVFile(client);
      }

      client.stop();
      Serial.println("Client Disconnected.");
    }
  }

  void resetLogData() { // LogData 리셋
    dataStorage.clear();  // 데이터 저장소 초기화
    timeStamps.clear();   // 타임스탬프 벡터 초기화
    Serial.println("LogData has been reset.");
  }
};

#endif  // LOGGER_H
