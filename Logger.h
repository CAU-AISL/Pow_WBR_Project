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
    WiFi.mode(WIFI_STA);  // 스테이션 모드로 설정
    WiFi.begin(ssid, password);

    while (WiFi.status() != WL_CONNECTED) {
      delay(1000);
      Serial.print("Connecting to WiFi...");
      Serial.print(" | WiFi Status: ");
      Serial.println(WiFi.status());  // 상태 코드 출력
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

    // HTTP 헤더 한번만 보냄
    client.println("HTTP/1.1 200 OK");
    client.println("Content-Type: text/csv");
    client.println("Connection: keep-alive");  // 연결을 유지하도록 설정
    client.println();

    // StringBuffer를 사용하여 데이터를 한번에 보내기 위한 버퍼 준비
    String csvData;
    csvData.reserve(2048);  // 적당한 크기로 예약

    // 헤더 생성
    csvData += "TimeStamp,";
    for (const auto& field : dataStorage) {
        csvData += field.first + ",";
    }
    csvData += "\n";  // 헤더 끝

    // 데이터를 청크 단위로 보내기 위한 설정
    size_t chunkSize = 1024;  // 한번에 전송할 데이터 크기 (최적화 필요)
    size_t currentSize = 0;

    // 데이터 생성
    for (size_t i = 0; i < dataSize; i++) {
        csvData += String(timeStamps[i]) + ",";
        for (const auto& field : dataStorage) {
            if (i < field.second.size()) {
                csvData += String(field.second[i], 8) + ",";  // 소수점 8자리까지
            } else {
                csvData += ",";
            }
        }
        csvData += "\n";  // 데이터 한 줄 끝

        // 청크 크기가 되면 데이터를 전송
        currentSize += csvData.length();
        if (currentSize >= chunkSize) {
            client.print(csvData);
            csvData = "";  // 버퍼 비우기
            currentSize = 0;  // 크기 초기화
        }
    }

    // 남은 데이터 전송
    if (csvData.length() > 0) {
        client.print(csvData);
    }

    Serial.println("CSV file sent to client.");
}

  void handleClientRequests() {
    WiFiClient client = server.available();

    // Wi-Fi 연결 상태 확인
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("WiFi not connected. Attempting to reconnect...");
      WiFi.disconnect();
      WiFi.reconnect();

      // 재연결 대기
      while (WiFi.status() != WL_CONNECTED) {
        delay(1000);
        Serial.print("Reconnecting to WiFi...");
        Serial.println(" | WiFi Status: " + String(WiFi.status()));
      }

      Serial.println("WiFi reconnected successfully.");
      Serial.print("IP Address: ");
      Serial.println(WiFi.localIP());
    }

    // 클라이언트 요청 처리
    if (client) {
      Serial.println("New Client connected.");
      delay(500);
      String request = client.readStringUntil('\r');
      client.flush();

      // 요청 처리 로직
      if (request.indexOf("/logdata") != -1) {
        delay(500);
        Serial.println("Start send CSV File!");
        sendCSVFile(client);
      }

      client.stop();
      Serial.println("Client disconnected.");
    }
  }


  void resetLogData() {   // LogData 리셋
    dataStorage.clear();  // 데이터 저장소 초기화
    timeStamps.clear();   // 타임스탬프 벡터 초기화
    Serial.println("LogData has been reset.");
  }
};

#endif  // LOGGER_H
