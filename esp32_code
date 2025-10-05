#include <WiFi.h>
#include <HTTPClient.h>
#include "HardwareSerial.h"

const char* ssid = "123456";  
const char* password = "88888887";
const char* serverURL = " "; //google app script

HardwareSerial mySerial(1);
char buffer[32];

void setup() {
    Serial.begin(115200);
    WiFi.begin(ssid, password);
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 20) {
        delay(1000);
        Serial.println("Đang kết nối WiFi...");
        attempts++;
    }
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("Không thể kết nối WiFi!");
        return;
    }
    Serial.println("WiFi đã kết nối!");
    mySerial.begin(115200, SERIAL_8N1, 16, 17); 
}

void loop() {
    if (WiFi.status() == WL_CONNECTED) {
        if (mySerial.available() >= 9) {
            int len = mySerial.readBytes(buffer, 32);  
            buffer[len] = '\0';
            Serial.printf("Chuỗi nhận được: '%s'\n", buffer);
            char mau_hong[4] = {0}, mau_cam[4] = {0}, mau_tim[4] = {0};
            memcpy(mau_hong, buffer, 3);
            memcpy(mau_cam, buffer + 3, 3);
            memcpy(mau_tim, buffer + 6, 3);
            if (strlen(mau_hong) == 3 && strlen(mau_cam) == 3 && strlen(mau_tim) == 3) {
                Serial.printf("Nhận dữ liệu - mauhong:%s, maucam:%s, mautim:%s\n", 
                            mau_hong, mau_cam, mau_tim);
                HTTPClient http;
                String url = String(serverURL) + "?mau_hong=" + String(mau_hong) + 
                           "&mau_cam=" + String(mau_cam) + "&mau_tim=" + String(mau_tim);
                http.begin(url);
                int httpCode = http.GET();
                if (httpCode > 0) {
                    Serial.println("Gửi thành công!");
                } else {
                    Serial.printf("Lỗi gửi HTTP: %s\n", http.errorToString(httpCode).c_str());
                }
                http.end();
                delay(100);
            } else {
                Serial.println("Dữ liệu không hợp lệ!");
            }
        }
    } else {
        Serial.println("WiFi chưa kết nối!");
        WiFi.reconnect();
        delay(5000);
    }
    delay(500);
}
