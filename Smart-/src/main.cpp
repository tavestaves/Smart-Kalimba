#include <Arduino.h>
#include <WiFi.h>
#include <WebSocketsServer.h>

const char* ssid     = "Ncube-Wifi";
const char* password = "Qwerty02.";

WebSocketsServer wsServer(81);

const int adcPin = 36;       // ADC1 pin (safe for analogRead)
const int sampleRate = 8000; // 8kHz stream (enough for testing)

unsigned long lastSampleMicros = 0;
const unsigned long sampleInterval = 1000000UL / sampleRate;

void onWsEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t len) {
  // You can add connection logging if you want.
}

void setup() {
  Serial.begin(115200);
  // Allow USB-serial to enumerate and stabilise, then print diagnostic header
  delay(2000);
  Serial.println("ESP ACTIVE");
  Serial.println("--- Serial test @115200 ---");

  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  unsigned long start = millis();
  const unsigned long timeout = 20000; // 20 seconds timeout
  while (WiFi.status() != WL_CONNECTED && (millis() - start) < timeout) {
    Serial.print('.');
    delay(500);
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("\nWiFi connected - IP: ");
    Serial.println(WiFi.localIP()); // Check the IP to open in browser.
  } else {
    Serial.println("\nWiFi connection FAILED (timeout).");
    Serial.print("WiFi status: ");
    Serial.println(WiFi.status());
  }

  wsServer.begin();
  wsServer.onEvent(onWsEvent);

  analogReadResolution(12);  // ESP32 default is 12-bit anyway
}

void loop() {
  wsServer.loop();

  unsigned long now = micros();
  if (now - lastSampleMicros >= sampleInterval) {
    lastSampleMicros = now;

    int value = analogRead(adcPin); // 0–4095
    Serial.println(value);

    uint16_t v16 = value;           // convert to 2 bytes

    uint8_t packet[2];
    packet[0] = v16 >> 8;           // high byte
    packet[1] = v16 & 0xFF;         // low byte

    wsServer.broadcastBIN(packet, 2);
  }
}
