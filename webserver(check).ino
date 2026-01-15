#include <WiFi.h>
#include <WebServer.h>

const char* ssid = "bus247";
const char* password = "ONETWOEIGHT";

WebServer server(80);

// UART2 → GPIO16 RX, GPIO17 TX
HardwareSerial STM32(2);
#define STM_BAUD 115200

float currentKp = 4.4;
float currentKd = 6.0;
int currentBaseSpeed = 170;
int currentMaxSpeed = 255;
int currentSlowSpeed = 165;
int currentLastEndTimeout = 200;
int currentDashGracePeriod = 150;
int currentTurnSpeed = 255;
int currentPivotSpeed = -255;
int currentJunctionSpeed = 120;

void sendCommandToSTM32(String cmd, String val) {
  STM32.print(cmd + ":" + val + "\n");
}

void handleSet() {
  if (server.hasArg("kp")) {
    currentKp = server.arg("kp").toFloat();
    sendCommandToSTM32("KP", String(currentKp, 3));
  }
  if (server.hasArg("kd")) {
    currentKd = server.arg("kd").toFloat();
    sendCommandToSTM32("KD", String(currentKd, 3));
  }
  if (server.hasArg("base_speed")) {
    currentBaseSpeed = server.arg("base_speed").toInt();
    sendCommandToSTM32("BS", String(currentBaseSpeed));
  }
  if (server.hasArg("max_speed")) {
    currentMaxSpeed = server.arg("max_speed").toInt();
    sendCommandToSTM32("MS", String(currentMaxSpeed));
  }
  if (server.hasArg("slow_speed")) {
    currentSlowSpeed = server.arg("slow_speed").toInt();
    sendCommandToSTM32("SS", String(currentSlowSpeed));
  }
  if (server.hasArg("junction_speed")) {
    currentJunctionSpeed = server.arg("junction_speed").toInt();
    sendCommandToSTM32("JS", String(currentJunctionSpeed));
  }

  server.sendHeader("Location", "/");
  server.send(303);
}

void setup() {
  Serial.begin(115200);
  STM32.begin(STM_BAUD, SERIAL_8N1, 16, 17);

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) delay(500);

  server.on("/set", HTTP_POST, handleSet);
  server.begin();
}

void loop() {
  server.handleClient();
}
