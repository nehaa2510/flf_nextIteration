#include <WiFi.h>
#include <WebServer.h>

const char* ssid = "bus247";
const char* password = "ONETWOEIGHT";

WebServer server(80);

// ================= UART =================
// UART2 → GPIO16 RX, GPIO17 TX
HardwareSerial STM32(2);
#define STM_BAUD 115200

// ================= CURRENT VALUES =================
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

// ================= UART SEND =================
void sendCommandToSTM32(String cmd, String val) {
  String msg = cmd + ":" + val + "\n";
  STM32.print(msg);
  Serial.print("UART → STM32: ");
  Serial.print(msg);
}

// ================= ROOT PAGE =================
void handleRoot() {
  server.send(200, "text/plain",
              "ESP32 Webserver OK\nUse POST /set to update parameters");
}

// ================= SET HANDLER =================
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

  if (server.hasArg("last_end_timeout")) {
    currentLastEndTimeout = server.arg("last_end_timeout").toInt();
    sendCommandToSTM32("LT", String(currentLastEndTimeout));
  }

  if (server.hasArg("dash_grace_period")) {
    currentDashGracePeriod = server.arg("dash_grace_period").toInt();
    sendCommandToSTM32("DG", String(currentDashGracePeriod));
  }

  if (server.hasArg("turn_speed")) {
    currentTurnSpeed = server.arg("turn_speed").toInt();
    sendCommandToSTM32("TS", String(currentTurnSpeed));
  }

  if (server.hasArg("pivot_speed")) {
    currentPivotSpeed = server.arg("pivot_speed").toInt();
    sendCommandToSTM32("PS", String(currentPivotSpeed));
  }

  server.sendHeader("Location", "/");
  server.send(303);
}

// ================= SETUP =================
void setup() {
  Serial.begin(115200);

  // UART to STM32
  STM32.begin(STM_BAUD, SERIAL_8N1, 16, 17);

  // WiFi with timeout
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  int retries = 0;
  while (WiFi.status() != WL_CONNECTED && retries < 20) {
    delay(500);
    Serial.print(".");
    retries++;
  }

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("\nWiFi FAILED");
    return;
  }

  Serial.println("\nWiFi connected");
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());

  server.on("/", HTTP_GET, handleRoot);
  server.on("/set", HTTP_POST, handleSet);
  server.begin();

  Serial.println("HTTP server started");
}

// ================= LOOP =================
void loop() {
  server.handleClient();
}
