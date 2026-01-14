#include <WiFi.h>
#include <WebServer.h>

/* ===================== WIFI CONFIG ===================== */
const char* ssid     = "bus247";
const char* password = "ONETWOEIGHT";

/* ===================== WEB SERVER ===================== */
WebServer server(80);

/* ===================== UART CONFIG ===================== */
// ESP32 UART2 → STM32 UART (PB6 / PB7)
#define STM32_RX 16   // ESP32 RX  ← STM32 PB6 (TX)
#define STM32_TX 17   // ESP32 TX  → STM32 PB7 (RX)
#define UART_BAUD 115200

/* ===================== CURRENT VALUES ===================== */
float currentKp = 4.4;
float currentKd = 6.0;

int currentBaseSpeed = 170;
int currentMaxSpeed = 255;
int currentSlowSpeed = 165;
int currentJunctionSpeed = 120;

int currentLastEndTimeout = 200;
int currentDashGracePeriod = 150;

int currentTurnSpeed = 255;
int currentPivotSpeed = -255;

/* ===================== DEBUG ===================== */
#define USB_DEBUG 1

/* ===================== WIFI CHECK ===================== */
unsigned long previousWifiCheckMillis = 0;
const long wifiCheckInterval = 10000;

/* ===================== UART SEND FUNCTION ===================== */
void sendCommandToSTM32(const String& command, const String& value) {
    String fullCmd = command + ":" + value + "\n";   // newline REQUIRED

    Serial2.print(fullCmd);

#if USB_DEBUG
    Serial.print("UART → STM32: ");
    Serial.print(fullCmd);
#endif
}

/* ===================== HTML PAGE ===================== */
void handleRoot() {
    String html = "<!DOCTYPE html><html><head><title>FLF Control</title>";
    html += "<meta name='viewport' content='width=device-width, initial-scale=1'>";
    html += "<style>";
    html += "body{font-family:sans-serif;background:#f4f4f4;margin:20px;}";
    html += "form{background:#fff;padding:20px;border-radius:8px;max-width:420px;margin:auto;}";
    html += "label{font-weight:bold;display:block;margin-top:10px;}";
    html += "input{width:100%;padding:8px;margin-top:5px;}";
    html += "button{margin-top:15px;padding:10px;width:100%;font-size:16px;}";
    html += "</style></head><body>";

    html += "<h2 align='center'>FLF Controller</h2>";

    html += "<form action='/set' method='POST'>";

    html += "<label>Kp</label><input type='number' step='0.01' name='kp' value='" + String(currentKp,3) + "'>";
    html += "<label>Kd</label><input type='number' step='0.01' name='kd' value='" + String(currentKd,3) + "'>";

    html += "<label>Base Speed</label><input type='number' name='base_speed' value='" + String(currentBaseSpeed) + "'>";
    html += "<label>Max Speed</label><input type='number' name='max_speed' value='" + String(currentMaxSpeed) + "'>";
    html += "<label>Slow Speed</label><input type='number' name='slow_speed' value='" + String(currentSlowSpeed) + "'>";
    html += "<label>Junction Speed</label><input type='number' name='junction_speed' value='" + String(currentJunctionSpeed) + "'>";

    html += "<label>Last End Timeout (ms)</label><input type='number' name='last_end_timeout' value='" + String(currentLastEndTimeout) + "'>";
    html += "<label>Dash Grace Period (ms)</label><input type='number' name='dash_grace_period' value='" + String(currentDashGracePeriod) + "'>";

    html += "<label>Line Lost Turn Speed</label><input type='number' name='turn_speed' value='" + String(currentTurnSpeed) + "'>";
    html += "<label>Line Lost Pivot Speed</label><input type='number' name='pivot_speed' value='" + String(currentPivotSpeed) + "'>";

    html += "<button type='submit'>Update</button></form>";
    html += "</body></html>";

    server.send(200, "text/html", html);
}

/* ===================== UPDATE HELPERS ===================== */
void updateInt(const char* arg, const char* cmd, int& var) {
    if (server.hasArg(arg)) {
        int v = server.arg(arg).toInt();
        if (v != var) {
            var = v;
            sendCommandToSTM32(cmd, String(v));
        }
    }
}

void updateFloat(const char* arg, const char* cmd, float& var) {
    if (server.hasArg(arg)) {
        float v = server.arg(arg).toFloat();
        if (v != var) {
            var = v;
            sendCommandToSTM32(cmd, String(v, 3));
        }
    }
}

/* ===================== HANDLE FORM ===================== */
void handleSet() {
    updateFloat("kp", "KP", currentKp);
    updateFloat("kd", "KD", currentKd);

    updateInt("base_speed", "BS", currentBaseSpeed);
    updateInt("max_speed", "MS", currentMaxSpeed);
    updateInt("slow_speed", "SS", currentSlowSpeed);
    updateInt("junction_speed", "JS", currentJunctionSpeed);

    updateInt("last_end_timeout", "LT", currentLastEndTimeout);
    updateInt("dash_grace_period", "DG", currentDashGracePeriod);

    updateInt("turn_speed", "TS", currentTurnSpeed);
    updateInt("pivot_speed", "PS", currentPivotSpeed);

    server.sendHeader("Location", "/");
    server.send(303);
}

/* ===================== SETUP ===================== */
void setup() {
    Serial.begin(115200);
    delay(500);

    Serial.println("\nESP32 WebServer + UART Controller");

    Serial2.begin(UART_BAUD, SERIAL_8N1, STM32_RX, STM32_TX);
    Serial.println("UART2 on GPIO16 (RX), GPIO17 (TX)");

    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);

    Serial.print("Connecting to WiFi");
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }

    Serial.println("\nConnected!");
    Serial.print("IP: http://");
    Serial.println(WiFi.localIP());

    server.on("/", HTTP_GET, handleRoot);
    server.on("/set", HTTP_POST, handleSet);
    server.begin();

    Serial.println("HTTP server started");
}

/* ===================== WIFI WATCHDOG ===================== */
void checkWiFi() {
    unsigned long now = millis();
    if (WiFi.status() != WL_CONNECTED &&
        now - previousWifiCheckMillis >= wifiCheckInterval) {
        Serial.println("WiFi lost, reconnecting...");
        WiFi.disconnect();
        WiFi.reconnect();
        previousWifiCheckMillis = now;
    }
}

/* ===================== LOOP ===================== */
void loop() {
    checkWiFi();
    server.handleClient();
    delay(5);
}
