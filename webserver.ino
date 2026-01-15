#include <WiFi.h>
#include <WebServer.h>

// ================= WIFI =================
const char* ssid = "bus247";
const char* password = "ONETWOEIGHT";

// ================= WEB SERVER =================
WebServer server(80);

// ================= UART =================
// UART2 → RX=16, TX=17
HardwareSerial STM32(2);
#define STM_BAUD 115200

// ================= CURRENT VALUES =================
float currentKp = 4.4;
float currentKd = 6.0;
int currentMinSpeed = -255;
int currentBaseSpeed = 170;
int currentMaxSpeed = 255;
int currentSlowSpeed = 165;
int currentLastEndTimeout = 200;
int currentDashGracePeriod = 150;
int currentTurnSpeed = 255;
int currentPivotSpeed = -255;
int currentJunctionSpeed = 120;

// ================= DEBUG =================
#define USB_DEBUG 0

// ================= WIFI RECHECK =================
unsigned long previousWifiCheckMillis = 0;
const long wifiCheckInterval = 10000;

// ================= UART SEND =================
void sendCommandToSTM32(String command, String value) {
    String msg = command + ":" + value + "\n";
    STM32.print(msg);

#if USB_DEBUG
    Serial.print("UART → STM32: ");
    Serial.print(msg);
#endif
}

// ================= ROOT PAGE =================
void handleRoot() {
    String html = "<!DOCTYPE html><html><head><title>FLF Control</title>";
    html += "<meta name='viewport' content='width=device-width, initial-scale=1'>";
    html += "<style>";
    html += "body{font-family:sans-serif;background:#f4f4f4;margin:20px}";
    html += "form{background:#fff;padding:20px;border-radius:8px;max-width:400px;margin:auto}";
    html += "label{display:block;margin-top:10px;font-weight:bold}";
    html += "input{width:100%;padding:8px;margin-top:4px}";
    html += "input[type=submit]{background:#007bff;color:white;border:none;margin-top:15px}";
    html += "</style></head><body>";

    html += "<h2 align='center'>FLF Controller</h2>";
    html += "<form action='/set' method='POST'>";
    html += "<label>Kp</label><input name='kp' value='" + String(currentKp,3) + "'>";
    html += "<label>Kd</label><input name='kd' value='" + String(currentKd,3) + "'>";
    html += "<label>Base Speed</label><input name='base_speed' value='" + String(currentBaseSpeed) + "'>";
    html += "<label>Max Speed</label><input name='max_speed' value='" + String(currentMaxSpeed) + "'>";
    html += "<label>Slow Speed</label><input name='slow_speed' value='" + String(currentSlowSpeed) + "'>";
    html += "<label>Junction Speed</label><input name='junction_speed' value='" + String(currentJunctionSpeed) + "'>";
    html += "<label>Last End Timeout</label><input name='last_end_timeout' value='" + String(currentLastEndTimeout) + "'>";
    html += "<label>Dash Grace Period</label><input name='dash_grace_period' value='" + String(currentDashGracePeriod) + "'>";
    html += "<label>Turn Speed</label><input name='turn_speed' value='" + String(currentTurnSpeed) + "'>";
    html += "<label>Pivot Speed</label><input name='pivot_speed' value='" + String(currentPivotSpeed) + "'>";
    html += "<input type='submit' value='Update'>";
    html += "</form></body></html>";

    server.send(200, "text/html", html);
}

// ================= HELPERS =================
void updateIntParam(const char* arg, const char* cmd, int& cur) {
    if (server.hasArg(arg)) {
        int v = server.arg(arg).toInt();
        if (v != cur) {
            cur = v;
            sendCommandToSTM32(cmd, String(v));
        }
    }
}

void updateFloatParam(const char* arg, const char* cmd, float& cur) {
    if (server.hasArg(arg)) {
        float v = server.arg(arg).toFloat();
        if (v != cur) {
            cur = v;
            sendCommandToSTM32(cmd, String(v, 3));
        }
    }
}

// ================= SET HANDLER =================
void handleSet() {
    updateFloatParam("kp", "KP", currentKp);
    updateFloatParam("kd", "KD", currentKd);
    updateIntParam("base_speed", "BS", currentBaseSpeed);
    updateIntParam("max_speed", "MS", currentMaxSpeed);
    updateIntParam("slow_speed", "SS", currentSlowSpeed);
    updateIntParam("junction_speed", "JS", currentJunctionSpeed);
    updateIntParam("last_end_timeout", "LT", currentLastEndTimeout);
    updateIntParam("dash_grace_period", "DG", currentDashGracePeriod);
    updateIntParam("turn_speed", "TS", currentTurnSpeed);
    updateIntParam("pivot_speed", "PS", currentPivotSpeed);

    server.sendHeader("Location", "/");
    server.send(303);
}

// ================= SETUP =================
void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("ESP32 UART Webserver");

    // UART INIT
    STM32.begin(STM_BAUD, SERIAL_8N1, 16, 17);
    Serial.println("UART initialized on GPIO16/17");

    WiFi.begin(ssid, password);
    Serial.print("Connecting WiFi");
    int tries = 0;
    while (WiFi.status() != WL_CONNECTED && tries < 20) {
        delay(500);
        Serial.print(".");
        tries++;
    }

    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("\nWiFi FAILED");
        while (1);
    }

    Serial.println("\nWiFi connected");
    Serial.print("IP: ");
    Serial.println(WiFi.localIP());

    server.on("/", HTTP_GET, handleRoot);
    server.on("/set", HTTP_POST, handleSet);
    server.begin();

    Serial.println("HTTP server started");
}

// ================= WIFI CHECK =================
void checkWiFi() {
    if (millis() - previousWifiCheckMillis > wifiCheckInterval) {
        previousWifiCheckMillis = millis();
        if (WiFi.status() != WL_CONNECTED) {
            Serial.println("WiFi lost, reconnecting...");
            WiFi.reconnect();
        }
    }
}

// ================= LOOP =================
void loop() {
    checkWiFi();
    server.handleClient();
}
