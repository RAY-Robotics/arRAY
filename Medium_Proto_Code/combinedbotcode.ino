#include <WiFi.h>
#include <WebServer.h>

const char* ssid = "Send Noodles";
const char* password = "Milomila";

// Pins
const int motorPin1 = 27;
const int motorPin2 = 33;

// PWM
const int freq = 30000;
const int resolution = 8;
const int motorSpeed = 255; // Max speed

WebServer server(80);

// --- HELPER: Send Response with CORS Headers ---
// This allows a website hosted elsewhere (your PC) to control this ESP32
void sendCORS(int statusCode, String content) {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.sendHeader("Access-Control-Allow-Methods", "GET");
  server.sendHeader("Access-Control-Allow-Headers", "Content-Type");
  server.send(statusCode, "text/plain", content);
}

void stopMotor() {
  ledcWrite(motorPin1, 0);
  ledcWrite(motorPin2, 0);
  sendCORS(200, "Stopped");
}

void moveForward() {
  ledcWrite(motorPin1, motorSpeed);
  ledcWrite(motorPin2, 0);
  sendCORS(200, "Forward");
}

void moveBackward() {
  ledcWrite(motorPin1, 0);
  ledcWrite(motorPin2, motorSpeed);
  sendCORS(200, "Backward");
}

void setup() {
  Serial.begin(115200);
  
  // PWM Setup (v3.0 syntax)
  ledcAttach(motorPin1, freq, resolution);
  ledcAttach(motorPin2, freq, resolution);

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi Connected!");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP()); // <-- WRITE THIS DOWN FOR EACH BOARD

  // Routes
  server.on("/forward", moveForward);
  server.on("/backward", moveBackward);
  server.on("/stop", stopMotor);
  
  // Handle "Pre-flight" requests (Browsers check this before sending commands)
  server.onNotFound([]() {
    if (server.method() == HTTP_OPTIONS) {
      server.sendHeader("Access-Control-Allow-Origin", "*");
      server.send(204);
    } else {
      server.send(404, "text/plain", "Not Found");
    }
  });

  server.begin();
}

void loop() {
  server.handleClient();
}