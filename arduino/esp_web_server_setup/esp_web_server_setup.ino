#include <String.h>

#include <ESP8266WiFi.h>
#include <ESPAsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <WebSerial.h>

#define MESSAGE_SIZE 35

/* Put your SSID & Password */
const char* ssid = "ESP8266 D1 Mini Network";  // Enter SSID here
const char* password = "12345678";  //Enter Password here
//const char* ssid = "Mechatronics";  // Enter SSID here
//const char* password = "6476775503";  //Enter Password here

// WASD Socket Connection
const char* host = "192.168.4.2";
const uint16_t port = 8090;

String message;
char c;
int ind = 0;

/* Put IP Address details */
//ESP8266WebServer server(80);

AsyncWebServer server(80);

void recvMsg(uint8_t *data, size_t len) {
  WebSerial.println("Received Data...");
  String d = "";
  for (int i = 0; i < len; i++) {
    d += char(data[i]);
  }
  WebSerial.println(d);
  if (d == "ON") {
    digitalWrite(LED_BUILTIN, LOW);
  } else if (d == "OFF") {
    digitalWrite(LED_BUILTIN, HIGH);
  }

  if (len == 1) {
    /* Switch for wasd movement (but s is stop not backwards) */
    switch (d[0]) {
      case 'w':
        Serial.write("forward");
        WebSerial.println("forward");
        break;
      case 'a':
        Serial.write("left");
        WebSerial.println("left");
        break;
      case 'd':
        Serial.write("right");
        WebSerial.println("right");
        break;
      case 's':
        Serial.write("stop");
        WebSerial.println("stop");
        break;
    }
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  delay(1000);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(100);

  /* for Access Point (ESP hosted network) */
    WiFi.softAP(ssid, password);
  
  /* for Station (ESP joins network) */
//  WiFi.mode(WIFI_STA);
//  WiFi.begin(ssid, password);
//  delay(1000);
//  if (WiFi.waitForConnectResult() != WL_CONNECTED) {
//    Serial.printf("WiFi Failed!\n");
//    return;
//  }
  
//  Serial.println("IP Address: ");
//  Serial.println(WiFi.localIP());
  
  // WebSerial is accessible at "<IP Address>/webserial" in browser
  WebSerial.begin(&server);
  WebSerial.msgCallback(recvMsg);
  server.begin();
}

void loop() {
  WiFiClient client;

  if (Serial.available() > 0) {
    message = Serial.readStringUntil('\n');
    WebSerial.println(message);
    //    if (message == "\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0Encoder Count: 0"){
    //      digitalWrite(LED_BUILTIN, LOW);
    //      delay(1000);
    //      digitalWrite(LED_BUILTIN, HIGH);
    //      delay(100);
    //    }
    //    c = Serial.read();
    //    if(ind > (MESSAGE_SIZE - 1)){
    //      WebSerial.println("Serial input message too large...");
    //    }
    //    message[ind] = c;
    //    ind++;
    //    if (c == '\n')
    //    {
    //      WebSerial.println(message);
    //      delay(100);
    //      memset(message, 0, sizeof(message));
    //      ind = 0;
    //    }
  }
  if (!client.connect(host, port)) {
      Serial.println("Connection to WASD host failed");
      delay(1000);
      return;
  } else {
    Serial.println("Connected to WASD server successful!");
    client.print("Hello from ESP8266!");
    while (client.connected()){
      if (client.available()) {
        char c = client.read();
//        rec_data += c;
        Serial.print(c);
        Serial.println();
      }
    }
    Serial.println("Disconnecting...");
    client.stop();
  }
//  WebSerial.println("Heartbeat <3");
  delay(100);
}
