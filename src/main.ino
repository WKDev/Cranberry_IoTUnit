//Comprehensive END-SIDE IoT UNIT
//Coded April 07, 2021
// Supports :
//OTA Upload
//HTTP Server(controls light, boiler remotely)
//AC Measurement(When attached DHT22)
// 192.168.0.5 ACUNIT
// 192,168.0.33 LIGHTUNIT

//Arduino OTA
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <ArduinoJson.h>

// HTTPServer
#include <WiFiClient.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>

#ifndef STASSID
#define STASSID "IoT_System"
#define STAPSK "nodemcu01"
#endif

//DHT22
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>

//DEFINE PINS --- SHOULD REMAP PIN!!!!!!!!!!!!!!!!
#define DHTTYPE DHT22 // DHT 22 (AM2302)
// #define LIGHT_CODE 1  // LIGHT_CODE는 임의 지정
#define DHTPIN 4 // DHT22 D2
// #define IR_PIN D3

#define UNIT_TYPE "Temp & Humidity"
#define RELAY1 14 // light D5
#define RELAY2 12 // light D6
#define RELAY3 13 // boiler D7
#define RELAY4 15 // light D8

DHT_Unified dht(DHTPIN, DHTTYPE);

// ACData DHT22 --- D2? --> 이부분 완성
// Boiler
// Light 1
// Light 2

// Common Relay --- D5, D6

float currentTemp = 0;
float currentHumid = 0;

const char *ssid = STASSID;
const char *password = STAPSK;

bool light1State = false;
bool light2State = false;

bool boilerState = false;

ESP8266WebServer server(80);

// Serving Hello world
void getHelloWord()
{
  DynamicJsonDocument doc(512);
  doc["name"] = "Hello world";

  Serial.print(F("Stream..."));
  String buf;
  serializeJson(doc, buf);
  server.send(200, "application/json", buf);
  Serial.print(F("done."));
}
void getACData()
{
  tempEvent();
  DynamicJsonDocument doc(512);
  doc["temp"] = currentTemp;
  doc["humid"] = currentHumid;

  Serial.print(F("Stream..."));
  String buf;
  serializeJson(doc, buf);
  server.send(200, "application/json", buf);
  Serial.print(F("done."));
}

void handleLight()
{
  Serial.println(server.args());
  Serial.print("target :  " + server.arg("target"));
  if (server.arg("target") == "light1")
  {
    if (server.arg("state") == "true")
    {
      digitalWrite(RELAY1, HIGH);
    }
    else
    {
      digitalWrite(RELAY1, LOW);
    }
  }

  if (server.arg("target") == "light2")
  {
    if (server.arg("state") == "true")
    {
      digitalWrite(RELAY2, HIGH);
    }
    else
    {
      digitalWrite(RELAY2, LOW);
    }
  }

  // GPIO 기준 제어
  DynamicJsonDocument doc(512);
  doc["light1"] = light1State;
  doc["light2"] = light2State;

  // Serial.print(F("Stream..."));
  String buf;
  serializeJson(doc, buf);
  server.send(200, "application/json", buf);
  // Serial.println(F("done."));
}

void handleBoiler()
{

  if (boilerState)
  {
    digitalWrite(RELAY3, LOW);
    boilerState = false;
  }
  else
  {
    digitalWrite(RELAY3, HIGH);
    boilerState = true;
  }

  // GPIO 기준 제어
  DynamicJsonDocument doc(512);
  doc["state"] = boilerState;

  Serial.print(F("Stream..."));
  String buf;
  serializeJson(doc, buf);
  server.send(200, "application/json", buf);
  Serial.println(F("done."));
}
// Serving Hello world
void getSettings()
{
  // Allocate a temporary JsonDocument
  // Don't forget to change the capacity to match your requirements.
  // Use arduinojson.org/v6/assistant to compute the capacity.
  //  StaticJsonDocument<512> doc;
  // You can use DynamicJsonDocument as well
  DynamicJsonDocument doc(512);

  doc["ip"] = WiFi.localIP().toString();
  doc["gw"] = WiFi.gatewayIP().toString();
  doc["nm"] = WiFi.subnetMask().toString();

  if (server.arg("signalStrength") == "true")
  {
    doc["signalStrengh"] = WiFi.RSSI();
  }

  if (server.arg("chipInfo") == "true")
  {
    doc["chipId"] = ESP.getChipId();
    doc["flashChipId"] = ESP.getFlashChipId();
    doc["flashChipSize"] = ESP.getFlashChipSize();
    doc["flashChipRealSize"] = ESP.getFlashChipRealSize();
  }
  if (server.arg("freeHeap") == "true")
  {
    doc["freeHeap"] = ESP.getFreeHeap();
  }

  Serial.print(F("Stream..."));
  String buf;
  serializeJson(doc, buf);
  server.send(200, F("application/json"), buf);
  Serial.print(F("done."));
}
void handleRoot()
{
  String message = "<h1>Welcome to Cranberry-IoT END UNIT.</h1>";
  message += "<h2>This is " + String(UNIT_TYPE) + " Unit</h2>";
  message += "<p>URI: ";
  message += server.uri();
  message += "</p><p>Method: ";
  message += (server.method() == HTTP_GET) ? "GET" : "POST";
  message += "\n</p><p>Arguments: ";
  message += server.args();
  message += "</p>";
  message += "<p>local IP : " + WiFi.localIP().toString() + "</p>";
  message += "<a href=/api/acdata>ACData</a>";
  message += "<a href=/api/light/refer>Light</a>";

  for (uint8_t i = 0; i < server.args(); i++)
  {
    message += " " + server.argName(i) + ": " + server.arg(i) + "\n";
  }
  server.send(200, "text/html", message);
}
// Define routing
void restServerRouting()
{
  // server.on("/", HTTP_GET, []() {
  //   server.send(200, F("text/html"),
  //               F("Welcome to the REST Web Server"));
  // });
  server.on("/", HTTP_GET, handleRoot);
  server.on(F("/helloWorld"), HTTP_GET, getHelloWord);
  server.on(F("/api/acdata"), HTTP_GET, getACData);
  server.on(F("/api/light"), HTTP_POST, handleLight);
  server.on(F("/api/boiler"), HTTP_GET, handleBoiler);

  server.on(F("/settings"), HTTP_GET, getSettings);
}

// Manage not found URL
void handleNotFound()
{
  String message = "File Not Found\n\n";
  message += "URI: ";
  message += server.uri();
  message += "\nMethod: ";
  message += (server.method() == HTTP_GET) ? "GET" : "POST";
  message += "\nArguments: ";
  message += server.args();
  message += "\n";
  for (uint8_t i = 0; i < server.args(); i++)
  {
    message += " " + server.argName(i) + ": " + server.arg(i) + "\n";
  }
  server.send(404, "text/plain", message);
}

void setup()
{
  //DHT22 SECTION
  dht.begin();
  sensor_t sensor;

  //WiFi CONNECTION
  Serial.begin(115200);
  Serial.println("Booting");
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.waitForConnectResult() != WL_CONNECTED)
  {
    Serial.println("Connection Failed! Rebooting...");
    delay(1000);

    ESP.restart();
  }

  // Port defaults to 8266
  // ArduinoOTA.setPort(8266);

  // Hostname defaults to esp8266-[ChipID]
  // ArduinoOTA.setHostname("myesp8266");

  // No authentication by default
  ArduinoOTA.setPassword("admin");

  // Password can be set with it's md5 value as well
  // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
  // ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");

  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH)
    {
      type = "sketch";
    }
    else
    { // U_FS
      type = "filesystem";
    }

    // NOTE: if updating FS this would be the place to unmount FS using FS.end()
    Serial.println("Start updating " + type);
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR)
    {
      Serial.println("Auth Failed");
    }
    else if (error == OTA_BEGIN_ERROR)
    {
      Serial.println("Begin Failed");
    }
    else if (error == OTA_CONNECT_ERROR)
    {
      Serial.println("Connect Failed");
    }
    else if (error == OTA_RECEIVE_ERROR)
    {
      Serial.println("Receive Failed");
    }
    else if (error == OTA_END_ERROR)
    {
      Serial.println("End Failed");
    }
  });
  ArduinoOTA.begin();
  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println("Ready to get OTAUpdate.");

  Serial.println(WiFi.localIP());

  // Activate mDNS this is used to be able to connect to the server
  // with local DNS hostmane esp8266.local
  if (MDNS.begin("esp8266"))
  {
    Serial.println("MDNS responder started");
  }

  // Set server routing
  restServerRouting();
  // Set not found response
  server.onNotFound(handleNotFound);
  // Start server
  server.begin();
  Serial.println("HTTP server started");

  pinMode(RELAY1, OUTPUT);
  digitalWrite(RELAY1, LOW);
  pinMode(RELAY2, OUTPUT);
  digitalWrite(RELAY2, LOW);
}

void loop()
{
  ArduinoOTA.handle();
  server.handleClient();
}

void tempEvent()
{
  /////////////////// DHT Section START ////////////////
  sensors_event_t event; //get TEMP Event
  dht.temperature().getEvent(&event);
  if (isnan(event.temperature))
  {
    Serial.println(F("Error reading temperature!"));
  }
  else
  {
    currentTemp = event.temperature;
  }

  dht.humidity().getEvent(&event); // Get HUMID Event
  if (isnan(event.relative_humidity))
  {
    Serial.println(F("Error reading humidity!"));
  }
  else
  {
    currentHumid = event.relative_humidity;
  }
  ///////////DHT Section END///////////////
}
