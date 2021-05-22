// DHT Section
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
//General
#include <ESP8266WiFi.h>
#include <ros.h>
#include <cranberry_topic/AcData.h>
DHT_Unified dht(DHTPIN, DHTTYPE);

#define DHTTYPE    DHT22     // DHT 22 (AM2302)
#define LIGHT_CODE 1 // LIGHT_CODE는 임의 지정
#define DHT_PIN D2
#define IR_PIN D3
#define RELAY1_PIN D4
#define RELAY2_PIN D5
#define MOTION_PIN D6

////////////WIFI/////////////////
const char* ssid     = "SSID";
const char* password = "PASSWORD";
IPAddress server(192, 168, 0, 19); // ROS가 실제로 작동하는 PC의 외부 ip주소
const uint16_t serverPort = 11411;
////////////WIFI_END/////////////////


float temp = 0;
float humid = 0;

//////////////ROS////////////////
ros::NodeHandle nh;

cranberry_topic::AcData ac_sensor_msg;
ros::Publisher pub_ac("ac_msg", &ac_sensor_msg);

void setup()
{
  Serial.begin(115200);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  // wifi AP와 NodeMCU 연결
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  // rosserial 소켓 서버와 연길 시도
  nh.getHardware()->setConnection(server, serverPort);
  nh.initNode();

  // ip 얻기 
  Serial.print("IP = ");
  Serial.println(nh.getHardware()->getLocalIP());

  nh.advertise(pub_ac);
}

void loop()
{
  if (nh.connected()) {
    Serial.print("Connected : sending :"); Serial.println(temp);
    // Say hello
    ac_sensor_msg.temp = temp; // remove numbers below decimal point 2
    ac_sensor_msg.humid = humid;
    pub_ac.publish( &ac_sensor_msg );
  } else {
    Serial.println("Not Connected");
  }
  nh.spinOnce();
  // Loop exproximativly at 1Hz
}
