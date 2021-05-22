#define UNIT_ID 1 // living room
#define UNIT_ID1 2 // kitchen
#define LIGHT_PIN D5
#define LIGHT_PIN1 D5


#include <ESP8266WiFi.h>
#include <ros.h>
#include <cranberry_topic/AcData.h> //arduino dosen'tsupport float64
#include <cranberry_topic/CommBoiler.h> //arduino dosen'tsupport float64


void callback(const cranberry_topic::CommLight::Request &req, cranberry_topic::CommLight::Response &res){
    // mode description
    // 1 : manual on/off
    // 2 : control light according to BMNT / EENT
    // 3 : Force on
    // 4 : Force off

    switch(req.mode){
        case 1:
            if(target == UNIT_ID) digitalWrite(LIGHT_PIN, state);
            if(target == UNIT_ID1) digitalWrite(LIGHT_PIN1, state);
            break;
        
        case 2:
            // to be continued...
            break;
        
        case 3:
            digitalWrite(LIGHT_PIN, HIGH);
            digitalWrite(LIGHT_PIN1, HIGH);
            break;

        case 4:
            digitalWrite(LIGHT_PIN, LOW);
            digitalWrite(LIGHT_PIN1, LOW);
            break;


    }  
}

////////////WIFI/////////////////
const char* ssid     = "IoT_System";
const char* password = "nodemcu01";
IPAddress server(192, 168, 0, 19); //input physical ip of which ros is running
const uint16_t serverPort = 11411;

//////////////ROS////////////////
ros::NodeHandle nh;
using cranberry_topic::CommBoiler;

ros::ServiceServer<cranberry_topic::CommLight::Request, cranberry_topic::CommLight::Response> srvServer("comm_boiler", &callback);

void setup(){

  // Use ESP8266 serial to monitor the process
  Serial.begin(9600);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  // Connect the ESP8266 the the wifi AP
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  // Set the connection to rosserial socket server
  nh.getHardware()->setConnection(server, serverPort);
  nh.initNode();

  // Another way to get IP
  Serial.print("IP = ");
  Serial.println(nh.getHardware()->getLocalIP());
  nh.advertiseService(srvServer);
}

void loop(){
    if (nh.connected()) {
    } 
    else {
        Serial.println("Not Connected");

    nh.spinOnce();
}