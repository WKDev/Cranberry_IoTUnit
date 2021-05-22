/*
   rosserial Publisher Example
   Prints "hello world!"
   This intend to connect to a Wifi Access Point
   and a rosserial socket server.
   You can launch the rosserial socket server with
   roslaunch rosserial_server socket.launch
   The default port is 11411
*/

#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#include <ESP8266WiFi.h>
#include <ros.h>
#include <cranberry_topic/AcData.h> //arduino dosen'tsupport float64
#include <cranberry_topic/CommBoiler.h> //arduino dosen'tsupport float64

#define DHTTYPE    DHT22     // DHT 22 (AM2302)
#define DHTPIN D2
#define RELAY_PIN D7

DHT_Unified dht(DHTPIN, DHTTYPE);
  
float currentTemp;
float currentHumid;

unsigned int long startTime=0;
unsigned int long elaspedSec=0;
unsigned int long elaspedMin=0;

long int relayState = 0;
long int boilerMode = 0;
long int boilerParam = 0;

// global Variables for boilerController
bool turnOnOffDescreminator = false; 

unsigned int long protectTimeCode = 0;
int protector = 0;
bool initProtect = true;

////////////WIFI/////////////////
const char* ssid     = "IoT_System";
const char* password = "nodemcu01";
IPAddress server(192, 168, 0, 19); //input physical ip of which ros is running
const uint16_t serverPort = 11411;


//////////////ROS////////////////
ros::NodeHandle nh;
using cranberry_topic::CommBoiler;

// 
// Request
// req.Mode : 0= off;
//            1 = keep temperture mode, parameter = temp;
//            2 = timer(if @param is 3, boiler turns on right now and turns off after 3 hrs.)
//            3 = repeated timer parameter = interval(if @param is 2, boiler turns on and off every 2hrs.)
//            4 = shower (it turns off in an hour automatically after triggered.)

void callback(const cranberry_topic::CommBoiler::Request &req, cranberry_topic::CommBoiler::Response &res){
  
  boilerMode = (long int)req.Mode;
  boilerParam = (long int)req.Parameter;
  if(req.Mode == 0){ // turn off
    Serial.println("mode : turn off");
  }
  else if (req.Mode == 1){ // keep temp mode
    Serial.println("mode : keep temp");
  }
  else if (req.Mode == 2){ // timer
    Serial.println("mode : timer");
    startTime = millis();
  }
  else if (req.Mode == 3){ // repeated timer
    Serial.println("mode : repeated timer");
    startTime = millis();
  }
  else if (req.Mode == 4){ // shower
  Serial.println("mode : shower");
  startTime = millis();
} 
}

cranberry_topic::AcData ac_sensor_msg;
ros::Publisher pub_ac("ac_msg", &ac_sensor_msg);
ros::ServiceServer<cranberry_topic::CommBoiler::Request, cranberry_topic::CommBoiler::Response> srvServer("comm_boiler", &callback);

void setup()
{
  pinMode(RELAY_PIN, OUTPUT);
  ///////////DHT Section START///////////////
  dht.begin();
  sensor_t sensor;
  //import sensor_data
  //  dht.temperature().getSensor(&sensor);
  //  // Print humidity sensor details.
  //  dht.humidity().getSensor(&sensor);
  ///////////DHT Section END///////////////

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

  // Start to be polite
  nh.advertise(pub_ac);
  nh.advertiseService(srvServer);
}

void boilerController(long int mode, long int param){
  if(mode == 0){ // turn off
    digitalWrite(RELAY_PIN, LOW);
  }

  else if(mode == 1){ //keep temp mode
  // i thought there would need tempmargin but after some investigation, it seems not necessary.
    if(currentTemp >=param){
      digitalWrite(RELAY_PIN, LOW);
    }
    else if(currentTemp< param)){
        digitalWrite(RELAY_PIN, HIGH);
    }
  }
  else if(mode == 2){ // timer, in this section, param means hour      
    //Serial.println((millis() - startTime));
    if(elaspedSec > 3600*param){
      Serial.println("time over. stop control");
      param = 0;
      digitalWrite(RELAY_PIN, LOW);
      Serial.println("param = " + param);
    }	
    else{
      digitalWrite(RELAY_PIN, HIGH);
      Serial.print("Boiler Working... | ");
      Serial.print(elaspedSec); //subtraction has to reach 3600 *1000
      //(millis() - startTime)/60000 means elasped time since the command delivered(scale : min)
      Serial.print("/");
      Serial.print(3600*param);
      Serial.println("sec has done.");
    }
  }
  else if(mode == 3){ // repeated timer, param means hour   
    Serial.println("turns on boiler now. it runs repeatedly.")
    if((elaspedMin % 60*param) == 0){ // 일정 시간 도래하면, 
      if(turnOnOffDescreminator) turnOnOffDescreminator = false;
      else turnOnOffDescreminator = true;

      Serial.print("current param : ");
      Serial.println((elaspedMin % 60*param));
    }

    if(turnOnOffDescreminator){
      digitalWrite(RELAY_PIN, HIGH);
    }
    else{
      digitalWrite(RELAY_PIN, LOW);
    }
  }

  else if(mode ==4){ // shower
      if(elaspedMin > 60){
      Serial.println("time over. stop control");
      param = 0;
      digitalWrite(RELAY_PIN, LOW);
      Serial.println("param = " + param);
    }	
    else{
      digitalWrite(RELAY_PIN, HIGH);
      Serial.print("shoer Working... | ");
      Serial.print(elaspedMin); //subtraction has to reach 3600 *1000
      //(millis() - startTime)/60000 means elasped time since the command delivered(scale : min)
      Serial.print("/");
      Serial.print(60*elaspedMin);
      Serial.println("sec has done.");
    }
  }

}

void tempEvent(){
/////////////////// DHT Section START ////////////////
  sensors_event_t event; //get TEMP Event
  dht.temperature().getEvent(&event);
    if (isnan(event.temperature)) {
      Serial.println(F("Error reading temperature!"));
    }
    else {   
      currentTemp = event.temperature;
    }
  
    dht.humidity().getEvent(&event); // Get HUMID Event
    if (isnan(event.relative_humidity)) {
      Serial.println(F("Error reading humidity!"));
    }
    else {
      currentHumid = event.relative_humidity;
    }
  ///////////DHT Section END///////////////
}

void loop()
{ 
  // this syntax used to run boilerController
  elaspedSec = (millis() - startTime) / 1000;
  elaspedMin = (elaspedSec / 60); 
  boilerController(boilerMode, boilerParam);

  tempEvent();
  
  delay(2000); //DHT MINIMUM_DELAY
  
  if (nh.connected()) {
    ac_sensor_msg.temp = currentTemp; 
    ac_sensor_msg.humid = currentHumid;
    pub_ac.publish( &ac_sensor_msg );
  } else {
    Serial.println("Not Connected");
  }
  nh.spinOnce();
  // Loop exproximativly at 1Hz

  // if(temp > 30.0){
  //   protector = 1;
  //   Serial.println("too high temp. protects overheat. timecode : " + protectTimeCode);
  //   delay(500);
    
  //   if(initProtect){ // to stop too much repeatition of relay);
  //     protectTimeCode = millis();
  //     digitalWrite(RELAY_PIN, LOW);
  //     delay(500);
  //     digitalWrite(RELAY_PIN, HIGH);
  //     delay(500);
  //     digitalWrite(RELAY_PIN, LOW);
  //     delay(500);
  //     digitalWrite(RELAY_PIN, HIGH);
  //     delay(500);
  //     digitalWrite(RELAY_PIN, LOW);
  //     delay(500);
  //     digitalWrite(RELAY_PIN, HIGH);
  //     delay(500);
  //     digitalWrite(RELAY_PIN, LOW);
  //     delay(500);   
  //     initProtect = false; 
  //     }
  //   else{
  //         digitalWrite(RELAY_PIN, LOW);
  //         Serial.println("High Temp Protecting...");
  //     }
  //     Serial.print((millis() - protectTimeCode)/1000);
  //     Serial.println("sec since protection starts");
  //   }
  // else if(((millis() - protectTimeCode)/1000 > 10) && (temp < 30.0) && (protectTimeCode != 0)){
  //     Serial.println("protection finished. follows previous command.");
  //     protectTimeCode = 0;
  //     initProtect = true;
  //     protector = 0;
  //     if(relayState >=1){
  //       digitalWrite(RELAY_PIN, HIGH);
  //     }
  //   }
}
