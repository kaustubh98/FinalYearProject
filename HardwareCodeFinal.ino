// adding sensor libraries
#include <Adafruit_Sensor.h>
#include <Arduino.h>
#include <DHT.h>
#include <DHT_U.h>

//libraries for transmitting data to firebase
#include <ESP8266WiFi.h>
#include <SoftwareSerial.h>
#include <FirebaseArduino.h>
#include <ArduinoJson.h>

//configuration for transmitting data to firebase
#define FIREBASE_HOST "plant-monitoring-system-93b1d.firebaseio.com"
#define FIREBASE_AUTH "jj6wnZ535qWk2oyxxadfUazSMDOAkRmggpike3WU"
#define WIFI_SSID "MRSHAH"
#define WIFI_PASSWORD "9824159892"

//Defining Pins
#define DHT_PIN 2
#define LDR_PIN 4
#define MOISTURE_PIN A0
#define BATTERYLEVEL_PIN 1
#define BATTERYCONTROL_PIN 3

//configuring DHT sensor
#define DHTTYPE    DHT11
DHT_Unified dht(DHT_PIN, DHTTYPE);

//Device ID
const String deviceID = "kaus23";

//other global variables
String userID,userIDPath;
float TempOld = -1;
float HumidOld = -1;
float MoistOld = -1;
float LightOld = -1;

void setup() {
  Serial.begin(9600);

  // Initialize DHT sensor
  dht.begin();
  sensor_t sensor;

  //setting control Pin for battery
  pinMode(BATTERYCONTROL_PIN,OUTPUT);
  digitalWrite(BATTERYCONTROL_PIN,LOW);

  //connecting to WiFi 
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.print(".");
    delay(300);
  }
  Serial.println();
  Serial.print("Connected with IP: ");
  Serial.println(WiFi.localIP());
  Serial.println();

  //connect to firebase
  Firebase.begin(FIREBASE_HOST,FIREBASE_AUTH);
  //get the corresponding user
  userIDPath = "/Devices/" + deviceID;
  userID = Firebase.getString(userIDPath);
  Serial.println("Path: "+userIDPath);
  Serial.println("User is: "+userID);

}

void loop() {
  // location in the database
  String path = "/"+userID+"/Zones/"+deviceID;

  //light Intensity
  int ldr_value = digitalRead(LDR_PIN);
  float LightNew = ldr_value * (5.0/1023.0);

  //transmit the value if change is recorded
  if(LightNew != LightOld){
    String nodePath = path + "/LightIntensity";
    Firebase.pushFloat(nodePath,LightNew);
    LightOld = LightNew;    
  }

  delay(1000);

  //soil moisture
  int soil_value = analogRead(MOISTURE_PIN);
  float MoistNew = ( 100 - ( (soil_value/1023.00) * 100 ) );

  //transmit the value if change is recorded
  if(MoistNew != MoistOld){
    String nodePath = path + "/Moisture";
    Firebase.pushFloat(nodePath,MoistNew);
    MoistOld = MoistNew;    
  }

  delay(1000);

  sensors_event_t event;
  
  //temperature
  dht.temperature().getEvent(&event);
  if (isnan(event.temperature)) {
    Serial.println(F("Error reading temperature!"));
  }
  else {
    Serial.print(F("Temperature: "));
    Serial.print(event.temperature);
    Serial.println(F("°C"));

    float TempNew = event.temperature;
    //transmit the value if change is recorced
    if(TempNew != TempOld){
      String nodePath = path + "/Temperature";
      Firebase.pushFloat(nodePath,TempNew);
      TempOld = TempNew; 
    }
  }

  delay(1000);

  //humidity
  dht.humidity().getEvent(&event);
  if (isnan(event.relative_humidity)) {
    Serial.println(F("Error reading humidity!"));
  }
  else {
    Serial.print(F("Humidity: "));
    Serial.print(event.relative_humidity);
    Serial.println(F("%"));

     float HumidNew = event.relative_humidity;
    //transmit the value if change is recorced
    if(HumidNew != HumidOld){
      String nodePath = path + "/Humidity";
      Firebase.pushFloat(nodePath,HumidNew);
      HumidOld = HumidNew; 
    }
    
  }

  //battery level check
  digitalWrite(BATTERYCONTROL_PIN,HIGH);
  delay(500);
  int BatteryStatus = !digitalRead(BATTERYLEVEL_PIN);
  Firebase.setInt(path + "/BatteryLevel",BatteryStatus);
  digitalWrite(BATTERYCONTROL_PIN,LOW);

}
