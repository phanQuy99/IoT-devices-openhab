/**************************** Library *********************************/
#include <FS.h>                   //this needs to be first, or it all crashes and burns...
#include <FastLED.h>
#include <ESP8266WiFi.h>          //https://github.com/esp8266/Arduino
#include <Servo.h>
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h>          //https://github.com/tzapu/WiFiManager
#include <PubSubClient.h>         //https://github.com/knolleary/pubsubclient
#include <ArduinoJson.h>          //https://github.com/bblanchon/ArduinoJson
/******************* HARDWARE PIN DEFINITIONS (Set to match your hardware)***********************/
#define  relay_fan  1    // TX
#define  relay_bed  3   // RX
#define  relay_kitchen  D7  //D7
#define  switch_fan D2
#define  switch_bed D3
#define  switch_kitchen D4
#define  sensor_door     D6
#define  LED_PIN  D1 // RGB
#define  NUM_LEDS 8
////config for Servo
//Servo servo_1;       // servo controller (multiple can exist)
//int servo_pin = D8; // PWM pin for servo control
//int pos = 0;       // servo starting position
/// LED RGB
CRGB leds[NUM_LEDS];
////// DHT 11
#include <DHT.h>
#define DHTPIN  D5
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);
/////// for sensor PM 2.5
int measurePin = A0;
int ledPower   = D0;
/************* Flags and Counts***********************/
int checkSensorDoor = 1;
/////// for sensor PM 2.5
int samplingTime = 280;
int deltaTime = 40;
int sleepTime = 9620;
float voMeasured = 0;
float calcVoltage = 0;
float dustDensity = 0.0;
unsigned long voMeasuredTotal = 0;
int voCount = 0; //biến lấy mẫu
// Use the typical sensitivity in units of V per 100ug/m3.
const float K = 1;
float Voc = 0.6;


static uint32_t  currentmls = millis(), intervalPM = currentmls;

/************* MQTT TOPICS (change these topics as you wish)  **************************/
///COMMAND
// Living, BedRoom, Kitchen
#define command_topic_relay_fan   "fan/command"
#define command_topic_relay_bed   "light_bed/command"
#define command_topic_relay_kitchen   "light_kitchen/command"
#define command_topic_switch_fan  "fan/state"
#define command_topic_switch_bed  "light_bed/state"
#define command_topic_switch_kitchen  "light_kitchen/state"
#define command_topic_door  "door/state"
//#define command_topic_servo    "DoAn/Servo"
#define command_topic_rgb_1   "rgb1/command"
#define command_topic_rgb_2 "rgb2/command"
#define command_topic_rgb_3    "rgb3/command"
#define command_topic_rgb     "rgb/command"

/** Door Sensor **/
const char* open_door = "OPEN";
const char* close_door = "CLOSE";
/** Relays **/
const char* turn_on_relay_fan  = "ON";
const char* turn_off_relay_fan = "OFF";

const char* turn_on_relay_bed  = "ON";
const char* turn_off_relay_bed = "OFF";

const char* turn_on_relay_kitchen  = "ON";
const char* turn_off_relay_kitchen = "OFF";
///** RGBs **/
//const char* turn_off_rgb    = "OFF";
const char* on_rgb_1 = "ON";
const char* on_rgb_2   = "ON";
const char* on_rgb_3    = "ON";
const char* off_rgb_1 = "OFF";
const char* off_rgb_2   = "OFF";
const char* off_rgb_3    = "OFF";
/********************************** START SETUP*****************************************/
WiFiClient espClient1;
PubSubClient client(espClient1);
long lastMsg = 0;
char msg[50];
int value = 0;
//define your default values here, if there are different values in config.json, they are overwritten.
char mqtt_server[40] = "broker.hivemq.com";
char mqtt_port[6]    = "1883";
char username[34]    = "";
char password[34]    = "";
//flag for saving data
bool shouldSaveConfig = false;
//callback notifying us of the need to save config

void saveConfigCallback () {
  Serial.println("Should save config");
  shouldSaveConfig = true;
}

/********************************** START MQTT CALLBACK*****************************************/

void callback(char* topic, byte* payload, unsigned int length) {
  char* payload_str;
  payload_str = (char*) malloc(length + 1);
  memcpy(payload_str, payload, length);
  payload_str[length] = '\0';

  Serial.println(String(payload_str));
  /********************************** Servos *****************************************/
//  if ( String(topic) == command_topic_servo ) {
//    if ( String(payload_str) == open_door )  {
//      for ( pos = 97; pos >= 0; pos -= 1) { // goes from 97 degrees to 0 degrees
//        // in steps of 1 degree
//        servo_1.write(pos);              // tell servo to go to position in variable 'pos'
//        delay(15); // delay to allow the servo to reach the desired position
//      }
//    }
//    else if ( String(payload_str) == close_door ) {
//      for ( pos = 0; pos <= 97; pos += 1) { // goes from 0 degrees to 97 degrees
//        servo_1.write(pos);              // tell servo to go to position in variable 'pos'
//        delay(15);
//      }
//    }
//  }
  /***********************************Relays*****************************************/
  /******** Relay 1**************/
  if ( String(topic) == command_topic_relay_fan ) {
    if (String(payload_str) == turn_on_relay_fan)
    {
      digitalWrite(relay_fan, LOW);
    }
    else if ( String(payload_str) == turn_off_relay_fan ) {
      digitalWrite(relay_fan, HIGH);
    }
  }
  /******** Relay 2**************/
  if ( String(topic) == command_topic_relay_bed ) {
    if (String(payload_str) == turn_on_relay_bed)
    {
      digitalWrite(relay_bed, HIGH);
    }
    else if ( String(payload_str) == turn_off_relay_bed) {
      digitalWrite(relay_bed, LOW);
    }
  }
  /******** Relay 3**************/
  if ( String(topic) == command_topic_relay_kitchen ) {
    if (String(payload_str) == turn_on_relay_kitchen)
    {
      digitalWrite(relay_kitchen, HIGH);
    }
    else if ( String(payload_str) == turn_off_relay_kitchen) {
      digitalWrite(relay_kitchen, LOW);
    }
  }

  /**************************************** RgB**********************************************/
//  if (String(topic) == command_topic_rgb_off ) {
//    if ( String(payload_str) == turn_off_rgb ) {
//      for (int i = 0; i <= 7; i++) {
//        leds[i] = CRGB (0, 0, 0);
//        FastLED.show();
//        delay(100);
//      }
//    }
//  }
//  /********* RGB1************/
//  if (String(topic) == command_topic_rgb_1 ) {
//    if ( String(payload_str) == on_rgb_1 ) {
//      for (int i = 0; i <= 7; i++) {
//        leds[i] = CRGB (0, 178, 191);
//        FastLED.show();
//      }
//    }
//    if ( String(payload_str) == off_rgb_1 ) {
//      for (int i = 0; i <= 7; i++) {
//        leds[i] = CRGB (0, 0, 0);
//        FastLED.show();
//      }
//    }
//  }
////  /******** RGB2***************/
//  if (String(topic) == command_topic_rgb_2) {
//    if ( String(payload_str) == on_rgb_2 ) {
//      for (int i = 0; i <= 7; i++) {
//        leds[i] = CRGB (249, 244, 0);
//        FastLED.show();
//      }
//    }
//    if ( String(payload_str) == off_rgb_2 ) {
//      for (int i = 0; i <= 7; i++) {
//        leds[i] = CRGB (0, 0, 0);
//        FastLED.show();
//      }
//    }
//  }
////  /***********RGB3**************/
//  if (String(topic) == command_topic_rgb_3 ) {
//    if ( String(payload_str) == on_rgb_3) {
//      for (int i = 0; i <= 7; i++) {
//        leds[i] = CRGB (229, 70, 70);
//        FastLED.show();
//      }
//    }
//    if ( String(payload_str) == off_rgb_3 ) {
//      for (int i = 0; i <= 7; i++) {
//        leds[i] = CRGB (0, 0, 0);
//        FastLED.show();
//      }
//    }
//  
//  
//}

//Control LED qua OpenHAB
  if ((char)payload[0] == '0')
  {
     for (int i = 0; i <= 7; i++) {
        leds[i] = CRGB (0, 0, 0);
        FastLED.show();
      }
   }
  else if ((char)payload[0] == '1') 
  {
    for (int i = 0; i <= 7; i++) {
        leds[i] = CRGB (223,0,41);
        FastLED.show();
      }
  }
   else if ((char)payload[0] == '2') 
  {
    for (int i = 0; i <= 7; i++) {
        leds[i] = CRGB (240, 156, 66);
        FastLED.show();
      }
  }
   else if ((char)payload[0] == '3') 
  {
    for (int i = 0; i <= 7; i++) {
        leds[i] = CRGB (252,  245, 76);
        FastLED.show();
      }
  }
   else if ((char)payload[0] == '4') 
  {
    for (int i = 0; i <= 7; i++) {
        leds[i] = CRGB (91,  189, 43);
        FastLED.show();
      }
  }
   else if ((char)payload[0] == '5') 
  {
    for (int i = 0; i <= 7; i++) {
        leds[i] = CRGB (0 , 178, 191);
        FastLED.show();
      }
  }
   else if ((char)payload[0] == '6') 
  {
    for (int i = 0; i <= 7; i++) {
        leds[i] = CRGB (58 , 40 , 133);
        FastLED.show();
      }
  }
   else if ((char)payload[0] == '7') 
  {
    for (int i = 0; i <= 7; i++) {
        leds[i] = CRGB (93 , 12 , 123);
        FastLED.show();
      }
  }
   else if ((char)payload[0] == '8') 
  {
    for (int i = 0; i <= 7; i++) {
        leds[i] = CRGB (162 , 0 , 124);
        FastLED.show();
      }
  }
   else if ((char)payload[0] == '9') 
  {
    for (int i = 0; i <= 7; i++) {
        leds[i] = CRGB (236 , 236 , 236);
        FastLED.show();
      }
  }







}
/********************************** START SETUP*****************************************/
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.println("device is in Wake up mode");
  //servo_1.attach(servo_pin); // start servo control
  dht.begin();
  FastLED.addLeds<WS2812, LED_PIN, GRB>(leds, NUM_LEDS);
  pinMode(relay_fan, OUTPUT);
  pinMode(relay_bed, OUTPUT);
  pinMode(relay_kitchen, OUTPUT);
  pinMode(switch_fan, INPUT_PULLUP);
  pinMode(switch_bed, INPUT_PULLUP);
  pinMode(switch_kitchen, INPUT_PULLUP);
  pinMode(sensor_door,     INPUT_PULLUP);
  pinMode(ledPower, OUTPUT);
  //clean FS, for testing
  SPIFFS.format();
  //read configuration from FS json
  Serial.println("mounting FS...");
  if (SPIFFS.begin()) {
    Serial.println("mounted file system");
    if (SPIFFS.exists("/config.json")) {
      //file exists, reading and loading
      Serial.println("reading config file");
      File configFile = SPIFFS.open("/config.json", "r");
      if (configFile) {
        Serial.println("opened config file");
        size_t size = configFile.size();
        // Allocate a buffer to store contents of the file.
        std::unique_ptr<char[]> buf(new char[size]);
        configFile.readBytes(buf.get(), size);
        DynamicJsonBuffer jsonBuffer;
        JsonObject& json = jsonBuffer.parseObject(buf.get());
        json.printTo(Serial);
        if (json.success()) {
          Serial.println("\nparsed json");
          strcpy(mqtt_server, json["mqtt_server"]);
          strcpy(mqtt_port, json["mqtt_port"]);
          strcpy(username, json["username"]);
          strcpy(password, json["password"]);

        } else {
          Serial.println("failed to load json config");
        }
      }
    }
  } else {
    Serial.println("failed to mount FS");
  }
  //end read
  // The extra parameters to be configured (can be either global or just in the setup)
  // After connecting, parameter.getValue() will get you the configured value
  // id/name placeholder/prompt default length
  
  WiFiManagerParameter custom_mqtt_server("server", "mqtt server", mqtt_server, 40);
  WiFiManagerParameter custom_mqtt_port("port", "mqtt port", mqtt_port, 5);
  WiFiManagerParameter custom_username("username", "username", username, 32);
  WiFiManagerParameter custom_password("password", "password", password, 32);
  WiFiManager wifiManager;
  // reset settings
  wifiManager.resetSettings();
  wifiManager.setSaveConfigCallback(saveConfigCallback);
  //add all your parameters here
  wifiManager.addParameter(&custom_mqtt_server);
  wifiManager.addParameter(&custom_mqtt_port);
  wifiManager.addParameter(&custom_username);
  wifiManager.addParameter(&custom_password);

  if (!wifiManager.autoConnect("PhanQuyWifi")) {
    Serial.println("failed to connect and hit timeout");
    delay(3000);
    //reset and try again, or maybe put it to deep sleep
    ESP.reset();
    delay(5000);
  }

  //if you get here you have connected to the WiFi
  Serial.println("connected...yeey :)");
  //read updated parameters
  strcpy(mqtt_server, custom_mqtt_server.getValue());
  strcpy(mqtt_port, custom_mqtt_port.getValue());
  strcpy(username, custom_username.getValue());
  strcpy(password, custom_password.getValue());
  //save the custom parameters to FS
  if (shouldSaveConfig) {
    Serial.println("saving config");
    DynamicJsonBuffer jsonBuffer;
    JsonObject& json = jsonBuffer.createObject();
    json["mqtt_server"] = mqtt_server;
    json["mqtt_port"] = mqtt_port;
    json["username"] = username;
    json["password"] = password;

    File configFile = SPIFFS.open("/config.json", "w");
    if (!configFile) {
      Serial.println("failed to open config file for writing");
    }
    json.printTo(Serial);
    json.printTo(configFile);
    configFile.close();
    //end save
  }
  Serial.println("local ip");
  Serial.println(WiFi.localIP());
  // mqtt
  client.setServer(mqtt_server, atoi(mqtt_port)); // parseInt to the port
  client.setCallback(callback);
}
/********************************** START RECONNECT*****************************************/
void reconnect() {
  // Loop until we're reconnected to the MQTT server
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("Phan Quy Wifi", username, password)) { // username as client ID
      Serial.println("connected");
      client.subscribe(command_topic_relay_fan);
      client.subscribe(command_topic_relay_bed);
      client.subscribe(command_topic_relay_kitchen);
      client.subscribe(command_topic_door);
//      client.subscribe(command_topic_servo);
      client.subscribe(command_topic_rgb_1);
      client.subscribe(command_topic_rgb_2);
      client.subscribe(command_topic_rgb_3);
      client.subscribe(command_topic_rgb);
    }
    else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void switchRelay1()
{
  if (digitalRead(switch_fan) == 0)
  {
    delay(150);
    while (digitalRead(switch_fan) == 0);
    if (digitalRead(relay_fan) == HIGH) {
      digitalWrite(relay_fan, LOW);
      client.publish("fan/state", "ON" );
    }
    else {
      digitalWrite(relay_fan, HIGH);
      client.publish("fan/state", "OFF" );
    }
  }
}

void switchRelay2()
{
  if (digitalRead(switch_bed) == 0)
  {
    delay(150);
    while (digitalRead(switch_bed) == 0);
    if (digitalRead(relay_bed) == LOW) {
      digitalWrite(relay_bed, HIGH);
      client.publish("light_bed/state", "ON" );
    }
    else {
      digitalWrite(relay_bed, LOW);
      client.publish("light_bed/state", "OFF" );
    }
  }
}

void switchRelay3()
{
  if (digitalRead(switch_kitchen) == 0)
  {
    delay(150);
    while (digitalRead(switch_kitchen) == 0);
    if (digitalRead(relay_kitchen) == LOW) {
      digitalWrite(relay_kitchen, HIGH);
      client.publish("light_kitchen/state", "ON" );
    }
    else {
      digitalWrite(relay_kitchen, LOW);
      client.publish("light_kitchen/state", "OFF" );
    }
  }
}

void openDoor()
{
  if ( (digitalRead(sensor_door) == 0) && (checkSensorDoor == 1))
  {
    client.publish("door/state", "CLOSE" );
    Serial.print("close");
    checkSensorDoor = 0;
  }
  if ( (digitalRead(sensor_door) == 1 ) && (checkSensorDoor == 0))
  {
    client.publish("door/state", "OPEN");
    Serial.print("open");
    checkSensorDoor = 1;
  }
}

void nhietDoAm()
{
  // Wait a few seconds between measurements.
  // Reading temperature or humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  float k = dht.readHumidity();
  float h = k;
  // Read temperature as Celsius (the default)
  float t = dht.readTemperature();
  // Read temperature as Fahrenheit (isFahrenheit = true)
  float f = dht.readTemperature(true);

  // Check if any reads failed and exit early (to try again).
  if (isnan(h) || isnan(t) || isnan(f)) {
    Serial.println("Failed to read from DHT sensor!");
    return;
  }

  // Compute heat index in Fahrenheit (the default)
  float hif = dht.computeHeatIndex(f, h);
  // Compute heat index in Celsius (isFahreheit = false)
  float hic = dht.computeHeatIndex(t, h, false);
  Serial.print("Humidity: ");
  Serial.print(h);
  Serial.print(" %\t");
  Serial.print("Temperature: ");
  Serial.print(t);
  Serial.print(" *C ");
  Serial.print(f);
  Serial.print(" *F\t");
  Serial.print("Heat index: ");
  Serial.print(hic);
  Serial.print(" *C ");
  //  Serial.print(hif);
  //  Serial.println(" *F");
  Serial.print("Temperature in Celsius:");
  Serial.println(String(t).c_str());
  client.publish("temperature/state", String(t).c_str(), true);

  Serial.print("Humidity:");
  Serial.println(String(h).c_str());
  client.publish("humidity/state", String(h).c_str(), true);
}

void PM(){
 voCount = 0;
  voMeasuredTotal = 0;
  while (voCount <= 10) {
    digitalWrite(ledPower, LOW);     // Bật IR LED
    delayMicroseconds(samplingTime);   //Delay 0.28ms
    voMeasured = analogRead(measurePin); // Đọc giá trị ADC V0 mất khoảng 0.1ms
    digitalWrite(ledPower, HIGH);    // Tắt LED
    delayMicroseconds(sleepTime);     //Delay 9.62ms
    voMeasuredTotal += voMeasured;    // Tính tổng lần lấy mẫu
    voCount ++;              // Đếm số lần lấy mẫu
  }
  voMeasured = 1.0 * voMeasuredTotal / 100; //Tính trung bình
  //****************************

  calcVoltage = voMeasured / 1024 * 5; //Tính điện áp Vcc của cảm biến (5.0 hoặc 3.3)
  dustDensity = calcVoltage / K * 100.0;
 
  client.publish("pm25/state", String(dustDensity).c_str());
  Serial.println(dustDensity);
}
/********************************** START MAIN LOOP***************************************/
void loop()
{
   if ( millis() - intervalPM > 30000 ) {
    // save the last time you updated the PM 2.5 values
    intervalPM = millis();
    PM();
  }
  switchRelay1();
  switchRelay2();
  switchRelay3();
  openDoor();
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
  if (millis() - currentmls > 30000) {
    // save the last time you updated the DHT values
    currentmls = millis();
    nhietDoAm();
  }
  
 
}
