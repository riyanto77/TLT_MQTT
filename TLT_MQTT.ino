

// Pemantau lokasi, waktu dan suhu pada range tertentu dengan MQTT V001
// Disusun oleh Riyanto dkk
// CP :  +6289663257596
// 2022 Juli 8
// Material : 
// 1. Wemos D1 Mini
// 2. SdCard Shield
// 3. DHT11 Shield
// 4. GY-GPS-6MV2
// 5. Powerbank
// 6. Modem 4GLTE

#include <ESP8266WiFi.h>
#ifdef ESP8266
 #include <ESP8266WiFi.h>  // Pins for board ESP8266 Wemos-NodeMCU
 #else
 #include <WiFi.h>  
#endif
 
#include <PubSubClient.h>
#include <WiFiClientSecure.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <Arduino.h>
#include <Hash.h>
#include <ESPAsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
// SD Card -----------------
#include <SPI.h>
#include <SD.h>

const int chipSelect = 4;
// ----------------------- SD card


TinyGPSPlus gps;
SoftwareSerial SerialGPS(4, 3); 

//const char* ssid = "HARD";
//const char* password = "timeisupbro";
//const char* ssid = "iot";
//const char* password = "12345678";
const char* ssid = "4G-UFI-606757";
const char* password = "12345678";

//---- MQTT Broker settings
const char* mqtt_server = "751cd6e9df164ebb895ed1933aa395af.s2.eu.hivemq.cloud"; // replace with your broker url
const char* mqtt_username = "riyanto";
const char* mqtt_password = "Mqttclient1";
const int mqtt_port =8883;

 

WiFiClientSecure espClient;   // for no secure connection use WiFiClient instead of WiFiClientSecure 
//WiFiClient espClient;
PubSubClient client(espClient);
unsigned long lastMsg = 0;

#define MSG_BUFFER_SIZE  (250)
char msg[MSG_BUFFER_SIZE];
//float sensor1 = 0;
//float sensor2 = 0;

float sensor1 = 0;
float sensor2 = 0;
float sensor3 = 0;
float sensor4 = 0;
float sensor5 = 0;
float sensor6 = 0;
float sensor7 = 0.00000;
float sensor8 = 0.00000;
float sensor9 = 0;
int command1 =0;

const char* sensor1_topic="sensor1";
const char* sensor2_topic="sensor2";
const char* sensor3_topic="sensor3";
const char* sensor4_topic="sensor4";
const char* sensor5_topic="sensor5";
const char* sensor6_topic="sensor6";
const char* sensor7_topic="sensor7";
const char* sensor8_topic="sensor8";
const char* sensor9_topic="sensor9";
//const char*  sensor2_topic="sensor3";

const char* command1_topic="command1";
//const char* command1_topic="command2";




#define DHTPIN 2     // Digital pin D7 => GPIO13 =>13
#define DHTTYPE    DHT11     // DHT 11

DHT dht(DHTPIN, DHTTYPE);
float t = 0.00;
float h = 0.00;
//AsyncWebServer server(80);


float Latitude , Longitude;
int year , month , date, hour , minute , second;
String DateString , TimeString , LatitudeString , LongitudeString;

unsigned long previousMillis = 0;    // will store last time DHT was updated
const long interval = 10000;  

WiFiServer server(80);

// Replaces placeholder with DHT values
String processor(const String& var){
  //Serial.println(var);
  if(var == "TEMPERATURE"){
    return String(t);
  }
  else if(var == "HUMIDITY"){
    return String(h);
  }
  return String();
}

//==========================================
void setup_wifi() {
  delay(10);
  Serial.print("\nConnecting to ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  randomSeed(micros());
  Serial.println("\nWiFi connected\nIP address: ");
  Serial.println(WiFi.localIP());
}


//=====================================
void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    String clientId = "ESP8266Client-";   // Create a random client ID
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (client.connect(clientId.c_str(), mqtt_username, mqtt_password)) {
      Serial.println("connected");

      client.subscribe(command1_topic);   // subscribe the topics here
      //client.subscribe(command2_topic);   // subscribe the topics here
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");   // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

//================================================ setup

void setup()
{
  Serial.begin(9600);
// ---------------------- IoT
 while (!Serial) delay(1);
  setup_wifi();
  pinMode(BUILTIN_LED, OUTPUT);     // Initialize the BUILTIN_LED pin as an output


  #ifdef ESP8266
    espClient.setInsecure();
  #else   // for the ESP32
    espClient.setCACert(root_ca);      // enable this line and the the "certificate" code for secure connection
  #endif
  
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
  // --------------------------- IoT



  
 // SD card ----------------------------------------------------
Serial.print("Initializing SD card...");

  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    return;
  }
  Serial.println("card initialized.");
// ---------------------------------------------------  SD card
  
  SerialGPS.begin(9600);
    dht.begin();
  Serial.println();
  Serial.print("Connecting");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");

  server.begin();
  Serial.println("Server started");
  Serial.println(WiFi.localIP());
  Serial.println(SerialGPS.available());


  
delay(100);

 
}

void loop()
{




//-------------------- IoT
if (!client.connected()) reconnect();
  client.loop();

  //---- example: how to publish sensor values every 5 sec
  unsigned long now = millis();
  if (now - lastMsg > 10000) {
    lastMsg = now;
 //   sensor1= (TimeString);       // replace the random value with your sensor value
 //   sensor2= (DateString);    // replace the random value  with your sensor value
 //    sensor1= (hour);       // replace the random value with your sensor value
 //   sensor2= (year);    // replace the random value  with your sensor value
    
    sensor1= (hour);       // replace the random value with your sensor value
    
    sensor2= (minute);
   
    sensor3= (second);
   
    sensor4= (year);    // Latitude
   
    sensor5= (month); 
    
    sensor6= (date); 
    
    sensor7= (Longitude); 
    
    sensor8= (Latitude); 
    
    sensor9= (t); 
    
    publishMessage(sensor1_topic,String(sensor1),true);    
   
    publishMessage(sensor2_topic,String(sensor2),true);
   
    publishMessage(sensor3_topic,String(sensor3),true);    
 
    publishMessage(sensor4_topic,String(sensor4),true);
 
    publishMessage(sensor5_topic,String(sensor5),true);    
   
    publishMessage(sensor6_topic,String(sensor6),true);
  
    publishMessage(sensor7_topic,String(sensor7),true);    
    
    publishMessage(sensor8_topic,String(sensor8),true);
 
    publishMessage(sensor9_topic,String(sensor9),true);
  }
 delay(50);
  
// ------------------------ IoT
  
  while (SerialGPS.available() > 0)
    if (gps.encode(SerialGPS.read()))
    {
      if (gps.location.isValid())
      {
        Latitude = gps.location.lat();
        LatitudeString = String(Latitude , 6);
        Longitude = gps.location.lng();
        LongitudeString = String(Longitude , 6);
      }

      if (gps.date.isValid())
      {
        DateString = "";
        date = gps.date.day();
        month = gps.date.month();
        year = gps.date.year();

        if (date < 10)
        DateString = '0';
        DateString += String(date);

        DateString += " / ";

        if (month < 10)
        DateString += '0';
        DateString += String(month);
        DateString += " / ";

        if (year < 10)
        DateString += '0';
        DateString += String(year);
      }

      if (gps.time.isValid())
      {
        TimeString = "";
        hour = gps.time.hour()+ 7; //adjust UTC
        minute = gps.time.minute();
        second = gps.time.second();
    
        if (hour < 10)
        TimeString = '0';
        TimeString += String(hour);
        TimeString += " : ";

        if (minute < 10)
        TimeString += '0';
        TimeString += String(minute);
        TimeString += " : ";

        if (second < 10)
        TimeString += '0';
        TimeString += String(second);
      }









    }

unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    // save the last time you updated the DHT values
    previousMillis = currentMillis;
    // Read temperature as Celsius (the default)
    float newT = dht.readTemperature();
    // Read temperature as Fahrenheit (isFahrenheit = true)
    //float newT = dht.readTemperature(true);
    // if temperature read failed, don't change t value
    if (isnan(newT)) {
      Serial.println("Failed to read from DHT sensor!");
    }
    else {
      t = newT;
    //  Serial.println(t);
    }
    // Read Humidity
    float newH = dht.readHumidity();
    // if humidity read failed, don't change h value 
    if (isnan(newH)) {
      Serial.println("Failed to read from DHT sensor!");
    }
    else {
      h = newH;
     // Serial.println(h);
    }
  }

    /*
Serial.println(t);
Serial.println(h);
      Serial.println(LatitudeString);
  Serial.println(LongitudeString);
      Serial.println(DateString);
  Serial.println(TimeString);
  */


  
// SD card ------------------------------------------------------------
// make a string for assembling the data to log:
  String dataString = "";

  // read three sensors and append to the string:
  for (int analogPin = 0; analogPin < 3; analogPin++) {
    int sensor = analogRead(analogPin);
    dataString += String(sensor);
    if (analogPin < 2) {
      dataString += ",";
    }
  }

  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  File dataFile = SD.open("datalog.txt", FILE_WRITE);

  // if the file is available, write to it:
  if (dataFile) {
    dataFile.print(TimeString);
    dataFile.print(",");
    dataFile.print(DateString);
    dataFile.print(",");
    dataFile.print(LatitudeString);
    dataFile.print(",");   
    dataFile.print(LongitudeString);
    dataFile.print(",");     
    dataFile.println(t);
    
    dataFile.close();

/*
//-------------------- IoT
if (!client.connected()) reconnect();
  client.loop();

  //---- example: how to publish sensor values every 5 sec
  unsigned long now = millis();
  if (now - lastMsg > 5000) {
    lastMsg = now;
 //   sensor1= (TimeString);       // replace the random value with your sensor value
 //   sensor2= (DateString);    // replace the random value  with your sensor value
 //    sensor1= (hour);       // replace the random value with your sensor value
 //   sensor2= (year);    // replace the random value  with your sensor value
    
    sensor1= (hour);       // replace the random value with your sensor value
    sensor2= (year);    // replace the random value  with your sensor value
    sensor3= (Latitude); 
    sensor4= (Longitude); 
    sensor5= (t); 
    publishMessage(sensor1_topic,String(sensor1),true);    
    publishMessage(sensor2_topic,String(sensor2),true);
    publishMessage(sensor3_topic,String(sensor3),true);    
    publishMessage(sensor4_topic,String(sensor4),true);
    publishMessage(sensor5_topic,String(sensor5),true);
  }
// ------------------------ IoT
*/


    
    // print to the serial port too:
   /* Serial.print(TimeString);
    Serial.print(",");
    Serial.print(DateString);
    Serial.print(",");
    Serial.print(LatitudeString);
    Serial.print(",");   
    Serial.print(LongitudeString);
    Serial.print(",");     
    Serial.println(t);
    */
  }
  // if the file isn't open, pop up an error:
  else {
    Serial.println("error opening datalog.txt");
  }
  
  // ------------------------------------------------------ SD card

}
// ----------------------- IoT 
void callback(char* topic, byte* payload, unsigned int length) {
  String incommingMessage = "";
  for (int i = 0; i < length; i++) incommingMessage+=(char)payload[i];
  
  Serial.println("Message arrived ["+String(topic)+"]"+incommingMessage);
  
  //--- check the incomming message
    if( strcmp(topic,command1_topic) == 0){
     if (incommingMessage.equals("1")) digitalWrite(BUILTIN_LED, LOW);   // Turn the LED on 
     else digitalWrite(BUILTIN_LED, HIGH);  // Turn the LED off 
  }

   //  check for other commands
 /*  else  if( strcmp(topic,command2_topic) == 0){
     if (incommingMessage.equals("1")) {  } // do something else
  }
  */
}



//======================================= publising as strings
void publishMessage(const char* topic, String payload , boolean retained){
  client.publish(topic, (byte*) payload.c_str(), 10, true);
  Serial.println("Message publised ["+String(topic)+"]: "+payload);
}
// ------------------ IoT
