#include <SPI.h>
#include <SD.h>
#include <DHT.h>

#define chipSelect D8
DHT dht(D4, DHT11);

void setup() {
  Serial.begin(9600);
  dht.begin();
  
  SD.begin(chipSelect); 
  if(!SD.begin(chipSelect)) {
    Serial.println("SD Card not Found!");
    return;
  }
  Serial.println("Program Data Logger");
}

void loop() {
  delay(5000);
  String dataString = "";
  float h = dht.readHumidity();
  float t = dht.readTemperature();
  
  dataString += String(h) + "#" + String(t) + "$";
  File dataFile = SD.open("datalog.txt", FILE_WRITE);
  if (dataFile) {
    dataFile.println(dataString);
    dataFile.close();
    Serial.println(dataString);
  }
  else {
    Serial.println("error opening datalog.txt");
  }
}
