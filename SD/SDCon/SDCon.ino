#include <SD.h>
#include <SDConfigFile.h>

#define pinSelectSD D8

const char CONFIG_FILE[] = "config.txt";
char *ssid;
char *pass;

void setup() {
  Serial.begin(9600);
  pinMode(pinSelectSD, OUTPUT);
  
  ssid = "multimedia3" ;
  pass = "tanpapassword" ;

  Serial.println("Read SD Configuration Saved");
  if (!SD.begin(pinSelectSD)) {
    Serial.println("SD.begin() failed");
    return;
  }

  readConfiguration();
}

void loop() {
  
}

void readConfiguration() {
  const uint8_t CONFIG_LINE_LENGTH = 127;
  SDConfigFile cfg;

  if (!cfg.begin(CONFIG_FILE, CONFIG_LINE_LENGTH)) {
    Serial.print("Failed to open configuration file: ");
    Serial.println(CONFIG_FILE);
  }

  while (cfg.readNextSetting()) {
    if (cfg.nameIs("ssid")) {
      ssid = cfg.copyValue();
      Serial.print("SSID: ");
    Serial.println(ssid);
    } 
  else if (cfg.nameIs("pass")) {
      pass = cfg.copyValue();
      Serial.print("Password: ");
    Serial.println(pass);
    }
  }
  cfg.end();
}
