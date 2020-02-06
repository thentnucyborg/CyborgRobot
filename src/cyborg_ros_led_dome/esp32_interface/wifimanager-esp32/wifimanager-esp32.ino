
//#include </home/johanndk/arduino-1.8.11/libraries/WiFi/src/WiFi.h>  //Arduino standard library
#include </home/johanndk/.arduino15/packages/esp32/hardware/esp32/1.0.4/libraries/WiFi/src/WiFi.h>  //ESP32 Core WiFi Library

#include <WebServer.h> //Local DNS Server used for redirecting all requests to the configuration portal (  https://github.com/zhouhan0126/DNSServer---esp32  )

#include </home/johanndk/arduino-1.8.11/libraries/zhouhan0126-DNSServer---esp32-master/src/DNSServer.h>  //Local WebServer used to serve the configuration portal (  https://github.com/zhouhan0126/DNSServer---esp32  )
//#include </home/johanndk/.arduino15/packages/esp32/hardware/esp32/1.0.4/libraries/DNSServer/src/DNSServer.h>  //ESP32 core library

#include </home/johanndk/arduino-1.8.11/libraries/zhouhan0126-WIFIMANAGER-ESP32-master/WiFiManager.h>   // (https://github.com/zhouhan0126/WIFIMANAGER-ESP32)


const int PIN_AP = 2;

void setup() {
  Serial.begin(9600);
  pinMode(PIN_AP, INPUT);
  WiFiManager wifiManager;

  //utilizando esse comando, as configurações são apagadas da memória
  //caso tiver salvo alguma rede para conectar automaticamente, ela é apagada.
  //  wifiManager.resetSettings();

  wifiManager.setAPCallback(configModeCallback);
  wifiManager.setSaveConfigCallback(saveConfigCallback);
  wifiManager.autoConnect("ESP_AP", "12345678");
}

//callback que indica que o ESP entrou no modo AP
void configModeCallback (WiFiManager *myWiFiManager) {  
  Serial.println("Entered config mode");
  Serial.println(WiFi.softAPIP());
  Serial.println(myWiFiManager->getConfigPortalSSID()); 
}

void saveConfigCallback () {
  Serial.println("Configuration saved");
  Serial.println(WiFi.softAPIP()); //imprime o IP do AP
}

void loop() {
  WiFiManager wifiManager;
  if (digitalRead(PIN_AP) == HIGH ) {
    Serial.println("reset");
    if (!wifiManager.startConfigPortal("ESP_AP", "12345678") ) {
      Serial.println("Failed to connect");
      delay(2000);
      ESP.restart();
      delay(1000);
    }
    Serial.println("Connected ESP_AP!!!");
  }
}
