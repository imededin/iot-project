#include <Arduino.h>
#include <string>
#include <WiFi.h>
#include <PubSubClient.h>
#include <WiFiClientSecure.h>
//sensor
#include "bsec.h"
#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"




#define SEALEVELPRESSURE_HPA (1013.25)

// Create an object of the class Bsec
Bsec iaqSensor;

const char *SSI = "Ooredoo 4G_0A6BD0";
//"ZTE""Flybox_2BC1";
const char *PWD ="79903678";
//"THmnFcnRTJHd"ytreza3210;

int TEMP_MIN=0; //seuil min de température
int TEMP_MAX=100; //seuil max de température


//server certificat
static const char *root_ca PROGMEM = R"EOF(
-----BEGIN CERTIFICATE-----
MIIFazCCA1OgAwIBAgIRAIIQz7DSQONZRGPgu2OCiwAwDQYJKoZIhvcNAQELBQAw
TzELMAkGA1UEBhMCVVMxKTAnBgNVBAoTIEludGVybmV0IFNlY3VyaXR5IFJlc2Vh
cmNoIEdyb3VwMRUwEwYDVQQDEwxJU1JHIFJvb3QgWDEwHhcNMTUwNjA0MTEwNDM4
WhcNMzUwNjA0MTEwNDM4WjBPMQswCQYDVQQGEwJVUzEpMCcGA1UEChMgSW50ZXJu
ZXQgU2VjdXJpdHkgUmVzZWFyY2ggR3JvdXAxFTATBgNVBAMTDElTUkcgUm9vdCBY
MTCCAiIwDQYJKoZIhvcNAQEBBQADggIPADCCAgoCggIBAK3oJHP0FDfzm54rVygc
h77ct984kIxuPOZXoHj3dcKi/vVqbvYATyjb3miGbESTtrFj/RQSa78f0uoxmyF+
0TM8ukj13Xnfs7j/EvEhmkvBioZxaUpmZmyPfjxwv60pIgbz5MDmgK7iS4+3mX6U
A5/TR5d8mUgjU+g4rk8Kb4Mu0UlXjIB0ttov0DiNewNwIRt18jA8+o+u3dpjq+sW
T8KOEUt+zwvo/7V3LvSye0rgTBIlDHCNAymg4VMk7BPZ7hm/ELNKjD+Jo2FR3qyH
B5T0Y3HsLuJvW5iB4YlcNHlsdu87kGJ55tukmi8mxdAQ4Q7e2RCOFvu396j3x+UC
B5iPNgiV5+I3lg02dZ77DnKxHZu8A/lJBdiB3QW0KtZB6awBdpUKD9jf1b0SHzUv
KBds0pjBqAlkd25HN7rOrFleaJ1/ctaJxQZBKT5ZPt0m9STJEadao0xAH0ahmbWn
OlFuhjuefXKnEgV4We0+UXgVCwOPjdAvBbI+e0ocS3MFEvzG6uBQE3xDk3SzynTn
jh8BCNAw1FtxNrQHusEwMFxIt4I7mKZ9YIqioymCzLq9gwQbooMDQaHWBfEbwrbw
qHyGO0aoSCqI3Haadr8faqU9GY/rOPNk3sgrDQoo//fb4hVC1CLQJ13hef4Y53CI
rU7m2Ys6xt0nUW7/vGT1M0NPAgMBAAGjQjBAMA4GA1UdDwEB/wQEAwIBBjAPBgNV
HRMBAf8EBTADAQH/MB0GA1UdDgQWBBR5tFnme7bl5AFzgAiIyBpY9umbbjANBgkq
hkiG9w0BAQsFAAOCAgEAVR9YqbyyqFDQDLHYGmkgJykIrGF1XIpu+ILlaS/V9lZL
ubhzEFnTIZd+50xx+7LSYK05qAvqFyFWhfFQDlnrzuBZ6brJFe+GnY+EgPbk6ZGQ
3BebYhtF8GaV0nxvwuo77x/Py9auJ/GpsMiu/X1+mvoiBOv/2X/qkSsisRcOj/KK
NFtY2PwByVS5uCbMiogziUwthDyC3+6WVwW6LLv3xLfHTjuCvjHIInNzktHCgKQ5
ORAzI4JMPJ+GslWYHb4phowim57iaztXOoJwTdwJx4nLCgdNbOhdjsnvzqvHu7Ur
TkXWStAmzOVyyghqpZXjFaH3pO3JLF+l+/+sKAIuvtd7u+Nxe5AW0wdeRlN8NwdC
jNPElpzVmbUq4JUagEiuTDkHzsxHpFKVK7q4+63SM1N95R1NbdWhscdCb+ZAJzVc
oyi3B43njTOQ5yOf+1CceWxG1bQVs5ZufpsMljq4Ui0/1lvh+wjChP4kqKOJ2qxq
4RgqsahDYVvTH9w7jXbyLeiNdd8XM2w9U/t7y0Ff/9yi0GE44Za4rF2LN9d11TPA
mRGunUHBcnWEvgJBQl9nJEiU0Zsnvgc/ubhPgXRR4Xq37Z0j4r7g1SgEEzwxA57d
emyPxgcYxn/eR44/KJ4EBs+lVDR3veyJm+kXQ99b21/+jh5Xos1AnX5iItreGCc=
-----END CERTIFICATE-----
)EOF";


//==========================================

//MQTT client
WiFiClientSecure wifiClient;
PubSubClient mqttClient(wifiClient);
const char* mqttServer ="036c07529d284e8a809ea7ca62593aa3.s2.eu.hivemq.cloud";// adresse de Broker hiveMQ

int mqttPort = 8883;


//fonction pour connexion au wifi
void connectToWiFi() {
  Serial.print("Connectiog to ");
  WiFi.mode(WIFI_STA);
  WiFi.begin(SSI, PWD);
  Serial.println(SSI);
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  Serial.print("Connected.");
  Serial.println("your IP is:");
  Serial.println(WiFi.localIP());
  
}
void callback(char* topic, byte* payload, unsigned int length) { // fonction pour récupérer les messages du broker et faire le traitement demander
  

  const char* message = (char*)payload;
  const String Topic = (char*)topic;
  

  if(Topic=="/actionneurs/chauffage"){//commande du chauffage
    if(message[0] == 'T'){
      Serial.println(message);
      digitalWrite(2, HIGH);
    
    }
    if(message[0] == 'F'){
     digitalWrite(2, LOW);
    }
  }
  else if(Topic=="/actionneurs/ventilateur"){//commande du ventilateur

    if(message[0] == 'T'){
      digitalWrite(4, HIGH);
    
    }
    if(message[0] == 'F'){
     digitalWrite(4, LOW);
    }
    
  }
   else if(Topic=="/actionneurs/electroV"){ //commande de l'electrovanne

    if(message[0] == 'T'){
      digitalWrite(15, HIGH);
    
    }
    if(message[0] == 'F'){
     digitalWrite(15, LOW);
    }
    
  }
   else if(Topic=="/actionneurs/lampe"){// commande des lampes
    if(message[0] == 'T'){
      digitalWrite(12, HIGH);
    
    }
    if(message[0] == 'F'){
     digitalWrite(12, LOW);
    }
    
  }
  else if(Topic=="/seuils/temp_min"){ //configurer seuil min de température
   Serial.println(message);
   TEMP_MIN= atoi(message);
   Serial.println(TEMP_MIN);
    
  }
  else if(Topic=="/seuils/temp_max"){  //configurer seuil max de la température
   Serial.println(message);
   TEMP_MAX= atoi(message);
   Serial.println(TEMP_MAX);
    
  }
  else ;
  
}
void setupMQTT() { //configuration de client MQTT
  mqttClient.setServer(mqttServer, mqttPort);
  // set the callback function
  mqttClient.setCallback(callback);
}
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  //setup bme sensor & bsec
   Wire.begin();
  iaqSensor.begin(UINT8_C(0x77), Wire);
  Serial.println(iaqSensor.status);

   bsec_virtual_sensor_t sensorList[7] = {
    BSEC_OUTPUT_RAW_TEMPERATURE,
    BSEC_OUTPUT_RAW_PRESSURE,
    BSEC_OUTPUT_RAW_HUMIDITY,
    BSEC_OUTPUT_RAW_GAS,
    BSEC_OUTPUT_IAQ,
    BSEC_OUTPUT_CO2_EQUIVALENT,
    BSEC_OUTPUT_BREATH_VOC_EQUIVALENT,
   
  };
  iaqSensor.updateSubscription(sensorList, 7, BSEC_SAMPLE_RATE_LP);

  
  connectToWiFi();

   wifiClient.setCACert(root_ca);
  
  setupMQTT();
   while (!Serial);
  Serial.println(F("BME680 async test"));

  if (iaqSensor.status != BSEC_OK & iaqSensor.bme680Status != BME680_OK) {
    Serial.println(F("Could not find a valid BME680 sensor, check wiring!"));
    while (1);
  }





//SETUP GPIO  pour le contrôle des actionneurs
   pinMode(2, OUTPUT);
   pinMode(4,OUTPUT);
   pinMode(12, OUTPUT);
   pinMode(15,OUTPUT);

}
void reconnect() { //fonction pour connecter au broker
  Serial.println("Connecting to MQTT Broker...");
  while(!mqttClient.connected()){
  
      Serial.println("Reconnecting to MQTT Broker..");
      String clientId = "my-iot-device";
     
      //,"imededin","11965844123Aa") )
      if (mqttClient.connect(clientId.c_str(),"imededin","xxxxxxxxxx"))
      {
        Serial.println("Connected.");
        mqttClient.subscribe("/actionneurs/+");
        mqttClient.subscribe("/seuils/+");
        
        // subscribe to topic
       
      }
     
  }
  
}
//paramètres climatiques
float temp;
float humidite;
float pression;
float co2;
float IAQ;
 float voc;

void loop() {
  // put your main code here, to run repeatedly:
 
   if (!mqttClient.connected())
    reconnect();
  mqttClient.loop() ;
 
 
if (iaqSensor.run()) { // If new data is available
   //recuperation des valeurs 
   Serial.print("temp");
   temp = iaqSensor.rawTemperature;
   Serial.println(temp);
   pression=iaqSensor.pressure/100;
   humidite=iaqSensor.humidity;
   co2=iaqSensor.co2Equivalent;
   voc=iaqSensor.breathVocEquivalent;
   IAQ=iaqSensor.iaq;
   //publish data to broker
    mqttClient.publish("/paramètres/IAQ",String(IAQ).c_str());
    mqttClient.publish("/paramètres/temp",String(temp).c_str());
    mqttClient.publish("/paramètres/pression",String(pression).c_str());
    mqttClient.publish("/paramètres/humidit",String(humidite).c_str());
    mqttClient.publish("/paramètres/co2",String(co2).c_str());
    mqttClient.publish("/paramètres/voc",String(voc).c_str());


     //autocontrôle des actionneurs
  if(temp>TEMP_MAX){
    digitalWrite(2, LOW);//Désactiver chauffage
    mqttClient.publish("/state/chauffage","False");//notification par le désactivation de chauffage
    
    digitalWrite(4, HIGH);//activer ventilateur
    mqttClient.publish("/state/ventilateur","True");
    
  }
  if(temp<TEMP_MIN){
    digitalWrite(4, LOW);//Desactiver ventilateur
     mqttClient.publish("/state/ventilateur","False");
  
    
    digitalWrite(2, HIGH);//activer chauffage
     mqttClient.publish("/actionneurs/chauffage","True");
    
  }
    
};


  }
