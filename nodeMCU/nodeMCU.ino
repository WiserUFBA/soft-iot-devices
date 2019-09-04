/*

 Node MCU Device - Internet of Things w/ TATU

*/
#include <FS.h>                    //this needs to be first, or it all crashes and burns...


#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <SoftwareSerial.h>
#include <Bounce2.h>
#include <EEPROM.h>

#include <TATUDevice.h>
#include <TATUInterpreter.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>

//library configurationBrowser
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h>          //https://github.com/tzapu/WiFiManager
#include <ArduinoJson.h>           //https://github.com/bblanchon/ArduinoJson

const char*   device_name     = "nodeMCU01";
const char*   mqtt_user       = "wiser";
const char*   mqtt_pass       = "wiser2014";
char          mqtt_server[40];
char          subsc_topic[20] = "dev/";
const int     mqttport = 1883;

//workaround
byte ip[5] = "11";


long lastMsg = 0;
char msg[50];

// Pins used

#define LUMINOSITY    A0
#define TEMPERATURE   16


//Hash that represents the attributes
#define H_lampActuator 0xEEF6732
#define H_temperatureSensor 0x5821FBAD
#define H_luminositySensor  0xF8D7A29C

// Variveis
int                 aux;
/*
float               luz;
float               v_out;
volatile int        t;
volatile int        luminosity;
volatile int        flag = 1;
*/
volatile int        state;
volatile int        state_relay;
char                response[15];

bool get(uint32_t hash, void* response, uint8_t code) {
  switch (hash) {
    case H_temperatureSensor:
      // The dht_temperatures_sensor supports INFO and VALUE requests.
      aux = temperature_sensor(TEMPERATURE);
      break;
    case H_luminositySensor:
      // The lumisity_sensor supports INFO and VALUE,requests.
      aux = luminosity_sensor(LUMINOSITY);
      break;
    /*
    case H_flow:
      STOS(response, vector_response);
      return true;
      break;
    */
    default:
      return false;
  }
}

bool set(uint32_t hash, uint8_t code, void* response) {
  switch (hash) {
    case H_lampActuator:
      Serial.println("Lamp!");
      break;
    default:
      return false;
  }

  return true;
}

//  Sensors functions
/*<sensorFunctions>*/
int luminosity_sensor(int PIN){
  return analogRead(PIN);
}

float temperature_sensor(int PIN){
  return ((float(analogRead(PIN))*5/(1023))/0.01);
}
/*</sensorFunctions>*/

//  Actuators functions
/*<actuatorFunctions>*/

int lamp_actuator(int PIN, int state ){
  if(state == 0){
    digitalWrite(PIN, LOW);
    state_relay =  state;
    Serial.println("Desligado");
  }
  else if(state == 2){
    state_relay =  !state_relay;
    digitalWrite(PIN, state_relay);
    state = state_relay;
    Serial.println("Alternou");
  }
  else{
    digitalWrite(PIN, HIGH);
    state_relay =  state;
    Serial.println("Ligado");
  }
}

/*</actuatorFunctions>*/

// System variables
WiFiClient    espClient;
TATUInterpreter interpreter;
MQTT_BRIDGE(bridge);
TATUDevice device(device_name, ip, 121, 88, 0, mqttport, 1, &interpreter, get, set, bridge);
MQTT_CALLBACK(bridge, device, mqtt_callback);
PubSubClient  client(espClient);
MQTT_PUBLISH(bridge, client);

//flag for saving data
bool shouldSaveConfig = false;

//callback notifying us of the need to save config
void saveConfigCallback () {
  Serial.println("Should save config");
  shouldSaveConfig = true;
}

void setup_wifi() {

  delay(10);

  // The extra parameters to be configured (can be either global or just in the setup)
  // After connecting, parameter.getValue() will get you the configured value
  // id/name placeholder/prompt default length
  WiFiManagerParameter custom_mqtt_server("Server", "mqtt server", mqtt_server, 40);
 
  WiFiManager wifiManager;
  wifiManager.setDebugOutput(false);

  //set config save notify callback
  wifiManager.setSaveConfigCallback(saveConfigCallback);
  
  //add all your parameters here
  wifiManager.addParameter(&custom_mqtt_server);

  wifiManager.startConfigPortal("NodeAP01", "node2014");
  

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  
  Serial.println("");
  Serial.println("WiFi conectado");

  WiFi.mode(WIFI_STA);
  
  //read updated parameters
  strcpy(mqtt_server, custom_mqtt_server.getValue());
  //save the custom parameters to FS
  if (shouldSaveConfig) {
    Serial.println("Saving config...");
    DynamicJsonBuffer jsonBuffer;
    JsonObject& json = jsonBuffer.createObject();
    json["mqtt_server"] = mqtt_server;
    File configFile = SPIFFS.open("/config.json", "w");
    if (!configFile) {
      Serial.println("Failed to open config file for writing");
    }

    json.printTo(Serial);
    json.printTo(configFile);
    configFile.close();
    //end save
  }
  
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void search_config() {
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
        } else {
          Serial.println("failed to load json config");
        }
      }
    }
  } else {
    Serial.println("failed to mount FS");
  }
}


void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    // Attempt to connect
    if (client.connect(device_name, mqtt_user, mqtt_pass)) {
      // Once connected, publish an announcement...
      client.publish("CONNECTED", device_name);
      // ... and resubscribe
      client.subscribe("dev/");
      client.subscribe(subsc_topic);
    }
    else {
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}



void setup() {
  EEPROM.begin(512);              // Begin eeprom to store on/off state
  Serial.begin(115200);
  Serial.write("Estou no setup!");

  //CONNECTIONS
  setup_wifi();
  search_config();

  strcpy(&subsc_topic[4], device_name);

  client.setServer(mqtt_server, 1883);
  client.setCallback(mqtt_callback);

  client.subscribe("dev/");
  client.subscribe(subsc_topic);

  client.publish("dev/CONNECTIONS", device_name);
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
}
