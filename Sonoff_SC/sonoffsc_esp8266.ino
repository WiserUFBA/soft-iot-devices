#include <FS.h>                    //this needs to be first, or it all crashes and burns...

#include <ESP8266WiFi.h>
#include <pgmspace.h>
#include <PubSubClient.h>  //https://github.com/knolleary/pubsubclient
#include <SoftwareSerial.h>
#include <Bounce2.h>  //https://github.com/thomasfredericks/Bounce2
#include <EEPROM.h>

#include <TATUDevice.h>
#include <TATUInterpreter.h>
#include <FlowController.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>

//library configurationBrowser
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h>          //https://github.com/tzapu/WiFiManager
#include <ArduinoJson.h>           //https://github.com/bblanchon/ArduinoJson

//workaround
byte ip[5] = "11";
//
char vector_response[1024];

#define stringSize 20
char device_name[stringSize]    = "sonoffsc01";
char mqtt_user[stringSize]      = "wiser";
char mqtt_pass[stringSize]      = "wiser2014";
char port[stringSize]      = "1883";
char mqtt_server[40];

char    subsc_topic[20] = "dev/";
int mqttport = 1883;

// wifi
char ssid[20];
char pass[20];

bool get(uint32_t hash, void* response, uint8_t code);
bool set(uint32_t hash, uint8_t code, void* response);

void requestSensor(int id, uint8_t type);
void requestActuator(int id, uint8_t type, int value);
void read_serial(double* response, int id);

// System variables
WiFiClient    espClient;
TATUInterpreter interpreter;
MQTT_BRIDGE(bridge);
TATUDevice device(device_name, 1, &interpreter, get, set, bridge);
MQTT_CALLBACK(bridge, device, mqtt_callback);
PubSubClient  client(espClient);
MQTT_PUBLISH(bridge, client);

Bounce debouncer = Bounce();
int oldValue = 0;
unsigned long previousMillis = 0;

#define SENSORS_SIZE 5
SensorMap sensors[SENSORS_SIZE];
FlowController fluxo(&device, vector_response,sensors);
FlowUnit unit1, unit2, unit3, unit4, unit5;

bool flow(uint32_t hash, uint8_t code, void* response) {
  fluxo.flowbuilder((char*)response, hash, code);
  return true;
}

const char set_flow[] = "FLOW INFO temperatureSensor {\"collect\":400,\"publish\":2000}";
const char set_f_luminosity[] = "FLOW INFO luminositySensor {\"collect\":400,\"publish\":2000}";
#define FLOW_BUFFER_SIZE 25
int buffer_int_flow[FLOW_BUFFER_SIZE];
int buffer_int_flow2[FLOW_BUFFER_SIZE];
int buffer_int_flow3[FLOW_BUFFER_SIZE];
int buffer_int_flow4[FLOW_BUFFER_SIZE];
int buffer_int_flow5[FLOW_BUFFER_SIZE];
//char buffer_char_flow[FLOW_BUFFER_SIZE][10];
// char buffer_char_flow[FLOW_BUFFER_SIZE][10];
// char buffer_char_flow2[FLOW_BUFFER_SIZE][10];

/*
  Sensor ID's defines
*/
//  <sensorsID>
#define ID_temp     1
#define ID_lumin    2
#define ID_sound    3
#define ID_dust     4
#define ID_hum      5
//  </sensorsID>

/*
  Actuators ID's defines
*/
//  <actuatorsID>
#define ID_lamp     0
//  </actuatorsID>

//Hash that represents the attributes
#define H_temperatureSensor 0x5821FBAD
#define H_luminositySensor  0xF8D7A29C
#define H_soundSensor       0x308B1BA8
#define H_dustSensor        0xD2A27EDF
#define H_humiditySensor    0x83D62D4C
//flow
#define H_flow              0x7C96D85D


//flag for saving data
bool shouldSaveConfig = false;

//callback notifying us of the need to save config
void saveConfigCallback () {
  Serial.println("Should save config");
  shouldSaveConfig = true;
}

bool get(uint32_t hash, void* response, uint8_t code) {
  uint8_t id;

  if (code == TATU_CODE_FLOW){
    STOS(response, vector_response);
    return true;
  }

  switch (hash) {
    case H_temperatureSensor:
      // The dht_temperatures_sensor supports INFO and VALUE requests.
      id = ID_temp;
      break;
    case H_humiditySensor:
      id = ID_hum;
      break;
    case H_luminositySensor:
      // The lumisity_sensor supports INFO and VALUE,requests.
      id = ID_lumin;
      break;
    case H_soundSensor:
      id = ID_sound;
      break;
      case H_dustSensor:
      id = ID_dust;
      break;

    case H_flow:
      STOS(response, vector_response);
      return true;
      break;
    default:
      return false;
  }

  //<bus>
  char bus_buffer[50];
  requestSensor(id, 's');
  //read_serial((int*)response,id);
  double bus_res;
  read_serial(&bus_res, id);
  //Handling types
  //int
  if (code == TATU_CODE_VALUE) {
    //*(int*)response = atoi(bus_buffer);
    *(int*)response = bus_res;
    //ESPSerial.print("Response: ");
    //ESPSerial.println(*(int*)response);
  }
  //</bus>

 /*
  WiFi.forceSleepBegin();
  delay(1000);
  WiFi.forceSleepWake();
  delay(100);
  search_config();
  */
  
  return true;
}

bool set(uint32_t hash, uint8_t code, void* response) {
  uint8_t id;
  switch (hash) {
    default:
      return false;
  }

  //<bus>
  //Handling types
  //int
  if (code == TATU_CODE_VALUE) {
    //*(int*)response = atoi(bus_buffer);
    requestActuator(id, 'a',(int)response);
  }
  //</bus>

  return true;
}

void setup_wifi() {

    Serial.println("Starting Wifi Web Manager...");
    delay(10);

 // The extra parameters to be configured (can be either global or just in the setup)
  // After connecting, parameter.getValue() will get you the configured value
  // id/name placeholder/prompt default length
  WiFiManagerParameter custom_device_name("device", "device name", device_name, 15);
  WiFiManagerParameter custom_mqtt_user("user", "mqtt user", mqtt_user, 20);
  WiFiManagerParameter custom_mqtt_pass("pass", "mqtt pass", mqtt_pass, 20);
  WiFiManagerParameter custom_mqtt_server("server", "mqtt server", mqtt_server, 40);
  WiFiManagerParameter custom_mqtt_port("port", "mqtt port", port, 5);

  WiFiManager wifiManager;
  wifiManager.setDebugOutput(false);

  //set config save notify callback
  wifiManager.setSaveConfigCallback(saveConfigCallback);

  //add all your parameters here
  wifiManager.addParameter(&custom_device_name);
  wifiManager.addParameter(&custom_mqtt_user);
  wifiManager.addParameter(&custom_mqtt_pass);
  wifiManager.addParameter(&custom_mqtt_server);
  wifiManager.addParameter(&custom_mqtt_port);

  wifiManager.startConfigPortal("SonoffAP01", "teste2014");
  //wifiManager.resetSettings();
  //wifiManager.autoConnect("UFBAinoAP", "wiser2014");

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected to: " + WiFi.SSID() +
                    "pass: " + WiFi.psk());

  WiFi.mode(WIFI_STA);
  // wifiManager

  //read updated parameters
  strcpy(device_name, custom_device_name.getValue());
  strcpy(mqtt_user, custom_mqtt_user.getValue());
  strcpy(mqtt_pass, custom_mqtt_pass.getValue());
  strcpy(mqtt_server, custom_mqtt_server.getValue());
  strcpy(port, custom_mqtt_port.getValue());
  mqttport = atoi(port);

  //save the custom parameters to FS
  if (shouldSaveConfig && SPIFFS.begin()) {
    Serial.println("Saving config...");
    DynamicJsonBuffer jsonBuffer;
    JsonObject& json = jsonBuffer.createObject();
    json["device_name"] = device_name;
    json["mqtt_user"] = mqtt_user;
    json["mqtt_pass"] = mqtt_pass;
    json["mqtt_server"] = mqtt_server;
    json["port"] = port;
    json["SSID"] = WiFi.SSID();
    json["pass"] = WiFi.psk();

    File configFile = SPIFFS.open("/config.json", "w");
    if (!configFile) {
      Serial.println("Failed to open config file for writing");
    }

    json.printTo(Serial);
    json.printTo(configFile);
    configFile.close();
    //end save
  }
  
/*
  device.init(device_name, 1, &interpreter);
  strcpy(&subsc_topic[4], device_name);

  client.setServer(mqtt_server, 1883);
  client.setCallback(mqtt_callback);

  client.subscribe("dev/");
  client.subscribe(subsc_topic);

  client.publish("dev/CONNECTIONS", device_name);
*/
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

}

bool search_config() {
  //read configuration from FS json

  Serial.println("Trying to load previous configuration...");

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

          strcpy(device_name, json["device_name"]);
          strcpy(mqtt_user, json["mqtt_user"]);
          strcpy(mqtt_pass, json["mqtt_pass"]);
          strcpy(mqtt_server, json["mqtt_server"]);
          strcpy(port, json["port"]);
          strcpy(ssid, json["SSID"]);
          strcpy(pass, json["pass"]);
          mqttport = atoi(port);
          if(WiFi.status() == WL_CONNECTED) return true;
          WiFi.begin(ssid,pass);
          for(int i=0;WiFi.status() != WL_CONNECTED && i < 10;i++) {
            delay(1000);
            Serial.print(".");
          }
          if(WiFi.status() != WL_CONNECTED){
            return false;
          }else{
            Serial.println("");
            Serial.println("WiFi connected to: " + WiFi.SSID() +
                            "pass: " + WiFi.psk());
            return true;
          }
        } else {
          Serial.println("Failed to load json config");
          return false;
        }
      }else{
        Serial.println("Couldn't open configuration file");
        return false;
      }
    }else{
      Serial.println("Didn't find previous configuration");
      return false;
    }
  } else {
    Serial.println("failed to mount FS");
    return false;
  }
}

void connectMqtt(){
  device.init(device_name, 1, &interpreter);
  strcpy(&subsc_topic[4], device_name);

  client.setServer(mqtt_server, 1883);
  client.setCallback(mqtt_callback);
  if (client.connect(device_name, mqtt_user, mqtt_pass)) {
      // Once connected, publish an announcement...
      client.publish("CONNECTED", device_name);
      // ... and resubscribe
      client.subscribe("dev/");
      client.subscribe(subsc_topic);
    }else{
      Serial.println("Couldn't connect in mqtt broker");
    }
 /* client.subscribe("dev/");
  client.subscribe(subsc_topic);

  client.publish("dev/CONNECTIONS", device_name);
  */
}

void requestSensor(int id, uint8_t type) {
  char req[8];
  sprintf(req, "%c:%d", type, id);
  Serial.println(req);
//  client.publish("debug","Req: ");
//  client.publish("debug",req);
}

void requestActuator(int id, uint8_t type, int value) {
  char req[8];
  sprintf(req, "%c:%d:%d", type, id, value);
  Serial.println(req);
}

//Predicado que verifica se o id de retorno está correto
bool pred_id(char* topic, int id) {
  int count = 0;
  if ((atoi(topic) != id)) {
    //ESPSerial.println("Mensagem Errada!");
    while (!Serial.available() && count < 10) {
      count++;
      delay(50);
    }
    return true;
  }

  return false;
}

void read_serial(double* response, int id) {
  int i;
  int count = 0;
  char msg[255];
  char topic[20];
  
  do {
    count++;
    if (count > 6) break;

    delay(50);
//    ESPSerial.println("ATMEGA Response");
//    ESPSerial.print("Topic = ");
    for (i = 0; Serial.available(); i++ ) {
      topic[i] = Serial.read();
      if (topic[i] == '>') break;
//      ESPSerial.print(topic[i]);
    }
//    ESPSerial.println();

    topic[i] = 0;
    //client.publish("debug","Topic = ");
    //client.publish("debug",topic);
//    ESPSerial.print("Message = ");Erro...
    for (i = 0; Serial.available(); i++) {
      msg[i] = Serial.read();
//      ESPSerial.print(msg[i]);
      if (msg[i] == '\n') break;
    }

    msg[i] = 0;
    //client.publish("debug","Message = ");
    //  client.publish("debug",msg);
    //fail safety
  } while (pred_id(topic, id));

  //int
  *response = atof(msg);
  ESPSerial.print("Response: ");
  ESPSerial.println(*response);
  //strcpy(response,msg);

}

void reconnect() {
  Serial.println("Trying reconnect in mqtt broker...");
  // Loop until we're reconnected
  if (!client.connected()) {
    // Attempt to connect
    if (client.connect(device_name, mqtt_user, mqtt_pass)) {
      // Once connected, publish an announcement...
      client.publish("CONNECTED", device_name);
      // ... and resubscribe
      client.subscribe("dev/");
      client.subscribe(subsc_topic);
    } 
    else {
      Serial.println("Couldn't connect in mqtt broker");
    }
  }

}

void extButton() {
  debouncer.update();
  // Get the update value
  int value = debouncer.read();
  if (value != oldValue && value==0) {
    saveConfigCallback();
    setup_wifi();
    connectMqtt();
  }
  oldValue = value;
}

void setup() {

  Serial.begin(115200);
  ESPSerial.begin(115200);
  ESPSerial.write("Starting setup procedure...");

  //flow settings
  device.flow_function = &flow;
  unit1.vector = buffer_int_flow;unit2.vector = buffer_int_flow2;unit3.vector = buffer_int_flow3;unit4.vector = buffer_int_flow4;unit5.vector = buffer_int_flow5;
  // unit1.vector = buffer_char_flow;
  // unit2.vector = buffer_int_flow2;
  unit1.used = false; unit2.used = false;unit3.used = false; unit4.used = false;unit5.used = false;
  fluxo.activity = &unit1;
  unit1.next = &unit2; unit2.next = &unit3; unit3.next = &unit4;unit4.next = &unit5;
  //

  //CONNECTIONS
  // setup_wifi();
  debouncer.attach(0);   // Use the bounce2 library to debounce the built in button
  debouncer.interval(3);         // Input must be low for 50 ms
  //setup_wifi();
  if (!search_config()){
    setup_wifi();
  }
  

  //
  //map settings
  String sensorNames[SENSORS_SIZE];
  sensorNames[0] = "temperatureSensor";
  sensorNames[1] = "soundSensor";
  sensorNames[2] = "luminositySensor";
  sensorNames[3] = "humiditySensor";
  sensorNames[4] = "dustSensor";

  SensorMap::init(SENSORS_SIZE,sensorNames,sensors);
  // strcpy(sensors[0].sensorName,"temperatureSensor");
  // strcpy(sensors[1].sensorName,"soundSensor");
  // strcpy(sensors[2].sensorName,"luminositySensor");
  // strcpy(sensors[3].sensorName,"humiditySensor");
  // strcpy(sensors[4].sensorName,"dustSensor");
  //sensors[0].sensorName = "temperatureSensor";
  //sensors[1].sensorName = "currentSensor01";

  // sensors[0].hash = H_temperatureSensor;
  // sensors[1].hash = H_soundSensor;
  // sensors[2].hash = H_luminositySensor;
  // sensors[3].hash = H_humiditySensor;
  // sensors[4].hash = H_dustSensor;
  //

  connectMqtt();
  /*byte req[80];

    strcpy((char*)req, set_flow);
    device.mqtt_callback("", req, strlen((char*)req) );
    strcpy((char*)req, set_f_luminosity);
    device.mqtt_callback("", req, strlen((char*)req) );*/
}


void loop(){
  unsigned long currentMillis = millis();
  extButton();
  //verify the mqtt connection each 5 seconds, without blocking
  if (currentMillis - previousMillis >= 5000) {
    previousMillis = currentMillis;
    if (!client.connected()) {
      reconnect();
    }
  }
  //ESPSerial.write("Cheguei no loop!");
  //extButton();
  client.loop();
  fluxo.loop();
}
