#include "OneWire.h"
#include "DallasTemperature.h"
#include <ArduinoJson.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include "FS.h"
#include "SPIFFS.h"
#include "ESPAsyncWebServer.h"
#include <PubSubClient.h>

/*===== MQTT broker/server ========*/
// const char* mqtt_server = "192.168.1.101";
// const char* mqtt_server = "public.cloud.shiftr.io"; // Failed in 2021
//  need login and passwd (public,public) mqtt://public:public@public.cloud.shiftr.io
// const char* mqtt_server = "broker.hivemq.com"; // anynomous Ok in 2021
const char* mqtt_server = "test.mosquitto.org"; // anynomous Ok in 2021
// const char *mqtt_server = "mqtt.eclipseprojects.io"; // anynomous Ok in 2021

/*===== MQTT TOPICS ===============*/
// #define TOPIC_TEMP "uca/M1/iot/temp"
// #define TOPIC_LED "uca/M1/iot/led"
#define TOPIC_PISCINE "uca/iot/piscine"

/*===== ESP is MQTT Client =======*/
WiFiClient espClient;           // Wifi
PubSubClient client(espClient); // MQTT client

/*============= GPIO ==============*/
const int ledPin = 19; // LED Pin

/*============= CONST ==============*/
#define WiFiMaxTry 10
#define SaveDisconnectTime 1000
const int SH = 25;
const int SB = 20;
unsigned long startTime = 0;
float temperature;
float lightLevel;
unsigned long uptime = 0;
const int LIGHT_PIN = 33;
const float LIGHT_FACTOR = 1000.0 / 1024.0;
const float VOLTAGE_REFERENCE = 5.0;
String color = "blue";

/* ---- TEMP ---- */
OneWire oneWire(23);                    // Pour utiliser une entite oneWire sur le port 23
DallasTemperature tempSensor(&oneWire); // Cette entite est utilisee par le capteur de temperature

float light = 0;

/*-----------CONNECT WIFI-------------*/

void wifi_connect_basic(String hostname, String ssid, String passwd)
{
  int nbtry = 0; // Nb of try to connect

  WiFi.mode(WIFI_OFF);
  WiFi.mode(WIFI_STA);
  WiFi.disconnect(true); // Disconnect from an AP if it was previously connected

  WiFi.setHostname(hostname.c_str());

  while (WiFi.status() != WL_CONNECTED && (nbtry < WiFiMaxTry))
  {
    nbtry++; // Reset the nbtry counter to zero before each attempt
    Serial.printf("\nAttempting %d to connect AP of SSID : %s", nbtry, ssid.c_str());
    WiFi.begin(ssid.c_str(), passwd.c_str());
    while (WiFi.status() != WL_CONNECTED && (millis() - startTime < SaveDisconnectTime))
    {
      delay(100);
      Serial.print(".");
    }
  }
}

String translateEncryptionType(wifi_auth_mode_t encryptionType)
{
  switch (encryptionType)
  {
  case (WIFI_AUTH_OPEN):
    return "Open";
  case (WIFI_AUTH_WEP):
    return "WEP";
  case (WIFI_AUTH_WPA_PSK):
    return "WPA_PSK";
  case (WIFI_AUTH_WPA2_PSK):
    return "WPA2_PSK";
  case (WIFI_AUTH_WPA_WPA2_PSK):
    return "WPA_WPA2_PSK";
  case (WIFI_AUTH_WPA2_ENTERPRISE):
    return "WPA2_ENTERPRISE";
  }
  return "unknown";
}

void wifi_status()
{
  String s = "WiFi Status : \n";
  s += "\tIP address : " + WiFi.localIP().toString() + "\n";
  s += "\tMAC address : " + String(WiFi.macAddress()) + "\n";
  s += "\tSSID : " + String(WiFi.SSID()) + "\n";
  s += "\tReceived Signal Strength Indication : " + String(WiFi.RSSI()) + " dBm\n";
  s += "\tReceived Signal Strength Indication : " + String(constrain(2 * (WiFi.RSSI() + 100), 0, 100)) + " %\n";
  s += "\tBSSID : " + String(WiFi.BSSIDstr()) + "\n";
  s += "\tEncryption type : " + translateEncryptionType(WiFi.encryptionType(0)) + "\n";
  Serial.print(s);
}

/*============= TO COMPLETE ===================*/
void set_LED(int v)
{
  digitalWrite(ledPin, v);
}

float get_Temperature()
{
  tempSensor.requestTemperatures();                  // Demande de mise à jour de la valeur de température
  float temperature = tempSensor.getTempCByIndex(0); // Récupération de la valeur de température
  return temperature;
}


/*============== CALLBACK ===================*/

float calculateDistance(float lat1, float lon1, float lat2, float lon2)
{
  // Convertit les coordonnées en radians
  float lat1Rad = radians(lat1);
  float lon1Rad = radians(lon1);
  float lat2Rad = radians(lat2);
  float lon2Rad = radians(lon2);

  // Rayon moyen de la Terre en kilomètres
  float radius = 6371.0;

  // Calcul des différences de latitude et de longitude
  float deltaLat = lat2Rad - lat1Rad;
  float deltaLon = lon2Rad - lon1Rad;

  // Calcul de la distance orthodromique
  float a = sin(deltaLat / 2) * sin(deltaLat / 2) +
            cos(lat1Rad) * cos(lat2Rad) *
                sin(deltaLon / 2) * sin(deltaLon / 2);
  float c = 2 * atan2(sqrt(a), sqrt(1 - a));
  float distance = radius * c;

  return distance;
}

void mqtt_pubcallback(char *topic, byte *message, unsigned int length)
{
  /*
   * Callback if a message is published on this topic.
   */

  if (strcmp(topic, TOPIC_PISCINE) == 0)
  {
    Serial.print("Message arrived on topic : ");
    Serial.println(topic);
    Serial.print("=> ");

    // Byte list to String and print to Serial
    String messageTemp;
    for (int i = 0; i < length; i++)
    {
      Serial.print((char)message[i]);
      messageTemp += (char)message[i];
    }
    Serial.println();

    StaticJsonDocument<2000> jsondoc;
    deserializeJson(jsondoc, message);
    float temperatureRecupere = jsondoc["status"]["temperature"].as<float>();
    float latitudeRecupere = jsondoc["info"]["loc"]["lat"].as<float>();
    float longitudeRecupere = jsondoc["info"]["loc"]["lon"].as<float>();
    String usernameRecupere = jsondoc["info"]["ident"].as<String>();

    // Check if the user is within the specified radius
    float currentLatitude = 43.5704187; // Your current latitude
    float currentLongitude = 6.9917197; // Your current longitude

    float distanceBetween = calculateDistance(currentLatitude, currentLongitude, latitudeRecupere, longitudeRecupere);

    if (distanceBetween < 25.0 && temperatureRecupere > temperature)
    {
      color = "green";
    }
    else if (temperatureRecupere < temperature)
    {
      color = "red";
    }
    
  }
}

/*----------------Serialize_ESPstatus-------------------------*/
String Serialize_ESPstatus(float temp, int light, unsigned long uptime)
{
  const boolean hot = (temp > SH);
  const boolean cold = (temp < SB);
  const boolean isRUNNING = (hot || cold);
  const boolean isFIRE = (light > 1000 && temp > 30.0);
  const String LOCATION = "342";
  const String isPresence = (light < 10000 ) ? "1" :"0";

  const String target_ip = "";
  const int target_port = 0;
  const int target_sp = 0;

  StaticJsonDocument<1000> jsondoc;
  jsondoc["status"]["temperature"] = temp;
  jsondoc["status"]["light"] = light;
  jsondoc["status"]["heat"] = hot;
  jsondoc["status"]["cold"] = cold;
  jsondoc["status"]["running"] = isRUNNING;
  jsondoc["status"]["fire"] = isFIRE;

  jsondoc["regul"]["sh"] = SH;
  jsondoc["regul"]["sb"] = SB;

  jsondoc["info"]["ident"] = "Lucas BLANC";
  jsondoc["info"]["user"] = "Lucas";
  jsondoc["info"]["loc"]["lat"] = "43.5704187";
  jsondoc["info"]["loc"]["lon"] = "6.9917197";
  jsondoc["info"]["description"] = "test";
  jsondoc["info"]["uptime"] = uptime;
  jsondoc["info"]["ssid"] = WiFi.SSID();
  jsondoc["info"]["ip"] = WiFi.localIP().toString();

  jsondoc["piscine"]["led"] = color ;
  jsondoc["piscine"]["presence"] = isPresence ;

  String data = "";
  serializeJson(jsondoc, data);
  Serial.println(data);
  return data;
}

/*============= SUBSCRIBE =====================*/
void mqtt_mysubscribe(char *topic)
{
  /*
   * Subscribe to a MQTT topic
   */
  while (!client.connected())
  { // Loop until we're reconnected

    Serial.print("Attempting MQTT connection...");
    // Attempt to connect => https://pubsubclient.knolleary.net/api
    if (client.connect("esp32", /* Client Id when connecting to the server */
                       NULL,    /* No credential */
                       NULL))
    {
      Serial.println("connected");
      // then Subscribe topic
      client.subscribe(topic);
    }
    else
    {
      Serial.print("failed, rc=");
      Serial.print(client.state());

      Serial.println(" try again in 5 seconds");
      delay(5000); // Wait 5 seconds before retrying
    }
  }
}
/*===== Arduino IDE paradigm : setup+loop =====*/
void setup()
{
  Serial.begin(9600);
  while (!Serial)
    ; // wait for a serial connection. Needed for native USB port only

  tempSensor.begin();

  String ssid = String("Freebox-A66E84");
  String password = String("lucaslucille2906");
  String hostname = String("ESP32-Lucas");

  wifi_connect_basic(hostname, ssid, password); // Connexion Wifi
  wifi_status();

  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.print(".");
  }

  Serial.print("WiFi connected: yes!\n");
  Serial.println("IP: " + WiFi.localIP().toString());
  wifi_status();

  if (!SPIFFS.begin(true))
  {
    Serial.println("An error occurred while mounting SPIFFS");
    return;
  }
  

  // Initialize the output variables as outputs
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW); // Set outputs to LOW

  // Init temperature sensor
  tempSensor.begin();

  // set server of our client
  client.setServer(mqtt_server, 1883);
  // set callback when publishes arrive for the subscribed topic

  client.setCallback(mqtt_pubcallback);
}

/*------------------ PUBLISH ----------------------*/
unsigned long lastPublishTime = 0; // Variable to store the time of the last publish

void publish()
{
  unsigned long currentTime = millis();
  
  // Check if enough time has elapsed since the last publish
  if (currentTime - lastPublishTime >= 15000) // 15 seconds timeout
  {
    lastPublishTime = currentTime;
    
    /*--- Publish Temperature periodically ---*/
    temperature = get_Temperature();
    String data = Serialize_ESPstatus(temperature, (int)lightLevel, uptime);
    
    // Serial info
    Serial.print("Published JSON: ");
    Serial.println(data);
    client.setBufferSize(3048);
    
    // MQTT Publish
    client.publish(TOPIC_PISCINE, data.c_str());
  }
}

/*================= LOOP ======================*/
void loop()
{
  uptime = (millis() - startTime) / 1000;
  int lightValue = analogRead(LIGHT_PIN);
  lightLevel = lightValue * VOLTAGE_REFERENCE * LIGHT_FACTOR;

  /*--- subscribe to TOPIC_LED if not yet ! */
  if (!client.connected())
  {
    mqtt_mysubscribe((char *)(TOPIC_PISCINE));
  }

  publish();

  /* Process MQTT ... une fois par loop() ! */
  client.loop();
}