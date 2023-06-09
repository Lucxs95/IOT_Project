#include "OneWire.h"
#include "DallasTemperature.h"
#include <ArduinoJson.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include "FS.h"
#include "SPIFFS.h"
#include "ESPAsyncWebServer.h"
#include <PubSubClient.h>

const char *mqtt_server = "mqtt.eclipseprojects.io";
#define TOPIC_PISCINE "uca/iot/piscine"
#define TOPIC_TELEPHONE_CONNECTED "uca/waterbnb/21904022/lucasPool"

#define latitudePiscine 43.5704187
#define longitudePiscine 6.9917197

WiFiClient espClient;
PubSubClient client(espClient);

const int ledPin = 2;

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

OneWire oneWire(23);
DallasTemperature tempSensor(&oneWire);

float light = 0;

void wifi_connect_basic(String hostname, String ssid, String passwd)
{
  int nbtry = 0;

  WiFi.mode(WIFI_OFF);
  WiFi.mode(WIFI_STA);
  WiFi.disconnect(true);

  WiFi.setHostname(hostname.c_str());

  while (WiFi.status() != WL_CONNECTED && (nbtry < WiFiMaxTry))
  {
    nbtry++;
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

void set_LED(int v)
{
  digitalWrite(ledPin, v);
}

float get_Temperature()
{
  tempSensor.requestTemperatures();
  float temperature = tempSensor.getTempCByIndex(0);
  return temperature;
}

float calculateDistance(float lat1, float lon1, float lat2, float lon2)
{
  float lat1Rad = radians(lat1);
  float lon1Rad = radians(lon1);
  float lat2Rad = radians(lat2);
  float lon2Rad = radians(lon2);

  float radius = 6371.0;

  float deltaLat = lat2Rad - lat1Rad;
  float deltaLon = lon2Rad - lon1Rad;

  float a = sin(deltaLat / 2) * sin(deltaLat / 2) +
            cos(lat1Rad) * cos(lat2Rad) *
                sin(deltaLon / 2) * sin(deltaLon / 2);
  float c = 2 * atan2(sqrt(a), sqrt(1 - a));
  float distance = radius * c;

  return distance;
}

bool isInsidePerimeter(float clientLatitude, float clientLongitude, float poolLatitude, float poolLongitude, float radius)
{
  float distance = calculateDistance(clientLatitude, clientLongitude, poolLatitude, poolLongitude);
  return distance <= radius;
}

void mqtt_pubcallback(char *topic, byte *message, unsigned int length)
{
  if (strcmp(topic, TOPIC_PISCINE) == 0)
  {
    Serial.print("Message arrived on topic : ");
    Serial.println(topic);
    Serial.print("=> ");

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

    float currentLatitude = latitudePiscine;
    float currentLongitude = longitudePiscine;

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

  if (strcmp(topic, TOPIC_TELEPHONE_CONNECTED) == 0)
  {
    Serial.print("Message arrived on topic : ");
    Serial.println(topic);
    Serial.print("=> ");
    String messageTemp;
    for (int i = 0; i < length; i++)
    {
      Serial.print((char)message[i]);
      messageTemp += (char)message[i];
    }
    Serial.println();

    DynamicJsonDocument jsondoc(4096);
    deserializeJson(jsondoc, message);

    const char *etatPorte = jsondoc["piscine"]["porte"]["etat"].as<const char *>();
    const char *userIdu = jsondoc["piscine"]["user"]["idu"].as<const char *>();
    const char *userIswp = jsondoc["piscine"]["user"]["idswp"].as<const char *>();
    float userIswpLon = jsondoc["piscine"]["user"]["lon"].as<float>();
    float userIswpLat = jsondoc["piscine"]["user"]["lat"].as<float>();
    const char *nomPiscine = jsondoc["piscine"]["nomPiscine"].as<const char *>();

    const float distanceBetween = calculateDistance(userIswpLat, userIswpLon, latitudePiscine, longitudePiscine);

    const boolean piscineOpenAndUserOutsidePerimeter = distanceBetween > 100;

    if (strcmp(etatPorte, "open") == 0)
    {
      digitalWrite(ledPin, HIGH);
      delay(30000);
      digitalWrite(ledPin, LOW);
      color = "yellow";
    }
    else if (strcmp(nomPiscine, "null") == 0 && piscineOpenAndUserOutsidePerimeter)
    {
      digitalWrite(ledPin, LOW);
      color = "blue";
    }
  }
}

String Serialize_ESPstatus(float temp, int light, unsigned long uptime)
{
  const boolean hot = (temp > SH);
  const boolean cold = (temp < SB);
  const boolean isRUNNING = (hot || cold);
  const boolean isFIRE = (light > 1000 && temp > 30.0);
  const String LOCATION = "342";
  const String isPresence = (light < 10000) ? "1" : "0";

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

  jsondoc["info"]["ident"] = "lucasPool";
  jsondoc["info"]["user"] = "Lucas";
  jsondoc["info"]["loc"]["lat"] = latitudePiscine;
  jsondoc["info"]["loc"]["lon"] = longitudePiscine;
  jsondoc["info"]["description"] = "test";
  jsondoc["info"]["uptime"] = uptime;
  jsondoc["info"]["ssid"] = WiFi.SSID();
  jsondoc["info"]["ip"] = WiFi.localIP().toString();

  jsondoc["piscine"]["led"] = color;
  jsondoc["piscine"]["presence"] = isPresence;

  String data = "";
  serializeJson(jsondoc, data);
  Serial.println(data);
  return data;
}

void mqtt_mysubscribe(const char *topic)
{
  while (!client.connected())
  {
    Serial.print("Attempting MQTT connection...");
    if (client.connect("esp32", NULL, NULL))
    {
      Serial.println("connected");
      client.subscribe(topic);
    }
    else
    {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

void setup()
{
  Serial.begin(9600);
  while (!Serial)
    ;

  tempSensor.begin();

  String ssid = String("Freebox-A66E84");
  String password = String("lucaslucille2906");
  String hostname = String("ESP32-Lucas");

  wifi_connect_basic(hostname, ssid, password);
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

  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);

  tempSensor.begin();

  client.setServer(mqtt_server, 1883);
  client.setCallback(mqtt_pubcallback);

  mqtt_mysubscribe(TOPIC_TELEPHONE_CONNECTED);
  mqtt_mysubscribe(TOPIC_PISCINE);
}

unsigned long lastPublishTime = 0;

void publish()
{
  unsigned long currentTime = millis();

  if (currentTime - lastPublishTime >= 5000)
  {
    lastPublishTime = currentTime;

    temperature = get_Temperature();
    String data = Serialize_ESPstatus(temperature, (int)lightLevel, uptime);

    Serial.print("Published JSON: ");
    Serial.println(data);
    client.setBufferSize(3048);

    client.publish(TOPIC_PISCINE, data.c_str());
  }
}

void loop()
{
  uptime = (millis() - startTime) / 1000;
  int lightValue = analogRead(LIGHT_PIN);
  lightLevel = lightValue * VOLTAGE_REFERENCE * LIGHT_FACTOR;

  if (!client.connected())
  {
    mqtt_mysubscribe(TOPIC_TELEPHONE_CONNECTED);
    mqtt_mysubscribe(TOPIC_PISCINE);
  }

  publish();

  client.loop();
}