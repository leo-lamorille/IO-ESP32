
#include <WiFi.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <Arduino.h>
#include <AsyncTCP.h>
#include "ESPAsyncWebServer.h"
#include "DNSServer.h"
#include "DHT.h"
#include <Preferences.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <AsyncTimer.h>

#define EEPROM_SIZE 12

#define DHTPIN 14
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

WiFiClient espClient;
PubSubClient client(espClient);

const char *ssid = "Group4";

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP);
const long gmtOffset_sec = 0;
const int daylightOffset_sec = 3600;

AsyncWebServer server(80);
DNSServer dnsServer;
Preferences preferences;

String networkSSID;
String networkPassword;
String ipServer;
int delayTime = 50;
bool networkSSID_received = false;
bool networkPassword_received = false;
bool ipServer_received = false;

String formattedDate;
String dayStamp;
String timeStamp;

AsyncTimer t;
unsigned short setIntervalId;
unsigned short setIntervalIdWifi;

// HTML du captive portal
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html><head>
  <title>Captive Portal Demo</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  </head><body>
  <h3>Captive Portal Demo</h3>
  <br><br>
  <form action="/get">
    <br>
    SSID: <input type="text" name="ssid">
    <br>
    <br>
    MDP: <input type="text" name="mdp">
    <br>
    <br>
    IP Server: <input type="text" name="ip">
    <br>
    <input type="submit" value="Submit">
  </form>
</body></html>)rawliteral";

bool modeConfig = true;
bool modeSender = false;

// prototype des fonctions
void printLocalTime();
void sendMQTTData(float humidity, float temperature, int airQuality);
void callback(char *topic, byte *payload, unsigned int length);
void clearPreferences();
void connectToWifi(String SSID, String password);
void collectData();
void savePreference(String ssid, String password, String ip, int delay);
void sendSensorParams();
void loopInCaptivePortalMode();
void loopInSenderMode();

// classe pour la gestion des requetes lors que le formulaire est envoyé par l'utilisateur dans le captive portal
class CaptiveRequestHandler : public AsyncWebHandler
{
public:
  CaptiveRequestHandler() {}
  virtual ~CaptiveRequestHandler() {}

  bool canHandle(AsyncWebServerRequest *request)
  {
    // request->addInterestingHeader("ANY");
    return true;
  }

  void handleRequest(AsyncWebServerRequest *request)
  {
    request->send_P(200, "text/html", index_html);
  }
};

// fonction pour mettre en place le captive portal et récupérer les données du formulaire
void setupServer()
{
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
            {
    request->send_P(200, "text/html", index_html); 
    Serial.println("Client Connected"); });

  server.on("/get", HTTP_GET, [](AsyncWebServerRequest *request)
            {
  
      if (request->hasParam("ssid")) {
        networkSSID = request->getParam("ssid")->value();
        networkSSID_received = true;
      }

      if (request->hasParam("mdp")) {
        networkPassword = request->getParam("mdp")->value();
        networkPassword_received = true;
      }

      if (request->hasParam("ip")) {
        ipServer = request->getParam("ip")->value();
        ipServer_received = true;
      }
      request->send(200, "text/html", "The values entered by you have been successfully sent to the device <br><a href=\"/\">Return to Home Page</a>"); });
}

void setupInCaptivePortalMode()
{
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid);
  Serial.print("AP IP address: ");
  Serial.println(WiFi.softAPIP());
  setupServer();
  dnsServer.start(53, "*", WiFi.softAPIP());
  server.addHandler(new CaptiveRequestHandler()).setFilter(ON_AP_FILTER);
  server.begin();
  Serial.println("All Done!");
}

void setupInSenderMode()
{
  modeConfig = false;
  connectToWifi(networkSSID, networkPassword);
  if (WiFi.status() == WL_CONNECTED)
  {
    timeClient.begin();
    timeClient.setTimeOffset(3600);
    Serial.println("WiFi connected");
    dht.begin();
    pinMode(35, INPUT);
    client.setServer(ipServer.c_str(), 1883);

    // setup du MQTT
    client.connect("ESP32Client");
    client.setCallback(callback);
    String topic = "sensor/params/" + String(WiFi.macAddress());
    client.subscribe(topic.c_str());
    Serial.println(delayTime);
    setIntervalId = t.setInterval(collectData, delayTime * 1000);
  }
  else
  {
    clearPreferences();
  }
}

void setup()
{
  Serial.begin(115200);

  preferences.begin("IO", false);
  networkSSID = preferences.getString("ssid");
  networkPassword = preferences.getString("password");
  ipServer = preferences.getString("ip");
  delayTime = preferences.getInt("delay");
  preferences.end();

  if (networkSSID != "" && networkPassword != "" && ipServer != "")
    setupInSenderMode();
  else
    setupInCaptivePortalMode();
}

void loop()
{
  t.handle();

  if (modeConfig)
    loopInCaptivePortalMode();
  else
    loopInSenderMode();
}

void loopInCaptivePortalMode()
{
  dnsServer.processNextRequest();
  if (ipServer_received && networkSSID_received && networkPassword_received)
  {
    savePreference(networkSSID, networkPassword, ipServer, delayTime);

    ipServer_received = false;
    networkSSID_received = false;
    networkPassword_received = false;
    ESP.restart();
  }
}

void loopInSenderMode()
{
  timeClient.update();
  client.loop();
  if (WiFi.status() == WL_CONNECTED)
  {
    // on vérifie si on est connecté au serveur MQTT
    // Si ce n'est pas le cas, alors on se reconnecte
    if (!client.connected())
    {
      client.connect("ESP32Client");
      client.setCallback(callback);
      String topic = "sensor/params/" + String(WiFi.macAddress());
      client.subscribe(topic.c_str());
    }
    else
    {
      // On vérifie s'il y a des données qui arrive depuis le serveur MQTT
      // Si on est pas en mode envoie de données, alors on renvoit les parametres du capteur pour s'enregistrer sur le MQTT
      if (!modeSender)
        sendSensorParams();
    }
  }
  else
  {
    // On essaie de se connecter au wifi
    // si l'ESP32 n'y arrive pas alors on redémarre l'ESP32 en mode captive portal
    connectToWifi(networkSSID, networkPassword);
    if (WiFi.status() != WL_CONNECTED)
      clearPreferences();
  }
}

// fonction pour envoyer les paramètres par defaut du capteur
void sendSensorParams()
{
  String jsonString = "{\"MacAddress\":\"" + String(WiFi.macAddress()) + "\",\"ip\":\"" + String(WiFi.localIP()) + "\",\"delay\": " + delayTime + "}";
  client.publish("sensor/register", jsonString.c_str());
}

// Fonction pour se connecter au réseau wifi
void connectToWifi(String SSID, String password)
{
  WiFi.begin(SSID.c_str(), password.c_str());
  int numberOfTry = 0;
  while (WiFi.status() != WL_CONNECTED && numberOfTry < 10)
  {
    delay(1000);
    Serial.print(".");
    numberOfTry++;
  }
}

// fonction de callback qui est appelée lorsqu'un message est reçu du serveur MQTT
void callback(char *topic, byte *payload, unsigned int length)
{
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] \n");
  DynamicJsonDocument doc(1024);
  deserializeJson(doc, payload);
  const int tempinterval = doc["interval"].as<int>();

  t.changeDelay(setIntervalId, delayTime * 1000);
  savePreference(networkSSID, networkPassword, ipServer, delayTime);
  modeSender = true;
}

// fonction qui récupère les données du capteur et appel la fonction qui envoie les données
void collectData()
{
  Serial.println("Collecting data");
  int humidity = dht.readHumidity();
  float temperature = dht.readTemperature();
  float airQuality = analogRead(35);
  sendMQTTData(humidity, temperature, airQuality);
}

// function qui crée un objet JSON et l'envoie au serveur MQTT
void sendMQTTData(float humidity, float temperature, int airQuality)
{
  String jsonString = "{\"Humidity\":" + String(humidity) + ",\"Temperature\":" + String(temperature) + ",\"AirQuality\":" + String(airQuality);
  jsonString += ",\"MacAddress\":\"" + String(WiFi.macAddress()) + "\",\"Time\":" + String(timeClient.getEpochTime()) + "}";
  client.publish("sensor/data", jsonString.c_str());
}

// fonction pour effacer les préférences dans la mémoire EEPROM
void clearPreferences()
{
  modeConfig = true;
  preferences.begin("IO", false);
  preferences.clear();
  preferences.end();
  ESP.restart();
}

// Fonction pour sauvegarder les préférences dans la mémoire EEPROM
void savePreference(String ssid, String password, String ip, int delay)
{
  preferences.begin("IO", false);
  preferences.putString("ip", ip);
  preferences.putString("ssid", ssid);
  preferences.putString("password", password);
  preferences.putInt("delay", delay);
  preferences.end();
}