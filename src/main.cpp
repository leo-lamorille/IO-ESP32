
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
int delayTime;
bool networkSSID_received = false;
bool networkPassword_received = false;
bool ipServer_received = false;

String formattedDate;
String dayStamp;
String timeStamp;

AsyncTimer t;
unsigned short setIntervalId;
unsigned short setIntervalIdWifi;

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

void printLocalTime();
void sendMQTTData(float humidity, float temperature, int airQuality);
void callback(char *topic, byte *payload, unsigned int length);
void clearPreferences();
void connectToWifi(String SSID, String password);
void collectData();
void savePreference(String ssid, String password, String ip, int delay);

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
  {
    modeConfig = false;
    connectToWifi(networkSSID, networkPassword);
    if (WiFi.status() == WL_CONNECTED)
    {
      timeClient.begin();
      timeClient.setTimeOffset(3600);
      Serial.println("WiFi connected");
      Serial.println("IP address: ");
      Serial.println(WiFi.localIP());
      dht.begin();
      pinMode(35, INPUT);
      client.setServer(ipServer.c_str(), 1883);
      client.connect("ESP32Client");
      client.setCallback(callback);
      String topic = "sensor/params/" + String(WiFi.macAddress());
      Serial.println(topic);
      client.subscribe(topic.c_str());
      setIntervalId = t.setInterval(collectData, delayTime);
    }
    else
    {
      clearPreferences();
    }
  }
  else
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
}

void loop()
{
  if (modeConfig)
  {
    dnsServer.processNextRequest();
    if (ipServer_received && networkSSID_received && networkPassword_received)
    {
      preferences.begin("IO", false);
      preferences.putString("ip", ipServer);
      preferences.putString("ssid", networkSSID);
      preferences.putString("password", networkPassword);
      preferences.putInt("delay", delayTime);
      preferences.end();

      ipServer_received = false;
      networkSSID_received = false;
      networkPassword_received = false;
      ESP.restart();
    }
  }
  else
  {

    delay(3000);
    timeClient.update();
    if (WiFi.status() == WL_CONNECTED)
    {
      if (!client.connected())
      {
        client.connect("ESP32Client");
        client.setCallback(callback);
        String topic = "sensor/params/" + String(WiFi.macAddress());
        Serial.println(topic.c_str());
        client.subscribe(topic.c_str());
      }
      else
      {
        client.loop();
        if (!modeSender)
        {
          String jsonString = "{\"MacAddress\":\"" + String(WiFi.macAddress()) + "\",\"ip\":\"" + String(WiFi.localIP()) + "\",\"delay\": " + delayTime + "}";
          client.publish("sensor/register", jsonString.c_str());
        }
        else
        {
          collectData();
        }
      }
    }
    else
    {
      connectToWifi(networkSSID, networkPassword);
      if (WiFi.status() != WL_CONNECTED)
      {
        clearPreferences();
      }
    }
  }
}

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

void callback(char *topic, byte *payload, unsigned int length)
{
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  DynamicJsonDocument doc(1024);
  deserializeJson(doc, payload);
  delayTime = doc["delay"];
  t.changeDelay(setIntervalId, delayTime);
  savePreference(networkSSID, networkPassword, ipServer, delayTime);
  modeSender = true;
}

void collectData()
{
  int humidity = dht.readHumidity();
  float temperature = dht.readTemperature();
  float airQuality = analogRead(35);
  sendMQTTData(humidity, temperature, airQuality);
}

void sendMQTTData(float humidity, float temperature, int airQuality)
{
  Serial.println("send data");
  String jsonString = "{\"Humidity\":" + String(humidity) + ",\"Temperature\":" + String(temperature) + ",\"AirQuality\":" + String(airQuality);
  jsonString += ",\"MacAddress\":\"" + String(WiFi.macAddress()) + "\",\"Time\":" + String(timeClient.getEpochTime()) + "}";
  client.publish("sensor/data", jsonString.c_str());
}

void clearPreferences()
{
  modeConfig = true;
  preferences.begin("IO", false);
  preferences.clear();
  preferences.end();
  ESP.restart();
}

void savePreference(String ssid, String password, String ip, int delay)
{
  preferences.begin("IO", false);
  preferences.putString("ip", ip);
  preferences.putString("ssid", ssid);
  preferences.putString("password", password);
  preferences.putInt("delay", delay);
  preferences.end();
}