

#include <Arduino.h>
#include <AsyncTCP.h>
#include "ESPAsyncWebServer.h"
#include "DNSServer.h"

const char *ssid = "LE GROUPE 4 EST LE MEILLEUR";
const char *password = "";

AsyncWebServer server(80);
DNSServer dnsServer;

void webServerSetup()
{

    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
              {
    request->send(200, "text/html", "<!DOCTYPE html><html><head><title>Success</title></head><body><p>Hooray</p></body>");
    Serial.println("requested /"); });

    server.on("/generate_204", HTTP_GET, [](AsyncWebServerRequest *request)
              {
    request->send(200, "text/plain", "You were sent here by a captive portal after requesting generate_204");
    Serial.println("requested /generate_204"); });

    server.onNotFound([](AsyncWebServerRequest *request)
                      {
    request->redirect("/");
    Serial.print("server.notfound triggered: ");
    Serial.println(request->url()); });

    server.begin();
    Serial.println("Web server started");
}

void setup()
{

    Serial.begin(115200);

    WiFi.softAP(ssid, password);
    Serial.println("Wifi initialized");

    Serial.println(WiFi.softAPIP());
    dnsServer.start(53, "*", WiFi.softAPIP());
    webServerSetup();
    Serial.println("Setup complete");
}

void loop()
{
    dnsServer.processNextRequest();
}