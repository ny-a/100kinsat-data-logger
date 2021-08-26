#pragma once

#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>

#define LOG_KIND 4

class WebController {
  public:
    WebController(const char * ssid, const char * password);

    void handleClient();
    void setValue(int index, char * value);

  private:
    WebServer server{80};
    const char * host = "esp32";
    String logs[LOG_KIND];
};

WebController::WebController(const char * ssid, const char * password) {
  WiFi.softAP(ssid, password);
  MDNS.begin(host);

  server.on("/", [&]() {
    String buffer;
    int sec = millis() / 1000;
    int min = sec / 60;
    int hr = min / 60;

    buffer = "<html>\
  <head>\
    <title>ESP32 Demo</title>\
    <style>\
      body { background-color: #cccccc; font-family: Arial, Helvetica, Sans-Serif; Color: #000088; }\
    </style>\
    <script>\
      setInterval(function() {\
        fetch('/log')\
          .then(res => res.text())\
          .then(text => document.getElementById('log').innerHTML = text)\
      }, 250);\
    </script>\
  </head>\
  <body>\
    <h1>Hello from ESP32!</h1>\
    <p>Uptime: ";
    buffer += String(hr);
    buffer += ":";
    buffer += String(min % 60);
    buffer += ":";
    buffer += String(sec % 60);
    buffer += "\
    </p>\
    <pre id='log'></pre>\
  </body>\
</html>";
    server.send(200, "text/html", buffer);
  });

  server.on("/log", [&]() {
    String buffer = "";

    for (int i = 0; i < LOG_KIND; i++) {
      buffer += logs[i];
      buffer += "\n";
    }
    server.send(200, "text/plain", buffer);
  });

  server.onNotFound([&]() {
    String message = "File Not Found\n\n";
    message += "URI: ";
    message += server.uri();
    message += "\nMethod: ";
    message += (server.method() == HTTP_GET) ? "GET" : "POST";
    message += "\nArguments: ";
    message += server.args();
    message += "\n";

    for (uint8_t i = 0; i < server.args(); i++) {
      message += " " + server.argName(i) + ": " + server.arg(i) + "\n";
    }

    server.send(404, "text/plain", message);
  });
  server.begin();
}

void WebController::handleClient() {
  server.handleClient();
}

void WebController::setValue(int index, char * value) {
  logs[index] = String(value);
}
