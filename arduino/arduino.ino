#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <coap-simple.h>
#include <DHT.h>


#define DHTPIN 4 //Connect Out pin to D2 in NODE MCU
#define DHTTYPE DHT11  
DHT dht(DHTPIN, DHTTYPE);

const char *ssid = "Casa Trianon"; // Enter your WiFi name
const char *password = "Trianon$2222"; // Enter WiFi password

// CoAP client response callback
void callback_response(CoapPacket &packet, IPAddress ip, int port);

double temperature = 0;
double humedity = 0;

WiFiUDP udp;
Coap coap(udp);

void readSensor()
{
    temperature = dht.readTemperature();
    humedity = dht.readHumidity();
    Serial.print("Temperature : ");
    Serial.print(temperature);
    Serial.print("    Humidity : ");
    Serial.println(humedity);
    if (isnan(humedity) || isnan(temperature)) {
      Serial.println("Failed to read from DHT sensor!");
    return;
  }
}


void callback_response(CoapPacket &packet, IPAddress ip, int port) {
  Serial.println("[Coap Response got]");
  
  char p[packet.payloadlen + 1];
  memcpy(p, packet.payload, packet.payloadlen);
  p[packet.payloadlen] = NULL;
  
  Serial.println(p);
}

void setup() {
  Serial.begin(115200);
  
  // connecting to a WiFi network
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println("Connecting to WiFi..");
  }
  Serial.println("Connected to the WiFi network");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  coap.response(callback_response);
  coap.start();
}


void loop() {
    delay(2000);
    // Read sensor and send message
    readSensor();
    String message = "{\"humidity\":" + String(humedity) + ",\"temperature\":" + String(temperature) + "}";
    int msgid = coap.put(IPAddress(10, 0, 2, 72), 5683, "alarm", message.c_str());
    coap.loop();
}