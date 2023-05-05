#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266WebServer.h>
#include <SimpleDHT.h>

// Replace with your network credentials
const char* ssid = "xxxx";
const char* password = "123456";

// Create an instance of the server
ESP8266WebServer server(80);

// Create an instance of the BMP280 sensor
Adafruit_BMP280 bmp280;

// For DHT11, 
//      VCC: 5V or 3V
//      GND: GND
//      DATA: 2
int pinDHT11 = 2;
SimpleDHT11 dht11(pinDHT11);

void setup() {
  Serial.begin(115200);
  Wire.begin(D2, D1);
  bmp280.begin(0x76);

  // Connect to Wi-Fi network
  WiFi.begin(ssid, password);
  Serial.println("");
  
  // Wait for connection
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }

  Serial.println("Connected to WiFi");

  // Print the IP address
  Serial.println(WiFi.localIP());

  // Start the server
  server.on("/", handleRoot);

  server.begin();
  Serial.println("HTTP server started");
}

void loop() {
  server.handleClient();
}

void handleRoot() {
  float temperature = bmp280.readTemperature();
  float pressure = bmp280.readPressure() / 100.0F;
  
  byte dht11_temperature = 0;
  byte dht11_humidity = 0;
  int dht11_err = SimpleDHTErrSuccess;
  if ((dht11_err = dht11.read(&dht11_temperature, &dht11_humidity, NULL)) != SimpleDHTErrSuccess) {
    Serial.print("Read DHT11 failed, err="); 
    Serial.print(SimpleDHTErrCode(dht11_err));
    Serial.print(","); 
    Serial.println(SimpleDHTErrDuration(dht11_err)); 
    delay(1000);
    return;
  }

  String html = "<html><head><title>BMP280 and DHT11 Sensor Data</title></head><body>";
  html += "<h1>BMP280 and DHT11 Sensor Data</h1>";
  html += "<p>BMP280 Temperature: " + String(temperature) + " &#8451;</p>";
  html += "<p>BMP280 Pressure: " + String(pressure) + " hPa</p>";
  html += "<p>DHT11 Temperature: " + String((int)dht11_temperature) + " &#8451;</p>";
  html += "<p>DHT11 Humidity: " + String((int)dht11_humidity) + " %</p>";
  html += "</body></html>";
  
  server.send(200, "text/html", html);
}
