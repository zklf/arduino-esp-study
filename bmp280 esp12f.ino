#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>
#include <ESP8266WiFi.h>

#define BMP_SCK  (13)
#define BMP_MISO (12)
#define BMP_MOSI (11)
#define BMP_CS   (10)
#define BMP280_ADDRESS (0x76)

Adafruit_BMP280 bmp; // I2C
//Adafruit_BMP280 bmp(BMP_CS); // hardware SPI
//Adafruit_BMP280 bmp(BMP_CS, BMP_MOSI, BMP_MISO,  BMP_SCK);

WiFiServer server(80);
const char* ssid = "xxx";
const char* password = "xxxx";

void setup() {
  Serial.begin(9600);
  Serial.println(F("BMP280 test"));
//注意0x67 I2C地址
  if (!bmp.begin(0x76)) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
    while (1);
  }

  Serial.print("Connecting to WiFi hotspot");
  WiFi.begin(ssid,password);
  while(WiFi.status() != WL_CONNECTED){
    delay(500);
    Serial.print(".");
    }
    Serial.println("");
    Serial.println("WiFi connected");
   //启动服务器
   server.begin();
   Serial.println("Server started");
   //打印出服务器IP地址
   Serial.println(WiFi.localIP());


  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
}

void loop() {
    Serial.print(F("Temperature = "));
    Serial.print(bmp.readTemperature());
    Serial.println(" *C");

    Serial.print(F("Pressure = "));
    Serial.print(bmp.readPressure());
    Serial.println(" Pa");

    Serial.print(F("Approx altitude = "));
    Serial.print(bmp.readAltitude(1013.25)); /* Adjusted to local forecast! */
    Serial.println(" m");

    Serial.println();

    //服务器传送数据
    WiFiClient client = server.available();
    if(!client){
      return;
      }
    while(!client.available()){
      delay(1);
      }
    //准备响应
   // String s = "HTTP/1.1 200 OK\r\nContent-Type:text/html\r\n\r\n<!DOCTYPE HTML>\r\n<body><h1 style='margin:100 auto;'>Weather station</h1><p>Temperature="+String(bmp.readTemperature())+"c</p><p>Pressure="+String(bmp.readPressure())+"pa</p><p>Approx altitude="+String(bmp.readAltitude(1013.25))+"m</p></body></HTML>\n";
    String s = "HTTP/1.1 200 OK\r\nContent-Type:text/html\r\n\r\n<!doctypehtml><html lang=en><meta charset=UTF-8><title>Weather station</title><body style=background:#eee><div style='width:600px;border:1px solid #090;margin:100px auto;padding:10px;background:#690;border-radius:5px'><h1 style=text-align:center>气象站</h1><p>温度:<span style=color:#c00>"+String(bmp.readTemperature())+"℃</span><p>气压:<span style=color:#c00>"+String(bmp.readPressure())+"pa</span><p>海拨:<span style=color:#c00>"+String(bmp.readAltitude(1013.25))+"m</span></div><script>function reloadPage(){setTimeout(function(){window.location.reload()},5e3)}reloadPage()</script>";
    
    //将响应发送给客户端
    client.print(s);
    delay(1);
    Serial.println("Client disonnected");

    
}
