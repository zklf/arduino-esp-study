#include <ESP8266WiFi.h>
#include <SimpleDHT.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>
#include <U8g2lib.h> // 添加U8g2库

U8G2_SSD1306_128X64_NONAME_F_SW_I2C u8g2(U8G2_R0, /* clock=*/ D5, /* data=*/ D6, /* reset=*/ U8X8_PIN_NONE); // 设置屏幕

Adafruit_BMP280 bmp; // I2C

#define TCP_SERVER_ADDR "bemfa.com"
//服务器端口//TCP创客云端口8344//TCP设备云端口8340
#define TCP_SERVER_PORT "8344"

///****************需要修改的地方*****************///

//WIFI名称，区分大小写，不要写错
#define DEFAULT_STASSID  "PDCN"
//WIFI密码
#define DEFAULT_STAPSW "1234567"
//用户私钥，可在控制台获取,修改为自己的UID
String UID = "7b1ff9ddb6f6";
//主题名字，可在控制台新建
String TOPIC = "temp"; //用于传输温湿度的主题
//DHT11引脚值
int pinDHT11 = D4;  //连接dht11的引脚
//单片机LED引脚值
const int LED_Pin = D2;  //假设连接led的引脚
//主题名字，可在控制台新建
String TOPIC2  = "light002";  //用于led控制的主题

///*********************************************///
//led 控制函数
void turnOnLed();
void turnOffLed();
//led状态状态
String my_led_status = "off";



//设置上传速率2s（1s<=upDataTime<=60s）
//下面的2代表上传间隔是2秒
#define upDataTime 5*1000


// for DHT11, 
//      VCC: 5V or 3V
//      GND: GND
//      DATA: 2

SimpleDHT11 dht11(pinDHT11);





//最大字节数
#define MAX_PACKETSIZE 512





//tcp客户端相关初始化，默认即可
WiFiClient TCPclient;
String TcpClient_Buff = "";
unsigned int TcpClient_BuffIndex = 0;
unsigned long TcpClient_preTick = 0;
unsigned long preHeartTick = 0;//心跳
unsigned long preTCPStartTick = 0;//连接
bool preTCPConnected = false;



//相关函数初始化
//连接WIFI
void doWiFiTick();
void startSTA();

//TCP初始化连接
void doTCPClientTick();
void startTCPClient();
void sendtoTCPServer(String p);





/*
  *发送数据到TCP服务器
 */
void sendtoTCPServer(String p){
  
  if (!TCPclient.connected()) 
  {
    Serial.println("Client is not readly");
    return;
  }
  TCPclient.print(p);
  Serial.println("[Send to TCPServer]:String");
  Serial.println(p);
}


/*
  *初始化和服务器建立连接
*/

void startTCPClient(){

  if(TCPclient.connect(TCP_SERVER_ADDR, atoi(TCP_SERVER_PORT))){
    Serial.print("\nConnected to server:");
    Serial.printf("%s:%d\r\n",TCP_SERVER_ADDR,atoi(TCP_SERVER_PORT));
    String tcpTemp="";
    tcpTemp = "cmd=1&uid="+UID+"&topic="+TOPIC2+"\r\n";

    sendtoTCPServer(tcpTemp);
    preTCPConnected = true;
    preHeartTick = millis();
    TCPclient.setNoDelay(true);
  }
  else{
    Serial.print("Failed connected to server:");
    Serial.println(TCP_SERVER_ADDR);
    TCPclient.stop();
    preTCPConnected = false;
  }
  preTCPStartTick = millis();
}



/*
  *检查数据，发送数据
*/
void doTCPClientTick(){
 //检查是否断开，断开后重连
   if(WiFi.status() != WL_CONNECTED) return;

  if (!TCPclient.connected()) {//断开重连

  if(preTCPConnected == true){

    preTCPConnected = false;
    preTCPStartTick = millis();
    Serial.println();
    Serial.println("TCP Client disconnected.");
    TCPclient.stop();
  }
  else if(millis() - preTCPStartTick > 1*1000)//重新连接
    startTCPClient();
  }
  else
  {
    if (TCPclient.available()) {//收数据
      char c =TCPclient.read();
      TcpClient_Buff +=c;
      TcpClient_BuffIndex++;
      TcpClient_preTick = millis();
      
      if(TcpClient_BuffIndex>=MAX_PACKETSIZE - 1){
        TcpClient_BuffIndex = MAX_PACKETSIZE-2;
        TcpClient_preTick = TcpClient_preTick - 200;
      }
      preHeartTick = millis();
    }
    if(millis() - preHeartTick >= upDataTime){//上传数据
      preHeartTick = millis();

      /*****************获取DHT11 温湿度*****************/
      // read without samples.
      byte temperature = 0;
      byte humidity = 0;
      int err = SimpleDHTErrSuccess;
      if ((err = dht11.read(&temperature, &humidity, NULL)) != SimpleDHTErrSuccess) {
        Serial.print("Read DHT11 failed, err="); Serial.println(err);delay(1000);
        return;
      }
      float temperature_bmp = bmp.readTemperature(); // Celsius
      float pressure_bmp = bmp.readPressure() / 100.0F; // hPa
      String temperature_bmp_str = String(temperature_bmp, 2); // 保留2位小数
      String pressure_bmp_str = String(pressure_bmp, 2); // 保留2位小数




      /*********************数据上传*******************/
      /*
        数据用#号包裹，以便app分割出来数据，&msg=#23#80#on#\r\n，即#温度#湿度#按钮状态#，app端会根据#号分割字符串进行取值，以便显示
        如果上传的数据不止温湿度，可在#号后面继续添加&msg=#23#80#data1#data2#data3#data4#\r\n,app字符串分割的时候，要根据上传的数据进行分割
      */
      String upstr = "";
      upstr = "cmd=2&uid="+UID+"&topic="+TOPIC+"&msg=#"+temperature+"#"+humidity+"#"+temperature_bmp+"#"+pressure_bmp+"#\r\n";
      sendtoTCPServer(upstr);
      upstr = "";
    }
  }
  if((TcpClient_Buff.length() >= 1) && (millis() - TcpClient_preTick>=200))
  {//data ready
    TCPclient.flush();
    Serial.println("Buff");
    Serial.println(TcpClient_Buff);
    //////字符串匹配，检测发了的字符串TcpClient_Buff里面是否包含&msg=on，如果有，则打开开关
    if((TcpClient_Buff.indexOf("&msg=on") > 0)) {
      turnOnLed();
    //////字符串匹配，检测发了的字符串TcpClient_Buff里面是否包含&msg=off，如果有，则关闭开关
    }else if((TcpClient_Buff.indexOf("&msg=off") > 0)) {
      turnOffLed();
    }
   TcpClient_Buff="";//清空字符串，以便下次接收
   TcpClient_BuffIndex = 0;
  }
}


//打开灯泡
void turnOnLed(){
  Serial.println("Turn ON");
  digitalWrite(LED_Pin,LOW);
  my_led_status = "on";
}
//关闭灯泡
void turnOffLed(){
  Serial.println("Turn OFF");
  digitalWrite(LED_Pin,HIGH);
  my_led_status = "off";
}


void startSTA(){
  WiFi.disconnect();
  WiFi.mode(WIFI_STA);
  WiFi.begin(DEFAULT_STASSID, DEFAULT_STAPSW);
}



/**************************************************************************
                                 WIFI
***************************************************************************/
/*
  WiFiTick
  检查是否需要初始化WiFi
  检查WiFi是否连接上，若连接成功启动TCP Client
  控制指示灯
*/
void doWiFiTick(){
  static bool startSTAFlag = false;
  static bool taskStarted = false;
  static uint32_t lastWiFiCheckTick = 0;

  if (!startSTAFlag) {
    startSTAFlag = true;
    startSTA();
    Serial.printf("Heap size:%d\r\n", ESP.getFreeHeap());
  }

  //未连接1s重连
  if ( WiFi.status() != WL_CONNECTED ) {
    if (millis() - lastWiFiCheckTick > 1000) {
      lastWiFiCheckTick = millis();
    }
  }
  //连接成功建立
  else {
    if (taskStarted == false) {
      taskStarted = true;
      Serial.print("\r\nGet IP Address: ");
      Serial.println(WiFi.localIP());
      startTCPClient();
    }
  }
}



// ...（省略其他代码）

// 初始化，相当于main 函数
void setup() {
  Serial.begin(115200);

  Wire.begin(D1, D2); //D1对应I2C总线的SDA，D2对应I2C总线的SCL
  if (!bmp.begin(0x76)) {  // 0x76是BMP280的I2C地址
    Serial.println("Could not find a valid BMP280 sensor, check wiring!");
    while (1);
  }

  // 初始化屏幕
  u8g2.begin();
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_ncenB08_tr);

  //初始化引脚为输出
  pinMode(LED_Pin,OUTPUT);
}

// 显示数据
void displayData(byte temperature, byte humidity, float temperature_bmp, float pressure_bmp) {
  u8g2.clearBuffer();
  u8g2.setCursor(0, 12);
  u8g2.print("DHT11 Temp: ");
  u8g2.print(temperature);
  u8g2.print(" C");
  u8g2.setCursor(0, 24);
  u8g2.print("DHT11 Humidity: ");
  u8g2.print(humidity);
  u8g2.print(" %");
  u8g2.setCursor(0, 36);
  u8g2.print("BMP280 Temp: ");
  u8g2.print(temperature_bmp, 2);
  u8g2.print(" C");
  u8g2.setCursor(0, 48);
  u8g2.print("Pressure: ");
  u8g2.print(pressure_bmp, 2);
  u8g2.print(" hPa");
  u8g2.sendBuffer();
}

//循环
void loop() {
  doWiFiTick();
  doTCPClientTick();

  // 读取数据并更新屏幕显示
  byte temperature = 0;
  byte humidity = 0;
  int err = SimpleDHTErrSuccess;
  if ((err = dht11.read(&temperature, &humidity, NULL)) != SimpleDHTErrSuccess) {
    Serial.print("."); Serial.println(err);delay(1000);
    return;
  }
  float temperature_bmp = bmp.readTemperature(); // Celsius
  float pressure_bmp = bmp.readPressure() / 100.0F; // hPa
  displayData(temperature, humidity, temperature_bmp, pressure_bmp);
}
