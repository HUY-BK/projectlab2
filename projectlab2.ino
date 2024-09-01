//code lăng phun
#include <Arduino.h>
#include <WiFi.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <PubSubClient.h>
 #include <SPI.h>

#define bienTro1 16
#define bienTro2 23

#define switch 22

unsigned long lastTime = 0, startTime = 0;

const char *ssid = "Tom Dong Bich";
const char *password = "hi09082015";

#define MQTT_SERVER "broker.hivemq.com"
#define MQTT_PORT 1883

#define MQTT_TOPIC1 "devID\valve"
#define MQTT_TOPIC2 "devID\mode"

WiFiClient wifiClient;
PubSubClient client(wifiClient);
Adafruit_
void setWiFi()
{
  Serial.print("Connecting");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.println("...");
  }
}
void setBNO055()
{
  while (!Serial)
  {
    delay(10);
  }
  Serial.println("Khoi tao BNO055");
  if (!bno.begin())
  {
    Serial.println("ERROR");
    while(1);
  }
  else
  {
    Serial.println("Khoi tao thanh cong");
  }
}

void connect_to_broker()
{
  while ((!client.connected()))
  {
    Serial.println("MQTT connection");
    String clientId = "esp32";
    clientId += String(random(0xffff), HEX);
    if (client.connect(clientId.c_str()))
    {
      Serial.println("NQTT Connected");
      client.subscribe(MQTT_TOPIC1);
      client.subscribe(MQTT_TOPIC2);
    }
    else
    {
      Serial.println("FAIED");
      delay(2000);
    }
  }
}
int giatri_bienTro1 = 0;
int giatri_bienTro2 = 0;
void read_bienTro1()
{
  giatri_bienTro1 = map(analogRead(bienTro1), 0, 4095, 0, 100);
  
}
void read_bienTro2()
{
  giatri_bienTro2 = map(analogRead(bienTro2), 0, 4095, 0, 20);
  
}
double tx, ty, tz, gx, gy, gz;
void readBNO055()
{
  tx=-1000000,ty=-1000000,tz=-1000000, gx=-1000000,gy=-1000000,gz=-1000000;
  sensors_event_t accelerometerData, orientationData;
  bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  tx =accelerometerData.acceleration.x;
  ty =accelerometerData.acceleration.y;
  tz =accelerometerData.acceleration.z;
  gx = orientationData.orientation.x;
  gy =orientationData.orientation.y;
  gz = orientationData.orientation.z;

}
void setup()
{
  Serial.begin(9600);
  setWiFi();
  client.setServer(MQTT_SERVER, MQTT_PORT);
  connect_to_broker();
  setBNO055();
  pinMode(bienTro1, INPUT);
  pinMode(bienTro2, INPUT);
  pinMode(switch, INPUT_PULLUP);
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_22, 0);
}
void sendMQTTMessage1(int f, unsigned long t){
  String message = "{\"f\":" + String(f) + ", \"t\":"String(t) + "}";
  client.publish(MQTT_TOPIC1, message.c_str()));
}
void sendMQTTMessage2(int f){
  String message = "{\"f\":" + String(f) + "}";
  client.publish(MQTT_TOPIC1, message.c_str()));
}

int count1 = 0;
int count2 = 0;
void loop()
{
 if (!client.connected()) {
    connect_to_broker();
  }
  client.loop();
  unsigned long currentTime = millis();

  if (digitalRead(nutAn) == 0)
  {
    readBNO055();
    if(currentTime - lastTime >= 0.1){

    read_bienTro1();
    read_bienTro2();
    lastTime = currentTime();
    }
    if ((giatri_bienTro1 == 0) && (count1 == 0)) 
    {
      client.publish(MQTT_TOPIC1, "Van Đóng");
      count1 = 1;
    }
    if ((giatri_bienTro2 == 0) && (count2 == 0)) 
    {
      client.publish(MQTT_TOPIC2, "Vòi Đóng");
      count1 = 1;
    }
    if (giatri_bienTro1 != 0)
    {
      if (count1 == 1)
      {
        startTime = millis();
      }
      if(currentTime - lastTime >= 0.5){
        sendMQTTMessage1(giatri_bienTro1,currentTime - startTime );
      }
      count = 0;
    }
    if (giatri_bienTro2 != 0){
      if(count2 == 1){
      
        count2 = 0;
      }
      sendMQTTMessage2(giatri_bienTro2);

    }
    
  }
  else
  {
    esp_deep_sleep_start();
  }
}