#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <PubSubClient.h>
#include <WiFi.h>
#include <ESP32Servo.h>

#define SERVOPIN 13
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define ANALOGPIN 33

// constants
Servo servo;
int releaseCount = 0;
int tableNumber = 0;

// mqtt constants
const char *ssid = "joey (2)";
const char *password = "12345678";
const char *mqtt_server = "172.20.10.6";

// set up WifiClient for mqtt server
WiFiClient wifiClient;
PubSubClient client(mqtt_server, 1883, wifiClient);

// initialise display
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

void callback(char *topic, byte *message, unsigned int length)
{
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  String msg;
  for (int i = 0; i < length; i++)
  {
    Serial.print((char)message[i]);
    msg += (char)message[i];
  }
  Serial.println();
  if (String(topic) == "dock")
  {
    if (msg == "Home")
    {
      releaseCount = 1;
      display.clearDisplay();
      display.setTextSize(2);
      display.setTextColor(WHITE);
      display.setCursor(0, 0);
      display.println("Home");
      display.display();
    }
  }
}

// connect to MQTT
void connect_MQTT()
{
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    displayWifiInformation(WiFi.status());
    Serial.print(".");
  }
  Serial.println("WiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  displayWifiInformation(WiFi.status());
  while (!client.connect(mqtt_server))
  {
    displayMQTTInformation(0);
    Serial.println("Connection to MQTT Broker failed…");
  }
  displayMQTTInformation(1);
  Serial.println("Connected to MQTT Broker!");
}

// sets up display and mqtt
void setup()
{
  Serial.begin(9600);
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C))
  {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;)
      ;
  }
  delay(2000);
  display.clearDisplay();
  connect_MQTT();
  client.setCallback(callback);
  client.subscribe("dock");
  servo.attach(SERVOPIN);
  servo.write(180);
}

void loop()
{
  client.loop();
  int value = analogRead(ANALOGPIN);
  if (findTable(value) != 0)
  {
    tableNumber = findTable(value);
  }
  displayInformation(tableNumber);
  Serial.println(tableNumber);
  if (releaseCount && tableNumber)
  {
    String msgPublish = String(tableNumber);
    client.publish("table", msgPublish.c_str());
    releaseCan();
    releaseCount = 0;
    tableNumber = 0;
  }
}

void releaseCan()
{
  servo.write(0);
  delay(3000);
  servo.write(180);
  delay(3000);
}

int findTable(int value)
{
  if (value > 3300 && value < 3500)
    return 4;
  else if (value > 3100 && value < 3300)
    return 5;
  else if (value > 2850 && value < 3100)
    return 6;
  else if (value > 2500 && value < 2800)
    return 3;
  else if (value > 1900 && value < 2500)
    return 2;
  else if (value < 500)
    return 1;
  else
    return 0;
}

void displayInformation(int tableNumber)
{
  display.clearDisplay();
  if (!tableNumber)
  {
    display.setTextSize(2);
    display.setTextColor(WHITE);
    display.setCursor(0, 0);
    display.println("TurtleBot");
    display.setCursor(0, 20);
    display.println("Dispenser");
    display.setCursor(0, 40);
    display.println("Group 8");
    display.display();
  }
  else
  {
    display.setTextSize(2);
    display.setTextColor(WHITE);
    display.setCursor(0, 0);
    display.println("Delivering");
    display.setCursor(0, 20);
    display.println("can to");
    display.setCursor(0, 40);
    String tableMessage = "table " + String(tableNumber);
    display.println(tableMessage);
    display.display();
    delay(1000);
  }
}

void displayWifiInformation(int connection)
{
  display.clearDisplay();
  if (connection == 3)
  {
    display.setTextSize(2);
    display.setTextColor(WHITE);
    display.setCursor(0, 0);
    display.println("Wifi");
    display.setCursor(0, 20);
    display.println("Connected");
    display.display();
  }
  else
  {
    display.setTextSize(2);
    display.setTextColor(WHITE);
    display.setCursor(0, 0);
    display.println("Wifi");
    display.setCursor(0, 20);
    display.println("Not");
    display.setCursor(0, 40);
    display.println("Connected");
    display.display();
  }
}

void displayMQTTInformation(bool connection)
{
  display.clearDisplay();
  if (connection)
  {
    display.setTextSize(2);
    display.setTextColor(WHITE);
    display.setCursor(0, 0);
    display.println("MQTT");
    display.setCursor(0, 20);
    display.println("Connected");
    display.display();
  }
  else
  {
    display.setTextSize(2);
    display.setTextColor(WHITE);
    display.setCursor(0, 0);
    display.println("MQTT");
    display.setCursor(0, 20);
    display.println("Not");
    display.setCursor(0, 40);
    display.println("Connected");
    display.display();
  }
}
