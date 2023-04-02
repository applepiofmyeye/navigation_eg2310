#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
//pubsub for esp32
#include <PubSubClient.h>
#include <WiFi.h>
//Servo motor
#include <ESP32Servo.h>

//Crreate servo object
Servo servo;
int releaseCount = 0;

#define SERVOPIN 13

// set up the wifi 
const char* ssid = "joey (2)";
const char* password = "12345678";

// MQTT info
const char* mqtt_server = "172.20.10.6"; // RPI IP address

// Initialise the WiFi and MQTT Client objects
WiFiClient wifiClient;

// 1883 is the listener port for the Broker  
PubSubClient client(mqtt_server, 1883, wifiClient);

int homeAway = 0;

//callback function to listen for any message
void callback(char* topic, byte* message, unsigned int length) {
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  String msg;
  
  for (int i = 0; i < length; i++) {
    Serial.print((char)message[i]);
    msg += (char)message[i];
  }
  Serial.println();

  // Feel free to add more if statements to control more GPIOs with MQTT

  // If a message is received on the topic esp32/output, you check if the message is either "on" or "off". 
  // Changes the output state according to the message
  if (String(topic) == "dock") {
    if(msg == "Home"){
      Serial.println("Home");
      homeAway = 1;
      releaseCount += 1;
    }
    else if(msg == "Away"){
      Serial.println("Away");
      homeAway = 0;
    }
  }
}

// Custom function to connect to the MQTT broker via WiFi
void connect_MQTT(){
  Serial.print("Connecting to ");
  Serial.println(ssid);

  // Connect to the WiFi
  WiFi.begin(ssid, password);

  // Wait until the connection is confirmed
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
 // Debugging – Output the IP Address of the ESP8266
  Serial.println("WiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  // Connect to MQTT Broker
  if (client.connect(mqtt_server)) {
    Serial.println("Connected to MQTT Broker!");
  }
  else {
    Serial.println("Connection to MQTT Broker failed…");
  }
}

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

#define ANALOGPIN 33
int tableNumber;

void setup(){
  Serial.begin(9600);

  connect_MQTT();
  client.setCallback(callback);

  servo.attach(SERVOPIN);

  // Setup OLED screen
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3D for 128x64
    Serial.println(F("SSD1306 allocation failed"));
    for(;;);
  }
  delay(2000);
  display.clearDisplay();
}

void loop() {
  /*
  // to publish to MQTT broker with the topic 'topic'
  //if (client.publish("esp32/cmd_vel", "hello")) {}
  // client.publish will return whether it was successful or not. 
  // can connect again if it isnt successful.
  
  else {
    Serial.println("Failed to send. Reconnecting to MQTT server..");
    client.connect(mqtt_server);
    delay(10);
    client.publish("esp32/cmd_vel", "hello");
  }*/
 
  client.subscribe("dock");
  client.loop(); //actively listen for msg from rpi
  
  
  // Read in analog value
  int value = analogRead(ANALOGPIN);

  // Determine table number
  tableNumber = findTable(value);

  //Display information on OLED screen
  if (homeAway) {
    displayInformation(tableNumber);
    String msgPublish = String(tableNumber);
    client.publish("esp32/cmd_vel", msgPublish.c_str());
    if (releaseCount) releaseCan();
    
  }
  
}


void releaseCan() {
  servo.write(180);
  delay(3000); //delay 3 seconds
  servo.write(0);
  releaseCount -= 1;
}


// Compare value with threshold
int findTable(int value) {
  if (value > 3300 && value < 3500) return 4;
  else if (value > 3100 && value < 3300 ) return 5;
  else if (value > 2850 && value < 3100) return 6;
  else if (value > 2500 && value < 2800) return 3;
  else if (value > 1900 && value < 2500) return 2;
  else if (value < 500) return 1;
  else return 0; // Nothing pressed
}


// OLED screen
void displayInformation(int tableNumber) {
  display.clearDisplay();
  
  if (!tableNumber){
    display.setTextSize(2);
    display.setTextColor(WHITE);
    display.setCursor(0, 0);
    // Display static text
    display.println("TurtleBot");
    display.setCursor(0, 20);
    display.println("Dispenser");
    display.setCursor(0, 40);
    display.println("Group 8");
    display.display(); 
  }
  else {
    display.setTextSize(2);
    display.setTextColor(WHITE);
    display.setCursor(0, 0);
    // Display static text
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
