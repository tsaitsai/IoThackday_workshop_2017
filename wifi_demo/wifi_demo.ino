/*
 Basic ESP8266 MQTT example

 This sketch demonstrates the capabilities of the pubsub library in combination
 with the ESP8266 board/library.


 To install the ESP8266 board, (using Arduino 1.6.4+):
  - Add the following 3rd party board manager under "File -> Preferences -> Additional Boards Manager URLs":
       http://arduino.esp8266.com/stable/package_esp8266com_index.json
  - Open the "Tools -> Board -> Board Manager" and click install for the ESP8266"
  - Select your ESP8266 in "Tools -> Board"
  
  Remember to change your ssid and password to match your network!
  Remember to change the MQTT broker IP address to match your Raspberry Pi!

*/

#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <Servo.h>
#include "SparkFunTMP102.h" // Used to send and recieve specific information from our sensor

// Update these with values suitable for your network.

const char* ssid = "netgear324";
const char* password = "somuchiot";
const char* mqtt_server = "192.168.2.24";

//////// FOR ESP8266 NODEMCU
// Connections
// VCC = 3.3V
// GND = GND
// SDA = A4  for EsP8266, D2 = GPIO 4
// SCL = A5  for esp8266, D1 = GPIO 5
// Note: this is using the wire.h library in Arduino core.


TMP102 sensor0(0x48); // Initialize sensor at I2C address 0x48


WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;
char msg[50];
char msg2[50];
int value = 0;

/*
static const uint8_t D0   = 16;
static const uint8_t D1   = 5;
static const uint8_t D2   = 4;
static const uint8_t D3   = 0;
static const uint8_t D4   = 2;
static const uint8_t D5   = 14;
static const uint8_t D6   = 12;
static const uint8_t D7   = 13;
static const uint8_t D8   = 15;
static const uint8_t D9   = 3;
static const uint8_t D10 = 1;
static const uint8_t SDA = 4;
static const uint8_t SCL = 5;
static const uint8_t LED_BUILTIN = 16;
static const uint8_t BUILTIN_LED = 16;
 */



//ultrasonic
//works, but used for temp sensor
//int trigPin = 5;    //Trig, pin 5=D1 on ESP8266
//int echoPin = 4;    //Echo, pin 4=D2 on ESP8266  does not work with D0/D4 either.
int trigPin = D6;    //Trig, GPIO 12=D6 on ESP8266
int echoPin = D7;    //Echo, GPIO 13=D7 on ESP8266


//servo pin
int servoPin = 15; // attach servo to pin D8, which is GPIO15 on NodeMCU

//ultrasonic
long duration, cm, inches;
long lastUltra;


//servo
Servo myservo;  // create servo object to control a servo
int servo_pos = 0;    // variable to store the servo position
const int servo_min = 10;
const int servo_max = 170;
long servo_time = 0;
int servo_target = 0;


//callback
long manual_time = 0;
bool flag_manual = 0;


void setup_wifi() {

  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  randomSeed(micros());

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address is is: ");
  Serial.println(WiFi.localIP());
}

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();

  flag_manual = 0;
  manual_time = millis();
  Serial.print("manual mode:");
  Serial.println(flag_manual);
  // Switch on the LED if an 1 was received as first character
  if ((char)payload[0] == '1') {
    digitalWrite(BUILTIN_LED, HIGH);   // Turn the LED on (Note that LOW is the voltage level
    // but actually the LED is on; this is because
    // it is acive low on the ESP-01)
    servo_target = servo_max;
  } else {
    digitalWrite(BUILTIN_LED, LOW);  // Turn the LED off by making the voltage HIGH
    servo_target = servo_min;
  }

}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "ESP8266Client-";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    //if (client.connect(clientId.c_str())) {
    if (client.connect("memq")) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      client.publish("outTopic", "hello world");
      // ... and resubscribe
      client.subscribe("inTopic");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void setup() {
  pinMode(BUILTIN_LED, OUTPUT);     // Initialize the BUILTIN_LED pin as an output
  Serial.begin(115200);
  sensor0.begin();  // Join I2C bus
  setup_wifi();
  // Initialize sensor0 settings
  // These settings are saved in the sensor, even if it loses power
  
  // set the number of consecutive faults before triggering alarm.
  // 0-3: 0:1 fault, 1:2 faults, 2:4 faults, 3:6 faults.
  sensor0.setFault(0);  // Trigger alarm immediately
  
  // set the polarity of the Alarm. (0:Active LOW, 1:Active HIGH).
  sensor0.setAlertPolarity(1); // Active HIGH
  
  // set the sensor in Comparator Mode (0) or Interrupt Mode (1).
  sensor0.setAlertMode(0); // Comparator Mode.
  
  // set the Conversion Rate (how quickly the sensor gets a new reading)
  //0-3: 0:0.25Hz, 1:1Hz, 2:4Hz, 3:8Hz
  sensor0.setConversionRate(2);
  
  //set Extended Mode.
  //0:12-bit Temperature(-55C to +128C) 1:13-bit Temperature(-55C to +150C)
  sensor0.setExtendedMode(0);

  //set T_HIGH, the upper limit to trigger the alert on
  sensor0.setHighTempF(85.0);  // set T_HIGH in F
  //sensor0.setHighTempC(29.4); // set T_HIGH in C
  
  //set T_LOW, the lower limit to shut turn off the alert
  sensor0.setLowTempF(84.0);  // set T_LOW in F
  //sensor0.setLowTempC(26.67); // set T_LOW in C
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);

  //ultrasonic
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  //servo
  myservo.attach(servoPin);  // attach servo to pin 2, which is D4 on NodeMCU
}

void loop() {

  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  long now = millis();

  float temperature;
  
  if (now - lastMsg > 2000) {
    lastMsg = now;
    ++value;

  }

  //ultrasonic
  if (now - lastUltra > 2000)
  {
    lastUltra = now;
    
    //ultrasonic
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    // Read the signal from the sensor: a HIGH pulse whose
    // duration is the time (in microseconds) from the sending
    // of the ping to the reception of its echo off of an object.
    //pinMode(echoPin, INPUT);
    duration = pulseIn(echoPin, HIGH);
    // convert the time into a distance
    cm = (duration/2) / 29.1;
    inches = (duration/2) / 74; 
    Serial.print(inches);
    Serial.print("in, ");
    Serial.print(cm);
    Serial.print("cm");
    Serial.println();
    //delay(250);

    //target servo
    //if (!flag_manual)
    if (0)
    {
      servo_target = (int)((inches * servo_max)/15);
      Serial.println("set target by ultrasonic");
    }

    
    //MQTT publish
    snprintf (msg, 50, "%ld", inches);
    Serial.print("Publish this distance: ");
    Serial.println(msg);
    client.publish("/wifi/A020A61225CE/distance", msg);
    //snprintf (msg, 75, "%ld", servo_target);
    //client.publish("target", msg);


    // Turn sensor on to start temperature measurement.
    // Current consumtion typically ~10uA.
    sensor0.wakeup();
  
    // read temperature data
    temperature = sensor0.readTempF();
    //temperature = sensor0.readTempC();
    //sensor0.sleep();
    Serial.print("Temperature: ");
    Serial.print(temperature);

    //char msg2[50];
    int tempINT=(int)temperature;
    Serial.print("Temp Int is: ");
    Serial.println(tempINT);
    snprintf (msg2, 50, "%d", tempINT);
    Serial.print("Publish this temperature: ");
    Serial.println(msg2);
    client.publish("/wifi/A020A61225CE/temperature", msg2);
  }//if lastUltra
  
  
  
  

  //servo control
  if (now - servo_time > 150)
  {
    servo_time = now;
    
    //slowly move towards target
    if (servo_pos < servo_target-9)     // 30 < 30 - 9
    {
      servo_pos = servo_pos + 10;
      
    }
    else if (servo_pos > servo_target+9) // 20 > 15+9
    {
      servo_pos = servo_pos - 10;
    }

    //check max/min
    if (servo_pos > servo_max)
    {
      servo_pos = servo_max;
    }
    else if (servo_pos < servo_min)
    {
      servo_pos = servo_min;
    }
    
    myservo.write(servo_pos);
  }//end if servo_time


  //if in manual for more than 5 seconds, turn off manual
  if ((flag_manual) && (now - manual_time > 20000))
  {
    Serial.println("manual mode off");
    flag_manual = 0; 
  }

  
} //end loop
  



