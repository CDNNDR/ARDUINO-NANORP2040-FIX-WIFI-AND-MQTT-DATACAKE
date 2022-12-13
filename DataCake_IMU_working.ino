
#include <SPI.h>
#include <WiFiNINA.h>
#include <Arduino_LSM6DSOX.h>
#include <PubSubClient.h>
#include <NTPClient.h>
#include <WiFiUdp.h>


float x;
float y;
float z;


const char* ssid = "";     
const char* pass = "";     

int status = WL_IDLE_STATUS;

WiFiSSLClient client;

// Libreries to get time for MQTT message
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org");

// Libreries for MQTT comunication
WiFiClient mqttWifiClient;
PubSubClient mqttClient(mqttWifiClient);


// Values for MQTT broker
const char *MQTT_HOST = "mqtt.datacake.co";
const int MQTT_PORT_INSECURE = 1883;
const int MQTT_PORT = 8883;
const char *MQTT_USER = "";
const char *MQTT_PASS = "";

#define mqtt_topic "dtck/test_imu/4de5e816-d506-49fa-b542-0c7ae500c44d/+"
#define mqtt_x_pub "dtck-pub/test_imu/4de5e816-d506-49fa-b542-0c7ae500c44d/X"
#define mqtt_y_pub "dtck-pub/test_imu/4de5e816-d506-49fa-b542-0c7ae500c44d/Y"
#define mqtt_z_pub  "dtck-pub/test_imu/4de5e816-d506-49fa-b542-0c7ae500c44d/Z"


void setup() {

 Serial.begin(9600);
WiFi.begin(ssid, pass);
  if (WiFi.status() == WL_CONNECTED) 
{
  Serial.print("connesso a: ");
  Serial.println(ssid);
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);
} else {
Serial.println("no internet found, resetting Nano in 5 seconds");
delay(5000);
digitalWrite( NINA_RESETN, LOW );  // Reset NINA module
delay(500);  // just making sure we get a clean pulse at the NINA_RESETN pin
NVIC_SystemReset();  // Reset RP2040
}
  
  mqttClient.setServer(MQTT_HOST, MQTT_PORT_INSECURE);
  while (!mqttClient.connected()) {
        Serial.print("MQTT connecting...");
        // MQTT Hostname should be empty for Datacake
        if (mqttClient.connect("", MQTT_USER, MQTT_PASS)) {
            Serial.println("connected");
            mqttClient.subscribe(mqtt_topic);
        } else {
            Serial.print("failed, status code =");
            Serial.print(mqttClient.state());
            Serial.println("try again in 5 seconds");
            digitalWrite( NINA_RESETN, LOW );  // Reset NINA module
            delay(500);  // just making sure we get a clean pulse at the NINA_RESETN pin
            NVIC_SystemReset();  // Reset RP2040
            // Wait 5 seconds before retrying
            //delay(5000);

        }
   }

IMU.begin();
 

}

void loop() {

  mqttClient.setServer(MQTT_HOST, MQTT_PORT_INSECURE);
  Serial.println("server setted");
 mqttClient.connect("", MQTT_USER, MQTT_PASS);
 Serial.println("connected");
  mqttClient.subscribe(mqtt_topic);
  Serial.println("subscribed");
  
if(!mqttClient.connected()){
  phoenix();
          
} 

        

          float xa;
          float ya;
          float za;
          
          IMU.readAcceleration(xa, ya, za);
          
          Serial.println(xa);
              //Serial.print('\t');
          Serial.println(ya);
              //Serial.print('\t');
          Serial.println(za);
          
          x = (float) xa*9.81;
          y = (float) ya*9.81;
          z = (float) za*9.81;


Serial.println(x);
Serial.println(y);
Serial.println(z);

sendMqttData(x, y, z);

delay(30000); 

phoenix();

}

void sendMqttData(float x, float y, float z)
{

  
//publish temperature
mqttClient.publish(mqtt_x_pub, String(x).c_str(), true);

//publish co2
mqttClient.publish(mqtt_y_pub, String(y).c_str(), true);

//publish humidity
mqttClient.publish(mqtt_z_pub, String(z).c_str(), true);

Serial.println("published all");

 }

 void phoenix(){

  digitalWrite( NINA_RESETN, LOW );  // Reset NINA module
            delay(500);  // just making sure we get a clean pulse at the NINA_RESETN pin
            NVIC_SystemReset();  // Reset RP2040
  
 }
