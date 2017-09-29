#include <Arduino.h>

#include <SimpleDHT.h>
#include <ESP8266WiFi.h>
#include "ThingSpeak.h"

#define MEASUREMENT_TIME_SECONDS 30
#define MEASUREMENT_INTERVAL_MINUTES 15

#define VOUT1 5 
#define VOUT2 4

struct system_globals_t {
  const char* ssid = "ssid";
  const char* password = "password";
  unsigned int v1_low_time;
  unsigned int v1_high_time;
  unsigned int v1_start_time;
  unsigned int v2_low_time;
  unsigned int v2_high_time;
  unsigned int v2_start_time;
  WiFiClient  client;
  unsigned long myChannelNumber = CHANNEL_NUMBER;
  const char * myWriteAPIKey = "API_KEY";
  SimpleDHT11 dht11;
  int pinDHT11 = 0;
} global;


void setLED(uint8_t led){
    digitalWrite(BUILTIN_LED, led);
}

void measure_dht(byte * temperature, byte * humidity)
{
  *temperature = 0;
  *humidity = 0;
  if (global.dht11.read(global.pinDHT11, temperature, humidity, NULL)) {
    Serial.print("Read DHT11 failed.");
    return;
  }
  Serial.print((int)*temperature); Serial.print(" *C, "); 
  Serial.print((int)*humidity); Serial.println(" %"); 
}



void start_air_measurement(void)
{
  setLED(LOW);
  global.v1_low_time = 0;
  global.v1_high_time = 0;
  global.v1_start_time = millis();
  global.v2_low_time = 0;
  global.v2_high_time = 0;
  global.v2_start_time = millis();
  
  Serial.println("Waiting for DSM501 to be in high state on V1 and V2");
  while(HIGH != digitalRead(VOUT1) || HIGH != digitalRead(VOUT2));
  Serial.println("Air measurement started");
  attachInterrupt(VOUT1, v1_lowInterrupt, FALLING);
  attachInterrupt(VOUT2, v2_lowInterrupt, FALLING);
}

void stop_air_measurement(void)
{
  float v1_low_ratio, v1_particle, v1_ppl;
  float v2_low_ratio, v2_particle, v2_ppl;
  byte temperature, humidity;
  
  detachInterrupt(VOUT1);
  detachInterrupt(VOUT2);
  if(LOW == digitalRead(VOUT1))
  {
    global.v1_low_time += (millis() - global.v1_start_time);
  }
  else
  {
    global.v1_high_time += (millis() - global.v1_start_time);
  }
  if(LOW == digitalRead(VOUT2))
  {
    global.v2_low_time += (millis() - global.v2_start_time);
  }
  else
  {
    global.v2_high_time += (millis() - global.v2_start_time);
  }
  
  setLED(HIGH);
  v1_low_ratio = ((float)100*global.v1_low_time)/(global.v1_high_time + global.v1_low_time);
  v1_particle = (v1_low_ratio * 14000) / 23;
  v1_ppl = 3.5336 * v1_particle;
  v2_low_ratio = ((float)100*global.v2_low_time)/(global.v2_high_time + global.v2_low_time);
  v2_particle = (v2_low_ratio * 14000) / 23;
  v2_ppl = 3.5336 * v2_particle;
  
  Serial.println("Air measurement stopped");

  Serial.println("High time for pcs>2.5um [ms]");
  Serial.println(global.v1_high_time);
  Serial.println("Low time for pcs>2.5um [ms]");
  Serial.println(global.v1_low_time);
  Serial.println("Low ratio for pcs>2.5um [percent]");
  Serial.println(v1_low_ratio);
  Serial.println("Particles for pcs>2.5um [pcs/283ml]");
  Serial.println(v1_particle);
  Serial.println("Particles for pcs>2.5um [pcs/1000ml]");
  Serial.println(v1_ppl);

  Serial.println("High time for pcs>1um [ms]");
  Serial.println(global.v2_high_time);
  Serial.println("Low time for pcs>1um [ms]");
  Serial.println(global.v2_low_time);
  Serial.println("Low ratio for pcs>1um [percent]");
  Serial.println(v2_low_ratio);
  Serial.println("Particles for pcs>1um [pcs/283ml]");
  Serial.println(v2_particle);
  Serial.println("Particles for pcs>1um [pcs/1000ml]");
  Serial.println(v2_ppl);

  measure_dht(&temperature, &humidity);
  
  ThingSpeak.setField(1, v1_low_ratio);
  ThingSpeak.setField(2, v1_particle);
  ThingSpeak.setField(3, v1_ppl);
  ThingSpeak.setField(4, v2_low_ratio);
  ThingSpeak.setField(5, v2_particle);
  ThingSpeak.setField(6, v2_ppl);
  ThingSpeak.setField(7, temperature);
  ThingSpeak.setField(8, humidity);
  ThingSpeak.writeFields(global.myChannelNumber, global.myWriteAPIKey); 
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  pinMode(VOUT1, INPUT);
  pinMode(VOUT2, INPUT);
  pinMode(BUILTIN_LED, OUTPUT);
  
  delay(10);
  // Connect to WiFi network
  Serial.println();
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(global.ssid);
  WiFi.begin(global.ssid, global.password);
 
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");
 
  // Start the server
  ThingSpeak.begin(global.client); 
}

void v1_highInterrupt(){
    
    detachInterrupt(VOUT1);
    attachInterrupt(VOUT1, v1_lowInterrupt, FALLING);
    global.v1_low_time += (millis() - global.v1_start_time);
    global.v1_start_time = (millis());
}

void v1_lowInterrupt(){   
    detachInterrupt(VOUT1);
    attachInterrupt(VOUT1,  v1_highInterrupt, RISING);
    global.v1_high_time += (millis() - global.v1_start_time);
    global.v1_start_time = (millis());
}

void v2_highInterrupt(){
    
    detachInterrupt(VOUT2);
    attachInterrupt(VOUT2, v2_lowInterrupt, FALLING);
    global.v2_low_time += (millis() - global.v2_start_time);
    global.v2_start_time = (millis());
}

void v2_lowInterrupt(){   
    detachInterrupt(VOUT2);
    attachInterrupt(VOUT2,  v2_highInterrupt, RISING);
    global.v2_high_time += (millis() - global.v2_start_time);
    global.v2_start_time = (millis());
}

void loop() {
  start_air_measurement();
  delay(MEASUREMENT_TIME_SECONDS*1000);
  stop_air_measurement();
  delay(MEASUREMENT_INTERVAL_MINUTES*60000);
}
