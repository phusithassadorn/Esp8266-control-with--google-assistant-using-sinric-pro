//#define ENABLE_DEBUG

#ifdef ENABLE_DEBUG
#define DEBUG_ESP_PORT Serial
#define NODEBUG_WEBSOCKETS
#define NDEBUG
#endif

#include <Arduino.h>
#ifdef ESP8266
#include <ESP8266WiFi.h>
#define DHT_PIN 12
#endif
#ifdef ESP32
#include <WiFi.h>
#define DHT_PIN 12
#endif

#include "DHT.h"
#include "SinricPro.h"
#include "SinricProSwitch.h"
#include "SinricProTemperaturesensor.h"
#include <BlynkSimpleEsp8266.h>


#define WIFI_SSID "wifiname"
#define WIFI_PASS "wifipassword"
#define APP_KEY "app key from sinric"
#define APP_SECRET "app secret from sinric"
#define SWITCH_ID "relay id"
#define TEMP_SENSOR_ID "dht22 id"
#define EVENT_WAIT_TIME 60000 // send event every 60 seconds
#define BAUD_RATE 9600
#define RELAY_PIN 14 // relay gpio pin

DHT dhtA(12, DHT22);
bool deviceIsOn;                              // Temeprature sensor on/off state
float temperature;                            // actual temperature
float humidity;                               // actual humidity
float lastTemperature;                        // last known temperature (for compare)
float lastHumidity;                           // last known humidity (for compare)
unsigned long lastEvent = (-EVENT_WAIT_TIME); // last time event has been sent


bool onPowerState(const String &deviceId, bool &state)
{
  digitalWrite(RELAY_PIN, state); // set pin state put ! in front of stat to invert relay 
  return true;                    // request handled properly
}
bool onPowerState1(const String &deviceId, bool &state)
{
  Serial.printf("Temperaturesensor turned %s (via SinricPro) \r\n", state ? "on" : "off");
  deviceIsOn = state; // turn on / off temperature sensor
  return true;        // request handled properly
}


void handleTemperaturesensor()
{
  if (deviceIsOn == false)
    return; // device is off...do nothing

  unsigned long actualMillis = millis();
  if (actualMillis - lastEvent < EVENT_WAIT_TIME)
    return; //only check every EVENT_WAIT_TIME milliseconds

  temperature = dhtA.readTemperature(); // get actual temperature in °C
                                        //  temperature = dht.getTemperature() * 1.8f + 32;  // get actual temperature in °F
  humidity = dhtA.readHumidity();       // get actual humidity

  if (isnan(temperature) || isnan(humidity))
  {                                           // reading failed...
    Serial.printf("DHT reading failed!\r\n"); // print error message
    return;                                   // try again next time
  }

  if (temperature == lastTemperature || humidity == lastHumidity)
    return; // if no values changed do nothing...

  SinricProTemperaturesensor &mySensor = SinricPro[TEMP_SENSOR_ID];    // get temperaturesensor device
  bool success = mySensor.sendTemperatureEvent(temperature, humidity); // send event
  if (success)
  { // if event was sent successfuly, print temperature and humidity to serial
    Serial.printf("Temperature: %2.1f Celsius\tHumidity: %2.1f%%\r\n", temperature, humidity);
  }
  else
  { // if sending event failed, print error message
    Serial.printf("Something went wrong...could not send Event to server!\r\n");
  }

  lastTemperature = temperature; // save actual temperature for next compare
  lastHumidity = humidity;       // save actual humidity for next compare
  lastEvent = actualMillis;      // save actual time for next compare
}

void setupWiFi()
{
  Serial.printf("\r\n[Wifi]: Connecting");
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.printf(".");
    delay(250);
  }
  IPAddress localIP = WiFi.localIP();
  Serial.printf("connected!\r\n[WiFi]: IP-Address is %d.%d.%d.%d\r\n", localIP[0], localIP[1], localIP[2], localIP[3]);
}

void setupSinricPro()
{
  // add device to SinricPro
  SinricProTemperaturesensor &mySensor = SinricPro[TEMP_SENSOR_ID];
  SinricProSwitch &mySwitch = SinricPro[SWITCH_ID];
  mySensor.onPowerState(onPowerState1);
  mySwitch.onPowerState(onPowerState);

  // setup SinricPro
  SinricPro.onConnected([]()
                        { Serial.printf("Connected to SinricPro\r\n"); });
  SinricPro.onDisconnected([]()
                           { Serial.printf("Disconnected from SinricPro\r\n"); });
  SinricPro.begin(APP_KEY, APP_SECRET);
  SinricPro.restoreDeviceStates(true); // get latest known deviceState from server (is device turned on?)
}

void climateRoutine() {
    byte h1 = dhtA.readHumidity();            // f1 and h1 are celsius and humidity readings
    // byte t1 = dhtA.readTemperature(true);  // for temperature in farenheits
    byte t1 = dhtA.readTemperature();         // from DHT/A
    Blynk.virtualWrite(V0, t1);               //  Set Virtual Pin 0 frequency to PUSH in Blynk app
    Blynk.virtualWrite(V1, h1);               //  Set Virtual Pin 1 frequency to PUSH in Blynk app
}

void setup()
{
  pinMode(RELAY_PIN, OUTPUT); // set relay-pin to output mode
  Serial.begin(BAUD_RATE);
  Serial.printf("\r\n\r\n");
  dhtA.begin();
  Blynk.begin(auth, WIFI_SSID, WIFI_PASS); 
  setupWiFi();
  setupSinricPro();
}

void loop()
{
  Blynk.run();
  SinricPro.handle(); // handle SinricPro commands
  handleTemperaturesensor();
  climateRoutine(); 
}
