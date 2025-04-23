#include "EmonLib.h"  
#include "ZMPT101B.h"
#include "WiFi.h"
#include "HTTPClient.h"
#include "Client.h"
#include "BH1750.h"
#include "Wire.h"
#include "DHT.h"

const char* ssid = "4G-MIFI-11D";
const char* pass = "1234567890";
const char* host = "null";
#define indikatorwifi 2
#define relaypin1 4
#define relaypin2 5
#define relaypin3 18
#define SENSITIVITY 267.250f
#define DHTPIN 13 
#define DHTTYPE DHT11 
DHT dht(DHTPIN, DHTTYPE);
EnergyMonitor emon1; 
EnergyMonitor emon2;
EnergyMonitor emon3; 
ZMPT101B voltageSensor(34, 50.0);
BH1750 lightMeter;
#define ANALOG_IN_PIN 36
float adc_voltage = 0.0;
float in_voltage = 0.0;
float R1 = 120000.0; // Can change the value of resistor 
float R2 = 10000.0;
float ref_voltage = 3.76;
int adc_value = 0;
const int NUM_SAMPLES = 100;
float vRef = 5.3;
float mVpA = 100.0;
float mVmA = 0.100;
float vZero = 0;
int counter = 0;
unsigned long previousMillis = 0;
const long interval = 5000;


float getVolt(){
  int raw;
  float v=0;
  float vBuff;
  for(int i=0; i<200; i++){
    raw = analogRead(39);
    vBuff = vRef * (raw / 4095.0);
    v = 0.9 * v + vBuff/10.0;
    delay(5);
  }
  return v;
}

void setup() 
{   
  Serial.begin(9600); 

  pinMode(indikatorwifi, OUTPUT);
  WiFi.hostname("ESP32");
  WiFi.begin(ssid, pass);
  while(WiFi.status() != WL_CONNECTED){
    digitalWrite(indikatorwifi, LOW);
    Serial.println("belum konek cuy");
    delay(500);
  }
  digitalWrite(indikatorwifi, HIGH);
  Serial.println("Terkoneksi");

  pinMode(relaypin1, OUTPUT);
  pinMode(relaypin2, OUTPUT);
  pinMode(relaypin3, OUTPUT);
  digitalWrite(relaypin1, HIGH);
  digitalWrite(relaypin2, HIGH);
  digitalWrite(relaypin3, HIGH);

  vZero = getVolt();

  emon1.current(35, 1.2); 
  emon2.current(32, 1.2);
  emon3.current(33, 1.2);

  voltageSensor.setSensitivity(SENSITIVITY);

  Wire.begin();
  lightMeter.begin();

  dht.begin();

  delay(1000);
}

void loop(){

  unsigned long currentMillis = millis();
  // AC Current 
  double Irms1 = emon1.calcIrms(1480);  
  double Irms2 = emon2.calcIrms(1480);  
  double Irms3 = emon3.calcIrms(1480);  
  Serial.println("Arus (A): ");
  Serial.println("Irms1 =" + String(Irms1/1000, 3) + "A");  
  Serial.println("Irms2 =" + String(Irms2/1000, 3) + "A");  
  Serial.println("Irms3 =" + String(Irms3/1000, 3) + "A");  
  
  if (currentMillis - previousMillis >= interval){
    float total_adc_voltage = 0;
    float lux = lightMeter.readLightLevel();  
    float temperature = dht.readTemperature();  
    float humidity = dht.readHumidity(); 
    float average_adc_voltage = total_adc_voltage / NUM_SAMPLES;
    in_voltage = average_adc_voltage / (R2/(R1+R2));
    float vDelta = getVolt() - vZero;
    float current_mA = abs(vDelta/mVmA);
    float daya_dc = in_voltage * current_mA;
    float maxVolt = 232.0;
    float voltage = voltageSensor.getRmsVoltage();
    float daya_beban1 = voltage * Irms1;
    float daya_beban2 = voltage * Irms2;
    float daya_beban3 = voltage * Irms3;

    for (int i = 0; i < NUM_SAMPLES; i++){
      adc_value = analogRead(ANALOG_IN_PIN);
      total_adc_voltage += adc_voltage;
      delay(1);
    }

    if (voltage > maxVolt){
      voltage = maxVolt;
    }

    if (daya_beban1 = 1){
      daya_beban1 = 0;
      
    } 

    if (daya_beban2 = 1){
      daya_beban2 = 0;
    }

    if(daya_beban3 = 1){
      daya_beban3 = 0;
    }  
  
  if (isnan(humidity) || isnan(temperature)) { 
    Serial.println(F("Gagal membaca sensor DHT11!"));  
    return;
  }

  Serial.print("Tegangan DC = ");
  Serial.println(in_voltage, 2);
  Serial.print("Arus DC = ");
  Serial.println(current_mA);
  Serial.print("Daya DC = ");
  Serial.println(daya_dc);
  Serial.print("Tegangan AC = ");
  Serial.println(voltage);
  Serial.print("Arus AC 1 = ");
  Serial.println(Irms1, 3);
  Serial.print("Arus AC 2 = ");
  Serial.println(Irms2, 3);
  Serial.print("Arus AC 3 = ");
  Serial.println(Irms3, 3);
  Serial.print("Cahaya = ");
  Serial.println(lux);
  Serial.print("Suhu = ");
  Serial.println(temperature);
  Serial.print("Kelembaban = ");
  Serial.println(humidity);
  Serial.print("Daya Beban 1 = ");
  Serial.println(daya_beban1);
  Serial.print("Daya Beban 2 = ");
  Serial.println(daya_beban2);
  Serial.print("Daya Beban 3 = ");
  Serial.println(daya_beban3);

  WiFiClient client;
  const int httpport = 80;
  if (!client.connect(host, httpport)) {
    Serial.println("Failed to connect to server");
    return;
  }

  String link;
  HTTPClient http;
  link = "http://" + String(host) + "/kirimdata.php?tdc=" + String(in_voltage) + "&adc=" + String(current_mA) + "&ddc=" + String(daya_dc) + "&tac=" + String(voltage) + "&aac1=" + String(Irms1) + "&aac2=" + String(Irms2) + "&aac3=" + String(Irms3) + "&dayabeban1=" + String(daya_beban1) + "&dayabeban2=" + String(daya_beban2) + "&dayabeban3=" + String(daya_beban3) + "&chy=" + String(lux) + "&sh=" + String(temperature) + "&klmb=" + String(humidity);

  http.begin(link);
  http.GET();
  http.end();
  delay(1000);
  }
}





// Calibration Sensitivity For ZMPT101B 
/**
 * This program shows how we can get (estimate) the right sensitivity value for
 * the sensor.
 * 
 * This program will swipe from the lowest sensitivity estimate value to the
 * highest. The program will stop if the voltage reading result from the sensor
 * is within the specified tolerance limit or the sensitivity value has exceeded
 * the highest predetermined value (which in this case is considered failed to
 * be determined).
*/

// #include <ZMPT101B.h>

// #define ACTUAL_VOLTAGE 210.0f // Change this based on actual voltage

// #define START_VALUE 0.0f
// #define STOP_VALUE 1000.0f
// #define STEP_VALUE 0.25f
// #define TOLLERANCE 1.0f

// #define MAX_TOLLERANCE_VOLTAGE (ACTUAL_VOLTAGE + TOLLERANCE)
// #define MIN_TOLLERANCE_VOLTAGE (ACTUAL_VOLTAGE - TOLLERANCE)

// // ZMPT101B sensor output connected to analog pin A0
// // and the voltage source frequency is 50 Hz.
// ZMPT101B voltageSensor(34, 50.0);

// void setup() {
//   Serial.begin(9600);
//   Serial.print("The Actual Voltage: ");
//   Serial.println(ACTUAL_VOLTAGE);

//   float senstivityValue = START_VALUE;
//   voltageSensor.setSensitivity(senstivityValue);
//   float voltageNow = voltageSensor.getRmsVoltage();

//   Serial.println("Start calculate");

//   while (voltageNow > MAX_TOLLERANCE_VOLTAGE || voltageNow < MIN_TOLLERANCE_VOLTAGE) {
//     if (senstivityValue < STOP_VALUE) {
//       senstivityValue += STEP_VALUE;
//       voltageSensor.setSensitivity(senstivityValue);
//       voltageNow = voltageSensor.getRmsVoltage();
//       Serial.print(senstivityValue);
//       Serial.print(" => ");
//       Serial.println(voltageNow);
//     } else {
//       Serial.println("Unfortunately the sensitivity value cannot be determined");
//       return;
//     }
//   }

//   Serial.print("Closest voltage within tolerance: ");
//   Serial.println(voltageNow);
//   Serial.print("Sensitivity Value: ");
//   Serial.println(senstivityValue, 10);
// }

// void loop() {}