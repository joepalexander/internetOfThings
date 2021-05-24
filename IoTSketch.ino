

#define BLYNK_PRINT Serial    // Comment this out to disable prints and save space
#include <SPI.h>
#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>
#include <SimpleTimer.h>
#include <DHT.h>
#include <MQUnifiedsensor.h>
#include "AnotherIFTTTWebhook.h"


//Data for get connected to the Wi-Fi and Blynk
char auth[] = "bJInzW7qvPcRDDUAGxg3GTGK47k7MKs-"; //Enter the Auth code which was send by Blink
char* WiFi_hostname = "dya-like-jazz"; //device local hostname
char ssid[] = "Oldfield WIFI";  //Enter your WIFI Name
char pass[] = "woodsideso249sf";  //Enter your WIFI Password
bool tempExceedNotifSent = false; //boolean operator to stop the spam of IFTTT notications 


//Defining the GPIO pins and sensor types
#define DHTPIN D3          // Digital pin 4
#define LED_PIN D1        // Digital pin 1
#define pirPin D7
#define DHTTYPE DHT11     // DHT 11
DHT dht(DHTPIN, DHTTYPE);
SimpleTimer timer;
#define IFTTT_API_KEY "bKQTgM0euJ98RaFQKI8HEa"
#define IFTTT_EVENT "ESP_DHT"
//Declare the MQ135 Sensor
#define placa "esp8266"
#define Voltage_Resolution 5
#define pin A0 //Analog input 0 of your arduino
#define type "MQ-135" //MQ135
#define ADC_Bit_Resolution 10 // For arduino UNO/MEGA/NANO
#define RatioMQ135CleanAir 3.6//RS / R0 = 3.6 ppm  
MQUnifiedsensor MQ135(placa, Voltage_Resolution, ADC_Bit_Resolution, pin, type);
int tempThreshold = 24; // the threshold when surpassed, triggers the IFTTT alert and LED



BLYNK_WRITE(V1) // Reset on Blynk
{
if (param.asInt()==1) {
digitalWrite(LED_BUILTIN, HIGH);
delay(1000);
digitalWrite(LED_BUILTIN, LOW);
delay(1000);
digitalWrite(LED_BUILTIN, HIGH);
delay(1000);
digitalWrite(LED_BUILTIN, LOW);
delay(10);
ESP.restart();
delay(5000);
}
}




void sendSensor()
{
  Blynk.virtualWrite(V10, "IP: " + WiFi.localIP().toString()); //shows the IP in the Blynk app
  Blynk.virtualWrite(V11, WiFi.macAddress()); //Shows the MAC address in the BLynk app
  float h = dht.readHumidity();
  float t = dht.readTemperature(); // or dht.readTemperature(true) for Fahrenheit

  
  pinMode(LED_BUILTIN, OUTPUT); //defines the onboard LED pin as an output
  pinMode(LED_PIN, OUTPUT);
    pinMode(pirPin, INPUT);
    digitalWrite(pirPin, HIGH);
//    Serial.println(digitalRead(pirPin));
    Blynk.virtualWrite(V9, digitalRead(pirPin));


  
  if (isnan(h) || isnan(t)) {
    Serial.println("Failed to read from DHT sensor!");
    digitalWrite(LED_BUILTIN, HIGH);
    digitalWrite(LED_BUILTIN, LOW);
    digitalWrite(LED_BUILTIN, HIGH);
    digitalWrite(LED_PIN, HIGH);
    //flashes the builtin LED if the DHT11 cannot be read from 


    return;
  };

  if (digitalRead(pirPin) == 1){
    digitalWrite(LED_BUILTIN, LOW);
    Serial.println("Motion/Light"); //prints if motion has been detected (USED FOR DEBUGGING)
  }else{
    digitalWrite(LED_BUILTIN, HIGH);

  }


  // You can send any value at any time.
  // Please don't send more that 10 values per second.
  Blynk.virtualWrite(V5, h);  //V5 is for Humidity
  Blynk.virtualWrite(V6, t);  //V6 is for Temperature
  
  if (t > tempThreshold && !tempExceedNotifSent) { // send the IFTTT webhook if the threshold is met (This also converts the floating point to the strong such that it can be read by the IFTTT app)
    tempExceedNotifSent = true;
    float PRETEMP = dht.readTemperature();
    char str[35];
    dtostrf(PRETEMP, 0, 1, str);
    {
      send_webhook(IFTTT_EVENT, IFTTT_API_KEY, str, "ESP device", "Webhook");
    }

  }

  if (tempThreshold < (t)) { //blink the LED if the threshhold is met 
    digitalWrite(LED_PIN, HIGH);
    delay(1000);
    digitalWrite(LED_PIN, LOW);
    delay(1000);
    digitalWrite(LED_PIN, HIGH);
    delay(1000);
    digitalWrite(LED_PIN, LOW);
    delay(4000);

  } else {
    return;
  }
}

BLYNK_WRITE(V0) //manual calibration
{
  digitalWrite(LED_PIN, HIGH); //flash the LED if manual calibration takes plce 
  MQ135.setRegressionMethod(1); //_PPM =  a*ratio^b
  MQ135.init();
  Serial.print("Calibrating please wait.");
  float calcR0 = 0;
  for (int i = 1; i <= 10; i ++)
  {
    MQ135.update(); // Update data, the arduino will be read the voltage on the analog pin
    calcR0 += MQ135.calibrate(RatioMQ135CleanAir);
    Serial.print(".");
  }
  MQ135.setR0(calcR0 / 10);
  Serial.println("  done!.");

  if (isinf(calcR0)) {
    Serial.println("Warning: Conection issue founded, R0 is infite (Open circuit detected) please check your wiring and supply");
    while (1);
  }
  if (calcR0 == 0) {
    Serial.println("Warning: Conection issue founded, R0 is zero (Analog pin with short circuit to ground) please check your wiring and supply");
    while (1);
  }
  /*****************************  MQ CAlibration ********************************************/
  //  Serial.println("** Lectures from MQ-135 ****");
  //  Serial.println("|    CO   |  Alcohol |   CO2  |  Tolueno  |  NH4  |  Acteona  |");

  // Setup a function to be called every second
  timer.setInterval(1000L, sendSensor);
  digitalWrite(LED_PIN, LOW);

};
void setup()
{
  Serial.begin(9600); // See the connection status in Serial Monitor
  Blynk.begin(auth, ssid, pass, "blynk-cloud.com", 8080); //connects to the Blynk network

  dht.begin();
  MQ135.setRegressionMethod(1); //_PPM =  a*ratio^b
  MQ135.init();
  Serial.print("Calibrating please wait.");
  float calcR0 = 0;
  for (int i = 1; i <= 10; i ++)
  {
    MQ135.update(); // Update data, the arduino will be read the voltage on the analog pin
    calcR0 += MQ135.calibrate(RatioMQ135CleanAir);
    Serial.print(".");
  }
  MQ135.setR0(calcR0 / 10);
  Serial.println("  done!.");

  if (isinf(calcR0)) {
    Serial.println("Warning: Conection issue founded, R0 is infite (Open circuit detected) please check your wiring and supply"); //prints if error is found 
    while (1);
  }
  if (calcR0 == 0) {
    Serial.println("Warning: Conection issue founded, R0 is zero (Analog pin with short circuit to ground) please check your wiring and supply");
    while (1);
  }
  /*****************************  MQ CAlibration ********************************************/
  //  Serial.println("** Lectures from MQ-135 ****");
  //  Serial.println("|    CO   |  Alcohol |   CO2  |  Tolueno  |  NH4  |  Acteona  |");

  // Setup a function to be called every second
  timer.setInterval(1000L, sendSensor);
}

void loop()
{
  Blynk.run(); // Initiates Blynk
  timer.run(); // Initiates SimpleTimer
  MQ135.update(); // Update data, the arduino will be read the voltage on the analog pin

  MQ135.setA(605.18); MQ135.setB(-3.937); // Configurate the ecuation values to get CO concentration
  float CO = MQ135.readSensor(); // Sensor will read PPM concentration using the model and a and b values setted before or in the setup

  //  MQ135.setA(77.255); MQ135.setB(-3.18); // Configurate the ecuation values to get Alcohol concentration
  //  float Alcohol = MQ135.readSensor(); // Sensor will read PPM concentration using the model and a and b values setted before or in the setup

  MQ135.setA(110.47); MQ135.setB(-2.862); // Configurate the ecuation values to get CO2 concentration
  float CO2 = MQ135.readSensor(); // Sensor will read PPM concentration using the model and a and b values setted before or in the setup

  //  MQ135.setA(44.947); MQ135.setB(-3.445); // Configurate the ecuation values to get Tolueno concentration
  //  float Tolueno = MQ135.readSensor(); // Sensor will read PPM concentration using the model and a and b values setted before or in the setup

  //  MQ135.setA(102.2 ); MQ135.setB(-2.473); // Configurate the ecuation values to get NH4 concentration
  //  float NH4 = MQ135.readSensor(); // Sensor will read PPM concentration using the model and a and b values setted before or in the setup
  //
  //  MQ135.setA(34.668); MQ135.setB(-3.369); // Configurate the ecuation values to get Acetona concentration
  //  float Acetona = MQ135.readSensor(); // Sensor will read PPM concentration using the model and a and b values setted before or in the setup

  Blynk.virtualWrite(V7, CO2 + 400);
  Blynk.virtualWrite(V8, CO);

  //  Serial.print("|   "); Serial.print(CO);
  //  Serial.print("   |   "); Serial.print(Alcohol);
  // Note: 200 Offset for CO2 source: https://github.com/miguel5612/MQSensorsLib/issues/29
  /*
    Motivation:
    We have added 200 PPM because when the library is calibrated it assumes the current state of the
    air as 0 PPM, and it is considered today that the CO2 present in the atmosphere is around 400 PPM.
    https://www.lavanguardia.com/natural/20190514/462242832581/concentracion-dioxido-cabono-co2-atmosfera-bate-record-historia-humanidad.html
  */
  //  Serial.print("   |   "); Serial.print(CO2 + 400);
  //  Serial.print("   |   "); Serial.print(Tolueno);
  //  Serial.print("   |   "); Serial.print(NH4);
  //  Serial.print("   |   "); Serial.print(Acetona);
  //  Serial.println("   |");
  /*
    Exponential regression:
    GAS      | a      | b
    CO       | 605.18 | -3.937
    Alcohol  | 77.255 | -3.18
    CO2      | 110.47 | -2.862
    Tolueno  | 44.947 | -3.445
    NH4      | 102.2  | -2.473
    Acetona  | 34.668 | -3.369
  */

  delay(1500); //Sampling frequency
}
