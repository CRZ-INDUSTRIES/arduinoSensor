#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#include <Adafruit_BMP085.h>
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27,20,4);



unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled
unsigned long debounceDelay = 1000;    // the debounce time; increase if the output flickers 
//pins defined for the bmp180 A4 and A5 because they use i2c

#define DHTPIN 2     // Digital pin connected to the DHT sensor 

int pinInterrupt = 3; //Digital pin connected to the anemometer
float valWind;
float Count=0;

#define DHTTYPE DHT11
Adafruit_BMP085 bmp;
#include <ArduinoJson.h> // Library for create JSON objects

DHT_Unified dht(DHTPIN, DHTTYPE);

void onChange()
{
   if ( digitalRead(pinInterrupt) == LOW )
      Count++;
}


void setup() {
  Serial.begin(9600);
  
  pinMode( pinInterrupt, INPUT_PULLUP);// set the interrupt pin

  //Enable
  attachInterrupt( digitalPinToInterrupt(pinInterrupt), onChange, FALLING);

  // Initialize device.
  dht.begin();
  bmp.begin();
  sensor_t sensor;
  lcd.init();                      // initialize the lcd 
  lcd.init();
  // Print a message to the LCD.
  lcd.backlight();
 

}

void loop() {
  // Delay between measurements.
  // delay(60000);
  
  delay(1000);
  lcd.setCursor(0,0);
   lcd.print("Presion: ");
  lcd.setCursor(0,1);
  lcd.print(bmp.readPressure());
  lcd.setCursor(7,1);
  lcd.print("Pa");
  DynamicJsonDocument doc(1024);
  JsonArray array = doc.to<JsonArray>();


  // Get temperature event and print its value.
  sensors_event_t event;
  dht.temperature().getEvent(&event);
  if (isnan(event.temperature)) {
    Serial.println(F("Error reading temperature!"));
  }
  else {
    JsonObject temperature = doc.createNestedObject();
    
    temperature["sensorType"] = "temperature";
    temperature["sensorValue"] = event.temperature;
    temperature["sensorMeasurement"] = "celcius";

    // array.add(temperature);`
    // Serial.println(F("°C"));
  }

  // Get humidity event and print its value.
  dht.humidity().getEvent(&event);
  if (isnan(event.relative_humidity)) {
    Serial.println(F("Error reading humidity!"));
  }
  else {
    JsonObject humidity = doc.createNestedObject();

    humidity["sensorType"] = "humidity";
    humidity["sensorValue"] = event.relative_humidity;
    humidity["sensorMeasurement"] = "percent";

    // array.add(humidity);
    // Serial.print(event.relative_humidity);

  }

  // Get pressure event and print its value.
  int32_t pressureVal = bmp.readPressure();  // Leer la presión
  JsonObject pressure = doc.createNestedObject();
 
  pressure["sensorType"] = "pressure";
  pressure["sensorValue"] = pressureVal;
  pressure["sensorMeasurement"] = "Pa";

  //Get wind event and print its value.
  JsonObject windSpeed  = doc.createNestedObject();
  if ((millis() - lastDebounceTime) > debounceDelay) {
    lastDebounceTime = millis();
    
    windSpeed["sensorType"] = "Wind";
    
    windSpeed["sensorValue"] = Count*8.75/100;
    Count=0;
    windSpeed["sensorMeasurement"] = "m/s";
    
  }
  
    Serial.println();
   

  serializeJson(array, Serial);
  Serial.println();
}


