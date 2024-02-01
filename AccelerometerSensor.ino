#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_MPU6050.h>
#include <DHTesp.h>
#include <LiquidCrystal_I2C.h>
#include <ESP32Servo.h>   //Library LCD I2C

// Define the pin for the DHT22 sensor
const int DHT_PIN = 22;
const int servoPin = 13;
int buka= 180;
int tutup= 0;
Servo servo1;
LiquidCrystal_I2C lcd(0x27,20,4);
// Define DHT sensor
DHTesp dht;

// Define MPU6050 sensor
Adafruit_MPU6050 mpu;

void setup() {
  Serial.begin(115200);
  servo1.attach(servoPin);

  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Status: ");
  lcd.clear();
  // Initialize DHT sensor
  dht.setup(DHT_PIN, DHTesp::DHT22);

  // Initialize MPU6050 sensor
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
}

void loop() {
  // Read temperature and humidity data from DHT sensor
  TempAndHumidity data = dht.getTempAndHumidity();
  float temperature = data.temperature;
  float humidity = data.humidity;
  // Read accelerometer data from MPU6050 sensor
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  float accelX = a.acceleration.x / (10); 
  float accelY = a.acceleration.y / (10);
  float accelZ = a.acceleration.z / (10);
  float rotationX = g.gyro.x / (10);
  float rotationY = g.gyro.y / (10);
  float rotationZ = g.gyro.z / (10);
  
  if(temperature <= 10){
    servo1.write(buka);
  }
    else if(temperature >=10){
    servo1.write(tutup);
  }


  // Print data to LCD16x2 i2c Monitor
  lcd.setCursor(0, 0);
  lcd.print("T:");
  lcd.print(temperature);
  lcd.print(" C");
  lcd.setCursor(11, 0);
  lcd.print("H: ");
  lcd.print(humidity);
  lcd.print("%");

  lcd.setCursor(0, 1);
  lcd.print("AccelerationX: ");
  lcd.print(accelX);
  lcd.setCursor(0, 2);
  lcd.print("AccelerationY: ");
  lcd.print(accelY);
  lcd.setCursor(0, 3);
  lcd.print("AccelerationZ: ");
  lcd.print(accelZ);

  // Print data to Serial Monitor
  Serial.print(" Temperature: ");
  Serial.print(temperature);
  Serial.print(" °C, Humidity: ");
  Serial.print(humidity);
  Serial.println(" %");
  Serial.print(" Accelerometer: X=");
  Serial.print(accelX);
  Serial.print(", Y=");
  Serial.print(accelY);
  Serial.print(", Z=");
  Serial.println(accelZ);
  Serial.print(" Rotation: X=");
  Serial.print(rotationX);
  Serial.print(", Y=");
  Serial.print(rotationY);
  Serial.print(", Z=");
  Serial.println(rotationZ);
  
  delay(1000); // delay 1s
}