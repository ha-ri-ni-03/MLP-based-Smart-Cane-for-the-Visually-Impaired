# Arduino IDE code
#include <Adafruit_MPU6050.h>
#include <WiFi.h>
#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include <Firebase_ESP_Client.h>
#include "addons/TokenHelper.h"
#include "addons/RTDBHelper.h"
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#define TRIG_PIN 5   
#define ECHO_PIN 18  
#define BUZZER_PIN 4 
#define LED 19
#define VIBMOT 23
#define RXD2 16  
#define TXD2 17  
#define GPS_BAUD 9600
#define WIFI_SSID "Redmi Note 11"
#define WIFI_PASSWORD "12345678"
#define API_KEY "AIzaSyC1Y3Y59si3NSdXleEfzCFf9lgzvQnkWy0"
#define DATABASE_URL 
"https://caneconnect-a0ed2-default-rtdb.asia-southeast1.firebasedatabase.app/"

FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;

unsigned long sendDataPrevMillis=0;
bool signupOK=false;
float accX, accY, accZ, rotX, rotY, rotZ, temp;
double latitude, longitude;

Adafruit_MPU6050 mpu;

TinyGPSPlus gps;
HardwareSerial gpsSerial(1);  // UART1 (GPIO16, GPIO17)

void setup() {
  Serial.begin(115200);
  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, RXD2, TXD2);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(LED, OUTPUT);
  pinMode(VIBMOT, OUTPUT);

  WiFi.begin(WIFI_SSID,WIFI_PASSWORD);
  while(WiFi.status()!=WL_CONNECTED){
    Serial.print("."); delay(300);
  }
  Serial.println();
  Serial.println(WiFi.localIP());
  Serial.println();

  config.api_key=API_KEY;
  config.database_url=DATABASE_URL;
  
  if(Firebase.signUp(&config, &auth, "", "")){
      Serial.println("signUp OK");
      signupOK = true;
    }
  else{
    Serial.printf("%s\n", config.signer.signupError.message.c_str());
  }
  config.token_status_callback = tokenStatusCallback;
  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);

  while (!Serial)
    delay(10); 
  Serial.println("Adafruit MPU6050 test!");

  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (!mpu.begin()) {
      delay(5000);
      Serial.println("Trying...");
    }
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
  case MPU6050_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case MPU6050_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case MPU6050_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case MPU6050_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
  case MPU6050_RANGE_250_DEG:
    Serial.println("+- 250 deg/s");
    break;
  case MPU6050_RANGE_500_DEG:
    Serial.println("+- 500 deg/s");
    break;
  case MPU6050_RANGE_1000_DEG:
    Serial.println("+- 1000 deg/s");
    break;
  case MPU6050_RANGE_2000_DEG:
    Serial.println("+- 2000 deg/s");
    break;
  }

  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
  case MPU6050_BAND_260_HZ:
    Serial.println("260 Hz");
    break;
  case MPU6050_BAND_184_HZ:
    Serial.println("184 Hz");
    break;
  case MPU6050_BAND_94_HZ:
    Serial.println("94 Hz");
    break;
  case MPU6050_BAND_44_HZ:
    Serial.println("44 Hz");
    break;
  case MPU6050_BAND_21_HZ:
    Serial.println("21 Hz");
    break;
  case MPU6050_BAND_10_HZ:
    Serial.println("10 Hz");
    break;
  case MPU6050_BAND_5_HZ:
    Serial.println("5 Hz");
    break;
  }

  Serial.println("");
  delay(100);
}

void obstacle() {
    long duration;
    float distance_cm;
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
    duration = pulseIn(ECHO_PIN, HIGH);
    distance_cm = (duration * 0.0343) / 2;
    Serial.print("Distance: ");
    Serial.print(distance_cm);
    Serial.println(" cm");

    if (distance_cm < 40) { 
        Serial.println("Obstacle detected! Activating buzzer.");
        digitalWrite(BUZZER_PIN, HIGH); 
        digitalWrite(LED, HIGH);
        digitalWrite(VIBMOT, HIGH);
    } else {
        digitalWrite(BUZZER_PIN, LOW); 
        digitalWrite(LED, LOW);
        digitalWrite(VIBMOT, LOW);
    }
    delay(500); 
}

void sendGPSData() {
    while (gpsSerial.available()>0) {
        gps.encode(gpsSerial.read());
        if (gps.location.isUpdated()) {
            latitude = gps.location.lat();
            longitude = gps.location.lng();
        }
    }
}

void loop() {
  obstacle();
  delay(100);
  if(Firebase.ready() && signupOK && (millis() 
  - sendDataPrevMillis > 5000 || sendDataPrevMillis ==0)){
    sendDataPrevMillis = millis();
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    sendGPSData();
    
    accX = a.acceleration.x;
    accY = a.acceleration.y;
    accZ = a.acceleration.z;

    rotX = g.gyro.x;
    rotY = g.gyro.y;
    rotZ = g.gyro.z;

    if(Firebase.RTDB.setFloat(&fbdo, "Sensor/accX", accX)){
      Serial.println();
      Serial.print(accX);
      Serial.print(" - successfully saved to: " + fbdo.dataPath());
      Serial.println(" (" + fbdo.dataType() + ") ");
    }
    else{
      Serial.println("FAILED: " + fbdo.errorReason());
    }

    if(Firebase.RTDB.setFloat(&fbdo, "Sensor/accY", accY)){
      Serial.println();
      Serial.print(accY);
      Serial.print(" - successfully saved to: " + fbdo.dataPath());
      Serial.println(" (" + fbdo.dataType() + ") ");
    }
    else{
      Serial.println("FAILED: " + fbdo.errorReason());
    }

    if(Firebase.RTDB.setFloat(&fbdo, "Sensor/accZ", accZ)){
      Serial.println();
      Serial.print(accZ);
      Serial.print(" - successfully saved to: " + fbdo.dataPath());
      Serial.println(" (" + fbdo.dataType() + ") ");
    }
    else{
      Serial.println("FAILED: " + fbdo.errorReason());
    }

    if(Firebase.RTDB.setFloat(&fbdo, "Sensor/rotX", rotX)){
      Serial.println();
      Serial.print(rotX);
      Serial.print(" - successfully saved to: " + fbdo.dataPath());
      Serial.println(" (" + fbdo.dataType() + ") ");
    }
    else{
      Serial.println("FAILED: " + fbdo.errorReason());
    }

    if(Firebase.RTDB.setFloat(&fbdo, "Sensor/rotY", rotY)){
      Serial.println();
      Serial.print(rotY);
      Serial.print(" - successfully saved to: " + fbdo.dataPath());
      Serial.println(" (" + fbdo.dataType() + ") ");
    }
    else{
      Serial.println("FAILED: " + fbdo.errorReason());
    }

    if(Firebase.RTDB.setFloat(&fbdo, "Sensor/rotZ", rotZ)){
      Serial.println();
      Serial.print(rotZ);
      Serial.print(" - successfully saved to: " + fbdo.dataPath());
      Serial.println(" (" + fbdo.dataType() + ") ");
    }
    else{
      Serial.println("FAILED: " + fbdo.errorReason());
    }

    if(Firebase.RTDB.setFloat(&fbdo, "Location/Latitude", latitude)){
      Serial.println();
      Serial.print(latitude);
      Serial.print(" - successfully saved to: " + fbdo.dataPath());
      Serial.println(" (" + fbdo.dataType() + ") ");
    }
    else{
      Serial.println("FAILED: " + fbdo.errorReason());
    }

    if(Firebase.RTDB.setFloat(&fbdo, "Location/Longitude", longitude)){
      Serial.println();
      Serial.print(longitude);
      Serial.print(" - successfully saved to: " + fbdo.dataPath());
      Serial.println(" (" + fbdo.dataType() + ") ");
    }
    else{
      Serial.println("FAILED: " + fbdo.errorReason());
    }
  }

  Serial.print("Acceleration X: ");
  Serial.print(accX);
  Serial.print(", Y: ");
  Serial.print(accY);
  Serial.print(", Z: ");
  Serial.print(accZ);
  Serial.println(" m/s^2");

  Serial.print("Rotation X: ");
  Serial.print(rotX);
  Serial.print(", Y: ");
  Serial.print(rotY);
  Serial.print(", Z: ");
  Serial.print(rotZ);
  Serial.println(" rad/s");

  Serial.print("Location - Latitude: ");
  Serial.print(latitude, 6);
  Serial.print(", Longitude: ");
  Serial.print(longitude, 6);
  Serial.println(" deg");

  Serial.println("");
  delay(500);
}
