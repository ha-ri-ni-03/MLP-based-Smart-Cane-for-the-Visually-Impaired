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

# DataProcessing via Firebase DB
import firebase_admin
from firebase_admin import credentials, db
import csv
import time
from google.colab import drive

drive.mount('/content/drive', force_remount=True)
url = 'https://caneconnect-a0ed2-default-rtdb.asia-southeast1.firebasedatabase.app/'
if not firebase_admin._apps:
    path = /content/drive/MyDrive/CaneConnect/JSON file.json
    cred = credentials.Certificate(path)
    firebase_admin.initialize_app(cred, {'databaseURL': url })
print("Firebase initialized:", firebase_admin._apps)

def get_sensor_data():
    ref = db.reference('/Sensor')
    data = ref.get()
    print("Full Database Snapshot:", data)
    return data if isinstance(data, dict) else None

csv_file_path = "/content/drive/MyDrive/Fallevents_data.csv"

try:
    with open(csv_file_path, mode='x', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(["AccX", "AccY", "AccZ", "RotX", "RotY", "RotZ"])
except FileExistsError:
    pass

while True:
    data = get_sensor_data()
    print("Fetched data:", data)

    if data and isinstance(data, dict):
        try:
            with open(csv_file_path, mode='a', newline='') as file:
                writer = csv.writer(file)
                writer.writerow([
                    data.get("accX", "N/A"),
                    data.get("accY", "N/A"),
                    data.get("accZ", "N/A"),
                    data.get("rotX", "N/A"),
                    data.get("rotY", "N/A"),
                    data.get("rotZ", "N/A")
                ])
            print("Data saved to CSV:", data)
        except Exception as e:
            print("Error writing to CSV:", e)

    time.sleep(1)

# Magnitude Warping Data Augmentation technique to artificially increase dataset size
import pandas as pd
import numpy as np
from tsgm.models.augmentations import MagnitudeWarping

from google.colab import drive
drive.mount('/content/drive')

file_path = '/content/drive/MyDrive/Nonfallevents_data.csv'
df = pd.read_csv(file_path)

columns = ['AccX', 'AccY', 'AccZ', 'RotX', 'RotY', 'RotZ']
data = df[columns].values

n_timesteps = 100  # Adjust based on your data
n_samples = data.shape[0] 
data = data[:n_sample*n_timesteps].reshape((n_samples,n_timesteps,len(columns)))

aug_model = MagnitudeWarping()

n_augmented_samples = 100  
sigma = 0.2
augmented_data=aug_model.generate(X=data, n_samples=n_augmented_samples,sigma=sigma)
augmented_data=augmented_data.reshape(-1, len(columns))
augmented_df = pd.DataFrame(augmented_data, columns=columns)
expanded_df = pd.concat([df, augmented_df], ignore_index=True)
expanded_file_path = '/content/drive/MyDrive/Nonfall_MDT1_dataset.csv'
expanded_df.to_csv(expanded_file_path, index=False)

print(f"Augmentation complete. Expanded dataset saved to {expanded_file_path}")

# MLP training
from google.colab import files
uploaded = files.upload()
import pandas as pd
file_path = "MDT_fallDataset_MPU6050.csv"
df = pd.read_csv(file_path)
df.head()
df.info()
df.describe()
print(df.isnull().sum())
df.dropna(inplace=True)
df = pd.get_dummies(df, drop_first=True)  
X = df.drop(columns=['Prediction'])  
y = df['Prediction']  
from sklearn.preprocessing import StandardScaler
from sklearn.model_selection import train_test_split

X = df.drop(columns=['Prediction'])   
y = df['Prediction']

scaler = StandardScaler()
X_scaled = scaler.fit_transform(X)

X_train, X_test, y_train, y_test = 
train_test_split(X_scaled, y, test_size=0.2, random_state=42)

print(f"Training samples: {len(X_train)}, Testing samples: {len(X_test)}")

from sklearn.neural_network import MLPClassifier
from sklearn.metrics import accuracy_score, classification_report, 
mlp = MLPClassifier(hidden_layer_sizes=(100,), 
activation='relu', solver='adam', max_iter=200, random_state=42)
mlp.fit(X_train, y_train)
y_pred = mlp.predict(X_test)

accuracy = accuracy_score(y_test, y_pred)
print(f"Accuracy: {accuracy * 100:.2f}%")
print("Classification Report:")
print(classification_report(y_test, y_pred))
print("Confusion Matrix:")
print(confusion_matrix(y_test, y_pred))
import matplotlib.pyplot as plt
import seaborn as sns

plt.figure(figsize=(8, 5))
sns.countplot(x=df["Prediction"], palette=["blue", "red"])
plt.xticks([0, 1], ["Non-Fall (0)", "Fall (1)"])
plt.xlabel("Event Type")
plt.ylabel("Count")
plt.title("Distribution of Fall and Non-Fall Events")
plt.show()

plt.figure(figsize=(8, 5))
sns.scatterplot(x=df["AccX"],
y=df["AccY"], hue=df["Prediction"], palette=["blue", "red"], alpha=0.6)
plt.xlabel("AccX (Acceleration in X-axis)")
plt.ylabel("AccY (Acceleration in Y-axis)")
plt.title("Fall (1) vs Non-Fall (0) based on AccX and AccY")
plt.legend(title="Prediction", labels=["Non-Fall (0)", "Fall (1)"])
plt.show()

import seaborn as sns
import matplotlib.pyplot as plt
 Define sensor features
features = ["AccX", "AccY", "AccZ", "RotX", "RotY", "RotZ"]
 
plt.figure(figsize=(12, 8))
for i, feature in enumerate(features, 1):
    plt.subplot(2, 3, i)  
    sns.boxplot(x="Prediction", y=feature, data=df, palette=["red", "blue"])
    plt.title(f"{feature} vs Fall Events")

plt.tight_layout()
plt.show()
import seaborn as sns
import matplotlib.pyplot as plt
 Filter dataset for non-fall events
df_non_fall = df[df["Prediction"] == 0]

features = ["AccX", "AccY", "AccZ", "RotX", "RotY", "RotZ"]

plt.figure(figsize=(12, 8))
for i, feature in enumerate(features, 1):
    plt.subplot(2, 3, i)  # 2 rows, 3 columns
    sns.boxplot(y=df_non_fall[feature], color="red")
    plt.title(f"{feature} Distribution (Non-Fall Events)")
plt.tight_layout()
plt.show()
import os
print(os.listdir())

# Live Fall Prediction mechanism
import firebase_admin
from firebase_admin import credentials, db
import pandas as pd
import numpy as np
import pickle
import time
from threading import Timer

from google.colab import drive
drive.mount('/content/drive/')

if not firebase_admin._apps:
    cred = credentials.Certificate
    ('/content/drive/MyDrive/CaneConnect/JSON file.json')
    firebase_admin.initialize_app
    (cred, {'databaseURL':
    'https://caneconnect-a0ed2-default-rtdb.asia-southeast1.
    firebasedatabase.app/'})
    print("Firebase initialized:", firebase_admin._apps)
else:
  print("Firebase app is already initialized.")

print("Connected to Firebase. Model and scaler loaded successfully!")

with open("model.pkl", "rb") as f:
    model = pickle.load(f)
with open("scaler.pkl", "rb") as f:
    scaler = pickle.load(f)

def fetch_live_sensor_data():
    ref = db.reference("Sensor")
    sensor_data = ref.get()

if sensor_data:
 df_live = pd.DataFrame([sensor_data])
 print("\n Live Sensor Data Fetched:")
 print(df_live)

 features = ["accX", "accY", "accZ", "rotX", "rotY", "rotZ"]
 X_live = df_live[features].values
 X_live_scaled = scaler.transform(X_live)
 return X_live_scaled

 print(" No sensor data available in Firebase.")
 return None

def predict_fall(X_live_scaled):
    if X_live_scaled is not None:
        prediction = model.predict(X_live_scaled)
        print(f"\n Model Prediction: {prediction[0]}")
        return prediction
    return None

alert_sent=False
while True:
    X_live_scaled = fetch_live_sensor_data()
    fall_prediction = predict_fall(X_live_scaled)

if fall_prediction is not None:
  if fall_prediction[0] == 1:
    print("\nFALL DETECTED! (High Risk)")
        if not alert_sent:
         sendAlert()
         alert_sent = True
         else:
         print("Alert already sent. Waiting for cooldown period to end.")

        else:
         print("\nNo Fall (Safe Condition)")
         alert_sent=False
    time.sleep(1)

# Emergency response module
import smtplib
message="""From: smart_cane@gmail.com
To: caretaker_mail@gmail.com
Subject: FALL ENCOUNTERED!!!



Please look out for the individual.
"""
def sendAlert():
  with smtplib.SMTP_SSL("smtp.gmail.com",465) as server:
  server.login("smart_cane@gmail.com","xxxx xxxx zxxx zxxf")
  server.sendmail
 ('smart_cane@gmail.com',['caretaker_mail1@gmail.com',
 'caretaker_mail2@gmail.com','caretaker_mail3@gmail.com'], message)
  print('Sent mail succesfully.')
