# Optimising Fall Detection with MLP
The Multi-Layer Perceptron (MLP) model excels at detecting falls because of its ability to extract complex and nonlinear patterns from sensor data, which helps it clearly tell the difference between an actual fall event and other non-fall events (e.g., sitting down rapidly). MLP, with its several hidden layers and activation functions, effectively processes these changes and finds important features from the sensor. It can achieve high accuracy with well-engineered features, such as acceleration thresholds, angular velocity, and statistical measures derived from sensor signals, and is more efficient in computing compared to deep learning models like convolutional neural networks (CNNs) or recurrent neural networks (RNNs) that may require large datasets. It is computatioally more efficient thereby making it suitable for use on devices with limited resources, like the ESP32. Even with its simpler design, MLP shows higher accuracy in classifying sensor data and also facilitates smooth integration with IoT-based fall detection systems and is compatible with real-time datasets, which could be noisy. The trained model could analyze sensor data either on the device or in the cloud, allowing for real-time analysis, quick emergency alerts, and long-term monitoring of mobility. Although MLP may sometimes struggle with lower recall rates for falls, this could be improved through better data pre-processing techniques.

Furthermore, MLP offers an effective balance between prediction accuracy and computational efficiency. It is a powerful and practical choice for fall detection, offering a balance of accuracy, efficiency, and scalability. Its ability to process sensor data in real-time while remaining computationally lightweight makes it an excellent choice for wearable and IoT-based health monitoring systems. It provides reliable classification while consuming less power and memory, making it ideal for embedded AI/ML applications in smart canes, wearable devices, and home monitoring systems.

# Process Workflow
### Obstacle Detction system
The fundamental component of the **Smart Cane** operates by measuring the distance of nearby obstacles using **sound waves**. It emits a **high-frequency sound pulse** and calculates the **time taken for the echo to return**, enabling precise obstacle detection. Objects within a range of **40 cm** are detected, allowing real-time obstacle detection and response.

If an obstacle is detected within the predefined threshold distance, the system provides *multi-modal feedback* through:

- **Haptic feedback:** Vibration motor  
- **Auditory feedback:** Buzzer  
- **Visual indication:** LED
  

## Dataset and Model Implementation

### Dataset Collection and Storage
The dataset is acquired using the **MPU6050** sensor module, which integrates a 3-axis accelerometer and a 3-axis gyroscope with the **ESP32** microcontroller featuring built-in Wi-Fi. The collected sensor data is stored in **Firebase Realtime Database (RTDB)**, ensuring real-time synchronization and secure transmission via **Transport Layer Security (TLS)**. Additionally, **Firebase Authentication** is employed to validate user access and enhance data security.

### Dataset Preparation and Labeling
After structuring the dataset, the samples are categorized into two primary classes:

1. **Fall Events:**  
   Comprising *10,127 samples* that include various fall scenarios such as forward, backward, and sideward (left/right) falls, as well as tripping and slipping events.

2. **Non-Fall Events:**  
   Comprising *11,305 samples* representing normal daily activities, including walking, sitting, turning, bending, climbing stairs, and jumping.

### Data Augmentation Technique
To improve model generalization and robustness, **magnitude warping** is applied as a data augmentation technique. This method introduces controlled variations in the amplitude of motion signals while preserving essential movement characteristics, thereby enhancing the model’s reliability in real-world fall detection scenarios.

### Live Implementation: Fall Detection & Alert System
After training with the labeled dataset, live motion data is continuously retrieved from **Firebase Realtime Database (RTDB)**. The trained model processes the incoming data in real time to classify events as either *fall (1)* or *non-fall (0)*.  
If a fall event is detected, the **emergency response module** is triggered, and instant alerts are sent to the designated **caregiver(s)** to ensure timely assistance.


```
