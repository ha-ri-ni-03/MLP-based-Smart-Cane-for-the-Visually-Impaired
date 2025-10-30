# Optimising Fall Detection with MLP

<p align="justify">
The <strong>Multi-Layer Perceptron (MLP)</strong> model excels at detecting falls because of its ability to extract complex and nonlinear patterns from sensor data, which helps it distinguish between an actual fall event and non-fall events (e.g., sitting down rapidly). MLP, with its multiple hidden layers and activation functions, effectively processes these changes and identifies key features from the sensor readings. It can achieve high accuracy with well-engineered features such as acceleration thresholds, angular velocity, and statistical measures derived from sensor signals. Moreover, it is computationally more efficient compared to deep learning models like convolutional neural networks (CNNs) or recurrent neural networks (RNNs), making it suitable for devices with limited resources such as the ESP32.
</p>

<p align="justify">
Despite its simpler architecture, MLP demonstrates strong accuracy in classifying sensor data and integrates seamlessly with IoT-based fall detection systems, even in noisy, real-time environments. The trained model can analyze sensor data either on the device or in the cloud, enabling real-time analysis, quick emergency alerts, and long-term monitoring. Although MLP may sometimes experience reduced recall for falls, this can be improved through effective data pre-processing and feature enhancement.
</p>

<p align="justify">
Overall, MLP offers an effective balance between prediction accuracy and computational efficiency. It is a practical and powerful choice for fall detection applications, providing real-time performance while maintaining low power and memory consumption. This makes it ideal for embedded AI/ML applications in smart canes, wearable devices, and home monitoring systems.
</p>

---

# Process Workflow

### Obstacle Detection System

<p align="justify">
The fundamental component of the <strong>Smart Cane</strong> operates by measuring the distance of nearby obstacles using <strong>sound waves</strong>. It emits a <strong>high-frequency sound pulse</strong> and calculates the <strong>time taken for the echo to return</strong>, enabling precise obstacle detection. Objects within a range of <strong>40 cm</strong> are detected, allowing real-time obstacle detection and response.
</p>

If an obstacle is detected within the predefined threshold distance, the system provides <em>multi-modal feedback</em> through:

- **Haptic feedback:** Vibration motor  
- **Auditory feedback:** Buzzer  
- **Visual indication:** LED  

---

## Dataset and Model Implementation

### Dataset Collection and Storage

<p align="justify">
The dataset is acquired using the <strong>MPU6050</strong> sensor module, which integrates a 3-axis accelerometer and a 3-axis gyroscope with the <strong>ESP32</strong> microcontroller featuring built-in Wi-Fi. The collected sensor data is stored in the <strong>Firebase Realtime Database (RTDB)</strong>, ensuring real-time synchronization and secure transmission via <strong>Transport Layer Security (TLS)</strong>. Additionally, <strong>Firebase Authentication</strong> is employed to validate user access and enhance data security.
</p>

### Dataset Preparation and Labeling

<p align="justify">
After structuring the dataset, the samples are categorized into two primary classes:
</p>

1. **Fall Events:**  
   Comprising *10,127 samples* that include various fall scenarios such as forward, backward, and sideward (left/right) falls, as well as tripping and slipping events.  

2. **Non-Fall Events:**  
   Comprising *11,305 samples* representing normal daily activities, including walking, sitting, turning, bending, climbing stairs, and jumping.  

---

### Data Augmentation Technique

<p align="justify">
To improve model generalization and robustness, <strong>magnitude warping</strong> is applied as a data augmentation technique. This approach introduces controlled variations in the amplitude of motion signals while preserving essential movement characteristics, thereby enhancing the model’s reliability in real-world fall detection scenarios.
</p>

---

### Live Implementation: Fall Detection & Alert System

<p align="justify">
After training with the labeled dataset, live motion data is continuously retrieved from the <strong>Firebase Realtime Database (RTDB)</strong>. The trained model processes incoming data in real time to classify events as either <em>fall (1)</em> or <em>non-fall (0)</em>. If a fall event is detected, the <strong>emergency response module</strong> is triggered, and instant alerts are sent to the designated <strong>caregiver(s)</strong> to ensure timely assistance.
</p>



