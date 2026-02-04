# 🌡️ Environmental Monitoring System (STM32 + Android)
This project is an integrated temperature and humidity monitoring system that utilizes the STM32F429I-Discovery microcontroller as the central processing unit, a DHT22 sensor for data acquisition, and an HC-05 Bluetooth module for wireless transmission to a mobile application developed in Kotlin.

## 🚀 Key Features

*Real-time Acquisition: Accurate environmental data reading using the Single-Bus protocol (DHT22).*

*Dual Display: Data visualization on both the integrated LCD (ILI9341) and an Android device.*

*Bluetooth Connectivity: Stable serial transmission via HC-05 using the Serial Port Profile (SPP).*

*Robust Processing: Implementation of Regular Expressions (Regex) in Android for precise data packet parsing.*

*Modern Interface: Intuitive Android app with dynamic color feedback based on temperature thresholds.*

## 🛠️ Hardware Components

*Microcontroller: STM32F429I-Discovery (Cortex-M4 @ 180MHz).*

*Sensor: DHT22 (AM2302) for high-precision temperature and humidity sensing.*

*Bluetooth: HC-05 Module (UART interface).*

## 📊 Connection Diagram
Component, STM32 Pin, Function

DHT22, PC3, Data (Single-Bus)

HC-05 TX, PA10, UART RX

HC-05 RX, PA9, UART TX

LCD, Internal, LTDC Interface

## 💻 Software Stack

#### Firmware (C/HAL): Developed in STM32CubeIDE. Implements sensor drivers and the LCD graphical interface.

#### Mobile App (Kotlin): Developed in Android Studio. Manages the Bluetooth RFCOMM socket and UI logic.

## ⚙️ Installation and Usage

*Hardware: Set up the connections as specified in the table above.*

*Firmware: Flash the .bin file or the project from the /STM32_Code folder onto the board.*

*Android: Install the APK provided in the /Android_App folder.*

*Setup: Pair your smartphone with the HC-05 module, launch the app, and enter the module's MAC address in the settings.*

## ⚠️ Known Issues and Troubleshooting

*Garbage Characters in Terminal: Ensure the HSE (High-Speed External) oscillator frequency is correctly configured to 8MHz in the clock tree.*

*Checksum Errors: Verify that the DHT22 sensor has a stable 3.3V power supply.*

*Connection Refused (Android): Manually check if the Nearby Devices permission is granted in the application settings (Android 12+).*

## 📚 Bibliography
Project developed under the guidance of Asst. Prof. Dr. Eng. Ionel Zagan, "Ștefan cel Mare" University of Suceava.

## ⭐ If you found this project helpful, please give it a Star on GitHub!
