# 🌤️ Arduino Weather Station

## 📚 Project Overview

This project is based on proposal **P3: Weather Station** from the Embedded Systems course (2024/2025). It involves designing and implementing a basic weather station using an Arduino board, capable of measuring and displaying environmental data in real time.

## 🎯 Objectives

- Develop a low-cost embedded system that simulates a weather station.
- Measure environmental variables (e.g., temperature, humidity, distance for rain level or wind movement).
- Display the data on an LCD.
- Optionally provide alerts (e.g., sound warnings for extreme conditions).

## 🛠️ Components Used

- Arduino UNO (or compatible)
- Ultrasonic Sensor HC-SR04 (e.g., to simulate rain or wind detection)
- I2C 16x2 LCD Display
- Buzzer (for alerts)
- (Optional: DHT11 or DHT22 for temperature and humidity measurement)
- Jumper wires
- Breadboard

## ⚙️ How It Works

> **Note:** The current version uses an ultrasonic sensor for distance measurement, which can simulate certain weather-related phenomena (like rainfall level in a container or wind movement via proximity).

1. The ultrasonic sensor measures distance — e.g., water level in a container (to simulate rain accumulation).
2. The value is calculated and shown on the LCD screen.
3. If the measured value drops below a defined threshold (e.g., high water level or close proximity), a buzzer sounds an alert.

## 🔌 Wiring Diagram

| Component        | Arduino Pin |
|------------------|-------------|
| HC-SR04 Trig     | D9          |
| HC-SR04 Echo     | D10         |
| Buzzer           | D8          |
| LCD (I2C) SDA    | A4          |
| LCD (I2C) SCL    | A5          |

> **Note:** SDA/SCL pins may vary depending on your Arduino model.

## 📦 Required Libraries

Before compiling the code, make sure to install the following:

- [`LiquidCrystal_I2C`](https://github.com/johnrickman/LiquidCrystal_I2C)

Install via the Arduino IDE Library Manager.

## 🚀 How to Use

1. Connect the components according to the wiring table.
2. Open the `projetoSE.ino` file in the Arduino IDE.
3. Upload the code to your Arduino board.
4. Observe the measured values on the LCD display.
5. Trigger the buzzer by reducing the measured distance (e.g., placing an object closer).

## 📈 Potential Enhancements

- Integrate a DHT11/DHT22 sensor for actual **temperature and humidity** measurement.
- Add a BMP180 or BME280 sensor for **atmospheric pressure**.
- Use an SD card module to **log historical weather data**.
- Display information via a **web interface or mobile app** using ESP8266/ESP32.
- Include a **real-time clock (RTC)** to timestamp readings.

## 🧑‍🎓 Authors

Group project developed for the Embedded Systems course – 2024/2025.

## 📄 License

This project is licensed under the MIT License. See the LICENSE file for more information.
