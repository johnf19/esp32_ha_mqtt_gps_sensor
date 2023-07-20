# esp32 ha mqtt gps sensor
## A gps, temperature and acceleration sensor based on esp32 and intergrated in Home Assistant by MQTT.
>Usage: Locating and anti-theft for vehicle(especially for 2-wheel-vehicles) in any place with wifi-connection(school, office .etc)

### Hardware Solution:
- MCU: ESP32C3.
- GPS module: ATGM336H(BDS/GPS/GLONASS/GALILEO/QZSS/SBAS).
- temperature & acceleration sensor: MPU 6050.

### Function:
- Publish GPS, temperature data(every 10 minutes).
- Movement Detection (IN PROGRESS).
- Reconnect if WiFi disconnected.

### Usage:
- This is a test project, now with temperature and gps data only. Motion sensor still work in progress.
- Edit essential elements properly in the main script & run.
- Campus wifi authentication may needed, I used MAC binding in my own case.
