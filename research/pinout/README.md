Github documentation:
https://github.com/LilyGO/TTGO-T-Beam 


#### NB:
- The sensors can share SCL / SDA pins, but ideally, find 4 pins that are free.
- The sensors will share GND & 5V pins

#### Features:
- GPS (make it work)
- Battery (make it work)
- On / Off / Reset switches
- Sleep
- Send to firebase db

POWER CHIP: AXP192
BATTERY: 18650
BUTTONS: reset, power, gpio_38

https://github.com/G6EJD/ESP32_LoRa_Examples/blob/master/ESP32_LoRaReceiver_Simple.ino

## Tiny GPS
- https://github.com/LilyGO/TTGO-T-Beam/blob/master/GPS/GPS.ino 
- https://github.com/mikalhart/TinyGPSPlus/blob/master/src/TinyGPS%2B%2B.h
- https://stackoverflow.com/questions/68083295/why-i-cant-get-gps-data-from-ttgo-tbeam-t22-v1-1-is-there-any-problem-with-my 
- https://www.thethingsnetwork.org/forum/t/ttgo-tbeam-v1-1-gps-fix/57999/3 
- https://github.com/glacierjay/T_beam/blob/master/gps/tbeam_gps/tbeam_gps.ino
- https://www.youtube.com/watch?v=Kgxx7jivSes&ab_channel=SayaneeBasu

## Firebase
- https://github.com/mobizt/FirebaseClient

