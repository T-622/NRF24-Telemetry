# NRF24-Telemetry
Project that utilises the NRF24 + PA modules to produce a working live telemetry system for RC cars or Planes.

Components:
- 1x MPU6050 Gyrometer
- 2x NRF24L01 + PA SPI modules
- 1x Arduino UNO
- 1x Arduino Mega 2560
- Optional (1x Battery voltage module) [Make this yourself]
- 1x DHT11 Temp & Hum sensor
- 1x Magnetic linear hall sensor with digital output
- 1x Thermistor probe and board (Schematic attached)
- 3D printed case parts (Provided .stl's)
- Optional (1x Current sensor)
- 1.8 Inch DSD Tech TFT lcd

Steps
1.) Conenct both respective transmitter and reciever sides accordingly with provided schematic
2.) Test and load code for respective boards. RX has only an LCD and the transmitter has all the components.
3.) Attach various sensors to parts of your RC vehicle. In my Case, I strapped the temperature probe to the Heatsink of my ESC, The voltage probe inline with the battery, and the hall sensor onto a wheel bracket with a neodymium magnet attached to the wheel itself. This way, each time the wheel rotates, the magnet passes and triggers the RPM counter. The GPS module should have an unobstructed view to the sky for the speed measurement.
