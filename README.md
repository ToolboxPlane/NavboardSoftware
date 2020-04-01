# NavboardSoftware
Code running on the navigation board, preliminary version running on an [Adafruit Feather M0 with RFM95 LoRa](https://www.adafruit.com/product/3178).

Additional hardware:
 * NXP MPL3115A2 connected via I²C
 * Pitot Tube with NXP MPXV5004DP Sensor connected to ADC 0
 * SRF02 connected via I²C

## Output Format
The UART-Baud Rate is 115200.

### Navboard Output (Transmitter ID 91)
The output package is a 11 bit, 4 Channel Package with the following data:

| Channel | Data |
| --- | --- |
| 0 | RSSI of the last package |
| 1 | Altimeter Data (Capped at 2047) (m) |
| 2 | ADC value of the pressure sensor for the Pitot-Tube (0-1023) |
| 3 | Ultrasonic Distance (cm) |

## LoRa
All data that is received via LoRa is directly forwarded, all data that is sent to the Navboard is forwarded as well.
