# AIGSG-TAG3-Software
Software for cattle's tag with SIM28ML gps and LoRa radio

Packet format:
* 0 byte - START byte
* 1, 2 bytes - ADDRESS
* 3 byte - GPS data ready (1 bit), // other bits "to be definded"
* 4 byte - UTC time from GPS
* 5, 6 bytes - Latitude
* 7, 8 bytes - Longtitude
* 9 byte - Battery Level
* 10 byte - Temperature (for future using)
* 11 byte - CRC
* 12 byte - END byte
