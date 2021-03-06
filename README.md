#  Embedded Systems - Wireless Sensor Networks
<p align="center">
  <img alt="WSN" width="760" height="308" src="repoassets/WSN.jpg">
</p>

Final assignment for the Embedded Systems course, at **Universidad Nacional del Sur**.

### Description
This project is about controlling 3 Mrf24j40MB module (IEEE 802.15.4) that are connected to the SPI hardware interface of two Arduino and one Raspberry PI. We used the arduinos as the leaf nodes of the network to read sensors values and send it to the coordinator node (Raspberry). The coordinator is connected to the internet and retransmit the values to a database, in this case Firebase. Then the information is displayed in a web dashboard. Moreover, commands can be sent from the web page to any nodes in the network (e.g turning on/off a motor or lights, change settings remotly, etc).

### Team
* Fassi Jeremias
* Salazar Gisbert Gabriel

### Technologies
* Mrf24j40MB (IEEE 802.15.4)
* Raspberry Pi 3B
* Arduino Uno
* Firebase realtime database
* Angular 7

### Sources and Acknowledgements
* Arduino driver for the mrf24j40 802.15.4 modules [GitHub](https://github.com/karlp/Mrf24j40-arduino-library).
* <a href="http://wiringpi.com/">WiringPi</a>, a PIN based GPIO access library written in C for Raspberry PI.
* WiringPi-Python, Unofficial Python-wrapped version of Gordon Henderson's WiringPi version 2 [GitHub](https://github.com/WiringPi/WiringPi-Python).
* Pyrebase, a simple python wrapper for the Firebase API [GitHub](https://github.com/thisbejim/Pyrebase).
* Big thanks to <a href="https://www.akveo.com">Akveo</a> for their great admin dashboard template [GitHub](https://github.com/akveo/ngx-admin)
