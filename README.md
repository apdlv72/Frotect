Frotect
=======

Arduino sketch to protect sensitive plants from frost damage. Uses DS18B20  temperature sensors and solid state relays to switch light strands on/off.
You need the DallasTemperature arduino library to compile the sketch. 

The device can control up to four strands (however this should not be a hard limit if you are willing to adjust some #defines) and up to 5 sensors.
You can define lower and upper temperature threshold per strand where it should be swicthed on/off. I used it in my garden to protect some young,
exotic plants to survive their first winter in Germany. I mounted the light strips around a simple plastic schaffold and enwrapped the whole setup
with protective popup covers, not only to minimize heat discipation, but also to protect the plants from winter storms and sunshine (yes - that in fact
is the most likely reason for e.g. Olive trees dying in norther areas, not the coldness).

The device will also do some data logging of min/max temperatures, record the duty cycle per strand and even compute overall costs.
The device communicates (actuall it can, there's no need to do so) with the outside world via a Bluetooth module and can be controlled
via simple commands sent to it with a PC/laptop or an Android app. See the folder android for details.

