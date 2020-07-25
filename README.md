# STM32-PID-IMPLEMENTATION-CMSIS

# TOPOLOGY:
![](photos/topology.png)
# REAL IMPLEMENTATION:
![](photos/IMG_20200204_113213.jpg)
**How does it work?**
You can inflict temperature with the encoder. LCD screen shows set temperature and actual temperature.  LCD show also a small menu where we can set PID values. With every click on the encoder button, we jump on the menu. 
BMP280 sensor measure temperature which is emitting by 5W resistor and send information to the STM. STM based on received data decides about the voltage on resistor.
The fan may be turned on to accelerate achieve the set temperature. In our application, we used a fan with an on–off controller.
