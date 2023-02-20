## Rationale
The reason why the project exists is the fact that currently drying module is not supported byt the roborock s7 pro ultra robot. 
To automize turning on of the drying module we would use 3 hardware parts:
1. Switchbot bot
2. Esp32 wroom v1
3. HC_SR04

The esp32 would track robot presents using HC_SR04 ultrasonic sensor and its embedded bluetooth to control switchbot arm.

## Schematics
Connect VIN and GND to VCC and GND HC_SR04 respectively.

The 12 pin is configured to be HC_SR04 trigger pin.

The 14 pin is configured to be HC_SR04 echo pin.

## State machine
![cached image](http://www.plantuml.com/plantuml/proxy?src=https://raw.githubusercontent.com/yota9/esp32_switchbot_dryer/main/states.uml?token=GHSAT0AAAAAAB2LTYYUBUOXBNBTGMBKNLHAY7TXAKQ)

## Results
<img src="https://github.com/yota9/esp32_switchbot_dryer/blob/main/switchbot.png" width=30% height=30%>
<img src="https://github.com/yota9/esp32_switchbot_dryer/blob/main/esp32.png" width=30% height=30%>
