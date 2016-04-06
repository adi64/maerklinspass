# Sensorboard

This program drives the sensorboard.
The sensorboard monitors the train detector switches by polling for events.
Up to 16 detector switches can be connected to this board (or 8 dual-direction switches ).
The inputs are read using two 1:8 multiplexers.

Everytime a train activates a detector switch, a CAN message with the switch's address is sent.
Each sensorboard has a base address that can be configured through jumper pins.