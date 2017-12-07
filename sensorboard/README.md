# Sensor board
(Also check out the [Wiki](https://github.com/adi64/maerklinspass/wiki), especially the page for the [sensor board](https://github.com/adi64/maerklinspass/wiki/Sensorboard)!)

This program drives the sensorboard.
The sensorboard monitors the train detector switches by polling for events.
Up to 16 detector switches can be connected to this board (or 8 dual-direction switches ).
The inputs are read using two 1:8 multiplexers.

Everytime a train activates a detector switch, a CAN message is sent.
The message's identifies corresponds to the address of the switch that was toggled.
Since every sensorboard has a base address that can be configured through jumper pins,
every message can be tracked to its originating sensorboard and detector switch.

The message data consists of a local timestamp and a duration value.
When a switch is toggled, the message contains the current local timestamp and a duration of 0.
When the switch returns to its default position, another message is sent, again with the
current local timestamp and the duration during which the switch was activated.
This way it is possible to distinguish between activation and deactivation messages.

Debouncing is applied to prevent sending multiple unwanted messages.
