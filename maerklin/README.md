# Train Controller Demo
(Also check out the [Wiki](https://github.com/adi64/maerklinspass/wiki)!)

This program controls the movement of 3 trains on 4 track segments (see track layout below)
ensuring that at most one train is on a particular segment.
The current position of a train is tracked by monitoring the segment borders.
Each passing train generates an event on the CAN bus. The event ID for each segment border
is marked on the track layout next to the respective parenthesis in hexadecimal format. (e.g. 308)

Track segments can only be crossed in one direction; all trains move counter-clockwise.
There are 2 switch arrays (SA0 and SA1) to connect the track segments.
Both trains and track segments have numbers starting at 0 (each physical train
or track segment has a label with its number attached) and after a controller reset
each train is expected to occupy the track segment of the same number.

## Track Layout

```
      ____<T2_____
     /  __<T3___  \
    |  /        \  |
 308) |          | )306
    | )30A    304) |
    |\|          |/|
    | |SA1    SA0| |
    |/|          |\|
    | )30C    302) |
 30E) |          | )300
    |  \___T1>__/  |
     \_____T0>____/
```

## Serial console

This code also provides a very basic train control via serial connection.
Trains can be stopped, started and their speed can be set.
This way it is possible to add some variety to the scenario.
Commands are:

* `H`: Stop all trains.
* `L[array-index][speed]`: Set the default speed of the locomotive at the given array-index.
  * Example: `L0E` sets the speed of the first locomotive to 14.
  * Example: `L20` stops the third locomotive.
  * Example: `L21` signals the third locomotive to change its direction. **Stop it first!**
* `W[array-index][configuration]`: Set the switch array at the given array-index to the given configuration.
  * Example: `W10` sets the second switch array to STRAIGHT
  * Example: `W11` sets the second switch array to IN2OUT
  * Example: `W12` sets the second switch array to OUT2IN
