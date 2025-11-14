# Stop-And-Go-Conveyor-RobotStudio-SmartComponent
Stop And Go Conveyor RobotStudio SmartComponent

This SmartComponent allow to simulate the behavior of a "stop and go" conveyor in a RobotStudio simulation.

Properties : 
Source Part : part to generate on the conveyor
Lenght : lenght of the conveyor, part are removed when reaching this dstance
Speed : speed of the conveyor
Stoppers : optionnal parameter that allow to add stopping positions (number of stopper)
Stopper position : position from the smartcomponent local origin of the stopper

Signals : 
Add Object : new object is added at the local origin of the conveyor
Clear : all the object are removed
Enable : objects on the conveyor are moving at the desired speed
Stopper Command : the part will be blocked when reaching the stopper position
Stopper Sensor : feedback when a part is blocked by the stopper

Behavior :
When "Enable" signal is active, the generated parts are moving in a linear direction at a given speed.
The direction of the movement is in X direction of the smartcomponent.
When multiple parts are moving on the conveyor, collision between part is handled so that they will not overlap.
When parts reach the "lenght" value, they are removed.
It is possible to add stoppers at desired distance.
When the "stopper command" signal is activated, and a part reach the stopper, the part will stop.
In that case the "stopper sensor" signal is high. When no part are blocked at a stopper, the stopper sensor signal is low.


