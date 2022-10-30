# pid_controller

## Introduction

This is a library for a PID (Proportional-Integral-Derivative) controller written in plain C.

The integral part includes an anti-windup protection using the integrator clamping technique, where integration is not done if the output is sarutated and the error and the output have the same sign. Otherwise, it integrates normally. This simple technique is found to give very good results with a faste response.

The derivative part includes a filter (TBD) in order to avoid undesired effects due to de derivation of noises.

A feed-forward input is also included, which is also taken into account when limitting the output, and thus, when clamping the integrator.

    TBD: [IMAGE]

Updating/creating and resetting functions are also included.

## Usage

TBD.
    
## References

This other works have been used as reference:
    
- https://github.com/pms67/PID
- https://www.youtube.com/watch?v=zOByx3Izf5U&feature=emb_title
- https://github.com/br3ttb/Arduino-PID-Library
- http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/
- https://github.com/akharsa/qPID
- https://github.com/uLipe/PidControlTemplate/tree/master/lib
- https://github.com/tcleg/PID_Controller
- https://github.com/geekfactory/PID

