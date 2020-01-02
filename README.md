# ECE560-Multithreaded-RTOS


Project - Displaying currents & Sharing the ADC

Area - Embedded Systems

Problem Statement - Implement mutual sharing of the ADC between the buck converter for high brightness LED and the touchscreen display, while maintaining correct timing for the buck converter. Alsp plot the setpoint and measured LED current graphically using RTOS primitives

Solution - Mutual sharing of ADC was achieved using RTOS primitive of message queues.
