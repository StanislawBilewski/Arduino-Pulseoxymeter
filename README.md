# Arduino Pulseoxymeter

This Arduino project focuses on building a pulseoxymeter with the feature of playing in the rhythm of the user's heart.

## Introduction
Pulseoxymeter is a device calculating the heartrate and blood oxygen saturation.  
Pulseoxymeters use two monochromatic LEDs with different frequencies. 
The light of these LEDs need to pass through patient's skin and be registered by a photodiode on the other side.  
From the difference between the two frequencies of light absorption spectrum can be determined, from which the oxygen saturation level can be calculated.

The standard levels for a healthy adult are: 
- oxygen saturation of 95-99% 
- heart rate of 60-100 beats per minute (BPM)  

The actual value of saturation, as well as heart rate, may change depending on patient's stress, exhaustion, age, weight and diet.

## Device description
Device was built using Arduino UNO with a ATMEGA 328P microcontroller.  
The sensor was built on a breadboard.  
The calculated heart rate and oxygen saturation are displayed on an LCD display with an I2C converter.  
The mechanism that allows to play the heart beat is a simple stick attached to the stepper motor, which turns by approx. 11 degrees whenever a pulse is detected - 
the stick then hits a wooden block (making a quiet 'click' sound).

In order to use the device one must place their finger on the photodiode (the light sensor). 
The two LEDs should be touching the fingernail so that the finger is lit as well as possible.
It is however important to remember to keep the hand relaxed - 
as putting too much pressure on the sensor may result in a change in the bloodflow, altering the results.
After placing the finger on the sensor the user must wait a few seconds in order for the signal to stabilize and 
for the device to make enough mesures to calculate the heart rate correctly

## Diagram
![Full device diagram](fulldiagram.png)
