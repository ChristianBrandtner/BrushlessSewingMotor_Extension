# BrushlessSewingMotor_Extension
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;
[![License: GPL v3](https://img.shields.io/badge/License-GPLv3-blue.svg)](https://www.gnu.org/licenses/gpl-3.0)


contact: github@brandtner.net

## STATUS

22.6.2022

Version 14.2
-
Änderungen an der Platine
Grund:
JST Stecker J3 Fehler in der Ausrichtung beseitigt


20.6.2022

Version 14.1
Änderungen an der Platine
Grund:
OK2 Anschlüsse vertauscht, optionale Widerstände R6,R30,R31
JST Stecker J3, Ausrichtung getauscht damit fertig konfektionierte Kabel zum SMC verwendet werden können.
Schaltung getestet (ohne RPM-Sensor und analog 0-10V Eingang, alles optional)

Wesentliche Funktionen der Software funktionieren:
Drehzahlregelung, Richtungsumschaltung, Drehzahlanzeige

03.06.2022 New PCB Version 12.2, Errors in PCB layout fixed. Basic software in progress.

12.05.2022 PCB Version 11.5 with galvanic isolation is in production 

30.04.2022 The goal is to fit the SMC controller board and everything else into the existing lathe front casing.

29.04.2022 optoisolation with separate powersupply is in work.

28.04.2022 under construction and brainstorming.


## INTRODUCTION

This is a extension for a cheap chinese 550/750W brushless servo motor for external control of speed, direction, etc.
to use in in e.g. Mini Lathe


first test with switch and potentiometer:
https://www.youtube.com/watch?v=qao7Rex1lZU

![SMC_Motor](https://user-images.githubusercontent.com/60114001/165738964-6df24e4b-6300-4330-b555-efc8d85aca5b.jpeg)

## PREVIEW

![Display3](https://user-images.githubusercontent.com/60114001/174744652-ce5d3712-e71a-4233-8b9c-0a11667b0ed3.jpg)
![Display4](https://user-images.githubusercontent.com/60114001/174744656-8c1be3cf-0af7-4bf9-b2b0-32b9f43f2d71.jpg)


## TODO

TODO: finalize software

## Features

- no changes on Controlerboard or Panelboard needed; 
- can be installed in the housing on existing attachments; 
- speedcontrol, potentiometer;
- start/stop control, switch; 
- rotation direction control, button with LED;
- securityswitch input for chuck guard;
- spindel RPM display (calculated)
- original SMC displaypanel can be used at same time 
- high temperture warning;
- isolated inputs/outputs for galvanic isolation from the mains voltage

## planned software features

- rpm calculation with gear reduction
- rampup and down at start/stop

- 0-10 V input to speed control (future?)
- optocoupler input for spindle speed measurement (future?)



