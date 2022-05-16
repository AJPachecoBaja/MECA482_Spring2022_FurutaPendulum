# MECA482_Spring2022_FurutaPendulum

A collaborative project report for CSU: Chico MECA 482 (Control Systems)

Created by Sean Black, Laine Wood, and Andrew Pacheco

--------------------------------------------------------
## Table of Contents

- [1 Introduction](#1-Introduction)

- [2 Modeling](#2-Modeling)

    - 2.1 Sketch
    - 2.2 Parameters
    - 2.3 Motion Equations
    - 2.4 State Space Representation
    
- [3 Simulation](#3-Simulation)

    - 3.1 Matlab
    - 3.2 Simulink
 - [4 Implementation](#4-Implementation)
 - [5 References](#5-References)
 
-------------------------------------------------------------
## 1 Introduction
A rotary inverted pendulum, also known as a Furuta Pendulum, is a non-linear, self-balancing control system. The basic design is created using two rotating masses connected to a base, this is a tool used far and wide to teach the basic principles of control systems designs.  The machine consists of a motor actuator connected to the base, an arm which is free to rotate around the horizontal axis, and an arm that is free to rotate around the vertical axis. The overall system goal is to balance the vertical arm, or pendulum, in an upright position by controlling the rotation of the motor about the horizontal axis. Shown below in Figure (1) is an example of a Furuta Pendulum.


<p align="center">
  <img 
    width="380"
    height="400"
    src="https://github.com/AJPachecoBaja/MECA482_Spring2022_FurutaPendulum/blob/main/Figure_1._Furuta_Pendulum.%20-%20Copy-modified.png?raw=true"
  >
</p>

<p align="center">
Figure 1. Photo of a Furuta Pendulum. 
    
</p>

This document summarizes the basic control theory, programming, and calculations necessary to create a working CoppeliaSim model with supporting state-space representation, equations of motion, and control code via Mathworks’ MATLAB and Simulink programs.

----------------------------------------------------------------------------
## 2 MODELING

### 2.1 Sketch

Our team followed the provided workbook Rotary Pendulum (ROTPEN) published by Quanser. The figure below is provided from the Quanser workbook, it depicts the rotarty pendulum in a freebody diagram.

<p align="center">
  <img 
    width="500"
    height="500"
    src="https://user-images.githubusercontent.com/104785921/168168774-9863f341-fba4-493f-b050-29c298c18598.png"
  >
</p>

<p align="center">
Figure 2. Free Body Diagram of Rotary Pendulum.
    </p>

By analyzing the figure above, the basic operation of the pendulum is controlled by two angles between the base to arm and arm to pendulum. The goal is to create script such that the two angles reach equilibrium and maintain a balanced pendulum in the upright position. Counter-Clockwise motion is depicted as a positve change in angle for the system, with this sign convention applied voltage to the motor matches so that positive voltage equates to positive angle.

### 2.2 Parameters

|  Symbol  | Description  |
| -------------: | :------------- |
| kg = |    kilograms                 |
| m =  |    meter                     |
| s =  |    second                    |
| N =  |    Newton                    |
| A =  |     Area                     |
| Ω =  |    Ohm                       |
| V =  |    Volt = [(kg*m^2)/(s^3*A)] |



|       Parameter            |       Symbol       |      Units       |
|     :-------------:        |   :-------------:  |  :-------------: |
| Mass of Pendulum           |       Mp = 0.128   |         kg       |
| Mass of Pendulum           | Mp1 = 0.128        |         kg       |
| Pendulum Length            |        Lp = 0.316  |    m             | 
| Pendulum Moment of Inertia | Jp = 5.43*10^(-3)  | kg*m^2           |
| Arm Length                 | Lr = 0.216         |   m              |
| Arm Moment of Inertia      | Jr = 20            |     kg*m^2       |
| Gravity                    | G = 9.81           |      m/s^2       |
| Total Gear Ratio           | Kg = 0.9           |        ----      |
| Motor Torque Constant      | Kt = 0.00768       |   (N.m)/A        |
| Motor back-EMF constant    | Km = 0.03          |    (V.s)/rad     |
|           ----             |  Dr = 1            |        ----      | 
| Motor Armature Resistance  |   Dp1 = 1          |        ----      |


--------------------------------------------------------------------------------
## 3 Simulation

### 3.1 Matlab

### 3.2 Simulink

--------------------------------------------------------------------------------
## 4 Implementation

--------------------------------------------------------------------------------
## 5 References

---------------------------------------------------------------------------------
