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


![image](https://user-images.githubusercontent.com/104785921/168161866-bff8b12b-9ba8-4771-abdf-470f46c5c86a.png)

Figure 1. Photo of a Furuta Pendulum.

This document summarizes the basic control theory, programming, and calculations necessary to create a working CoppeliaSim model with supporting state-space representation, equations of motion, and control code via Mathworks’ MATLAB and Simulink programs.

----------------------------------------------------------------------------
## 2 MODELING

### 2.1 Sketch

Our team followed the provided workbook Rotary Pendulum (ROTPEN) published by Quanser. The figure below is provided from the Quanser workbook, it depicts the rotarty pendulum in a freebody diagram.

![image](https://user-images.githubusercontent.com/104785921/168168774-9863f341-fba4-493f-b050-29c298c18598.png)

Figure 2. Free Body Diagram of Rotary Pendulum

By analyzing the figure above, the basic operation of the pendulum is controlled by two angles between the base to arm and arm to pendulum. The goal is to create script such that the two angles reach equilibrium and maintain a balanced pendulum in the upright position. Counter-Clockwise motion is depicted as a positve change in angle for the system, with this sign convention applied voltage to the motor matches so that positive voltage equates to positive angle.

### 2.2 Parameters

--------------------------------------------------------------------------------
## 3 Simulation

### 3.1 Matlab

### 3.2 Simulink

--------------------------------------------------------------------------------
## 4 Implementation

--------------------------------------------------------------------------------
## 5 References

---------------------------------------------------------------------------------
