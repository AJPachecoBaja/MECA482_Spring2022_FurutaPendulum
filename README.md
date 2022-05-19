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
    src="https://user-images.githubusercontent.com/103708956/168511751-b11f5390-5207-4fea-b512-11c63e4dbe67.png"
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
| Pendulum Length            |        Lp = 0.316  |    m             | 
| Pendulum Moment of Inertia | Jp = 5.43*10^(-3)  | kg*m^2           |
| Arm Length                 | Lr = 0.216         |   m              |
| Arm Moment of Inertia      | Jr = 20            |     kg*m^2       |
| Gravity                    | g = 9.81           |      m/s^2       |
| Total Gear Ratio           | Kg = 0.9           |        ----      |
| Motor Torque Constant      | kt = 0.00768       |   (N.m)/A        |
| Motor back-EMF constant    | km = 0.03          |    (V.s)/rad     |
| Motor armaturce resistance | Rm =               |   (ohm)          |

|           ----             |  Dr = 1            |        ----      | 
| Motor Armature Resistance  |   Dp1 = 1          |        ----      |


### 2.3 Motion Eqautions

This section is to establish a mathmatical connection between all parameters and actual motion code used to control the Furuta Pendulum Simulink and CoppeliaSim models. We shall cover the derivations of motion for the pendulum and rotary arms. The following equations were provided by Quanser workbook and have been tweaked slightly to create working matlab code. Brief descriptions of the equations will be stated above the equation image.


   
   <p align="center">
    Eq. 1 : Euler-Lagrange Equation
    <br>
  <img
       width=612
       height=116
       src="https://user-images.githubusercontent.com/104785921/168964314-175fbad0-ff7d-46d0-af66-45df029143f6.png"
  >
</p>



<p align="center">
    Eq. 2 : Generalized Polar Coordinates
  <img
       width=621
       height=90
       src="https://user-images.githubusercontent.com/104785921/168964761-589147dd-5a80-4de6-97bf-51672c79d8f7.png"
  >
</p>



<p align="center">
    Eq. 3 : Generalized Force on Arm (Theta)
  <img
       width=612
       height=89
       src="https://user-images.githubusercontent.com/104785921/168965306-bfeddff3-4d13-479d-a498-b0246f08375d.png"
  >
</p>

 

<p align="center">
    Eq. 4 : Generalized Force on Pendulum (Alpha)
  <img
       width=615
       height=103
       src="https://user-images.githubusercontent.com/104785921/168965471-db2f72a8-63d7-464a-8f63-7ed810c8e94b.png"
  >
</p>



<p align="center">
    Equations. (5-6) : Non-conservative forces acting on arm and pendulum respectively
  <img
       width=607
       height=192
       src="https://user-images.githubusercontent.com/104785921/168965840-ebca4c93-d595-4ecc-a6fc-3b5356b4e0da.png"
  >
</p>



<p align="center">
    Eq. 7 : Equation of Motion (Theta) for rotary Arm
  <img
       width=1000
       height=214
       src="https://user-images.githubusercontent.com/104785921/168976456-7059417a-9641-4c54-9a25-77985ec5160a.png"
  >
</p>



<p align="center">
    Eq. 8 : Equation of Motion (Alpha) for rotary Pendulum
  <img
       width=935
       height=175
       src="https://user-images.githubusercontent.com/104785921/168976702-fa84a9e1-d24e-4221-9ac1-2fbf34994877.png"
  >
</p>



<p align="center">
    Eq. 9 : Torque Equation
  <img
       width=686
       height=101
       src="https://user-images.githubusercontent.com/104785921/168977004-829036e9-9813-4861-88d3-e55ed5525fe6.png"
  >
</p>



<p align="center">
    Eq. 10 : Typical format of a Equation of Motion
  <img
       width=665
       height=65
       src="https://user-images.githubusercontent.com/104785921/168980696-282e93e6-3ea2-45bd-9e22-348609c08e4c.png"
  >
</p>



<p align="center">
    Eq. 11 : Generalized Eq. 10 to work with vector q
  <img
       width=785
       height=73
       src="https://user-images.githubusercontent.com/104785921/168981026-c7918b93-5a7b-4ada-ad07-5e08a826b287.png"
  >
</p>

Eq. 12 : 
--------------------------------------------------------------------------------
## 3 Simulation

### 3.1 Matlab

### 3.2 Simulink

--------------------------------------------------------------------------------
## 4 Implementation

--------------------------------------------------------------------------------
## 5 References

---------------------------------------------------------------------------------

