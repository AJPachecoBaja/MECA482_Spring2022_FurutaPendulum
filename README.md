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
    
- [3 Simulation](#3-Simulation)

    - 3.1 Matlab
    - 3.2 Simulink
 - [4 Implementation](#4-Implementation)
 - [5 Video Presentation](#5-Video Presentation)
 - [6 References](#6-References)
 
-------------------------------------------------------------
## 1 Introduction
A rotary (<i>inverted</i>) pendulum, also known as a Furuta Pendulum, is a non-linear, self-balancing control system. The basic design is created using two rotating masses connected to a base, this is a tool used far and wide to teach the basic principles of control systems designs.  The machine consists of a motor actuator connected to the base, an arm which is free to rotate around the horizontal axis, and an arm that is free to rotate around the vertical axis. The overall system goal is to balance the vertical arm, or pendulum, in an upright position by controlling the rotation of the motor about the horizontal axis. Shown below in Figure (1) is an example of a Furuta Pendulum.



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
## 2 Determining a Physical System

### 2.1 Sketch

Our team depicts the rotary pendulum in a freebody diagram.

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

By analyzing the figure above, the basic operation of the (<i>inverted</i>) Pendulum is controlled by two angles between the base-to-Rotary arm (Θ) and Rotary arm-to-Pendulum (α). The goal is to create script such that the two angles reach equilibrium and maintain a balanced pendulum in the upright position. Counterclockwise (CCW) motion is depicted as a positve change in angle (Θ) for the system, with this sign convention, voltage was applied to the motor. This resulted in a positive voltage equaling a positive angle.

To begin this project the group created operational and functional viewpoint diagrams, this is a key step to control system theory. With these diagrams we are able to get a sense of the physical, mechanical, and electrical model as a project overview. The diagrams are shown below.

<p align="center">
    Operational Viewpoint
     <br>
  <img
       width=757
       height=406
       src="https://user-images.githubusercontent.com/103708956/169402363-5b39790b-4f21-4e13-8e62-c6d75d5010b0.PNG"

            
            
  >
</p>

<p align="center">
    Functional Viewpoint
     <br>
  <img
       width=616
       height=539
       src="https://user-images.githubusercontent.com/104785921/169254229-862943d5-e514-473a-bb0d-2f478a26c9de.png"
  >
</p>

### 2.2 Parameters

|  Symbol  | Description  |
| -------------: | :------------- |
| kg = |    kilograms                 |
| m =  |    meter                     |
| s =  |    second                    |
| N =  |    Newton                    |
| A =  |     Area                     |
| Ω =  |    Ohm                       |
| V =  |    Volt                      |
| J =  |    Joule                     |


|       Parameter            |       Symbol       |      Units       |
|     :-------------:        |   :-------------:  |  :-------------: |
| Mass of Pendulum           |       Mp = 0.128   |         (kg)      |
| Pendulum Length            |        Lp = 0.316  |         (m)       | 
| Pendulum Moment of Inertia | Jp = 5.43*10^(-3)  |       kg*m^2      |
| Arm Length                 | Lr = 0.216         |       (m)         |
| Arm Moment of Inertia      | Jr = 20            |     kg*m^2        |
| Gravity                    | g = 9.81           |      m/s^2        |
| Total Gear Ratio           | Kg = 0.9           |        ----       |
| Motor Torque Constant      | kt = 0.00768       |     (N.m)/A       |
| Motor back-EMF constant    | km = 0.03          |    (V.s)/rad      |
| Motor armature resistance  | Rm =  2.6          |       (Ω)         |
| Mass of Rotary Arm         |  Mr = 0.250        |      (kg)         |
| Pendulum Viscious Friction coefficient    | Dr = 1                |        ----               | 
| Pendulum Viscous Damping coefficient      | Dp = 1                |        ----               |
| Gearbox Efficiency                        | eta_g =  0.85         |       (η)<sub>g</sub>     | 
| Motor Efficiency                          | eta_m =  0.87         |       (η)<sub>m</sub>     |
| Rotation of the motor                     | tau_1 = 0             |       (τ)<sub>1</sub>     |
| Rotation of the motor                     | tau_2 = 0             |      (τ)<sub>2</sub>      |
| Arm Viscious Friction coefficient         | Br = 0                |      ----                 |
| Pendulum Viscous Damping coefficient      | Bp = 0                |      ----                 |
| Potential Energy                          | Ep =  Mp * g * Lp     |        (J)                |







### 2.3 Motion Equations

This section is to establish a mathematical connection between all parameters and actual motion code used to control the Furuta Pendulum Simulink and CoppeliaSim models. We shall cover the derivations of motion for the pendulum and rotary arms. The following equations are used to create a working MATLAB environmental architecture. Brief descriptions of the equations are provided, preceding each respective image.


   
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
     <br>
  <img
       width=621
       height=90
       src="https://user-images.githubusercontent.com/104785921/168964761-589147dd-5a80-4de6-97bf-51672c79d8f7.png"
  >
</p>



<p align="center">
    Eq. 3 : Generalized Force on Arm (Theta)
     <br>
  <img
       width=612
       height=89
       src="https://user-images.githubusercontent.com/104785921/168965306-bfeddff3-4d13-479d-a498-b0246f08375d.png"
  >
</p>

 

<p align="center">
    Eq. 4 : Generalized Force on Pendulum (Alpha)
     <br>
  <img
       width=615
       height=103
       src="https://user-images.githubusercontent.com/104785921/168965471-db2f72a8-63d7-464a-8f63-7ed810c8e94b.png"
  >
</p>



<p align="center">
    Eq. 5 & Eq. 6 : Non-conservative forces acting on arm and pendulum respectively
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
     <br>
  <img
       width=686
       height=101
       src="https://user-images.githubusercontent.com/104785921/168977004-829036e9-9813-4861-88d3-e55ed5525fe6.png"
  >
</p>



<p align="center">
    Eq. 10 : Typical format of a Equation of Motion
     <br>
  <img
       width=665
       height=65
       src="https://user-images.githubusercontent.com/104785921/168980696-282e93e6-3ea2-45bd-9e22-348609c08e4c.png"
  >
</p>



<p align="center">
    Eq. 11 : Generalized Eq. 10 to work with vector 
    <br>
  <img
       width=785
       height=73
       src="https://user-images.githubusercontent.com/104785921/168981026-c7918b93-5a7b-4ada-ad07-5e08a826b287.png"
  >
</p>

<p align="center">
    Eq. 12 : Linearization of Equations of Motion for both Arm and Pendulum 
    <br>
  <img
       width=908
       height=88
       src="https://user-images.githubusercontent.com/104785921/169220386-41414de8-5061-4f1a-a1c4-aa37217737d4.png"
  >
</p> 

<p align="center">
    Eq. 12.1 : Suplemental equation for <b>Z</b><sup>T</sup> matrix in Eq. 12 
    <br>
  <img
       width=157
       height=80
       src="https://user-images.githubusercontent.com/104785921/169221844-11b8d7b9-1db3-4bc0-88d7-2667998c04eb.png"
  >
</p>

<p align="center">
    Eq. 12.2 : Suplemental equation for <b>Z</b><sub>o</sub><sup>T</sup> matrix in Eq. 12
    <br>
  <img
       width=139
       height=83
       src="https://user-images.githubusercontent.com/104785921/169222213-a0b92097-9d40-4e24-b81a-f69e6a8764be.png"
  >
</p>

<p align="center">
    Eq. 13 & Eq. 14 : Linear State Space Equations (Denoted with [A], [B], [C], and [D] matrices)
    <br>
  <img
       width=531
       height=99
       src="https://user-images.githubusercontent.com/104785921/169222550-efff155b-d8ee-40e1-b847-419c7716794b.png"
  >
</p>

<p align="center">
    Eq. 15: State of the Rotary Pendulum (<i>q</i>)
    <br>
  <img
       width=769
       height=88
       src="https://user-images.githubusercontent.com/104785921/169223314-b27dbefc-313b-44ce-8dfb-55dd4555ae38.png"
  >
</p>

<p align="center">
    Eq. 16: Output of the System (<i>q</i>)
    <br>
  <img
       width=780
       height=81
       src="https://user-images.githubusercontent.com/104785921/169224531-05e292cf-f880-4011-8e72-71acf4a02498.png"
  >
</p>

<p align="center">
    Eq. 17 & Eq. 18: C and D matrices, corresponding to the output equations (<i>q</i>)
    <br>
  <img
       width=646
       height=194
       src="https://user-images.githubusercontent.com/104785921/169225543-6e8367f5-ffd9-436f-9145-112891b51f87.png"
  >
</p>

<p align="center">
    Eq. 19 & Eq. 20: Linearized from Eq. 7 & Eq. 8 respectively. Initial conditions for all variables set to zero (<i>q</i>)
    <br>
  <img
       width=783
       height=178
       src="https://user-images.githubusercontent.com/104785921/169225847-ad597110-43df-4a95-a68a-cb72dc1687ef.png"
  >
</p>

<p align="center">
    Eq. 21 & Eq. 22: Displaying the Matrix and Determinite  (<i>q</i>)
    <br>
  <img
       width=681
       height=72
       src="https://user-images.githubusercontent.com/104785921/169226274-ac748590-4564-4365-996c-aa5e77d8d948.png"
       <br>
  <img
       width=775
       height=188
       src="https://user-images.githubusercontent.com/104785921/169226430-d0b0fb3f-0830-42bf-b32b-e40ffff103fe.png"
  >
</p>

<p align="center">
    Eq. 23 & Eq. 24: Calculation for the angular accelerations for the Arm and Pendulum (<i>q</i>)
    <br>
  <img
       width=786
       height=181
       src="https://user-images.githubusercontent.com/104785921/169232158-c44980cd-73c6-4057-aa29-04d63dcd0a8d.png"
  >
</p>

<p align="center">
    Eq. 25 & Eq. 26: Substitute x3 for (Θ) and x4 for (α) (<i>q</i>)
    <br>
  <img
       width=970
       height=272
       src="https://user-images.githubusercontent.com/104785921/169232728-e746d48a-9a3b-452c-b655-96390ac26fb2.png"
  >
</p>

<p align="center">
    <font size="5">Eq. 27: Final [A] and [B] matrices from state space equation Eq. 13 (<i>q</i>)</font>
    <br>
  <img
       width=782
       height=392
       src="https://user-images.githubusercontent.com/104785921/169233939-de70608c-c501-4fbd-a9ae-cf40e2ef7cc2.png"
  >
</p>

With this mathmatical model of the angular aceleration, state space equations/matrices, and linearized motions the matlab system models were then coded with the intent to connect to a CoppeliaSim model and operate synchronously with one another. The code written can be seen below in the next section.

--------------------------------------------------------------------------------
## 3 Simulation



 ### 3.1 Matlab

 
   #### Creating a System Matrix
   
 In this Matlab code, the final [A], [B], [C] and [D] matrices are calculated using Eq. 27, Eq. 17 and Eq. 18, to be used in the Steady State Transfer function. 
 
 The raw matlab code can be found here [ROTPEN_ABCD_eqns](https://github.com/AJPachecoBaja/MECA482_Spring2022_FurutaPendulum/blob/main/FinalProject/ROTPEN_ABCD_eqns.m)
 
 #### Creating and Modeling the Steady State Transfer function for the System</font>
 
 The ROTPEN_ABCD_eqns code uses supporting matlab functions [setup_dbip.m](https://github.com/AJPachecoBaja/MECA482_Spring2022_FurutaPendulum/blob/main/setup_dbip.m) and working in conjunction with the raw matlab code [FURPEN_SSR_eqns](https://github.com/AJPachecoBaja/MECA482_Spring2022_FurutaPendulum/blob/main/FinalProject/FURPEN_SSR_eqns.m)
 
 To create the Furuta Pendulum transfer function, with this model we are able to establish a connection between output and input of the overall system. The FURPEN_SSR_eqns supporting code is shown below.
 
    %This runs furuta pendulum model and sets up its state space representation
    FURPEN_SSR_eqns;
    
    % Transfer function system
    [num,den]=ss2tf(A,B,C,D);
    TF = tf(num, den);
    disp("Transfer Function Furuta Pendulum");
    TF
    
    disp("Zeros");
    disp(roots(num));
    disp("Poles");
    disp(roots(den));
    figure(134);

#### Modeling the System Response

    sys_ss = ss(A,B,C,D);
    % Converting control canonical form
    t = 0:0.1:10;
    u = ones(size(t,2),1);
    % Ramp Input - (Right now we dont need it)
    %u=t;
    u=sin(t);
    x0 = [ 0 0 0 0];
    X0_pendulum = [0 270 0 0];

    %[Y, T, X] = lsim(sys_ss,u,t)
        % Plotting the result   (Initial Results)
    figure; hold on;
    plot(t,u,'r')                       % Reference Signal
    %plot(t,Y,'m-.','linewidth',2);      % Systemn Response
    axis([-1.5 t(end) -.5 1.5]);
    legend('reference','response');

#### Consideration of Actuator Dynamics

    B = Kg * kt * B / Rm;
    A(3,3) = A(3,3) - Kg^2*kt*km/Rm*B(3);
    A(4,3) = A(4,4) - Kg^2*kt*km/Rm*B(4);
    states(:) = {'theta' 'theta_dot' ' alpha' 'alpha_dot'};
    States = reshape(states,[4,1])
    inputs = {'u'};
    outputs = {'theta';'alpha'};

#### Making the Open Loop

    % Open Loop System Model
    % The Requirements of the System
    zetaf_1 = 0.7;
    wn = 4;
    %     Desired Pole Location     (too fast so we can Neglect)
    d_p3 = -30;
    %     Desired Pole Location     (too fast so we can Neglect)
    d_p4 = -40; 
    % Converting control canonical form(Method 1)
    [numtf, dentf] = ss2tf(A, B, C, D);
    [Acc, Bcc, Ccc, Dcc] = tf2ss(numtf, dentf);
    % Alternativelty one can use similarity transformation
    %     State-Feedback GAIN
    K = control_FURPEN(Acc, Bcc, zetaf_1, wn);
    %K = control_FURPEN(Acc, Bcc, zetaf_1, wn, d_p3, d_p4);
    % Requirements are: Acc-matrix, Bcc-matrix, zetaf_1, wn-Omega_n
    % Using MATLAB code 'eig(A)' to find the open-loop poles of the system
    eig(A);
    %    Find Closed-Loop Poles
    eig(A-B*K)
    %mu = 2.3;
    %   Ep : Potential Energy
    Ep = Mp * g * Lp;
    % New sytem with closed-loop
    Anew = A-B*K;
    %       Find Closed Loop Poles
    eig(Anew)
    %K_ENC = 2 * pi / ( 4 * 1024 )

### 3.2 Simulink

   The Simulink program is a platform to create Model-Based, systen level design structures. With this we are able to simulate the Matlab code and verify our systems input/output values with auto generated graphs. Using a block diagram format the Simulink program is easy to understand and follow, our raw simulink file can be found at [s_rotpen_bal](https://github.com/AJPachecoBaja/MECA482_Spring2022_FurutaPendulum/blob/main/s_rotpen_bal.slx). Shown below is our final Simulink model diagram.
   
   <p align="center">
  <img
       width=1423
       height=368
       src="https://user-images.githubusercontent.com/104785921/169392955-8538fa66-713e-4f63-abe2-b20745965494.png"
  >
    </p>
    
   <br>
  
  
   Supporting structures for the Simulink diagram, seperate <b>Find X State</b> and <b>Scopes</b> block diagrams that were created, are illustrated below.
   
   <br>
   
   <p align="center">
    Find X State, Block Diagram:
    <br>
  <img
       width=882
       height=565
       src="https://user-images.githubusercontent.com/104785921/169409579-b9a5ea7c-0d19-46f2-91c4-d1156d9823ed.png"
  >
    </p>
<br>
<p align="center">
    Scopes, Block Diagram:
    <br>
  <img
       width=1001
       height=582
       src="https://user-images.githubusercontent.com/104785921/169409759-9b0d15b8-369f-4249-be6f-08d7378ace51.png"
  >
  
   
<br>
    <br>
    
    
  <p align="center">
  The output graphs of the Simulink testing and verification can be seen below:
   </p>
  <p align="center">
    Scopes, Output Graph:
    <br>
  <img
       width=750
       height=538
       src="https://user-images.githubusercontent.com/104785921/169410305-8beeeb0d-cad2-4d1e-857d-a2203eed6a8d.png"
  >
    </p>
    
  <p align="center">
    Theta (degrees) [Θ], Output Graph:
    <br>
  <img
       width=748
       height=383
       src="https://user-images.githubusercontent.com/104785921/169410420-daf9d4c7-7c30-4474-8bb6-dc05c4d00ad9.png"
  >
    </p>
    
  <p align="center">
    Alpha (degrees) [α] Output Graph:
    <br>
  <img
       width=784
       height=626
       src="https://user-images.githubusercontent.com/104785921/169410493-8d790e02-4ba5-4e92-a31f-7123692c3d08.png"
  >
    </p>
    
  <p align="center">
    Voltage (Vm), Output Graph:
    <br>
  <img
       width=901
       height=676
       src="https://user-images.githubusercontent.com/104785921/169410533-7feb0b31-f563-45de-b739-920b24979868.png"
  >
    </p>

--------------------------------------------------------------------------------
## 4 Implementation
    
### CoppeliaSim
    
This an industry-grade robotics simulator, allowing the team to fully simulate and implement our verified Matlab code to control a simplified model of a Furuta Pendulum. To establish a connection between the two programs we used a remote api function called B-Zero (b0). All of the B-Zero (b0) files can be found in the [FinalProject](https://github.com/AJPachecoBaja/MECA482_Spring2022_FurutaPendulum/tree/main/FinalProject) folder. Additionally within the same folder one can find the supporting CoppeliaSim scene [3D_Model_482Project_v20_CoppeliaFeedback.ttt](https://github.com/AJPachecoBaja/MECA482_Spring2022_FurutaPendulum/blob/main/FinalProject/3D_Model_482Project_v20_CoppeliaFeedback.ttt) and Matlab feedback scene [3D_Model_482Project_v20_Matlabfeedback.ttt](https://github.com/AJPachecoBaja/MECA482_Spring2022_FurutaPendulum/blob/main/FinalProject/3D_Model_482Project_v20_Matlabfeedback.ttt).
    
The Furuta Pendulum model seen below is made by connecting the Base, Arm, and Pendulum to two servo-actuator motors. With the code we created and tested we can control the model to self balance in an (<i>inverted</i>) upright position for an indefinite amount of time. [Coppelia-Furuta Control Balancing Video](https://github.com/AJPachecoBaja/MECA482_Spring2022_FurutaPendulum/blob/main/recording_2022_05_19-13_11-11.mp4) is a video recording has been slowed down and is being submitted for our finalized deliverable simulation.
    
   <p align="center">
        <img
             src="https://user-images.githubusercontent.com/104785921/169405956-3b06a414-db69-439a-a605-12414ce5c457.gif"       
     </p>

    
With a functioning simulation the project can be considered completed, although with the model that we have designed it would be possible to create a physical model of our system. The requirements for a functional physical model would include Sensor Calibration, manufacturing of parts (base, arm, and pendulumm), as well as selecting motors and servos to control the system
    
--------------------------------------------------------------------------------
## 5 Video Presentation
 <br>
    
   The raw mp4 file of our Project Video Presentation can be found [HERE](https://github.com/AJPachecoBaja/MECA482_Spring2022_FurutaPendulum/blob/main/SeanBlack_LaineWood_AndrewPacheco_MECA-482_Spring-2022.mp4) 

    
   
--------------------------------------------------------------------------------
## 6 References
    
Quanser, Rotary Inverted Pendulum, Retrieved by Jan, 27, 2020 from
https://www.quanser.com/products/rotary-inverted-pendulum/
    
Control System Tutorials for MATLAB and Simulink, Retrieved by Jan, 27, 2020 from
http://ctms.engin.umich.edu/CTMS/index.php?example=InvertedPendulum&section=SystemModeling
    
Kildare, R., Hansen E., Leon, E., PID Control of Furuta Pendulum, Control System Design Project Fall
2019

---------------------------------------------------------------------------------

