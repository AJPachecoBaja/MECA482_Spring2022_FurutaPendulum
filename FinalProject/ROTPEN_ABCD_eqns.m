%%RUN setup_dbip.m 
%  Project_Project > Quasar > Advanced Application > Software > setup_dbip.m
%  NEED config_Sp.m   &  config_srv02.m
%function [K,N,info,Ep, Ek,K_swing1,K_swing2,mu,eps] = setup_FURPEN()

    % Set the electromechanical parameters
    eta_g = 0.85e0;
    eta_m = 0.87e0;
    Kg = 70;
    km = 0.76e-2;
    kt = 0.76e-2;
    Rm = 0.26e1;
    Vm = 0;
    
    % Set the rotary arm parameters
    Jp = 0.23e-2;
    Mp = 0.125e0;
    Lr = 0.215e0;
    Mr = 0.250;
    Lp = 0.335e0;
    Jr = 0.23e-2;
    Bp = 0.0e0;
    tau_1 = 0;
    g = 0.981e1;
    tau_2 = 0;
    Br = 0.0e0;
    
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
    
%end
Dr = 1;
Dp1 = 1;    % Viscous Damping coefficient (Short Bottom) Pendulum

% Jp            Pendulum moment of inertia                     (kg.m^2)
% Mp            Pendulum Mass with T-fitting                   (kg)
% Lr            Arm Lrngth (horizontal-rotational arm)         (m)
% Jr            Arm Moment of Inertia                          (kg.m^2)
% Lp            Full Length of the pendulum (w/ T-fitting)     (m)
% g             Gravitational constant                         (m/s^2)
% Br        Viscous damping coefficient as seen at the
%                rotary ARM  (Horizontal) axis                         (N.m.s/rad)
% Bp        Viscous damping coefficient as seen at the 
%               pendulum axis                                  (N.m.s/rad)
% Kg        Total gear ratio
% kt        Motor torque constant                               (N.m/A)
% km        Motor back-EMF constant                             (V.s/rad)
% Rm        Motor armaturce resistance                          (ohm)
% Mr        Mass of Rotary ARM                                  (kg)
% Dp        Desired Poles (-30 and -40 are given)
% -------------------------------------------------------------------------
% lp1            Distance from pivot to centre Of gravity       (m)
% lp2            Distance from pivot to centre Of gravity       (m)
% Jp            Pendulum moment of inertia                     (kg.m^2)
% RtpnOp        Used in Simulink model to identify rotpen option.
%               1 for potentiometer, 2 for encoder
% RtpnOff       Pendulum offset for starting inverted pendulum. (rad)
% K_POT_PEN     Potentiometer calibration gain for ROTPEN.      (rad/V)
% Rm        Motor armaturce resistance                          (ohm)
% eta_g     Gearbox efficiency
% eta_m     Motor efficiency
% Beq       Equivalent viscous damping coefficient w.r.t. load  (N.m.s/rad)
% Jm        Motor armature moment of inertia                    (kg.m^2)
% Jeq       Equivalent moment of inertia w.r.t. load            (kg.m^2)
% K_POT     Potentiometer Sensitivity                           (rad/V)
% K_TACH    Tachometer Sensitivity                              (rad/s/V)
% K_ENC     Encoder Resolution                                  (rad/count)
% K_amp : Amplifier GAIN
% VMAX_AMP  Amplifier Maximum Output Voltage                          (V)
% IMAX_AMP  Amplifier Maximum Output Current                          (A)

%%  Identify the System
%Jt = Jp*Mp*Lr^2+Jr*Jp+(1/4)*Jr*Mp*Lp^2
Jt = Jr*Jp + Mp*(Lp/2)^2*Jr + Jp*Mp*Lr^2;


A = [0 0 1 0; 
     0 0 0 1; 
     0 Lp ^ 2 * Lr * g * Mp ^ 2 / (4 * Jp * Lr ^ 2 * Mp + Jr * Lp ^ 2 * Mp + 4 * Jp * Jr) -(Lp ^ 2 * eta_g * eta_m * Kg ^ 2 * km * kt * Mp + 4 * Jp * eta_g * eta_m * Kg ^ 2 * km * kt + Br * Lp ^ 2 * Mp * Rm + 4 * Br * Jp * Rm) / Rm / (4 * Jp * Lr ^ 2 * Mp + Jr * Lp ^ 2 * Mp + 4 * Jp * Jr) -2 * Bp * Lp * Lr * Mp / (4 * Jp * Lr ^ 2 * Mp + Jr * Lp ^ 2 * Mp + 4 * Jp * Jr); 
     0 -2 / Rm / (4 * Jp * Lr ^ 2 * Mp + Jr * Lp ^ 2 * Mp + 4 * Jp * Jr) * (-Lp * Lr ^ 2 * g * Mp ^ 2 * Rm - Jr * Lp * g * Mp * Rm) -2 / Rm / (4 * Jp * Lr ^ 2 * Mp + Jr * Lp ^ 2 * Mp + 4 * Jp * Jr) * (Lp * Lr * eta_g * eta_m * Kg ^ 2 * km * kt * Mp + Br * Lp * Lr * Mp * Rm) -2 / Rm / (4 * Jp * Lr ^ 2 * Mp + Jr * Lp ^ 2 * Mp + 4 * Jp * Jr) * (2 * Bp * Lr ^ 2 * Mp * Rm + 2 * Bp * Jr * Rm);];
B = [0; 0; -(-Lp ^ 2 * eta_g * eta_m * Kg * kt * Mp - 4 * Jp * eta_g * eta_m * Kg * kt) / Rm / (4 * Jp * Lr ^ 2 * Mp + Jr * Lp ^ 2 * Mp + 4 * Jp * Jr); 2 / Rm / (4 * Jp * Lr ^ 2 * Mp + Jr * Lp ^ 2 * Mp + 4 * Jp * Jr) * Lp * Lr * eta_g * eta_m * Kg * kt * Mp;];
C = [1 0 0 0];
D = 0;


%%
close all

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

% Actuator dynamics
B = Kg * kt * B / Rm;
A(3,3) = A(3,3) - Kg^2*kt*km/Rm*B(3);
A(4,3) = A(4,4) - Kg^2*kt*km/Rm*B(4);
states(:) = {'theta' 'theta_dot' ' alpha' 'alpha_dot'};
States = reshape(states,[4,1])
inputs = {'u'};
outputs = {'theta';'alpha'};


%% Making the Open Loop
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