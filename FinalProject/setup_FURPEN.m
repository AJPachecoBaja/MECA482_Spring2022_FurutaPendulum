%This function compute the gain controller K and the scaling block N to use
%the system as a tracker of the reference signal which is the angle 
%theta(t) of the rotating arm

%% SETUP Rotpen
function [K,N,info,Ep, Ek,K_swing1,K_swing2,mu,eps] = setup_FURPEN()
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
    rlocus(TF)
    title("Root Locus Furuta Pendulum");
    %% Balance controller
    % The requirements of the system
    perc_OS = 10;   %10 Overshoot
    Ts = 3;       %1.5s Settling time
    disp("Desired Overshoot:");
    disp(perc_OS);
    disp("Desired Settling Time:");
    disp(Ts);
    %Specifications
    zeta = (-(log(perc_OS/100))/sqrt(pi^2+(log(perc_OS/100))^2));
    wn = 4/(zeta*Ts);
    sigma = zeta * wn;
    wd = wn*(1-zeta^2)^0.5;
    % Desired poles location
    p1 = -sigma+sqrt(-1)*wd;
    p2 = conj(p1);
    p3 = -sigma*10;
    % One of the excessive poles has a negative real part. So this poles
    % can be cancelled and pass from 4th to 3rd order system.
    p4 = -5.9473;
    % State-feedback gain setup
    K = control_FURPEN(A, B, p1, p2, p3, p4);
    %% Gain Block Input Tracker 
    disp(' ');
    disp('Control gains: ');
    disp(K);
    disp('Closed loop eigenvalues');
    disp(eig(A-B*K));
    [num,den]=ss2tf(A-B*K,B,C,D);
    TF = tf(num,den);
    DC = dcgain(TF);
    %Computing gain
    N = 1/DC;
    info = stepinfo(TF);
    disp("Transfer Function Closed Loop");
    TF
    disp("Zeros");
    disp(roots(num));
    disp("Poles");
    disp(roots(den));
    figure(223);
    rlocus(TF)
    title("Root Locus Closed Loop");
    disp("Closed Loop Overshoot");
    disp(info.Overshoot);
    disp("Closed Loop Settling Time");
    disp(info.SettlingTime);
    %% Energy Shaping Controller Setup
    eps = 0.2;
    mu = 25; % You choose this value
    K_swing1 = Rm*Mr*Lr/(eta_g*Kg*eta_m*kt);
    K_swing2 = Kg*km;
    Ek = 1/2*Jp;
    Ep = 1/2*Mp*g*Lp;
end
%% Plot the result function [K,N,info,Ep, Ek,K_swing1,K_swing2,mu,eps] = setup_FURPEN()

   

