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
    mr = 0.250;
    Lp = 0.335e0;
    Jr = 0.23e-2;
    Bp = 0.0e0;
    tau1 = 0;
    g = 0.981e1;
    tau2 = 0;
    Br = 0.0e0;
% State Space Representation
A = [0 0 1 0; 
     0 0 0 1; 
     0 Lp ^ 2 * Lr * g * Mp ^ 2 / (4 * Jp * Lr ^ 2 * Mp + Jr * Lp ^ 2 * Mp + 4 * Jp * Jr) -(Lp ^ 2 * eta_g * eta_m * Kg ^ 2 * km * kt * Mp + 4 * Jp * eta_g * eta_m * Kg ^ 2 * km * kt + Br * Lp ^ 2 * Mp * Rm + 4 * Br * Jp * Rm) / Rm / (4 * Jp * Lr ^ 2 * Mp + Jr * Lp ^ 2 * Mp + 4 * Jp * Jr) -2 * Bp * Lp * Lr * Mp / (4 * Jp * Lr ^ 2 * Mp + Jr * Lp ^ 2 * Mp + 4 * Jp * Jr); 
     0 -2 / Rm / (4 * Jp * Lr ^ 2 * Mp + Jr * Lp ^ 2 * Mp + 4 * Jp * Jr) * (-Lp * Lr ^ 2 * g * Mp ^ 2 * Rm - Jr * Lp * g * Mp * Rm) -2 / Rm / (4 * Jp * Lr ^ 2 * Mp + Jr * Lp ^ 2 * Mp + 4 * Jp * Jr) * (Lp * Lr * eta_g * eta_m * Kg ^ 2 * km * kt * Mp + Br * Lp * Lr * Mp * Rm) -2 / Rm / (4 * Jp * Lr ^ 2 * Mp + Jr * Lp ^ 2 * Mp + 4 * Jp * Jr) * (2 * Bp * Lr ^ 2 * Mp * Rm + 2 * Bp * Jr * Rm);];
     
B = [0; 0; -(-Lp ^ 2 * eta_g * eta_m * Kg * kt * Mp - 4 * Jp * eta_g * eta_m * Kg * kt) / Rm / (4 * Jp * Lr ^ 2 * Mp + Jr * Lp ^ 2 * Mp + 4 * Jp * Jr); 2 / Rm / (4 * Jp * Lr ^ 2 * Mp + Jr * Lp ^ 2 * Mp + 4 * Jp * Jr) * Lp * Lr * eta_g * eta_m * Kg * kt * Mp;];
C = [1 0 0 0];
D = 0;

% Load into state-space system
sys_FURPEN_ol = ss(A,B,C,D); % Open loop system model
