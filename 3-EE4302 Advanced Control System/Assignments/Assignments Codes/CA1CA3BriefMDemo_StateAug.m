% CA1CA3 Briefing MATLAB Demostration (State Augmentation)
clear
close all
%% ---------------------------------------------------------------------- %
% Formulate the Augmented State-Space Model

Fbar = [0,1,0;-1.10,-3.01,0;1,0,0]; % The process matrix for the augmented system
Gu = [0;1;0]; % The input matrix for u
Gr = [0;0;-1]; % The input matrix for r
Gv = [0;1;0]; % Used for Disturbance Analysis; The input matrix for disturbance v
Hbar = [1,0,0]; % The output matrix for y

%% ---------------------------------------------------------------------- %
% Design Using Ackermann's Formula

ITAEPoles = 3*[-0.7081;-0.5210+1.068*1i;-0.5210-1.068*1i];
K_ITAE = acker(Fbar,Gu,ITAEPoles);
sys_ag_ITAE = ss(Fbar-Gu*K_ITAE,Gr,Hbar,0);
% sys_ag_ITAE is already closed-loop system since 'F-GK' is used as the
% process matrix
% There is no need to use function 'feedback' to build the closed-loop
% system
figure(12)
bode(sys_ag_ITAE)
figure(13)
step(sys_ag_ITAE)
grid on

BesselPoles = 3*[-0.9420;-0.7455+0.7112*1i;-0.7455-0.7112*1i];
K_Bessel = acker(Fbar,Gu,BesselPoles);
sys_ag_Bessel = ss(Fbar-Gu*K_Bessel,Gr,Hbar,0);
figure(14)
bode(sys_ag_Bessel)
figure(15)
step(sys_ag_Bessel)
grid on

SODPoles = [-12;-3.6+2.7*1i;-3.6-2.7*1i];
K_SOD = acker(Fbar,Gu,SODPoles);
sys_ag_SOD = ss(Fbar-Gu*K_SOD,Gr,Hbar,0);
figure(16)
bode(sys_ag_SOD)
figure(17)
step(sys_ag_SOD)
grid on

%% ---------------------------------------------------------------------- %
% Design Using LQR

Q_LQR = [1,0,0;0,1,0;0,0,1];
R_LQR = 1;
[K_LQR,~,~] = lqr(Fbar,Gu,Q_LQR,R_LQR);

sys_ag_LQR = ss(Fbar-Gu*K_LQR,Gr,Hbar,0);
figure(18)
bode(sys_ag_LQR)
figure(19)
step(sys_ag_LQR)
grid on

sys_ag_LQR_Control = ss(Fbar-Gu*K_LQR,Gr,-K_LQR,0);
% In sys_ag_LQR_Control, Hbar is changed to -K_LQR
% In this case, MATLAB would treat -K_LQR*x as the output of the system,
% which is nothing else but the control signal
% Therefore, by doing a unit step response analysis, the control signal
% response can be displayed
figure(20)
step(sys_ag_LQR_Control)
grid on

%% ---------------------------------------------------------------------- %
% Disturbance Study

sys_ag_Disturb = ss(Fbar-Gu*K_LQR,Gv,Hbar,0);
% Gr is changed to Gv
% For sys_ag_Disturb, the input matrix is set Gv in order to study the
% system response when disturbance v comes in
% In this case study, the reference signal r is zero.
figure(21)
bode(sys_ag_Disturb)
figure(22)
step(sys_ag_Disturb)
grid on
sys_ag_Disturb_Control = ss(Fbar-Gu*K_LQR,Gv,-K_LQR,0);
figure(23)
step(sys_ag_Disturb_Control)
grid on







