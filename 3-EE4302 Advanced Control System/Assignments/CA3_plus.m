% CA3 plus
clear
close all

% Formulate the Augmented State-Space Model
%% 对比宽带宽和窄带宽的输入响应
Fbar = [0,     1,   0;
      -1.20, -3.71, 0;
        1,     0,   0]; % The process matrix for the augmented system
Gu = [0; 1; 0]; % The input matrix for u
Gr = [0; 0; -1]; % The input matrix for r
Gv = [0; 1; 0]; % Used for Disturbance Analysis; The input matrix for disturbance v
% 这里干扰为什么加在x2上面
%窄带宽
Hbar = [1, 0, 0]; % The output matrix for y
ITAEPoles_narrow = 1.5*[-0.7081; -0.5210+1.068*1i; -0.5210-1.068*1i];
K_ITAE_narrow = acker(Fbar,Gu,ITAEPoles_narrow);
sys_ag_ITAE_narrow = ss(Fbar-Gu*K_ITAE_narrow, Gr, Hbar, 0);
subplot(2,2,1)
bode(sys_ag_ITAE_narrow)
sys_ag_Disturb = ss(Fbar-Gu*K_ITAE_narrow, Gv, Hbar, 0);
sys_ag_ITAE_Control_narrow = ss(Fbar-Gu*K_ITAE_narrow,Gr,-K_ITAE_narrow,0);
% Gr is changed to Gv
% For sys_ag_Disturb, the input matrix is set Gv in order to study the
% system response when disturbance v comes in
% In this case study, the reference signal r is zero.
subplot(2,2,2)
title("input step response")
step(sys_ag_ITAE_Control_narrow)
% 宽带宽
ITAEPoles_wide = 3.5*[-0.7081; -0.5210+1.068*1i; -0.5210-1.068*1i];
K_ITAE_wide = acker(Fbar,Gu,ITAEPoles_wide);
sys_ag_ITAE_wide = ss(Fbar-Gu*K_ITAE_wide, Gr, Hbar, 0);
subplot(2,2,3)
bode(sys_ag_ITAE_wide)
sys_ag_Disturb_wide = ss(Fbar-Gu*K_ITAE_wide, Gv, Hbar, 0);
% Gr is changed to Gv
% For sys_ag_Disturb, the input matrix is set Gv in order to study the
% system response when disturbance v comes in
% In this case study, the reference signal r is zero.
sys_ag_ITAE_Control_wide = ss(Fbar-Gu*K_ITAE_wide,Gr, -K_ITAE_wide,0);
subplot(2,2,4)
title("input step response")
step(sys_ag_ITAE_Control_wide)

% 计算传输函数
[num, den] = ss2tf(Fbar-Gu*K_ITAE_narrow, Gv, Hbar, 0);
sys_tf = tf(num,den)


%% Additional exploration

% Open Loop System Analysis
F_Plant = [0, 1; -0.77, -3.51]; % Process Matrix
G_Plant = [0; 1.1]; % Input Matrix
H_Plant_State1 = [1,0]; % Output Matrix
% H_Plant_State2 = [0,1]; % Output Matrix Assuming x2 is the Output
sys1 = ss(F_Plant, G_Plant, H_Plant_State1, 0); % Use function 'ss' to establish state-space model
% sys2 = ss(F_Plant,G_Plant,H_Plant_State2,0);
figure(1) % Open a new figure window named 'figure 1'
% open loop bode plot
bode(sys1) % Bode plot from u to x1; Use function 'bode' to plot bode graph

Poles = eig(F_Plant); % The open-loop poles are the eigenvalue of the original plant process matrix
% Use function 'eig' to calculate the eigenvalue of a square matrix
disp('The open-loop system transfer function poles are located at:') 
display(Poles)

% 计算是否 controllable
ControllabilityMatrix = [G_Plant,F_Plant*G_Plant];
if (det(ControllabilityMatrix)==0)
    error('The original plant is not controllable.') 
    % An 'error' command will stop the MATLAB program and pop-up an error message
    % If the controllability matrix has determinant of zero, the plant is not controllable.
else
    disp("This system is controllable")
end

%% scaling gain
% ITAE Design Poles
% Design Using Ackermann's Formula
ITAEPoles = 3 * [-0.7071+0.7071*1i; -0.7071-0.7071*1i]; % ITAE Design Poles as a column vector
% In MATLAB, 'a+b*1i' is common way of defining complex value
K_ITAE = acker(F_Plant, G_Plant, ITAEPoles); 
% Use function 'acker' to calculate the controller gain using Ackermann's Formula
sys_fb_ITAE = ss(F_Plant-G_Plant*K_ITAE, G_Plant, H_Plant_State1, 0); 
Ks_2st = 1/dcgain(sys_fb_ITAE); % Calculate the steady state gain of the feedback system
% Use function 'dcgain' to calculate the steady state gain of a system
% The returned value of a function can be directly used as part of
% other calculations.
G_disturbance= [0; 0.1]; 
sys_fb_ITAE_dis = ss(F_Plant-G_Plant*K_ITAE, G_disturbance, H_Plant_State1, 0); 

sys_cl_2st_r = Ks_2st*sys_fb_ITAE_dis; 
% The feedback system together with the scaling gain Ks forms the final design of closed-loop system
% Note that sys_cl_ITAE is SINGLE output system with output y=x1
disp("ITAE Design Poles scaling gain:")
disp(Ks_2st)

figure(4)
bode(sys_cl_2st_r) % Bode plot from r to y
figure(5)
step(sys_cl_2st_r) % Unit step response from r to y; Use function 'step' to observe the step response of a system
grid on % Add grids to the step response figure

%% augment state variable
Fbar = [0,     1,   0;
      -0.77, -3.51, 0;
        1,     0,   0]; % The process matrix for the augmented system
Gu = [0; 1.1; 0]; % The input matrix for u
Gr = [0; 0; -1]; % The input matrix for r
 Gv = [0; -1; 0]; % Used for Disturbance Analysis; The input matrix for disturbance v
Hbar = [1, 0, 0]; % The output matrix for y

ITAEPoles = 3*[-0.7081; -0.5210+1.068*1i; -0.5210-1.068*1i];
K_ITAE = acker(Fbar,Gu,ITAEPoles);
sys_ag_ITAE = ss(Fbar-Gu*K_ITAE, Gr+Gv, Hbar, 0);
% sys_ag_ITAE is already closed-loop system since 'F-GK' is used as the
% process matrix
% There is no need to use function 'feedback' to build the closed-loop
% system
figure(12)
bode(sys_ag_ITAE)
figure(13)
step(sys_ag_ITAE)
grid on
title("Step response with disturbance")

%这里想转换成离散系统研究下
%[num, den] = ss2tf(F_Plant, G_Plant, H_Plant_State1, 0);
%open_sys_tf = tf(num, den);
%open_sys_tf_d = c2d(open_sys_tf, 0.1)
% sys2 = ss(F_Plant,G_Plant,H_Plant_State2,0);
%figure(1) % Open a new figure window named 'figure 1'
%bode(open_sys_tf_d) % Bode plot from u to x1; Use function 'bode' to plot bode graph
