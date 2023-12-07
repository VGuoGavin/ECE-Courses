% Q1
% My matriculation number is A0260014Y
clc
clear
close all

a = 0;  b = 0;  c = 1;  d = 4;

A = [ -8.8487+(a-b)/5,           -0.0399,                     -5.55+(c+d)/10,               3.5846;
        -4.574                             2.501*(d+5)/(c+5),   -4.3662,                           -1.1183-(a-c)/20 ;
        3.7698,                          16.1212-c/5,              -18.2103 + (a+d)/(b+4),  4.4936;
      -8.5645-(a-b)/(c+d+2),  8.3742,                     -4.4331 ,                          -7.7181*(c+5)/(b+5) ];

B = [0.0564+b/(10+c),                       0.0319;
        0.0165-(c+d-5)/(1000+20*a),   -0.02;
       4.4939,                                       1.5985*(a+10)/(b+12);
      -1.4269,                                     -0.273 ];
C = [-3.2988,              -2.1932+(10*c+d)/(100+5*a),  0.037,      -0.0109;
    0.2922-a*b/500,    -2.1506,                                      -0.0104,    0.0163];

% Reference second order system
% Overshoot less than 10%      exp(-pi* sigma)/sqrt(1-sigma^2) < 10%
% Settling time less than 20 seconds   4/(sigma*wn) <20s
syms wn sigma s
equ1 = exp((-pi* sigma)/sqrt(1-sigma^2)) == 10;
res = solve(equ1);
upper_bound =double(res(1));      % 0.5912
%lower_bound =double(res(2));   % -0.5912
%equ2 = sigma <  upper_bound;
%equ3 = sigma >  lower_bound;
sigma = 0.5;
equ4 = 4/(sigma*wn) <20;
%equs = [equ2, equ3, equ4];
res = solve(equ4);  % wn >= 0.4 let wn =0.4

% The reference second order mdoel is 0.16 / (s^2 + 0.4s + 0.16)
equ5 = s^2 + 0.4*s + 0.16 == 0;
poles  = solve(equ5);
% pole 1:  -0.2000 - 0.3464i
% pole 2:  -0.2000 + 0.3464i

%% openloop model
open_loop =  ss(A, B, C, 0); 
dcgain(open_loop);
step(open_loop)
open_gain = dcgain(open_loop);

%% 使用second-order reference model 模型
sys1 = ss(A, B, C, 0); 
Second_ref_poles = [-2, -2, -0.2000 - 0.3464i,  -0.2000 + 0.3464i]; % set the other two poles 5 times the domonator poles
K_SOD = place(A, B, Second_ref_poles); 

% B = Kx + Fr
% F is scalling gain, r is the reference signal
%r = [2 0;0 2;2 0;0 2];
Af=A-B*K_SOD;
sys_close=ss(Af, B, C, 0);  %Should be careful about the B here.
step(sys_close)
% close_gain = dcgain(sys_close);
% F = 1./close_gain;
% sys_feedback = F*sys;
step(sys_close)

t=0:0.1:40;
len = size(t,2);
u0=10*zeros(len,2);
u1=[ones(len,1),zeros(len,1)];
u2=[zeros(len,1),ones(len,1)];
u3=[ones(len,1),ones(len,1)];
% zero inputs and x0 initial state
x0 = [1; 1; 1; 1];
[y, tout, x]=lsim(sys_close, u0, t, x0);
%y  = y * F;
figure()
plot(t,x)
grid on
legend('x1','x2','x3','x4')
xlabel('time')
ylabel('state')
title('zero inputs and x0 initial state')

figure()
plot(t, y)
grid on
legend('y1','y2')
xlabel('time')
ylabel('output')
title('zero inputs and x0 initial state')



% sys_ag_ITAE = ss(A-B*K_SOD,  [0; 0; 0; 1], C, 0);
% DC_gain = dcgain(sys_ag_ITAE);
% dcgain(sys_ag_ITAE);
% F = 1./dcgain(sys_ag_ITAE);
% sys_feedback= F' * sys_ag_ITAE;
% step(sys_feedback)



