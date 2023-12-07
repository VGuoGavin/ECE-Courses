clc
clear
close all
%% full order poles placement

A = [ 0 1; -1.2  -3.71];
B = [1 0; 0 2 ];
C = [1  0];


% K_ITAE_narrow = acker(Fbar,Gu,ITAEPoles_narrow);
sys_open = ss(A, B, C, 0);
% bode(sys_open) 

t=0:0.1:40;
len = size(t,2);
x0 = [0; 0];
u0=10*zeros(len,2);
u1=[ones(len,1),zeros(len,1)];
u2=[zeros(len,1),ones(len,1)];
% zero inputs and x0 initial state
[y, tout, x]=lsim(sys_open, u1, t, x0);
figure(1)
subplot(121)
plot(t, y)
grid on
%legend('y1','y2')
xlabel('time')
ylabel('output')
title('Open loop step response u1')
subplot(122)
[y, tout, x]=lsim(sys_open, u2, t, x0);
plot(t, y)
grid on
%legend('y1','y2')
xlabel('time')
ylabel('output')
title('Open loop step response u2')


%% Unity Rank Method 

% ITAEPoles_narrow = 1.5*[-0.7081; -0.5210+1.068*1i; -0.5210-1.068*1i];
q = [1; 2];
C2 = [B*q, A*B*q];
rank(C2) % 4
syms k1 k2
k = [k1 k2];
K = q*k;
Feedback_m = A - B*K;
ploy_f = charpoly(Feedback_m);

equ1 = ploy_f(2) == 3 ;
equ2 = ploy_f(3) == 3 ;

equs=[equ1, equ2];
res = (solve(equs));
k1 = double(res.k1);
k2 =double(res.k2);

K2 = q*[k1 k2];

sys1 = ss(A, B, C, 0); 

Af=A-B*K2;
sys_close=ss(Af, B, C, 0);  %Should be careful about the B here.
%bode(sys_close)
%step(sys_close)

t=0:0.1:40;
len = size(t,2);
x0 = [0; 0];
u0=10*zeros(len,2);
u1=[ones(len,1),zeros(len,1)];
u2=[zeros(len,1),ones(len,1)];
% zero inputs and x0 initial state
[y, tout, x]=lsim(sys_close, u1, t, x0);
figure(1)
subplot(121)
plot(t, y)
grid on
%legend('y1','y2')
xlabel('time')
ylabel('output')
title('Unity order poles placement step response u1')
subplot(122)
[y, tout, x]=lsim(sys_close, u2, t, x0);
plot(t, y)
grid on
%legend('y1','y2')
xlabel('time')
ylabel('output')
title('Unity order poles placement step response u2')

