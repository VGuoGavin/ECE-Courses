% Q1
% My matriculation number is A0260014Y
% LQR approach
clc
clear
close all

a = 0;  b = 0;  c = 1;  d = 4;

A = [ -8.8487+(a-b)/5,           -0.0399,                     -5.55+(c+d)/10,               3.5846;
        -4.574                             2.501*(d+5)/(c+5),   -4.3662,                           -1.1183-(a-c)/20 ;
        3.7698,                          16.1212-c/5,               -18.2103 + (a+d)/(b+4),  4.4936;
      -8.5645-(a-b)/(c+d+2),  8.3742,                       -4.4331 ,                          -7.7181*(c+5)/(b+5) ];

B = [0.0564+b/(10+c),                       0.0319;
        0.0165-(c+d-5)/(1000+20*a),   -0.02;
       4.4939,                                       1.5985*(a+10)/(b+12);
      -1.4269,                                     -0.273 ];
C = [-3.2988,              -2.1932+(10*c+d)/(100+5*a),  0.037,      -0.0109;
    0.2922-a*b/500,    -2.1506,                                      -0.0104,    0.0163];

%%%%%
Q1=[1 0 0 0
   0 2 0 0 
   0 0 3 0 
   0 0 0 4];
% 
% Q2=[1 0 0 0
%    0 1 0 0 
%    0 0 1 0
%    0 0 0 1 ].*0.5;

R=[1 0 
   0 1];

Tau = [A  -B*inv(R)*B'; 
          -Q1  -A' ];



e = eig(Tau) ;
[V, D] = eig(Tau);
vectors = V(:,[1,2,6,8]); 
mu = vectors(5:8, :);
v = vectors(1:4, :);
P = mu*inv(v);
K_LQR = inv(R)*B'*P;

Af=A-B*K_LQR;
sys_close=ss(Af, B, C, 0);  %Should be careful about the B here.
%step(sys_close)

t=0:0.1:40;
len = size(t,2);
x0 = [0; 0; 0; 0];
u0=10*zeros(len,2);
u1=[ones(len,1),zeros(len,1)];
u2=[zeros(len,1),ones(len,1)];
% zero inputs and x0 initial state
[y, tout, x]=lsim(sys_close, u2, t, x0);
figure()
plot(t, y)
grid on
legend('y1','y2')
xlabel('time')
ylabel('output')
title('LQR control step response u2')
