% Q1
% My matriculation number is A0260014Y
clc
clear
close all
%% full order poles placement
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
%% 
Wc = [B A*B A^2*B A^3*B];
disp(rank(Wc));
C_bar1 = [Wc(:,1), Wc(:,3),  Wc(:,5), Wc(:,7), Wc(:,2), Wc(:,4), Wc(:,6), Wc(:,8)];
C_bar2 = C_bar1(:,1:4);
C_bar_inv = inv(C_bar2);
q1T = C_bar_inv(1,:);
T = [q1T ;  q1T*A;  q1T*A^2;  q1T*A^3];
A_bar = T*A*inv(T);
B_bar = T*B;

syms k11 k12 k13 k14 k21 k22 k23 k24 s
equ = expand((s+2)*(s+2)*(s+0.2+0.3465i)*(s+0.2-0.3464i));
% close loop transfer matrix
K_bar = [1 2 3 4; k21 k22 k23 k24];
Feedback = simplify(A_bar - B_bar*K_bar);
p = simplify(charpoly(Feedback));
equ1 = p(2) == 22/6 ;
equ2 = p(3) == 14400069/2500000 ;
equ3 = p(4) == (1400069/625000*1.3);
equ4 = p(5) == (400069/625000 *0.8);
equs=[equ1, equ2, equ3, equ4];
res = (solve(equs));

k21 = double(res.k21);
k22 =double(res.k22);
k23 =double(res.k23);
k24 =double(res.k24);
K_bar = [1 2 3 4; k21 k22 k23 k24];
K = K_bar*T;

% Second_ref_poles = [-2, -2, -0.2000 - 0.3464i,  -0.2000 + 0.3464i]; % set the other two poles 5 times the domonator poles
% K = place(A, B, Second_ref_poles)

Af=A-B*K;
sys_close=ss(Af, B, C, 0);  %Should be careful about the B here.
step(sys_close)

t=0:0.1:40;
len = size(t,2);
x0 = [0; 0; 0; 0];
u0=10*zeros(len,2);
u1=[ones(len,1),zeros(len,1)];
u2=[zeros(len,1),ones(len,1)];
% zero inputs and x0 initial state
[y, tout, x]=lsim(sys_close, u1, t, x0);
figure()
plot(t, y)
grid on
legend('y1','y2')
xlabel('time')
ylabel('output')
title('Full order poles placement step response u1')



%% Unity Rank Method 
q = [4;6];
C2 = [B*q, A*B*q, A^2*B*q, A^3*B*q];
rank(C2) % 4
syms k1 k2 k3 k4
k = [k1 k2 k3 k4];
K = q*k;
Feedback_m = A - B*K;
ploy_f = charpoly(Feedback_m);

equ1 = ploy_f(2) == 22/6 ;
equ2 = ploy_f(3) == 14400069/2500000 ;
equ3 = ploy_f(4) == (1400069/625000 *1.3);
equ4 = ploy_f(5) == (400069/625000*0.8 );
equs=[equ1, equ2, equ3, equ4];
res = (solve(equs));
k1 = double(res.k1);
k2 =double(res.k2);
k3 =double(res.k3);
k4 =double(res.k4);
% 
K2 = q*[k1 k2 k3 k4];

sys1 = ss(A, B, C, 0); 

Af=A-B*K2;
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
title('Unity Rank Method step response u2')