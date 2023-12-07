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

%% decoupling performance
syms s
for i = 1:4
    if C(1,:)*A^(i-1)*B ~= 0
        degree1 = i;
        break
    end
end

for i = 1:4
    if C(2,:)*A^(i-1)*B ~= 0
        degree2 = i;
        break
    end
end

B_star = [C(1,:)*A^(degree1-1)*B;
                C(2,:)*A^(degree2-1)*B];

target_poles1 = (s+2)*(s+3);
target_poly1 = expand(target_poles1);
%  s^2 + 5*s + 6
target_poles2 = (s+4)*(s+5);
target_poly2 = expand(target_poles2);
% s^2 + 9*s + 20

Phi_A1=A^2 + 5*A+6*eye(4);
Phi_A2=A^2 + 9*A+2*eye(4);
C_star = [C(1,:)*(A+2*eye(4));
                C(2,:)*(A+3*eye(4))];

F = inv(B_star);
K = F*C_star;
Bf=B*F;
Af = A-B*K;9
decouple_model=ss(Af, Bf, C, 0);
W=[Bf  Af*Bf  Af^2*Bf   Af^3*Bf];

%assert(rank(W)==4);

p=pole(decouple_model);
H=C*inv(s*eye(4)-Af)*Bf;
[~,eigenvalue] = eig(Af);

%% Plot
% non-zero inputs and zero initial state
figure()
step(decouple_model);
grid on

% zero inputs and x0 initial state
t=0:0.01:10;
len = size(t,2);
u0=zeros(len,2);
u1=[ones(len,1),zeros(len,1)];
u2=[zeros(len,1),ones(len,1)];
x0 = [0.5; -0.1; 0.1; -0.8];
[y,tout,x]=lsim(decouple_model,u1,t,x0);

figure()
plot(t,x)
grid on
legend('x1','x2','x3','x4','x5','x6')
xlabel('time')
ylabel('state')
title('zero inputs and x0 initial state')

figure()
plot(t,y)
grid on
legend('y1','y2')
xlabel('time')
ylabel('output')
title('zero inputs and x0 initial state')

