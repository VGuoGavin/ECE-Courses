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

F = [1 0;0 1];
%% evaluate LQR performance when changing Q and R 
Q = [0.2 2 4 10];
Q1=[1 0 0 0
   0 1 0 0 
   0 0 1 0 
   0 0 0 1].*2;
%R=[1 0
   %0 1].*0.5;

Q2=[1 0 0 0
   0 1 0 0 
   0 0 1 0
   0 0 0 1 ].*0.5;

R=[1 0 
   0 1];

[K_LQR,~,~] = lqr(A, B, Q1, R);
sys_ag_LQR = ss(A-B * K_LQR, B*F , C, 0);

t=0:0.1:40;
len = size(t,2);
x0 = [0; 0; 0; 0];
u0=10*zeros(len,2);
%u1=[ones(len,1),zeros(len,1)];
%u2=[zeros(len,1),ones(len,1)];
u3=[ones(len,1),ones(len,1)];
% zero inputs and x0 initial state
[y, tout, x]=lsim(sys_ag_LQR, u3, t, x0);


for i =1:4
    %[K_LQR,~,~] 
    K_LQR = lqr(A, B, Q2.*Q(i), R); 
    A_f=A-B*K_LQR;
    sys_close=ss(A_f, B, C, 0);  %Should be careful about the B here.
    [y, ~, x]=lsim(sys_close, u0, t, x0);

    subplot(2,2,i)
    plot(t, y)
    grid on
    legend('y1','y2')
    xlabel('time')
    ylabel('output')
    titles = {['Non-zero state zero input Q:' ,num2str(Q(i) )]};
    title(titles)
end

for i =1:4
    %[K_LQR,~,~] 
    K_LQR = lqr(A, B, Q2.*Q(i), R); 
    A_f=A-B*K_LQR;
    sys_close=ss(A_f, B, C, 0);  %Should be careful about the B here.
    [y, ~, x]=lsim(sys_close, u0, t, x0);

    subplot(2,2,i)
    plot(t,x)
    grid on
    legend('x1','x2','x3','x4')
    xlabel('time')
    ylabel('state')
    titles = {['Non-zero state zero input Q:' ,num2str(Q(i) )]};
    title(titles)
end


for i =1:4
    K_LQR =  lqr(A, B, Q2.*Q(i), R); 
    A_f=A-B*K_LQR;
    sys_close=ss(A_f, B, -K_LQR, 0);  %Should be careful about the B here.
    [y, ~, x]=lsim(sys_close, u0, t, x0);
    subplot(2,2,i)
    plot(t, y)
    grid on
    legend('u1','u2')
    xlabel('time')
    ylabel('state')
    titles = {['Non-zero state zero input Q:' ,num2str(Q(i) )]};
    title(titles)
end



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


%%
%% monitor control signal size


sys1 = ss(A, B, C, 0); 
Second_ref_poles = [-2, -2, -0.2000 - 0.3464i,  -0.2000 + 0.3464i]; % set the other two poles 5 times the domonator poles
K_SOD = place(A, B, Second_ref_poles); 
Af=A-B*K_SOD;
sys_close_ctr=ss(Af, B, -K_SOD, 0);  %Should be careful about the B here.
step(sys_close_ctr)

t=0:0.1:40;
len = size(t,2);
x0_zeros = [0; 0; 0; 0];
x0_non_zeros = [1; 1; 1; 1];
u0=10*zeros(len,2);
%u1=[ones(len,1),zeros(len,1)];
%u2=[zeros(len,1),ones(len,1)];
u3=[ones(len,1),ones(len,1)];
% zero inputs and x0 initial state
[y1, tout1, x1]=lsim(sys_close_ctr, u3, t, x0_zeros);
[y2, tout2, x2]=lsim(sys_close_ctr, u0, t, x0_non_zeros);

figure()
plot(t, y1)
grid on
legend('y1','y2')
xlabel('time')
ylabel('output')
title('Control signal under step reponse situation')

figure()
plot(t, y2)
grid on
legend('y1','y2')
xlabel('time')
ylabel('output')
title('Control signal under non zero state and zeros input')








%% change Q
gamma1=[A -B/R*B';  -Q1 -A'];
gamma2=[A -B/R*B';  -Q2 -A'];

[eig_vector1, eig_value1]=eig(gamma1);
eig_value1_sum=sum(eig_value1);
vueigen1=eig_vector1(:, real(eig_value1_sum)<0);

P1=vueigen1(7:12,:)/vueigen1(1:6,:);
K1=inv(R)*B'*P1;

[eig_vector2,eig_value2]=eig(gamma2);
eig_value2_sum=sum(eig_value2);
vueigen2=eig_vector2(:,real(eig_value2_sum)<0);
P2=vueigen2(7:12,:)/vueigen2(1:6,:);
K2=inv(R)*B'*P2;

%% PLOT figure
t=0:0.01:10;
Af1=A-B*K1;
Af2=A-B*K2;
sys1=ss(Af1,B,C,D);
sys2=ss(Af2,B,C,D);

len = size(t,2);
u0=zeros(len,2);
u1=[ones(len,1),zeros(len,1)];
u2=[zeros(len,1),ones(len,1)];

[y1,tout1,x1]=lsim(sys1,u0,t,x0);
figure()
plot(t,x1)
legend('x1 Q=50I','x2 Q=50I','x3 Q=50I','x4 Q=50I')
xlabel('time')
ylabel('state')
grid on
figure()
plot(t,y1)
legend('out1 Q=50I','out2')
grid on

%hold on
[y2,tout2,x2]=lsim(sys2,u0,t,x0);
figure()
plot(t,x2)
legend('x1 Q=0.5I','x2 Q=0.5I','x3 Q=0.5I','x4 Q=0.5I')
xlabel('time')
ylabel('state')
grid on
figure()
plot(t,y2)
legend('out1 Q=0.5I','out2')
grid on
