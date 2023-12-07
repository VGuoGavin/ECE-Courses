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

%% Servo control + LQR
w = [0.3; 0.2];     

% verify controlbility
Qc = [A B;C zeros(2,2)];
assert(rank(Qc)==6);

A_bar=[A zeros(4,2); 
            -C zeros(2,2)];

B_bar=[B; zeros(2,2)];
B_w_bar=[B; zeros(4,2)];
B_r_bar=[zeros(4,2); eye(2)];
C_bar=[C, zeros(2,2)];

Q=[1 0 0 0 0 0;
      0 1 0 0 0 0;
     0 0 1 0 0 0;
    0 0 0 1 0 0;
   0 0 0 0 1 0;
  0 0 0 0 0 1]*10;

R=[1 0
     0 1]*1;

gamma=[A_bar -B_bar/R*(B_bar');-Q -A_bar'];

[eig_vector,eig_value]=eig(gamma);
eig_value_sum=sum(eig_value);
vueogen=eig_vector(:,real(eig_value_sum)<0);
P=vueogen(7:12,:)/vueogen(1:6,:);
K_calculated=real(inv(R)*(B_bar')*P);

K1=K_calculated(:,1:4);
K2=K_calculated(:,5:6);

%% full order Observer LQR method
% A_ba = A', B_ba = C', K_ba = L'
Qbar=[1 0 0 0 
      0 1 0 0 
      0 0 1 0 
      0 0 0 1]*5;
Rbar=[1 0 
       0 1 ]*1;
x0 = [0.5; -0.1; 0.1; -0.8];
Phi1 = [A',-C'/Rbar*C;-Qbar,-A];
[eig_vector_observed,eig_value_observed]=eig(Phi1);
eig_value_observed_sum=sum(eig_value_observed);
vueigen_observered=eig_vector_observed(:,real(eig_value_observed_sum)<0);
P_observed=vueigen_observered(5:8,:)/vueigen_observered(1:4,:);
Kbar=real(inv(Rbar)*C*P_observed);
L=Kbar';
L=real(L);

