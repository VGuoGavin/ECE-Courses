%%  HVAC system for a 2-stories office building with 2 rooms in each floor
clc;
close all;
clear all;

Ti = 30;
% simulation approach two: variable step solver between control updates

Te = 110; 
Tf = 60*4;        %total simulation time (s)
Tu = 0.1;           %120;10;    %control update period (s)

%% plant (system) variables
Ns = 6; %number of states

% c=0.001;  %%capacitors(farads)
%% Defining A, B, C, and D matrices for state space representation

A =   [ -0.1061    0.0503         0         0.0558         0                0
          0.0168     -0.1086     0.0360        0           0.0558          0
                0         0.0216    -0.0775        0              0            0.0558
            0.1408         0              0        -0.1858     0.0392          0
                0         0.1408          0         0.0131     -0.1878      0.0281
                0              0         0.1408         0          0.0168      -0.1636 ];


B = 1.0e-03 *[  0.2325                 0                          0
                            0                    0.2325                     0
                            0                        0                      0.2325
                            0                        0                          0
                            0                        0                          0
                            0                        0                          0];
 
I = eye(6);

sys = struct('A',A, ...
    'B', B, ...
    'I', I);

t = 0:Tu:Tf; %times at which we sample system state and update control input
x = zeros(length(t),Ns); %system state at sampling times
x(1,:) = Ti * ones(1,Ns); %initial state of system

%% controller variables
Nu = 3; %number of inputs to system
u = zeros(Nu,1); %initial control input

% PI controller gains for HVAC output
kp1 = 7;     kp2 = 7;   kp3 = 7;
ki1 = 1;     ki2 = 1;   ki3 = 1; 

kp = [kp1;kp2;kp3];
ki = [ki1;ki2;ki3];

p = 0;              %proportional term of PI controller
pint = 0;        %integral term of PI controller

% derivative gains for decoupling controller

%% evaluate plant response
tic;
for i = 2:length(t)-60
    if i == 600 || i == 1200
        x(i-1,3:6) = 30;
        pint = 0;
    end
        % find plant response over previous sampling period
        f_w = @(T,Y)plant(T,Y,sys,u); %handle for ode solver (function it evaluates)
        
        [~,xds] = ode23(f_w, t(i-1:i+60), x(i-1,:)');
        x(i,:) = xds(end,:);
        
        % update control input for next sampling period
        e = (Te - x(i,:))'; %desired vs. actual
        %p = Der*e; %p(t)
        % calculate integral for PI controller: use discretized version from SATS project
        dt = t(i) - t(i-1);
        pint = pint + e*dt;    %integral(p,0,t)
        
        % find control input
        u =   kp.*e(4:6,1) + ki.*pint(4:6,1);
        
        umax = [1500,1500,1500]'; 
        umin = [-1500,-1500,-1500]';

        u = min(u,umax);
        u = max(u,umin);
    %     u = zeros(Ns,1);
        disp(u);
    %     null = input('...');
    
end
toc;
% x=G*u+K*dist;


figure;plot(t(1:1800),x(1:1800,:));hold on;

%plot(t,5*sin(2*pi*t/86400)+298.15);
xlabel('time [s]');
ylabel('Temperature [C]');
title('Temperature controller in the baking');
legend('Plate zone1','Plate zone2','Plate zone3', ...
    'wafer zone1','wafer zone2','wafer zone3');


function dx = plant(t,x,sys,u)
dx = sys.A*x + sys.B*u;
end