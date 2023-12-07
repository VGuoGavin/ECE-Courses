close all
clear
clc

%% PI Control part
T_target = 150;           %Target temperature
T_surrending = 25;     % Surrending temperature
dt = 0.1;    % Simulation step length
t   = 180;   % Simulation duration
N = t/dt;

T = zeros(6,N);   % Initial temperature
v = zeros(6,N);   % The first derivative of temperature
e = zeros(6,N);   % Temperature error
tlist=1:N;

text=cell(1,6); %
line=cell(1,6); %
line{1}='k:';  text{1} = "Plate point 1";
line{2}='k-.'; text{2} = "Plate point 2";
line{3}='k-';  text{3} = "Plate point 3";
line{4}='k:';  text{4} = "Wafer point 1";
line{5}='k-.'; text{5} = "Wafer point 2";
line{6}='k-';  text{6} = "Wafer point 3";

T(:,1)=T_surrending;    %Initial surrounding temperature
u = [20,20,20]';              %Initial input power
e(:, 1)=T_target - T(:,1);
%hold on

Kp=[6, 6, 6]'; 
Ki=[0.19, 0.19, 0.19]'; 
%Kd = 0.010;
 
v(:, 1) = 1;
T(:, 2) = T(:, 1) + v(:, 1)*dt;
e(:, 2) = T_target - T(:,  2);  
u = u + diag( Kp*e(1:3, 2)') + diag(Ki*(e(1:3, 1) - e(1:3,  2))');    %+Kd*(e(2)-e(1))/dt;

v(:, 2) = ibc_modelt23(u', T(:, 2));
%     if lim_v==1       %Power limitation;
%         if v(2)>100
%             v(2)=100;
%         end
%         if v(2)<-100
%             v(2)=-100;
%         end
%     end
for k=3:N
    T(:,k) = T(:, k-1)+v(:, k-1)*2;
    if k == 400 || k==800 || k ==1200 || k == 1500
        T(4:6,k) = T_surrending;
    end
    e(:,k) = T_target - T(:, k);
    error = e(1:3, k) ;
    if k > 30
        for i =1:30
            error = error - e(1:3, k-i);
        end
    else
        for i =1:k-1
            error = error - e(1:3, k-i);
        end
    end
    u = u + diag(Kp * e(1:3, k)')+ diag(Ki * (error')); %+Kd*(e(k)-e(k-1))/dt;
    v(:,k) = ibc_modelt23(u', T(:,k));
end

plot(tlist,T(1:6,:))
%text{i}=['Kp=',num2str(Kp),' Ki=',num2str(Ki),' Kd=',num2str(Kd)];
%end

legend(text{1},text{2},text{3},text{4},text{5},text{6})
xlabel('Time')
ylabel('Temperature')
%set(gca, 'XTick',0:N/10:N,'XTicklabel',dt*[0:N/10:N]); 
grid on




%% ibc_modelt23 function
function plantop = ibc_modelt23(u,T)
%global u % control signal
q = u';
%Geometry
N = 3;			% number of zones
% The total number of states is 2N, the first N states are the plate temperature
% while the next N states are the wafer temperature

D = 0.3;		        % 300mm
tp = 1.778e-3;	% plate thickness
ta = 127e-6;	    % airgap (6mils)
tw = 0.926e-3;    % wafer thickness
dr = D/2/N;		    % zone thickness
ta = 125e-6*ones(1,N);

for i=1:N
   Aps(i) = 2*pi*i*dr*tp;
   Apz(i) = pi*(i*i - (i-1)*(i-1))*dr*dr;
   Aws(i) = 2*pi*i*dr*tw;
   Vp(i) = Apz(i)*tp;
   Vw(i) = Apz(i)*tw;
end

%Thermophysical Properties
% rho - density
% Cp - specific heat
% k - thermal conductivity
% Silicon
rho_si = 2330;
Cp_si = 790;
k_si = 99;

% Aluminum
rho_al = 2700;
Cp_al = 896;
k_al = 167;

% Quatz
rho_q = 2800;
Cp_q = 833;

% Air gap
k_a = 0.03;
ka = k_a;
rho_a = 1.1; 
Cp_a = 1000;
V_a = 10;
h = 10;

%Plate (aluminum)
kp = k_al;
Cpp = Cp_al;
rhop = rho_al;

%Wafer (silicon)
kw = k_si;
Cpw = Cp_si;
rhow = rho_si;

%Modeling
%Resistance and Capacitance 
for i=1:N-1
   Rpr(i) = log((i+1/2)/(i-1/2))/(2*pi*kp*tp);
   Rwr(i) = log((i+1/2)/(i-1/2))/(2*pi*kw*tw);
end

for i=1:N
   Ra(i) = ta(i)/(ka*Apz(i));
   Rwz(i) = 1/(h*Apz(i));
end
Rpr(N) = 1/(h*Aps(N));
Rwr(N) = 1/(h*Aws(N));

for i=1:N
   Cp(i) = rhop*Cpp*Vp(i);
   Cw(i) = rhow*Cpw*Vw(i);
end

%Energy Balance Equations
% In state-space format
% define A and B matrix here, C assumed to be I (all states available)
% A = |App Apw|
%       |Awp Aww|
App = zeros(N,N);		%note that App is tridiagonial
App(1,1) = -(1/Rpr(1) + 1/Ra(1))/Cp(1);
for i=2:N
   App(i,i) = -(1/Rpr(i) + 1/Rpr(i-1) + 1/Ra(i))/Cp(i);
   App(i,i-1) = (1/Rpr(i-1))/Cp(i);
end
for i=1:N-1
   App(i,i+1) = (1/Rpr(i))/Cp(i);
end
Apw = diag(1./(Ra.*Cp));  % will be different if the airgap is varying
Awp = diag(1./(Ra.*Cw));
Aww = zeros(N,N);		%note that Aww is tridiagonial
Aww(1,1) = -(1/Rwr(1) + 1/Ra(1) + 1/Rwz(1))/Cw(1);
for i=2:N
   Aww(i,i) = -(1/Rwr(i) + 1/Rwr(i-1) + 1/Ra(i) + 1/Rwz(i))/Cw(i);
   Aww(i,i-1) = (1/Rwr(i-1))/Cw(i);
end
for i=1:N-1
   Aww(i,i+1) = (1/Rwr(i))/Cw(i);
end

for i=1:N
   temp(i)=Apz(i)/Cp(i);
end
Bpp = diag(temp);
Bww = zeros(N,N);
%q = u2;
%Combining everything
A = [App Apw;Awp Aww];
B = [Bpp;Bww];
C = eye(2*N,2*N);
D = [zeros(N,N);zeros(N,N)];
sys = ss(A,B,C,D);
%step(sys);
%Tw = 100*ones(N,1);
%Tp = -(Aww/Awp)*Tw;
plantop = A*T + B*q;
end
