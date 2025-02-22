%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Modeling of IBC
% Arthur Tay
% Date: 22/8/2023
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function plantop = ibc_modelt23(t,T)
global u % control signal

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

% [ Plate tempeature
%     Wafer tempeature]

% A =
%    -0.1061    0.0503        0         0.0558         0                0
%     0.0168   -0.1086    0.0360         0          0.0558          0
%          0         0.0216   -0.0775        0             0            0.0558
%     0.1408         0             0        -0.1858     0.0392          0
%          0        0.1408         0         0.0131     -0.1878      0.0281
%          0             0        0.1408         0          0.0168      -0.1636

%  1.0e-03 * B =
%     0.2325         0         0
%          0    0.2325         0
%          0         0    0.2325
%          0         0         0
%          0         0         0
%          0         0         0