% Example system matrices
A = [0.8 0;
     0.8  1];
B = [0.1; 0.1];
C = [0 1];
D = 0;

% Error covariance matrices
Q = diag([1 0]);
R = 0.1;

% Solve discrete-time algebraic Riccati equation
[P,K] = idare(A',C',Q,R,[],[]);
K = K';

% Check results satisfy eqn.s
assert(all(A*(P - P*C'*(C*P*C' + R)^-1*C*P)*A' + Q - P < 1e-14, [1 2]))
assert(all(K - A*P*C'*(C*P*C' + R)^-1 < 1e-14, [1 2]))

K