%%解方程
syms s x wn
eqn = 1/(2*x*sqrt(1-x^2)) == 2;
S = solve(eqn)

%Sigma = (-1/(3^(1/2) - 2))^(1/2)/2 = 0.9659
%Sigma = (1/(3^(1/2) + 2))^(1/2)/2 = 0.2588

x = 0.9659 % 0.2588
eqn2 = wn*(sqrt(1-2*x^2+sqrt(2-4*x^2+4*x^4))) == 3.5;
S = solve(eqn2)

% Wn = 5.1782
% Wn = 2.3657

x = 0.9659
wn = 5.1782

eqn3 = s^2 + 2*x*wn*s + wn^2 == 0;
S = solve(eqn3)

% poles: -5.0016 - 1.3407i
% poles: -5.0016 + 1.3407i

x = 0.2588
wn = 2.3657

eqn3 = s^2 + 2*x*wn*s + wn^2 == 0;
S = solve(eqn3)

% poles: -0.6122 + 2.2851i
% poles: -0.6122 - 2.2851i


%%解方程
syms sgm wn s
eqn1 = abs(sgm) < 1.8478;
eqn2 = abs(sgm) > 0.7654;
eqn3 = wn*(sqrt(1-2*sgm^2+sqrt(2-4*sgm^2+4*sgm^4))) > 3.5;
conds = [eqn1, eqn2, eqn3];
sol = solve(conds, [wn, sgm], 'ReturnConditions', true);
sol.sgm
S = solve(eqn3)

sgm = 0.8
wn = 4.0188
eqn3 = s^2 + 2*sgm*wn*s + wn^2 == 0;
S = solve(eqn3)


