%% Root locus analysis
clc
clear
close all

syms s z
K = [0.1, 0.3, 0.5, 1];
for i = 1 : length(K)
    k = K(i);
    equ = s^3+k*(s+1)^2== 0;
    poles = vpa(solve(equ, s), 4);
    pole1_r = real(poles(1)); 
    pole1_i = imag(poles(1)); 
    pole2_r = real(poles(2)); 
    pole2_i = imag(poles(2)); 
    pole3_r = real(poles(3)); 
    pole3_i = imag(poles(3)); 
    x = [pole1_r, pole2_r, pole3_r];
    y = [pole1_i, pole2_i, pole3_i];
    plot(x,y, '*', 'Linewidth', 2)
    hold on
end
legend({[[ 'K:'; 'K:'; 'K:'; 'K:'] ,num2str(K' )]});
%legend(num2str(K))
title('The different poles placements with different K')

%% gain
% K = 1
% when error input bigger than -1 and smaller than 1 the feedback transfer
% function is 1/s^2, or it will be 1/s
num = [1 2 1];
den = [1 1 2 1];
sys1 = tf(num,den);
DC_gain = dcgain(sys1);
t = linspace(0, 10, 200);
y = zeros(1,length(t));
y(1, 1:20) = DC_gain;
y(1, 21:200) = 1./t(21:200);
plot(t,y)
xlabel('error input')
ylabel('K')
title('The gain with different input error')

%% different input response

t = 0:0.1:20;  % 201 points
u = zeros(1, length(t));
I = [0.5 1 1.5 2];
for i = 1 : length(I)
    if I(i) <=1
        k = 1;
    else
        k = 1/I(i);
    end
    num = [k 2*k k];
    den = [1 k 2*k k];
    sys = tf(num,den);
    u = I(i) + u;
    y = lsim(sys,u,t);

    plot(t, y)
    hold on
end
legend({[[ 'Input:'; 'Input:'; 'Input:'; 'Input:'] ,num2str(I' )]});
%legend(num2str(K))
title('The step response with different input')


