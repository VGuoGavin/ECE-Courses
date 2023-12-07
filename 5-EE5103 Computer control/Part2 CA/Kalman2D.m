clc
clear
close all

N = 25;
theta = pi / 12;
A = [cos(theta) -sin(theta); 
        sin(theta) cos(theta)];

R1 = [1 0; 
         0 1];  % process uncertainty %%
R2 = [1 0; 
         0 1];  % measurement uncertainty
C = eye(2);
Pm(:,:,1) = 1e5*eye(2); %P(N|N-1) %%
% kalman gain and estimate uncertainty

for k = 1:24
    r_x(k) = 10*cos(k*pi/12);
    r_y(k) = 10*sin(k*pi/12);
end
plot(r_x, r_y, 'o')

for k=1: N
    Kf(:,:, k) = Pm(:, :, k) * C' * (C * Pm(:, :, k) * C' + R2) ^ (-1);
    K(:,:,k)=(A*Pm(:,:,k)*C')*(C*Pm(:,:,k)*C'+R2)^(-1);
    %K(:,:, k) = A * Kf(:,:, k);  
    P(:, :, k) = Pm(:, :, k) - Pm(:, :, k) * C' * (C * Pm(:, :, k) * C' + R2) ^ (-1) * C * Pm(:, :, k);
    Pm(:, :, k+1) = A * Pm(:, :, k)*A' - K(:,:, k)*(C*Pm(:, :,k)*C' + R2)*K(:,:, k)' + R1;
end
w = random('normal', 0, 0, 1, N);
v = random('normal', 0, 1, 2, N);
x(:, 1) = [10; 0];
xhm(:, 1) = [10; 0];%%
% put measurement to y
y=[7.1165 9.6022 8.9144 9.2717 6.3400 4.0484 0.3411 -0.6784 -5.7726 -5.4925 -9.4523 -9.7232 -9.5054 -9.7908 -7.7300 -5.9779  -4.5535 -1.5042 -0.7044 3.2406 8.3029 6.1925 9.1178 9.0904 9.0662;
    0.000 3.1398 6.3739 9.5877 10.1450 10.1919 9.0683 10.2254 7.5799 7.7231 5.4721 3.3990 0.9172 -1.3551 -5.2708 -9.7011 -9.4256 -9.3053 -9.3815 -9.8822 -8.1876 -8.7501 -4.5653 -1.9179 -1.0000];

T=1;   
for k=1: N
    % x(:, k+1) = A*x(:, k) + [1; 1]*w(:, k);
    %x(:, k+1) = A*x(:, k);
    x(:, k+1)=A*x(:,k) + [1; 1]*w(:,k);
    %yh(:, k)=C*x(:,k)+v(1,k);
    xh(:, k) = xhm(:, k) + Kf(:,:, k)*(y(:,k)-C*xhm(:,k));
    xhm(:, k+1) = A*xhm(:, k) + K(:,:, k)*(y(:, k)-C*xhm(:, k));
end

figure(1)
hold on
% plot([0:N-1],x(1,1:N),'x');
% plot([0:N-1],x(2,1:N),'o');
plot(xh(1,1:N),xh(2,1:N),'-');
plot(xh(1,1:N),xh(2,1:N),'rx', 'MarkerSize',10);
plot(r_x, r_y, 'o')
%plot(x(1,1:N),x(2,1:N),'o');
plot(y(1,:), y(2,:), '.b', 'MarkerSize',12);
xlim([-12 12])
ylim([-12 12])

% biases:
num = 0;
for k = 1: N
    %num = num + 10 * cos(2 * pi * (k-1) / (N-1)) - xh(1, k);
    num = num + x(1,k) - xh(1,k);
end
y_bias = num / N;
disp(y_bias)

num = 0;
for k = 1: N
    num = num + x(2,k) - xh(2,k);
end
z_bias = num / N;
disp(z_bias)

% variance:
num = 0;
for k = 1: N
    num = num + (x(1,k) - xh(1,k))^2;
end
y_variance = num / N;
disp(y_variance)

num = 0;
for k = 1: N
    num = num + (x(2,k) - xh(2,k))^2;
end
z_variance = num / N;
disp(z_variance)




Kf11= squeeze(Kf(1,1,1:N));
Kf22= squeeze(Kf(2,2,1:N));

figure(2)


subplot(1,2,1)
plot([0:N-1],Kf11,'x');
grid
xlabel('k');
ylabel('Kf11');

subplot(1, 2, 2)
plot([0 : N-1], Kf22, 'x');
grid
xlabel('k');
ylabel('Kf21');

%%%%%%

K11= squeeze(K(1,1,1:N));
K22= squeeze(K(2,2,1:N));

figure(3)

subplot(1,2,1)
plot([0:N-1],K11,'x');
grid
xlabel('k');
ylabel('K11');

subplot(1,2,2)
plot([0:N-1],K22,'x');
grid
xlabel('k');
ylabel('K22');

%%%%%%

P11= squeeze(P(1,1,1:N));
P22= squeeze(P(2,2,1:N));

figure(4)
subplot(1,2,1)
plot([0:N-1],P11,'x');
grid
xlabel('k');
ylabel('P11');

subplot(1,2,2)
plot([0:N-1],P22,'x');
grid
xlabel('k');
ylabel('P22');

%%%%%%

Pm11= squeeze(Pm(1,1,1:N));
Pm22= squeeze(Pm(2,2,1:N));

figure(5)
subplot(1,2,1)

plot([0:N-1],Pm11,'x');
grid
xlabel('k');
ylabel('Pm11(k+1|k)');

subplot(1,2,2)
plot([0:N-1],Pm22,'x');
grid
xlabel('k');
ylabel('Pm22(k+1|k)');

