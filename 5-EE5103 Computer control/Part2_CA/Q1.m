clear
close all
clc

N=10000;
Nrun=1; 
T=1;       
sw=0.1;
sv=1;

A=1;
C=1;
R1=sw^2;
R2=sv^2;
Pm(:,1)=1e5*eye(1);

for k=1:N
    Kf(1,k)=Pm(1,k)*C'*(C*Pm(1,k)*C'+R2)^(-1);
    K(1,k)=(A*Pm(1,k)*C')*(C*Pm(1,k)*C'+R2)^(-1);
    P(1,k)=Pm(1,k)-(Pm(1,k)*C')*(C*Pm(1,k)*C'+R2)^(-1)*C*Pm(1,k);
    Pm(1,k+1)=A*Pm(1,k)*A'+R1-K(1,k)*(C*Pm(1,k)*C'+R2)*K(1,k)';
end

w=random('normal',0,sw,Nrun,N);
v=random('normal',0,sv,Nrun,N);

x_hatm(:,1)=0;
x(:,1)=5;

for k=1:N
    x(1,k+1)=A*x(1,k)+w(1,k);
    y(k)=C*x(1,k)+v(1,k); 
    x_hat(1,k)=x_hatm(:,k)+Kf(1,k)*(y(k)-C*x_hatm(1,k));
    x_hatm(1,k+1)=A*x_hatm(1,k)+K(1,k)*(y(k)-C*x_hatm(1,k));
end

Pf=squeeze(P);
S=x(:,[1:N])-x_hat;
Bias=1/(N+1)*sum(S);
Var=1/(N+1)*sum(S.^2);


figure(1)
hold on
plot([0:N-1], x(1,1:N));
plot([0:N-1], x_hat(1,1:N), '.');
title('Graph 1')
label1 = '$x(k)$';
label2 = '$\hat{x}(k|k)$';
legend(label1,label2,'Interpreter','latex') % Add legend
hold off
grid
xlabel('k');
ylabel('x');
xs(:,1)=x(:,N);
x_hats(:,1)=x_hat(:,N);

figure(2)
plot([0:N-1],Pf,'x');
title('Graph 2 P(k|k)')
xlabel('k');
ylabel('P(k|k)');
grid

figure(3)
plot([0:N-1],Kf,'x');
title('Graph 3 Kf(k)')
xlabel('k');
ylabel('Kf(k)');
grid

disp("Bias:"+ Bias);
disp("Var:"+ Var);
