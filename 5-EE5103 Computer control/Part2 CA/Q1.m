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
Pm(:,:,1)=1e5*eye(1);

for k=1:N
    Kf(:,k)=Pm(:,:,k)*C'*(C*Pm(:,:,k)*C'+R2)^(-1);
    K(:,k)=(A*Pm(:,:,k)*C')*(C*Pm(:,:,k)*C'+R2)^(-1);
    P(:,:,k)=Pm(:,:,k)-(Pm(:,:,k)*C')*(C*Pm(:,:,k)*C'+R2)^(-1)*C*Pm(:,:,k);
    Pm(:,:,k+1)=A*Pm(:,:,k)*A'+R1-K(:,k)*(C*Pm(:,:,k)*C'+R2)*K(:,k)';
end

w=random('normal',0,sw,Nrun,N);
v=random('normal',0,sv,Nrun,N);

x_hatm(:,1)=0;
x(:,1)=5;

for k=1:N
    x(:,k+1)=A*x(:,k)+w(1,k);
    y(k)=C*x(:,k)+v(1,k); 
    x_hat(:,k)=x_hatm(:,k)+Kf(:,k)*(y(k)-C*x_hatm(:,k));
    x_hatm(:,k+1)=A*x_hatm(:,k)+K(:,k)*(y(k)-C*x_hatm(:,k));
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
