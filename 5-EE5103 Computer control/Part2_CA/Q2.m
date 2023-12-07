clear
close all 
clc

N=10000;
Nrun=1; 
T=1;       
sw=0.1;
sv=1;

A=[1 T;0 1];
C=[1 0];
B=[T^2/2; T];

R1=sw^2*B*B';
R2=sv^2;
Pm(:,:,1)=1e5*eye(2);

for k=1:N
    Kf(:,k)=Pm(:,:,k)*C'*(C*Pm(:,:,k)*C'+R2)^(-1);
    K(:,k)=(A*Pm(:,:,k)*C')*(C*Pm(:,:,k)*C'+R2)^(-1);
    P(:,:,k)=Pm(:,:,k)-(Pm(:,:,k)*C')*(C*Pm(:,:,k)*C'+R2)^(-1)*C*Pm(:,:,k);
    Pm(:,:,k+1)=A*Pm(:,:,k)*A'+R1-K(:,k)*(C*Pm(:,:,k)*C'+R2)*K(:,k)';
end

w=random('normal',0,sw,Nrun,N);
v=random('normal',0,sv,Nrun,N);

for run=1:Nrun
    x_hatm(:,1)=[0 0]';
    x(:,1)=[0 30]';
    for k=1:N
        x(:,k+1)=A*x(:,k)+[T^2/2 T]'*w(run,k);
        y(k)=C*x(:,k)+v(run,k);
        x_hat(:,k)=x_hatm(:,k)+Kf(:,k)*(y(k)-C*x_hatm(:,k));
        x_hatm(:,k+1)=A*x_hatm(:,k)+K(:,k)*(y(k)-C*x_hatm(:,k));
    end
end

figure(4)
hold on
plot([0:N-1], x(1,1:N));
plot([0:N-1], x_hat(1,1:N), '.');
title('Graph 4')
label1 = '$x_1(k)$';
label2 = '$\hat{x_1}(k|k)$';
legend(label1,label2,'Interpreter','latex') % Add legend
hold off
grid
xlabel('k');
ylabel('x1');
xs(:,1)=x(:,N);
x_hats(:,1)=x_hat(:,N);


figure(5)
hold on
plot([0:N-1], x(2,1:N));
plot([0:N-1], x_hat(2,1:N), '.');
title('Graph 5')
label1 = '$x_2(k)$';
label2 = '$\hat{x_2}(k|k)$';
legend(label1,label2,'Interpreter','latex') % Add legend
xlabel('k');
ylabel('x2');
hold off
grid


figure(6)
P11=squeeze(P(1,1,:));
hold on
plot([0:N-1],P11,'x');
title('Graph 6 P11');
hold off
grid
xlabel('k');
ylabel('P11');

figure(7)
P22=squeeze(P(2,2,:));
hold on
plot([0:N-1],P22,'x');
title('Graph 7 P22');
hold off
grid
xlabel('k');
ylabel('P22');

figure(8)
hold on
plot([0:N-1],Kf(1,:),'x');
title('Graph 8 Kf1');
hold off
grid
xlabel('k');
ylabel('Kf1');

figure(9)
hold on
plot([0:N-1],Kf(2,:),'x');
title('Graph 9 Kf2');
hold off
grid
xlabel('k');
ylabel('Kf2');

S=x(:,[1:N])-x_hat(:,:);
Bias1=1/(N+1)*sum(S(1,:));
Var1=1/(N+1)*sum(S(1,:).^2);

Bias2=1/(N+1)*sum(S(2,:));
Var2=1/(N+1)*sum(S(2,:).^2);


disp("Bias1:"+ Bias1);
disp("Var1:"+ Var1);
disp("Bias2:"+ Bias2);
disp("Var2:"+ Var2);