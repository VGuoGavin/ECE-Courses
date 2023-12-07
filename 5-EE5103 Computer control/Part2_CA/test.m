clear all;
close all;
clc;

%% Init values
dT = 0.1; 
duration = 10;
sampleCnt = duration / dT;

sigma_l_x_real = 0.1;
sigma_l_y_real = 10;
eps = [sigma_l_x_real; sigma_l_y_real];
Q_ll_real = [sigma_l_x_real^2 0;0 sigma_l_y_real^2]; %Real cofactor matrix of measurements

sigma_w_x_real = 2;
sigma_w_y_real = -3;
w_real = [sigma_w_x_real; sigma_w_y_real];
Q_ww_real = [sigma_w_x_real^2 0;0 sigma_w_y_real^2]; %Real cofactor matrix of process error
  
x_real = [0; 0; 5; 5];

T = [1 0 dT 0; 0 1 0 dT;0 0 1 0; 0 0 0 1]; %Transition Matrix

%A priori estimated erros
sigma_l_x = 0.1;
sigma_l_y = 10;
Q_ll = [sigma_l_x^2 0;0 sigma_l_y^2];

sigma_w_x = 2;
sigma_w_y = -3;
w_err_factor = 1;
w = [sigma_w_x; sigma_w_y] * w_err_factor;
Q_ww = [sigma_w_x^2 0;0 sigma_w_y^2] * w_err_factor ^ 2;

%Init estimates
S = [0.5 * dT^2 0;0 0.5 * dT^2;dT 0;0 dT];
x_hat = [0; 0; 5; 5];
Q_xx_hat = zeros(4,4);

pos_real = zeros(sampleCnt , 2);
pos_meas = zeros(sampleCnt , 2);
pos_filter = zeros(sampleCnt , 2);

periodCnt = 1;
sharpChange = 0;
changeFrame = 50;

for t = 0 : dT : duration
    white_noise = randn;
    %Simulate real state vector
    x_real = T * x_real  + S * (w_real * white_noise);
    if(periodCnt == changeFrame)
        x_real(2) = x_real(2) + sharpChange;
    end
    pos_real(periodCnt, :) = x_real(1:2)';
    
    %Simulate measured positions
    A = [1 0 0 0; 0 1 0 0];
    l_t = A * x_real + eps * randn;
    pos_meas(periodCnt, :) = l_t;
     
    %Perform filter using estimated errors
    x_dash = T * x_hat + S * (w_real * white_noise);
    Q_xx_dash = T * Q_xx_hat * T' + S * Q_ww * S';
    
    d = l_t - A * x_dash; %Innovation
    Q_dd = Q_ll + A * Q_xx_dash * A';
    
    K = Q_xx_dash * A' * Q_dd^-1;
    x_hat = x_dash + K * d;
   % Q_xx_hat = Q_xx_dash - K * Q_dd * K';
    Q_xx_hat = (eye(4) - K * A) * Q_xx_dash;
    
    pos_filter(periodCnt, :) = x_hat(1:2)';
    
    periodCnt = periodCnt + 1;
end
figure, plot(pos_real(:,1),pos_real(:,2),'-xr');
hold on; 
plot(pos_meas(:,1),pos_meas(:,2),'-xb');
plot(pos_filter(:,1),pos_filter(:,2),'-xg');
grid on;
legend('Model','Measured','Filtered','Location','best');