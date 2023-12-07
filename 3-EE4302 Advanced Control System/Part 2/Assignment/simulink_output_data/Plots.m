%% SLIDING MODE CONTROL DESIGN
% gTruth0=evalin('base', 'out_2.mat');
% signal0 = gTruth0.Phase.signals;
% data0_1 = signal0(1).values;
% data0_2 = signal0(2).values;
% ctr_input0 = signal0(3).values;
% save('out_2')
% 
% gTruth1=evalin('base', 'out_1_5');
% signal1 = gTruth1.Phase.signals;
% data1_1 = signal1(1).values;
% data1_2 = signal1(2).values;
% ctr_input1 = signal1(3).values;
% save('out_1_5')
% 
% gTruth2=evalin('base', 'out_1');
% signal2 = gTruth2.Phase.signals;
% data2_1 = signal2(1).values;
% data2_2 = signal2(2).values;
% ctr_input2= signal2(3).values;
% save('out_1')
% 
% gTruth3=evalin('base', 'out_min1');
% signal3 = gTruth3.Phase.signals;
% data3_1 = signal3(1).values;
% data3_2 = signal3(2).values;
% ctr_input3 = signal3(3).values;
% save('out_min1')
% 
% gTruth4=evalin('base', 'out_min1_5');
% signal4 = gTruth4.Phase.signals;
% data4_1 = signal4(1).values;
% data4_2 = signal4(2).values;
% ctr_input4 = signal4(3).values;
% save('out_min1_5')
% 
% gTruth5=evalin('base', 'out_min2');
% signal5 = gTruth5.Phase.signals;
% data5_1 = signal5(1).values;
% data5_2 = signal5(2).values;
% ctr_input5 = signal5(3).values;
% save('out_min2')

load('out_2.mat')

N = length(data0_2);
t = linspace(0,10,N);

figure(1)
plot(linspace(0,10,length(data0_1)), data0_1)
hold on
plot(linspace(0,10,length(data1_1)), data1_1)
plot(linspace(0,10,length(data2_1)), data2_1)
plot(linspace(0,10,length(data3_1)), data3_1)
plot(linspace(0,10,length(data4_1)), data4_1)
plot(linspace(0,10,length(data5_1)), data5_1)
legend('x=-2', 'x=-1.5', 'x=-1', 'x=1', 'x=1.5', 'x=2')
title('x1')
hold off

figure(2)
plot(linspace(0,10,length(data0_2)), data0_2)
hold on
plot(linspace(0,10,length(data1_2)), data1_2)
plot(linspace(0,10,length(data2_2)), data2_2)
plot(linspace(0,10,length(data3_2)), data3_2)
plot(linspace(0,10,length(data4_2)), data4_2)
plot(linspace(0,10,length(data5_2)), data5_2)
legend('x=-2', 'x=-1.5', 'x=-1', 'x=1', 'x=1.5', 'x=2')
hold off
title('x2')

figure(3)
plot(data0_1, data0_2)
hold on
plot(data1_1, data1_2)
plot(data2_1, data2_2)
plot(data3_1, data3_2)
plot(data4_1, data4_2)
plot(data5_1, data5_2)
legend('x=-2', 'x=-1.5', 'x=-1', 'x=1', 'x=1.5', 'x=2')
hold off
title('Phase trajectory')

figure(4)
plot(linspace(0,10,length(ctr_input0)), ctr_input0)
hold on
plot(linspace(0,10,length(ctr_input1)), ctr_input1)
plot(linspace(0,10,length(ctr_input2)), ctr_input2)
plot(linspace(0,10,length(ctr_input3)), ctr_input3)
plot(linspace(0,10,length(ctr_input4)), ctr_input4)
plot(linspace(0,10,length(ctr_input5)), ctr_input5)
legend('x=-2', 'x=-1.5', 'x=-1', 'x=1', 'x=1.5', 'x=2')
hold off
title('Control input')