% Lin vel : 0.4 m/s
skp_04_01 = rosbagReader('D:\Work\HuskySkidpadConfusion\test_04_01_2023-10-04-13-52-31.bag');

real_04_01 = skp_04_01.gx5_imu_data.AngularVelocity.Z;
sim_04_01 = csvread('confusion_sim/z_ang_04_01.csv');


%%
%%%%%%%%%%%%% Select data samples %%%%%%%%%%%%% 
% You need  to plot them to see which ones are constant for a test

% T1 
figure
plot(-1*real_04_01(1000:2000))
hold on
plot(sim_04_01(1:1000))
legend('real','sim')

%% How to get V

% for now use command V - 0.01
% Ex : if cmd V = 0.5 m/s , set V = 0.49

%% Get gap

gap_04_01 = getSimGap(sim_04_01(1:1000),-1*real_04_01(1001:2000),V);

gap = gap_04_01.mean;

