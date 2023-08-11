%% Extract all the topics of interest from ROSBAG
% % requirements - 
% 
% ROS toolbox
% 
% Mapping toolbox
% 
% rosbagReader.m script in same location as this file

clc
clear
% Each bag can be passed directly and named after the experiment
% the output of custom rosbagReader function is to generate dataset of all
% topics and combine all the readings in entire rosbag under one header


% Linear Velocity : 0.1 m/s

for i=1:50
    straight(i) =  rosbagReader(['D:\husky_sysID\dataset_batteryID\battery_id\test' '_' num2str(i) '.bag']);
end







%% 
% 
% 
% *INPUT : Command Velocity*
for i=1:50
    cmd(i).linX = straight(i).husky_velocity_controller_cmd_vel.Linear.X;
    Reltime(i).cmd_vel = straight(i).husky_velocity_controller_cmd_vel.RelTime;
end
%cmd.angZ = clothoid_1.husky_velocity_controller_cmd_vel.Angular.Z;


fcmd = figure();
subplot(1,2,1)
plot(Reltime,cmd.linX)
title('cmd lin X')
xlabel('Time [s] ')
ylabel('Linear velocity (m/s)')

% subplot(1,2,2)
% plot(Reltime,cmd.angZ)
% title('cmd ang Z')
% xlabel('Time [s] ')
% ylabel('Angular velocity(rad/s)')
% set(fcmd,'Units','normalized','Position',[0 0 0.5 0.4])
%%

%% 
% *Realized IMU Data*

for i=1:50
    acc(i).X = straight(i).gx5_imu_data.LinearAcceleration.X;
    acc(i).Y = straight(i).gx5_imu_data.LinearAcceleration.Y;
    acc(i).Z = straight(i).gx5_imu_data.LinearAcceleration.Z;
    ang(i).X = straight(i).gx5_imu_data.AngularVelocity.X;
    ang(i).Y = straight(i).gx5_imu_data.AngularVelocity.Y;
    ang(i).Z = straight(i).gx5_imu_data.AngularVelocity.Z;
    Reltime(i).imu = straight(i).gx5_imu_data.RelTime;
end
% reltime is relative time added to original struct it contains all
% timestamp - timestamp at 1 sec 

figure
for i=1:10
    subplot(2,5,i)
    plot(Reltime(5*i).imu,acc(5*i).X)
end
title('Linear Acc X')
xlabel('Time [s] ')
ylabel('Acceleration [m/s2]')

figure
for i=1:10
    subplot(2,5,i)
    plot(Reltime(5*i).imu,acc(5*i).Y)
end
title('Linear Acc Y')
xlabel('Time [s] ')
ylabel('Acceleration [m/s2]')

figure
for i=1:10
    subplot(2,5,i)
    plot(Reltime(5*i).imu,ang(5*i).Z)
end
title('Angular vel Yaw rate')
xlabel('Time [s] ')
ylabel('Acceleration [m/s2]')


f0 = figure();
subplot(2,3,1)
plot(Reltime,acc.X)
title('Linear Acc X')
xlabel('Time [s] ')
ylabel('Acceleration [m/s2]')

subplot(2,3,2)
plot(Reltime,acc.Y)
title('Linear Acc Y')
xlabel('Time [s] ')
ylabel('Acceleration [m/s2]')

subplot(2,3,3)
plot(Reltime,acc.Z)
title('Linear Acc Z')
xlabel('Time [s] ')
ylabel('Acceleration [m/s2]')

subplot(2,3,4)
plot(Reltime,ang.X)
title('Angular Vel X')
xlabel('Time [s] ')
ylabel('Angular Vel [rad/s]')

subplot(2,3,5)
plot(Reltime,ang.Y)
title('Angular Vel Y')
xlabel('Time [s] ')
ylabel('Angular Vel [rad/s]')

subplot(2,3,6)
plot(Reltime,ang.Z)
title('Angular Vel Z')
xlabel('Time [s] ')
ylabel('Angular Vel [rad/s]')

set(f0,'Units','normalized','Position',[0 0 1.5 1.5])

%% Wheel Encoder data processing

%flat_concrete.joint_states.Velocity
% Plotting each topic / any quantity becomes easier
for i=1:50
    wheel_enc(i).vel= straight(i).joint_states.Velocity;
    wheel_enc(i).pos= straight(i).joint_states.Position;
    Reltime(i).wheel_enc = straight(i).joint_states.RelTime;
end

% reltime is relative time added to original struct it contains all
% timestamp - timestamp at 1 sec 
for j=1:50
    for i=1:length(wheel_enc(j).vel)
        wheel1(j).vel(i) = wheel_enc(j).vel{i,1}(1,1);
        wheel2(j).vel(i) = wheel_enc(j).vel{i,1}(2,1);
        wheel3(j).vel(i) = wheel_enc(j).vel{i,1}(3,1);
        wheel4(j).vel(i) = wheel_enc(j).vel{i,1}(4,1); 
    end
end

for j=1:50
    for i=1:length(wheel_enc(j).pos)
        wheel1(i).pos(i) = wheel_enc(j).pos{i,1}(1,1);
        wheel2(i).pos(i) = wheel_enc(j).pos{i,1}(2,1);
        wheel3(i).pos(i) = wheel_enc(j).pos{i,1}(3,1);
        wheel4(i).pos(i) = wheel_enc(j).pos{i,1}(4,1); 
    end
end

figure
subplot(2,2,1)
for i = 1:10
    plot([wheel1(5*i).vel(:)])
    hold on
end
legend('0.1','0.2','0.3','0.4','0.5','0.6','0.7','0.8','0.9','1.0')
title('front left vel')

subplot(2,2,2)
for i = 1:10
    plot([wheel2(5*i).vel(:)])
    hold on
end
legend('0.1','0.2','0.3','0.4','0.5','0.6','0.7','0.8','0.9','1.0')
title('front right vel')

subplot(2,2,3)
for i = 1:10
    plot([wheel3(5*i).vel(:)])
    hold on
end
legend('0.1','0.2','0.3','0.4','0.5','0.6','0.7','0.8','0.9','1.0')
title('rear left vel')

subplot(2,2,4)
for i = 1:10
    plot([wheel4(5*i).vel(:)])
    hold on
end
legend('0.1','0.2','0.3','0.4','0.5','0.6','0.7','0.8','0.9','1.0')
title('rear right vel')

%%

figure
subplot(2,2,1)
for i = 1:10
    plot([wheel1(5*i).pos(:)])
    hold on
end
legend('0.1','0.2','0.3','0.4','0.5','0.6','0.7','0.8','0.9','1.0')
title('front left pos')

subplot(2,2,2)
for i = 1:10
    plot([wheel2(5*i).pos(:)])
    hold on
end
legend('0.1','0.2','0.3','0.4','0.5','0.6','0.7','0.8','0.9','1.0')
title('front right pos')

subplot(2,2,3)
for i = 1:10
    plot([wheel3(5*i).pos(:)])
    hold on
end
legend('0.1','0.2','0.3','0.4','0.5','0.6','0.7','0.8','0.9','1.0')
title('rear left pos')

subplot(2,2,4)
for i = 1:10
    plot([wheel4(5*i).pos(:)])
    hold on
end
legend('0.1','0.2','0.3','0.4','0.5','0.6','0.7','0.8','0.9','1.0')
title('rear right pos')



%%






figure
%for i=1:length(wheel_enc(3).vel)
    plot([wheel_enc(3).vel{:,1}(1,1)])
    %hold on
%end
title('Left front encoder : velocity')
%xlabel('Time [s] ')
ylabel('rad/s')


%%
% Battery voltage plot
  

figure
for i=1:10
    plot([straight(5*i).status.BatteryVoltage])
    hold on
end
title('Voltage')
%xlabel('Time [s] ')
ylabel('volt')
legend('0.1','0.2','0.3','0.4','0.5','0.6','0.7','0.8','0.9','1.0')

%%

%right left motor voltages

figure
for i=1:10
    plot([straight(5*i).status.LeftDriverVoltage])
    hold on
end
title('Left Voltage')
%xlabel('Time [s] ')
ylabel('volt')
legend('0.1','0.2','0.3','0.4','0.5','0.6','0.7','0.8','0.9','1.0')

figure
for i=1:10
    plot([straight(5*i).status.RightDriverVoltage])
    hold on
end
title('Right Voltage')
%xlabel('Time [s] ')
ylabel('volt')
legend('0.1','0.2','0.3','0.4','0.5','0.6','0.7','0.8','0.9','1.0')

%%


%right left motor Current

figure
for i=1:10
    plot([straight(5*i).status.LeftDriverCurrent])
    hold on
end
title('Left Current')
%xlabel('Time [s] ')
ylabel('Amp')
legend('0.1','0.2','0.3','0.4','0.5','0.6','0.7','0.8','0.9','1.0')

figure
for i=1:10
    plot([straight(5*i).status.RightDriverCurrent])
    hold on
end
title('Right Current')
%xlabel('Time [s] ')
ylabel('Amp')
legend('0.1','0.2','0.3','0.4','0.5','0.6','0.7','0.8','0.9','1.0')

%%

%right left motor Current

figure
for i=1:10
    plot([straight(5*i).status.LeftDriverCurrent])
    hold on
end
title('Left Current')
%xlabel('Time [s] ')
ylabel('Amp')
legend('0.1','0.2','0.3','0.4','0.5','0.6','0.7','0.8','0.9','1.0')

figure
for i=1:10
    plot([straight(5*i).status.RightDriverCurrent])
    hold on
end
title('Right Current')
%xlabel('Time [s] ')
ylabel('Amp')
legend('0.1','0.2','0.3','0.4','0.5','0.6','0.7','0.8','0.9','1.0')

%%


%right left driver temp

figure
for i=1:10
    plot([straight(5*i).status.LeftDriverTemp])
    hold on
end
title('Left Temp')
%xlabel('Time [s] ')
ylabel('Temp')
legend('0.1','0.2','0.3','0.4','0.5','0.6','0.7','0.8','0.9','1.0')

figure
for i=1:10
    plot([straight(5*i).status.RightDriverTemp])
    hold on
end
title('Right Temp')
%xlabel('Time [s] ')
ylabel('Temp')
legend('0.1','0.2','0.3','0.4','0.5','0.6','0.7','0.8','0.9','1.0')

%%

%right left motor temp

figure
for i=1:10
    plot([straight(5*i).status.LeftMotorTemp])
    hold on
end
title('Left Motor Temp')
%xlabel('Time [s] ')
ylabel('Temp')
legend('0.1','0.2','0.3','0.4','0.5','0.6','0.7','0.8','0.9','1.0')

figure
for i=1:10
    plot([straight(5*i).status.RightMotorTemp])
    hold on
end
title('Right Motor Temp')
%xlabel('Time [s] ')
ylabel('Temp')
legend('0.1','0.2','0.3','0.4','0.5','0.6','0.7','0.8','0.9','1.0')

%%


