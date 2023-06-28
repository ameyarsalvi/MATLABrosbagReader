function [fig1,fig2, fig3] = plotImuData(inputStructCell)
numPlots = numel(inputStructCell);
fig1 = figure;
hold on;
title('LinearAccel X [m/s^2]');
grid on;
xlabel('relative timestamp in secs');
ylabel('linear Acc X [m/s^2]');

fig2 = figure;
hold on;
title('LinearAccel Y [m/s^2]');
grid on;
xlabel('relative timestamp in secs');
ylabel('linear Acc Y [m/s^2]');

fig3 = figure;
hold on;
title('LinearAccel Z [m/s^2]');
grid on;
xlabel('relative timestamp in secs');
ylabel('linear Acc Z [m/s^2]');

    for i=1:numPlots
        if isempty(inputStructCell{i})
            continue
        else
            ax = inputStructCell{i}.gx5_imu_data.LinearAcceleration.X;
            ay = inputStructCell{i}.gx5_imu_data.LinearAcceleration.Y;
            az = inputStructCell{i}.gx5_imu_data.LinearAcceleration.Z;
            Reltime = inputStructCell{i}.gx5_imu_data.RelTime;
            figure(fig1);
            plot(Reltime,ax);
            figure(fig2);
            plot(Reltime,ay);
            figure(fig3);
            plot(Reltime,az);
        end
    end
end