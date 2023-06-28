function [fig1, fig2] = velocityPlotter(inputCell)
    numPlots = numel(inputCell);

    % Create the first plot
    fig1 = figure;
    hold on;
    xlabel('Longitudinal velocity [m/s]');
    ylabel('number of data points');
    title('Linear Velocity');
    grid on;

    % Create the second plot
    fig2 = figure;
    hold on;
    xlabel('Yaw rate [rad/s]');
    ylabel('number of data points');
    title('Angular Velocity');
    grid on;

    for i = 1:numPlots
        if isempty(inputCell{i})
            continue
        else
            % Plot on the first figure
            figure(fig1);
            plot(inputCell{i}.husky_velocity_controller_cmd_vel.Linear.X);
            
            % Plot on the second figure
            figure(fig2);
            plot(inputCell{i}.husky_velocity_controller_cmd_vel.Angular.Z);
            
            % Additional plot customization or processing can be done here
        end
    end
end
