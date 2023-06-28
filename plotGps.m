function [fig1] = plotGps(inputStructCell)
numPlots = numel(inputStructCell);


fig1 = figure;
hold on
grid on
xlabel('xEast [m]');
ylabel('yNorth [m]');
title('Cartesian position relative to starting point @(0,0)');

% fig2 = figure;
% hold on;
% geobasemap satellite
% title('Plotting GPS Lat Lon data on geomap');
% xlabel('Latitude');
% ylabel('Longitude');

for i = 1:numPlots
    if isempty(inputStructCell{i})
        continue
    else     
    gpsLat = inputStructCell{i}.piksi_multi_position_navsatfix_best_fix.Latitude;
    gpsLon = inputStructCell{i}.piksi_multi_position_navsatfix_best_fix.Longitude;
    gpsAlt = inputStructCell{i}.piksi_multi_position_navsatfix_best_fix.Altitude;
    gpsReltime = inputStructCell{i}.piksi_multi_position_navsatfix_best_fix.RelTime;
    % Define the reference coordinates, corresponding to (0,0,0) coordinates
    % let's pick the starting point of the trajectory according to the /bestfix
        % topic
    [xEastGPS, yNorthGPS, zUpGPS] = geodetic2enu(gpsLat ,gpsLon,gpsAlt,...
                                        gpsLat(1), gpsLon(1), gpsAlt(1),...
                                        wgs84Ellipsoid,"degrees");

    figure(fig1);
    plot(xEastGPS,yNorthGPS);
    % figure(fig2);
    % geoplot(gpsLat,gpsLon);
    end
end
end