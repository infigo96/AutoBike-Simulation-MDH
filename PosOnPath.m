% clear;
% close all;
% clc;
bikePos = [[5 5]; [5 6]; [5 7]; [6 5]; [6 7]; [7 5]; [7 6]; [7 7]];
%[6+3*cos(deg2rad((30)))+cos(deg2rad((30+90))) 6+3*sin(deg2rad((30)))+sin(deg2rad((30+90)))]
% scatter(bikePos(:,1),bikePos(:,2))
curvepos = [6 6];
angle = [0 30 45 60 90 120 135 150 180 -150 -135 240 270 300 -45 330]';

for j= 1:16

    subplot(4,4,j);
    line([curvepos(1) curvepos(1)+cos(deg2rad(angle(j)))],[curvepos(2) curvepos(2)+sin(deg2rad(angle(j)))]);
    hold on;
    xlim([4 8])
    ylim([4 8])
    n = [cos(deg2rad(angle(j))) sin(deg2rad(angle(j)))];

    for i = 1:8
        usnit = dot([bikePos(i,1)-curvepos(1), bikePos(i,2)-curvepos(2)],n); 
        if(usnit > 0)
            scatter(bikePos(i,1),bikePos(i,2),'g');
        elseif(abs(usnit) == 0) 
            scatter(bikePos(i,1),bikePos(i,2),'b');
        else
            scatter(bikePos(i,1),bikePos(i,2),'r');
        end
    end
end

%Same as in Simulink choosing where the bicycle is on the planned path so
%the bicycle have a correct point on the path to measure the error. 
%Also required when lateral control is implemented on a real bicycle.
function [y,io] = SimulinkPosChooser(path,bike,i)
    j = i;
    usnit = 1;
    y = path(j,:);
    io = 1;
    while(j < length(path) && usnit >= 0)
        n = [cos(path(j,3)) sin(path(j,3))];
        usnit = dot([bike(1)-path(j,1), bike(2)-path(j,2)],n); 
        if(usnit >= 0)
           y =  path(j,:);
           io = j;
        end
        j = j+1;
    end
end