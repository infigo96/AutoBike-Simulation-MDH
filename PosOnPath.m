clear;
close all;
clc;
bikePos = [[5 5]; [5 6]; [5 7]; [6 5]; [6 7]; [7 5]; [7 6]; [7 7]];
% scatter(bikePos(:,1),bikePos(:,2))
curvepos = [6 6];
angle = [0 30 45 90 135 180 240 270 -45]';

for j= 1:9

    subplot(3,3,j);
    line([curvepos(1) curvepos(1)+cos(deg2rad(angle(j)))],[curvepos(2) curvepos(2)+sin(deg2rad(angle(j)))]);
    hold on;
    xlim([4 8])
    ylim([4 8])
    n = [sin(wrapToPi(deg2rad(angle(j)+90))) -cos(wrapToPi(deg2rad(angle(j)+90)))];

    for i = 1:8
        usnit = dot([bikePos(i,1)-curvepos(1), bikePos(i,2)-curvepos(2)],n); 
        if(usnit > 0.00000000001)
            scatter(bikePos(i,1),bikePos(i,2),'g');
        elseif(abs(usnit) < 0.00000000001) 
            scatter(bikePos(i,1),bikePos(i,2),'b');
        else
            scatter(bikePos(i,1),bikePos(i,2),'r');
        end
    end
end