close all;
% clc;
%w = warning ('off','all');
distanceStep = (v)*Ts; %run simulation Main first

%%%%%%%% 90 Degree %%%%%%%%%%%
distance = 20;
xc = 0:0.1:distance;
yc = zeros(1,length(xc));
radius = 10;
xc = [xc radius*cos(pi/2:-pi/64:pi/4)+distance];
yc = [yc radius*sin(pi/2:-pi/64:pi/4)-radius];
ye = yc(end);
xe = xc(end);
yb = ye:-0.1*sin(pi/4):ye-2*distance*sin(pi/4);
xb = xe:0.1*cos(pi/4):xe+2*distance*cos(pi/4);
xc = [xc xb];
yc = [yc yb];
TestPath = [xc' yc'];
total_length = arclength(TestPath(:,1),TestPath(:,2),'linear');
PreparedPath{1} = interparc(0:(distanceStep/total_length):1,TestPath(:,1),TestPath(:,2),'linear');
yd = diff(PreparedPath{1}(:,2));
xd = diff(PreparedPath{1}(:,1));
vd = [atan2(yd,xd); atan2(yd(end),xd(end))];
PreparedPath{1}(:,3) = vd;
PathStop(1) = length(PreparedPath{1})-1;

%%%%%%%%%%   Straight %%%%%%%%
distance = 100;
xc = 0:0.1:distance;
yc = zeros(1,length(xc));
TestPath = [xc' yc'];
total_length = arclength(TestPath(:,1),TestPath(:,2),'linear');
PreparedPath{2} = interparc(0:(distanceStep/total_length):1,TestPath(:,1),TestPath(:,2),'linear');
yd = diff(PreparedPath{2}(:,2));
xd = diff(PreparedPath{2}(:,1));
vd = [atan2(yd,xd); atan2(yd(end),xd(end))];
PreparedPath{2}(:,3) = vd;
PathStop(2) = length(PreparedPath{2})-1;


%%%%%%%% 180 Degree %%%%%%%%%%%
distance = 10;
xc = 0:0.1:distance;
yc = zeros(1,length(xc));
radius = 15;
xc = [xc radius*cos(-pi/2:pi/64:pi/2)+distance];
yc = [yc radius*sin(-pi/2:pi/64:pi/2)+radius];
ye = yc(end);
xe = xc(end);
xb = xe:-0.1:xe-distance;
yc = [yc ye*ones(1,length(xb))];
xc = [xc xb];
TestPath = [xc' yc'];
total_length = arclength(TestPath(:,1),TestPath(:,2),'linear');
PreparedPath{3} = interparc(0:(distanceStep/total_length):1,TestPath(:,1),TestPath(:,2),'linear');
yd = diff(PreparedPath{3}(:,2));
xd = diff(PreparedPath{3}(:,1));
vd = [atan2(yd,xd); atan2(yd(end),xd(end))];
PreparedPath{3}(:,3) = vd;
PathStop(3) = length(PreparedPath{3})-1;

%%%%%%%%%%RRT Path%%%%%
load('RTPt.mat')
poses(:,1) = poses(:,1)-10;
poses(:,2) = poses(:,2)-10;
total_length = arclength(poses(:,1),poses(:,2),'linear');
PreparedPath{4} = interparc(0:(distanceStep/total_length):1,poses(:,1),poses(:,2),'linear');
yd = diff(PreparedPath{4}(:,2));
xd = diff(PreparedPath{4}(:,1));
vd = [atan2(yd,xd); atan2(yd(end),xd(end))];
PreparedPath{4}(:,3) = vd;
PathStop(4) = length(PreparedPath{4})-1;


%%%%%%%%%%%%%%%%%%%%%%%%
%gains = [0 0 0 -0.4 0 0.04;-0.2 -0.1 0 -0.4 0 0.05;-0.2 -0.1 0 0 0 0];
gains = [0 0 0 -0.3 0 0];%0 0 0 -0.4 0 0]      %Heading 10Hz
[m, n] = size(gains);
for(j=1:1)
    meen(j,:) = [inf inf inf inf];
    RMSE(j,:) = [inf inf inf inf];
    stid(j,:) = [inf inf inf inf];
    for(i=1:4)
       
        P_Lateral = 18;
        I_Lateral = 0.2;
        D_Lateral = 0;
        
        P_Heading = -1;
        I_Heading = 0;
        D_Heading = 0.001;
        
        SimulinkPath = PreparedPath{i};
        PathData = PathStop(i);
        try
            sim('All_Controllers') %Run simulation
            SLatErr{j,i} = LatError;
            SRoll{j,i} = roll;
            SPosition{j,i} = position;
            Sheading{j,i} = heading;
            
            SHeadErr{j,i} = HeadError;
            SHeadErr{j,i}.Data = squeeze(SHeadErr{j,i}.Data);
            
            meen(j,i) = mean(LatError.Data);
            RMSE(j,i) = sqrt((sum((LatError.Data).^2))/(length(LatError.Data)));
            stid(j,i) = std(LatError.Data);
        end
    end
end
RMSE
PIDS = [P_Lateral I_Lateral D_Lateral P_Heading I_Heading D_Heading];
%%
b = 1;
subplot(2,2,b)
hold on;
plot(SPosition{b}.Data(:,1),SPosition{b}.Data(:,2));
plot(SPosition{b}.Data(:,3),SPosition{b}.Data(:,4));
% plot(linspace(1,30,length(SLatErr{b}.Data)),SLatErr{b}.Data);
% plot(linspace(1,30,length(SHeadErr{b}.Data(1,1,:))),SHeadErr{b}.Data(1,1,:));
legend('Reference path', 'Bicycle Path');

b = 2;
subplot(2,2,b)
hold on;
plot(SPosition{b}.Data(:,1),SPosition{b}.Data(:,2));
plot(SPosition{b}.Data(:,3),SPosition{b}.Data(:,4));
% plot(linspace(1,30,length(SLatErr{b}.Data)),SLatErr{b}.Data);
% plot(linspace(1,30,length(SHeadErr{b}.Data(1,1,:))),SHeadErr{b}.Data(1,1,:));
legend('Reference path', 'Bicycle Path');

ylim([-1 1]);

b = 3;
subplot(2,2,b)
hold on;
plot(SPosition{b}.Data(:,1),SPosition{b}.Data(:,2));
plot(SPosition{b}.Data(:,3),SPosition{b}.Data(:,4));
% plot(linspace(1,30,length(SLatErr{b}.Data)),SLatErr{b}.Data);
% plot(linspace(1,30,length(SHeadErr{b}.Data(1,1,:))),SHeadErr{b}.Data(1,1,:));
legend('Reference path', 'Bicycle Path');


b = 4;
subplot(2,2,b)
hold on;
plot(SPosition{b}.Data(:,1),SPosition{b}.Data(:,2));
plot(SPosition{b}.Data(:,3),SPosition{b}.Data(:,4));
% plot(linspace(1,30,length(SLatErr{b}.Data)),SLatErr{b}.Data);
% plot(linspace(1,30,length(SHeadErr{b}.Data(1,1,:))),SHeadErr{b}.Data(1,1,:));
legend('Reference path', 'Bicycle Path');


% figure;
% hold on;
% plot(SPosition{1,4}.Data(:,3),SPosition{1,4}.Data(:,4));
% plot(SPosition{2,4}.Data(:,3),SPosition{2,4}.Data(:,4));
% plot(SPosition{3,4}.Data(:,3),SPosition{3,4}.Data(:,4));
% %plot(SPosition{4,1}.Data(:,3),SPosition{4,1}.Data(:,4));
% %plot(SPosition{5,1}.Data(:,3),SPosition{5,1}.Data(:,4));
% plot(SimulinkPath(:,1),SimulinkPath(:,2));
% legend;
PosPlot