% Patched for AB3, without Bob
clc
clear
close all
Simulink.sdi.clear %Clear simulink data inspector
%%

%Point-mass model parameters
% AB3 data from PM15... (without Bob)
a=0.505; 
v=10/3.6;
h=0.562;
b=1.115; 
g=9.82;
m = 25;
lambda = deg2rad(24); % angle of the fork axis [deg]
bike_params = [g h b a lambda m]; % Store bike parameters in a vector

%Different sample rates
Ts=0.01;
TsD = 0.1;
Tsm=Ts;
matrixIx = 1;

%8 Km/h balancing
% P_Balancing = 7.791;
% I_Balancing = 2.523;
% D_Balancing = 0.0524;

% 10 Km/h balancing 2 weak
P_Balancing = 4.8049;
I_Balancing = 1.1908;
D_Balancing = 0.0566;

%10 Km/h balancing
% P_Balancing = 5.7844;
% I_Balancing = 1.6066;
% D_Balancing = 0.0545;

%14 Km/h balancing
% P_Balancing = 3.418;
% I_Balancing = 1.327;
% D_Balancing = 0.0646;

% P_Heading = 0;
% I_Heading = 0;
% D_Heading = 0;
% 
% P_Lateral = -0.4;
% I_Lateral = 0;
% D_Lateral = 1.5;

%% P, PD, Fuzzy
%steering system step response matching
%k=110 ; % AB3 33.9;
%steer_sys = tf(k^2,[1,2*0.7*k,k^2],'InputDelay',0.015); % AB3: 0.7 (was 0.6)
%sys_dicretePID=c2d(steer_sys,Tsm,'matched');

steer_sys = tf(33.9^2, [1, 2*0.6*33.9, 33.9^2], 'InputDelay', 0.0150);  %Unsure which is the right one. 
sys_dicretePID = c2d(steer_sys, Tsm, 'matched'); % GF: Convert model from continuous to discrete time


%Transfer function for point mass model from steering pos to lean angle
sys = tf((a*v/(h*b))*[1,v/a],[1,0,-g/h]);


%Gain for noise, if set to 0 we got no noise at all.
Noise=1;

%Initialise the state space with Init_Angle degree lean angle
AngleGain = 0.5; %1 for model 1 guess and 0.5 for "new" model 1
Init_Angle=0.2;
Init_Yaw = -90;
Init_X = -1.3;
Init_Y = 50;
Init_condLQR=[0; deg2rad(Init_Angle); 0; 0];
FinalValue=1; %disturbance amplitude


%Run simulation
[A,B,C,D]=tf2ss(sys.Numerator{1}, sys.Denominator{1});

%%
w = warning ('off','all');
distanceStep = 0.001; 
distance = -30; 
startY = 50; 
startX = -1.3; 
radius = 15; 
yc = startY+0:-0.1:startY+distance; 
xc = startX*ones(1,length(yc)); 
xc = [xc radius*cos(0:-pi/128:-pi/2)-radius+xc(end)]; 
yc = [yc radius*sin(0:-pi/128:-pi/2)+yc(end)]; 
ye = yc(end); 
xe = xc(end); 
xb = xe:-0.1:xe+distance; 
yb = 0*ones(1,length(xb))+ye; 
 
xc = [xc xb]; 
yc = [yc yb]; 
TestPath = [xc' yc']; 
total_length = arclength(TestPath(:,1),TestPath(:,2),'linear'); 
SimulinkPath = interparc(0:(distanceStep/total_length):1,TestPath(:,1),TestPath(:,2),'linear'); 
yd = diff(SimulinkPath(:,2)); 
xd = diff(SimulinkPath(:,1)); 
vd = wrapTo180([atan2(yd,xd); atan2(yd(end),xd(end))]); 
SimulinkPath(:,3) = vd; 
PathData = length(SimulinkPath)-1; 
%%
P_Lateral = 5;
I_Lateral = 0.1;
D_Lateral = 0;

P_Heading = -0.2;
I_Heading = 0;
D_Heading = 0;
sim('All_Controllers') %Run simulation

hold on;
plot(position.Data(:,1),position.Data(:,2));
plot(position.Data(:,3),position.Data(:,4));
legend('Reference path', 'Bicycle Path');
RMSE = sqrt((sum((LatError.Data).^2))/(length(LatError.Data)))
stid = std(LatError.Data)
set(gca,'FontSize',18) % Creates an axes and sets its FontSize to 18
xlabel('X (m)')
ylabel('Y (m)')

PosPlot

