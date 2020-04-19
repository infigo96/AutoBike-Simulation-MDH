% close all;
% subplot(1,2,1);
% plot(position.Data(:,1),position.Data(:,2));
% hold on;
% plot(position.Data(:,3),position.Data(:,4));
% legend('Reference path', 'Bicycle path');
% set(gca,'FontSize',18) % Creates an axes and sets its FontSize to 18
% xlabel('X (m)');
% ylabel('Y (m)');
% ylim([-50 20])
% xlim([0 70])
% 
% 
% subplot(1,2,2);
% plot(heading.Time,heading.Data(:,1));
% hold on;
% plot(heading.Time,heading.Data(:,2));
% legend('Reference heading', 'Bicycle heading');
% set(gca,'FontSize',18) % Creates an axes and sets its FontSize to 18
% xlabel('Time (s)');
% ylabel('Angle (degrees)');

%%
figure
subplot(1,2,1)
plot(LatError,'blue')
hold on
plot(LatOut,'black')
plot(roll,'red')
ylim([-30 30])
%xlim([0 6])
legend('lateral error (m)','lean setpoint (degrees)','lean angle (degrees)'); 
set(gca,'FontSize',18) % Creates an axes and sets its FontSize to 18

subplot(1,2,2)
plot(LatError,'blue')

hold on
plot(PIDSteer1,'red')

ylim([-30 30])
%xlim([0 6]);
legend('lateral error (m)','lean angle (degrees)'); 
set(gca,'FontSize',18) % Creates an axes and sets its FontSize to 18
