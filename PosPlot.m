close all;
subplot(1,2,1);
plot(position.Data(:,1),position.Data(:,2));
hold on;
plot(position.Data(:,3),position.Data(:,4));
legend('Reference path', 'Bicycle path');
set(gca,'FontSize',18) % Creates an axes and sets its FontSize to 18
xlabel('X (m)');
ylabel('Y (m)');

subplot(1,2,2);
plot(heading.Time,heading.Data(:,1));
hold on;
plot(heading.Time,heading.Data(:,2));
legend('Reference heading', 'Bicycle heading');
set(gca,'FontSize',18) % Creates an axes and sets its FontSize to 18
xlabel('Time (s)');
ylabel('Angle (degrees)');