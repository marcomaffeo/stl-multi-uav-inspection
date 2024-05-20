%% Position Plot

t1=0:sampling_time:57;

subplot(3,3,1)
title('Drone n.1')
hold on 
plot(t1,pos_x(:,1),'-b');
hold on
plot(t1,pos_y(:,1),'--r');
hold on 
plot(t1,pos_z(:,1),'-.g');
legend('p^{(1)}','p^{(2)}','p^{(3)}','Orientation','horizontal','Location','north')
ylabel('[m]')
ylim([-20 40])
grid on

subplot(3,3,2)
title('Drone n.2')
hold on
plot(t1,pos_x(:,2),'-b');
hold on
plot(t1,pos_y(:,2),'--r');
hold on 
plot(t1,pos_z(:,2),'-.g');
legend('p^{(1)}','p^{(2)}','p^{(3)}','Orientation','horizontal','Location','north')
ylabel('[m]')
ylim([-20 30])
grid on

subplot(3,3,3)
title('Drone n.3')
hold on
plot(t1,pos_x(:,3),'-b');
hold on
plot(t1,pos_y(:,3),'--r');
hold on 
plot(t1,pos_z(:,3),'-.g');
legend('p^{(1)}','p^{(2)}','p^{(3)}','Orientation','horizontal','Location','north')
ylabel('[m]')
ylim([-20 30])
grid on

%% Velocity Plot

vel_x = diff(pos_x) / sampling_time;
vel_y = diff(pos_y) / sampling_time;
vel_z = diff(pos_z) / sampling_time;

% Poiché la differenza riduce la dimensione di un elemento, definisci un nuovo asse temporale
t2 = t1(1:end-1);

subplot(3,3,4)
plot(t2, vel_x(:,1), '-b');
hold on
plot(t2, vel_y(:,1), '--r');
hold on
plot(t2, vel_z(:,1), '-.g');
legend('v^{(1)}','v^{(2)}','v^{(3)}','Orientation','horizontal','Location','north')
ylabel('[ms^{-1}]')
ylim([-4 5])
grid on


subplot(3,3,5)
plot(t2, vel_x(:,2), '-b');
hold on
plot(t2, vel_y(:,2), '--r');
hold on
plot(t2, vel_z(:,2), '-.g');
legend('v^{(1)}','v^{(2)}','v^{(3)}','Orientation','horizontal','Location','north')
ylabel('[ms^{-1}]')
ylim([-4 6])
grid on

subplot(3,3,6)
hold on
plot(t2,vel_x(:,3),'-b');
hold on
plot(t2,vel_y(:,3),'--r');
hold on 
plot(t2,vel_z(:,3),'-.g');
legend('v^{(1)}','v^{(2)}','v^{(3)}','Orientation','horizontal','Location','north')
ylabel('[ms^{-1}]')
ylim([-4 6])
grid on

%% Acceleration Plot

acc_x = diff(vel_x) / sampling_time;
acc_y = diff(vel_y) / sampling_time;
acc_z = diff(vel_z) / sampling_time;

% Poiché la differenza riduce la dimensione di un elemento, definisci un nuovo asse temporale per le accelerazioni
t3 = t2(1:end-1);

subplot(3,3,7)
plot(t3, acc_x(:,1), '-b');
hold on
plot(t3, acc_y(:,1), '--r');
hold on
plot(t3, acc_z(:,1), '-.g');
legend('a^{(1)}','a^{(2)}','a^{(3)}','Orientation','horizontal','Location','north')
xlabel('Time [s]')
ylabel('[ms^{-2}]')
ylim([-5 7])
grid on

subplot(3,3,8)
plot(t3, acc_x(:,2), '-b');
hold on
plot(t3, acc_y(:,2), '--r');
hold on
plot(t3, acc_z(:,2), '-.g');
legend('a^{(1)}','a^{(2)}','a^{(3)}','Orientation','horizontal','Location','north')
xlabel('Time [s]')
ylabel('[ms^{-2}]')
ylim([-5 7])
grid on

subplot(3,3,9)
hold on
plot(t3,acc_x(:,3),'-b');
hold on
plot(t3,acc_y(:,3),'--r');
hold on 
plot(t3,acc_z(:,3),'-.g');
legend('a^{(1)}','a^{(2)}','a^{(3)}','Orientation','horizontal','Location','north')
xlabel('Time [s]')
ylabel('[ms^{-2}]')
ylim([-5 7])
grid on