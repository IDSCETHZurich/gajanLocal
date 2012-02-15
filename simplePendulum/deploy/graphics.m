%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% OROCOS LOGGER (FRI PC side) %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% logfile = load('orocosKUKA16.log');
% 
% TsOROCOSlogger = round(1000*mean(diff(logfile(:,1))))/1000
% t_ini = 0.912; % Time it takes to start the deployer
% 
% figure
% plot(logfile(:,1)-t_ini,logfile(:,3:9))
% legend 0 1 2 3 4 5 6
% grid on
% title('Commanded positions from remote PC (OROCOS LOGGER)')
% xlabel('t(s)')
% ylabel('joint angles(rad)')
% 
% figure
% plot(logfile(2:end,1)-t_ini,diff(logfile(:,3:9))./TsOROCOSlogger)
% legend 0 1 2 3 4 5 6
% grid on
% title('Commanded velocities from remote PC (OROCOS LOGGER) using timestamp')
% xlabel('t(s)')
% ylabel('joint veloticies(rad/s)')
% 
% figure
% plot([logfile(1,1):TsOROCOSlogger:logfile(end-1,1)]-t_ini,diff(logfile(:,3:9))./TsOROCOSlogger)
% legend 0 1 2 3 4 5 6
% grid on
% title('Commanded velocities from remote PC (OROCOS LOGGER) using constant Ts')
% xlabel('t(s)')
% ylabel('joint veloticies(rad/s)')
% 


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% OROCOS REPORTER (FRI PC side) %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % NOTE: This script assumes that the following columns are available in the
% % logger file: 1->time ; 2-8->cmd_pos ; 9-15->->msr_trq ; 16-22->msr_pos
% % Otherwise, modify it to make it match your logging
reports = load('reports.dat');

% figure
% plot(reports(:,1),reports(:,2:8))
% legend 0 1 2 3 4 5 6
% grid on
% title('Commanded positions from remote PC')
% xlabel('t(s)')
% ylabel('joint angles(rad)')
% 
% Deleting points that are not taken at the right time (duplicity of
% points)
tol = 0.001; % Minimum time between measurements
bad_points = [];
% Se puede hacer con un find. Pensar c'omo si hay tiempo.
for i=2:size(reports,1)
    % If time between measurements is too small, we delete one of them
    if (abs(reports(i,1)-reports(i-1,1)) < tol)
        bad_points = [bad_points i-1];
    end
end
% Deleting bad_points
reports(bad_points,:) = [];

% Automatically calculates the sampling time as the average (rounded to
% miliseconds) of the time difference between samples
TsOROCOSreporter = round(1000*mean(diff(reports(:,1))))/1000

figure
plot(reports(:,1),reports(:,2:8))
legend 0 1 2 3 4 5 6
grid on
title('Commanded positions from remote PC')
xlabel('t(s)')
ylabel('joint angles(rad)')

figure
plot(reports(2:end,1),diff(reports(:,2:8))./TsOROCOSreporter)
legend 0 1 2 3 4 5 6
grid on
title('Commanded velocities from remote PC')
xlabel('t(s)')
ylabel('joint veloticies(rad/s)')

figure
plot(reports(2:end-1,1),diff(diff(reports(:,2:8)))./TsOROCOSreporter^2)
legend 0 1 2 3 4 5 6
grid on
title('Commanded accelerations from remote PC')
xlabel('t(s)')
ylabel('joint accelerations(rad/s)')


figure
subplot(3,1,1)
plot(reports(:,1),reports(:,2:8))
legend 0 1 2 3 4 5 6
grid on
ylabel('joint angles(rad)')

subplot(3,1,2)
plot(reports(2:end,1),diff(reports(:,2:8))./TsOROCOSreporter)
grid on
ylabel('joint veloticies(rad/s)')

subplot(3,1,3)
plot(reports(2:end-1,1),diff(diff(reports(:,2:8)))./TsOROCOSreporter^2)
grid on
xlabel('t(s)')
ylabel('joint accelerations(rad/s)')

figure
subplot(3,1,1)
plot(reports(:,1),reports(:,16:22))
legend 0 1 2 3 4 5 6
grid on
ylabel('joint angles(rad)')

subplot(3,1,2)
plot(reports(2:end,1),diff(reports(:,16:22))./TsOROCOSreporter)
grid on
ylabel('joint veloticies(rad/s)')

subplot(3,1,3)
plot(reports(2:end-1,1),diff(diff(reports(:,16:22)))./TsOROCOSreporter^2)
grid on
xlabel('t(s)')
ylabel('joint accelerations(rad/s)')


figure
plot(reports(:,1),reports(:,2:8))
hold on
plot(reports(:,1),reports(:,16:22), '--')
legend 0 1 2 3 4 5 6
grid on
title('OROCOS LOGGER POSITIONS Solid:cmd Dashed:msr')
xlabel('t(s)')
ylabel('joint angles(rad)')



%%%%%%%%%%%%%%%%%%%%%%%%%%%
% FRI LOGGER (Robot Side) %
%%%%%%%%%%%%%%%%%%%%%%%%%%%
FRIC006
FRIM006

% Automatically calculates the sampling time as the average (rounded to
% miliseconds) of the time difference between samples
TsFRIlogger = round(1000*mean(diff(FRIC_time)))/1000


figure
plot(FRIC_time-FRIC_time(1),FRIC_cmd_jntPos)
hold on
plot(FRIM_time-FRIM_time(1),FRIM_data_msrJntPos,'--')
grid on
title('FRI POSITIONS Robot Side Solid:cmd Dashed:msr')
xlabel('t(s)')
ylabel('joint angles(rad)')

figure
plot(FRIC_time(1:end-1)-FRIC_time(1),diff(FRIC_cmd_jntPos)/TsFRIlogger)
hold on
plot(FRIM_time(1:end-1)-FRIM_time(1),diff(FRIM_data_msrJntPos)/TsFRIlogger,'--')
grid on
title('FRI VELOCITIES Robot Side Solid:cmd Dashed:msr')
xlabel('t(s)')
ylabel('joint velocities (rad/s)')


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% ROBOT LOGGER (internal data) %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
ROB_006
ACYC006
KRC_006

% Automatically calculates the sampling time as the average (rounded to
% miliseconds) of the time difference between samples
TsKRClogger = round(1000*mean(diff(KRC_time)))/1000

% Plotting robot info
figure
ROB_sampletime = ROB_time(2)-ROB_time(1);
plot(ROB_time-ROB_time(1), ROB_cmd_angle)
hold on
plot(ROB_time-ROB_time(1), ROB_msr_angle, '--')
legend 0 1 2 3 4 5 6
title('ROBOT POSITIONS Solid:cmd Dashed:msr')
xlabel('t(s)')
ylabel('joint angles(rad)')
grid on

% NOTE: Velocity provided by logger is exactly the same than
% differentiating the position values, so, it will be used instead
figure
plot(ROB_time-ROB_time(1), ROB_cmd_speed)
hold on
plot(ROB_time-ROB_time(1), ROB_msr_speed, '--')
legend 0 1 2 3 4 5 6
title('ROBOT VELOCITIES (PROVIDED BY LOGGER) Solid:cmd Dashed:msr')
xlabel('t(s)')
ylabel('joint velocities (rad/s)')
grid on

% Plotting KRC info
figure
plot(KRC_time-KRC_time(1), KRC_cmd_angle)
hold on
plot(KRC_time-KRC_time(1), KRC_msr_angle, '--')
legend 0 1 2 3 4 5 6
title('KRC POSITIONS Solid:cmd Dashed:msr')
xlabel('t(s)')
ylabel('joint angles(rad)')
grid on

figure
plot(KRC_time(1:end-1)-KRC_time(1), diff(KRC_cmd_angle)/TsKRClogger)
hold on
plot(KRC_time(1:end-1)-KRC_time(1), diff(KRC_msr_angle)/TsKRClogger, '--')
legend 0 1 2 3 4 5 6
title('KRC VELOCITIES (calculated with diff(angle)) Solid:cmd Dashed:msr')
xlabel('t(s)')
ylabel('joint velocities (rad/s)')
grid on

figure
plot(ROB_time-ROB_time(1), ROB_cmd_torque)
hold on
plot(ROB_time-ROB_time(1), ROB_msr_torque,'--')
legend 0 1 2 3 4 5 6
title('KRC TORQUES Solid:cmd Dashed:msr')
xlabel('t(s)')
ylabel('Joint torques (N*m)')
grid on


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% ERROR LOGGER (Robot Side) %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
ERR_006

% Automatically calculates the sampling time as the average (rounded to
% miliseconds) of the time difference between samples
TsERRlogger = round(1000*mean(diff(ERR_time)))/1000

figure
plot(ERR_time-ERR_time(1),ERR_cmd_angle)
hold on
plot(ERR_time-ERR_time(1),ERR_msr_angle,'--')
legend 0 1 2 3 4 5 6
grid on
title('POSITIONS ERR LOGGER Solid:cmd Dashed:msr')
xlabel('t(s)')
ylabel('joint angles (rad)')

figure
plot(ERR_time(1:end-1)-ERR_time(1),diff(ERR_cmd_angle)/0.001)
hold on
plot(ERR_time(1:end-1)-ERR_time(1),diff(ERR_msr_angle)/0.001,'--')
legend 0 1 2 3 4 5 6
grid on
title('VELOCITIES ERR LOGGER Solid:cmd Dashed:msr')
xlabel('t(s)')
ylabel('joint velocities (rad/s)')



