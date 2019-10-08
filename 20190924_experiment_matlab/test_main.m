%% setup
clc
clear all
close all
rosshutdown;

% % ip_address = ('192.168.0.101'); % in ACSL
ip_address = ('192.168.1.46'); % in experiment
% ip_address = ('192.168.0.11'); % in HOME
%%%%%%%%%%%%%%%%%%%%%%%%%% CASE 1. TWO ROBOT %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%     main_name = '/tb3_0/odom';
%     sub1_name = '/tb3_1/odom';
%     sub2_name = '/tb3_2/odom';
%     main_name = '/main/odom';
%     sub1_name = '/sub1/odom';
%     sub2_name = '/sub2/odom';
%     sampling_time = 0.4;
%     test_bot = turtle_bot(ip_address, main_name, sub1_name, sub2_name, sampling_time); % case : two robot
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%% CASE 2. THREE ROBOT %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%     main_name = '/tb3_0/odom';
%     sub1_name = '/tb3_1/odom';
%     sub2_name = '/tb3_2/odom';
    main_name = '/main/odom';
    sub1_name = '/sub1/odom';
    sub2_name = '/sub2/odom';
    sampling_time = 0.4;
    test_bot = turtle_bot_three(ip_address, main_name, sub1_name, sub2_name, sampling_time); % case : three robot
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% for save data
global data data_num

save_step = 100;
data_num = 1;
data.x_main_hat_PEFFM_data = zeros(3,save_step);
data.x_main_hat_EKF_data = zeros(3,save_step);
data.measurement_main_data = zeros(5,save_step);

data.x_sub1_hat_PEFFM_data = zeros(3,save_step);
data.x_sub1_hat_EKF_data = zeros(3,save_step);
data.measurement_sub1_data = zeros(3,save_step);
data.control_input_sub1 = zeros(2,save_step);

data.x_sub2_hat_PEFFM_data = zeros(3,save_step);
data.x_sub2_hat_EKF_data = zeros(3,save_step);
data.measurement_sub2_data = zeros(3,save_step);
data.control_input_sub2 = zeros(2,save_step);

data.lidar_x_data_sub1 = zeros(360,save_step);
data.lidar_y_data_sub1 = zeros(360,save_step);
data.lidar_x_data_sub2 = zeros(360,save_step);
data.lidar_y_data_sub2 = zeros(360,save_step);

data.plot_measurement_sub1_data = zeros(2, save_step);
data.plot_measurement_sub2_data = zeros(2, save_step);
data.local_measurement_sub1 = zeros(2,save_step);

data.real_sub1_data = zeros(3,save_step);
data.real_sub2_data = zeros(3,save_step);

data.flag_sub1_data = zeros(1,save_step);
data.flag_sub2_data = zeros(1,save_step);
data.calibration_theta_sub1_data = zeros(1,save_step);
data.calibration_theta_sub2_data = zeros(1,save_step);
%% algorithm loop
test_timer = timer('Busymode', 'drop', 'ExecutionMode', 'fixedRate', 'Period', sampling_time, 'TimerFcn', {@turtle_timer_fcn, test_bot});
% test_timer = timer('TimerFcn', {@turtle_timer_fcn, test_bot});
start(test_timer);
%  stop(test_timer)
% % delete(timerfindall)