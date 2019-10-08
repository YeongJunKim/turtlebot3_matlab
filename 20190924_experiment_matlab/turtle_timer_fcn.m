function turtle_timer_fcn(obj, event, turtle_bot)
persistent step 
perception_fcn(turtle_bot);

if turtle_bot.imu_step_main <= 11
    initialization_imu_fcn(turtle_bot);
    turtle_bot.imu_step_main = turtle_bot.imu_step_main + 1;
    turtle_bot.imu_step_sub1 = turtle_bot.imu_step_sub1 + 1;
    turtle_bot.imu_step_sub2 = turtle_bot.imu_step_sub2 + 1;
    disp('ok intailization')
    step = 0;
elseif turtle_bot.imu_step_main ==12
    turtle_bot.imu_step_main = turtle_bot.imu_step_main + 1;
    disp('intailization complete')
else
     tic
    localization_main(turtle_bot);
%     %     disp('ok main localization')
    lidar_localization_sub1(turtle_bot);
    lidar_localization_sub2(turtle_bot);
%     %     disp('ok sub localization')
%     
    if step > 17
    control_fcn(turtle_bot);
    end
    plot_fcn(turtle_bot);
% %     %     disp('ok plot')
      toc
%     turtle_bot.sampling_time = toc;
end


%% for saving data

global data data_num

%     data.tic_toc_data(1,data_num) = tic;
%     data.tic_toc_data(2,data_num) = toc;

data.x_main_hat_PEFFM_data(1,data_num) = turtle_bot.x_main_hat_PEFFME(1,1);
data.x_main_hat_PEFFM_data(2,data_num) = turtle_bot.x_main_hat_PEFFME(2,1);
data.x_main_hat_PEFFM_data(3,data_num) = turtle_bot.x_main_hat_PEFFME(3,1);
data.x_main_hat_EKF_data(1,data_num) = turtle_bot.x_main_hat_EKF(1,1);
data.x_main_hat_EKF_data(2,data_num) = turtle_bot.x_main_hat_EKF(2,1);
data.x_main_hat_EKF_data(3,data_num) = turtle_bot.x_main_hat_EKF(3,1);
data.measurement_main_data(1,data_num) = turtle_bot.measurement_main(1,1);
data.measurement_main_data(2,data_num) = turtle_bot.measurement_main(1,2);
data.measurement_main_data(3,data_num) = turtle_bot.measurement_main(1,3);
data.measurement_main_data(4,data_num) = turtle_bot.measurement_main(1,4);
data.measurement_main_data(5,data_num) = turtle_bot.yaw_angle_main;
data.control_input_main(1,data_num) = turtle_bot.linear_vel_main;
data.control_input_main(2,data_num) = turtle_bot.angular_vel_main;

data.x_sub1_hat_PEFFM_data(1,data_num) = turtle_bot.x_sub1_hat_PEFFME(4,1);
data.x_sub1_hat_PEFFM_data(2,data_num) = turtle_bot.x_sub1_hat_PEFFME(5,1);
data.x_sub1_hat_PEFFM_data(3,data_num) = turtle_bot.x_sub1_hat_PEFFME(6,1);
data.x_sub1_hat_EKF_data(1,data_num) = turtle_bot.x_sub1_hat_EKF(4,1);
data.x_sub1_hat_EKF_data(2,data_num) = turtle_bot.x_sub1_hat_EKF(5,1);
data.x_sub1_hat_EKF_data(3,data_num) = turtle_bot.x_sub1_hat_EKF(6,1);
data.measurement_sub1_data(1,data_num) = turtle_bot.measurement_sub1(1,1);
data.measurement_sub1_data(2,data_num) = turtle_bot.measurement_sub1(1,2);
data.measurement_sub1_data(3,data_num) = turtle_bot.measurement_sub1(1,3);
data.control_input_sub1(1,data_num) = turtle_bot.linear_vel_sub1;
data.control_input_sub1(2,data_num) = turtle_bot.angular_vel_sub1;
data.real_sub1_data(:,data_num) = turtle_bot.real_sub1_state(:,1);
data.flag_sub1_data(:,data_num) = turtle_bot.alpha_sub1;
% 
data.x_sub2_hat_PEFFM_data(1,data_num) = turtle_bot.x_sub2_hat_PEFFME(4,1);
data.x_sub2_hat_PEFFM_data(2,data_num) = turtle_bot.x_sub2_hat_PEFFME(5,1);
data.x_sub2_hat_PEFFM_data(3,data_num) = turtle_bot.x_sub2_hat_PEFFME(6,1);
data.x_sub2_hat_EKF_data(1,data_num) = turtle_bot.x_sub2_hat_EKF(4,1);
data.x_sub2_hat_EKF_data(2,data_num) = turtle_bot.x_sub2_hat_EKF(5,1);
data.x_sub2_hat_EKF_data(3,data_num) = turtle_bot.x_sub2_hat_EKF(6,1);
data.measurement_sub2_data(1,data_num) = turtle_bot.measurement_sub2(1,1);
data.measurement_sub2_data(2,data_num) = turtle_bot.measurement_sub2(1,2);
data.measurement_sub2_data(3,data_num) = turtle_bot.measurement_sub2(1,3);
data.control_input_sub2(1,data_num) = turtle_bot.linear_vel_sub2;
data.control_input_sub2(2,data_num) = turtle_bot.angular_vel_sub2;
data.real_sub2_data(:,data_num) = turtle_bot.real_sub2_state(:,1);
data.flag_sub2_data(:,data_num) = turtle_bot.alpha_sub2;
data.plot_measurement_sub1_data(:,data_num) = turtle_bot.plot_measurement_sub1(:,1);
% data.plot_measurement_sub2_data(:,data_num) = turtle_bot.plot_measurement_sub2(:,1);
% data.lidar_x_data_sub1(:,data_num) = turtle_bot.lidar_x_position_sub1(:,1);
% data.lidar_y_data_sub1(:,data_num) = turtle_bot.lidar_y_position_sub1(:,1);
% data.lidar_x_data_sub2(:,data_num) = turtle_bot_sub2.lidar_x_position(:,1);
% data.lidar_y_data_sub2(:,data_num) = turtle_bot_sub2.lidar_y_position(:,1);
% %
data.calibration_theta_sub1_data(:,data_num) = turtle_bot.calibration_theta_sub1(:,1);
% data.calibration_theta_sub2_data(:,data_num) = turtle_bot.calibration_theta_sub2(:,1);
data_num = data_num + 1
step = data_num;
end