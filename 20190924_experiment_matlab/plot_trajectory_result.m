close all
data_num =85;
main_x_data = data.x_main_hat_PEFFM_data(1,13:data_num-1);
main_y_data = data.x_main_hat_PEFFM_data(2,13:data_num-1);
sub1_x_data = data.x_sub1_hat_PEFFM_data(1,13:data_num-1);
sub1_y_data = data.x_sub1_hat_PEFFM_data(2,13:data_num-1);
sub1_theta_hat = data.x_sub1_hat_PEFFM_data(3,13:data_num-1);
sub2_x_data = data.x_sub2_hat_PEFFM_data(1,1:data_num-1);
sub2_y_data = data.x_sub2_hat_PEFFM_data(2,1:data_num-1);
sub2_theta_hat = data.x_sub2_hat_PEFFM_data(3,13:data_num-1);

% main_x_data = data.x_main_hat_EKF_data(1,13:data_num-1);
% main_y_data = data.x_main_hat_EKF_data(2,13:data_num-1);
% sub1_x_data = data.x_sub1_hat_EKF_data(1,13:data_num-1);
% sub1_y_data = data.x_sub1_hat_EKF_data(2,13:data_num-1);
% sub1_theta_hat = data.x_sub1_hat_EKF_data(3,13:data_num-1);
% sub2_x_data = data.x_sub2_hat_EKF_data(1,1:data_num-1);
% sub2_y_data = data.x_sub2_hat_EKF_data(2,1:data_num-1);
% sub2_theta_hat = data.x_sub2_hat_EKF_data(3,13:data_num-1);

sub1_x_measurement = data.plot_measurement_sub1_data(1,13:data_num-1);
sub1_y_measurement = data.plot_measurement_sub1_data(2,13:data_num-1);
sub1_theta_measurement = data.measurement_sub1_data(3,13:data_num-1);
sub2_theta_measurement = data.measurement_sub2_data(3,13:data_num-1);
real_x_position_sub1 = data.real_sub1_data(1,13:data_num-1);
real_y_position_sub1 = data.real_sub1_data(2,13:data_num-1);
sub1_theta_real = data.real_sub1_data(3,13:data_num-1);
real_x_position2 = data.real_sub2_data(1,13:data_num-1);
real_y_position2 = data.real_sub2_data(2,13:data_num-1);
sub2_theta_real = data.real_sub2_data(3,13:data_num-1);
flag_sub1_data = data.flag_sub1_data(1,13:data_num-1);
flag_sub2_data = data.flag_sub2_data(1,13:data_num-1);
sub1_x_position_local=data.local_measurement_sub1(1,13:data_num-15);
sub1_y_position_local=data.local_measurement_sub1(2,13:data_num-15);
% calibration_theta_sub1 =data.calibration_theta_sub1_data(1,13:data_num-1);
% calibration_theta_sub2 =data.calibration_theta_sub2_data(1,13:data_num-1);
control_input_angular_sub1 = data.control_input_sub1(2,13:data_num-1);
control_input_angular_sub2 = data.control_input_sub2(2,13:data_num-1);
tile_size = 0.6;
tile_num = 9;
plot_limit1 = tile_size*tile_num;

figure(4)
plot(main_x_data,main_y_data,'*-','color',[0.3,0.3,0.3],'DisplayName','main real');hold on; grid on;
% plot(real_x_position_sub1,real_y_position_sub1,'o-','color',[0.0,0.0,0.0],'DisplayName','sub1 real');
% plot(real_x_position2,real_y_position2,'d-','color',[0.0,0.0,0.0],'DisplayName','sub2 real');
plot(sub1_x_data,sub1_y_data,'o-','color',[0.9,0.3,0.3],'DisplayName','sub1 hat');
plot(sub2_x_data,sub2_y_data,'d-','color',[0.3,0.3,0.9],'DisplayName','sub2 hat');
% plot(sub1_x_measurement,sub1_y_measurement,'*-','color',[0.3,0.9,0.3],'DisplayName','sub1 measurement');
xlim([0 plot_limit1])
ylim([0 plot_limit1])
legend('Location','northwest');
title('trajectory')
% % 
% figure(5)
% subplot(2,1,1)
% plot(flag_sub1_data,'*-','color',[0.9,0.3,0.3]);
% % subplot(2,1,2)
% % plot(flag_sub2_data,'*-','color',[0.3,0.3,0.9]);
% 
% figure(6)
% subplot(2,1,1)
% % plot(sub1_theta_real,'d-','color',[0.0,0.0,0.0],'DisplayName','sub1 real theta');hold on; grid on;
% % plot(sub1_theta_measurement ,'*-','color',[0.3,0.3,0.9],'DisplayName','sub1 measurement theta');
% plot(calibration_theta_sub1,'*-','color',[0.3,0.3,0.9],'DisplayName','sub1 calibration theta');
% plot(sub1_theta_hat,'*-','color',[0.9,0.3,0.3],'DisplayName','sub1 estimated theta');
% legend('Location','northwest');
% title('theta 1')
% subplot(2,1,2)
% % plot(sub2_theta_real,'d-','color',[0.0,0.0,0.0],'DisplayName','sub2 real theta');hold on; grid on;
% % % plot(sub1_theta_measurement ,'*-','color',[0.3,0.3,0.9],'DisplayName','sub1 measurement theta');
% plot(calibration_theta_sub2,'*-','color',[0.3,0.3,0.9],'DisplayName','sub2 calibration theta');
% plot(sub2_theta_hat,'*-','color',[0.9,0.3,0.3],'DisplayName','sub2 estimated theta');
% legend('Location','northwest');
% title('theta 2')
% 
% figure(7)
% subplot(2,1,1)
% plot(control_input_angular_sub1,'*-','color',[0.9,0.3,0.3],'DisplayName','sub1 angular input');hold on; grid on;
% legend('Location','northwest');
% title('sub1 angular input')
% subplot(2,1,2)
% plot(control_input_angular_sub2,'*-','color',[0.3,0.3,0.9],'DisplayName','sub2 angular input');hold on; grid on;
% legend('Location','northwest');
% title('sub2 angular input')






