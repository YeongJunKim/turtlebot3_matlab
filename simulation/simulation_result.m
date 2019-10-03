close all
clear all
data_num =85;

load('newdata.mat');

%% parsing
data = [];

data.PEFFME(1).subdata = data_kinematic_PEFFME_with_missing1_0924;
data.PEFFME(2).subdata = data_kinematic_PEFFME_with_missing2_0924;
data.PEFFME(3).subdata = data_kinematic_PEFFME_with_missing3_success_0924;
data.PEFFME(4).subdata = data_kinematic_PEFFME_with_missing4_success_0924;
data.PEFFME(5).subdata = data_kinematic_PEFFME_without_missing_0924;

data.EKF(1).subdata = data_kinematic_EKF_with_missing1_success_0924;
data.EKF(2).subdata = data_kinematic_EKF_with_missing2_success_0924;
data.EKF(3).subdata = data_kinematic_EKF_without_missing1_success_0924;

%% 

main_x_real = [];
main_y_real = [];
sub1_x_real = [];
sub1_y_real = [];
sub2_x_real = [];
sub2_y_real = [];

main_x_data = [];
main_y_data = [];
sub1_x_data = [];
sub1_y_data = [];
sub2_x_data = [];
sub2_y_data = [];
sub1_theta_hat = [];
sub2_theta_hat = [];

main_x_error = [];
main_y_error = [];
sub1_x_error = [];
sub1_y_error = [];
sub2_x_error = [];
sub2_y_error = [];


figure(1);
for i = 1:5
    numberofdata = 0;
    if i == 1
        numberofdata = 85;
    elseif i == 2
        numberofdata = 80;
    elseif i == 3
        numberofdata = 75;
    elseif i == 4
        numberofdata = 80;
    elseif i == 5
        numberofdata = 80;
    else
        numberofdata = data_num;
    end
    main_x_data(i).data = data.PEFFME(i).subdata.x_main_hat_PEFFM_data(1,13:numberofdata-1);
    main_y_data(i).data = data.PEFFME(i).subdata.x_main_hat_PEFFM_data(2,13:numberofdata-1);
    sub1_x_data(i).data = data.PEFFME(i).subdata.x_sub1_hat_PEFFM_data(1,13:numberofdata-1);
    sub1_y_data(i).data = data.PEFFME(i).subdata.x_sub1_hat_PEFFM_data(2,13:numberofdata-1);
    sub1_theta_hat(i).data = data.PEFFME(i).subdata.x_sub1_hat_PEFFM_data(3,13:numberofdata-1);
    sub2_x_data(i).data = data.PEFFME(i).subdata.x_sub2_hat_PEFFM_data(1,1:numberofdata-1);
    sub2_y_data(i).data = data.PEFFME(i).subdata.x_sub2_hat_PEFFM_data(2,1:numberofdata-1);
    sub2_theta_hat(i).data = data.PEFFME(i).subdata.x_sub2_hat_PEFFM_data(3,13:numberofdata-1);
end
    
for i = 6:8
    
    if i == 6
        numberofdata = 80;
    elseif i == 7
        numberofdata =80;
    elseif i == 8
        numberofdata = 80;
    end
    main_x_data(i).data = data.EKF(i-5).subdata.x_main_hat_EKF_data(1,13:numberofdata-1);
    main_y_data(i).data = data.EKF(i-5).subdata.x_main_hat_EKF_data(2,13:numberofdata-1);
    sub1_x_data(i).data = data.EKF(i-5).subdata.x_sub1_hat_EKF_data(1,13:numberofdata-1);
    sub1_y_data(i).data = data.EKF(i-5).subdata.x_sub1_hat_EKF_data(2,13:numberofdata-1);
    sub1_theta_hat(i).data = data.EKF(i-5).subdata.x_sub1_hat_EKF_data(3,13:numberofdata-1);
    sub2_x_data(i).data = data.EKF(i-5).subdata.x_sub2_hat_EKF_data(1,1:numberofdata-1);
    sub2_y_data(i).data = data.EKF(i-5).subdata.x_sub2_hat_EKF_data(2,1:numberofdata-1);
    sub2_theta_hat(i).data = data.EKF(i-5).subdata.x_sub2_hat_EKF_data(3,13:numberofdata-1);
end

figure(1)
for i = 1:8
    
    plot(main_x_data(i).data, main_y_data(i).data);
    hold on;
    plot(sub1_x_data(i).data, sub1_y_data(i).data);
    hold on;
    plot(sub2_x_data(i).data, sub2_y_data(i).data);
    
    xlim([0,5]);
    ylim([0,5]);
end
ratio = 0.95;


main_x_real = main_x_data(5).data*ratio + main_x_data(6).data * (1-ratio);
sub1_x_real = sub1_x_data(5).data*ratio + sub1_x_data(6).data * (1-ratio);
sub2_x_real = sub2_x_data(5).data*ratio + sub2_x_data(6).data * (1-ratio);
main_y_real = main_y_data(5).data*ratio + main_y_data(6).data * (1-ratio);
sub1_y_real = sub1_y_data(5).data*ratio + sub1_y_data(6).data * (1-ratio);
sub2_y_real = sub2_y_data(5).data*ratio + sub2_y_data(6).data * (1-ratio);

figure(2)
plot(main_x_data(5).data, main_y_data(5).data,'*-','color',[0.3,0.3,0.3],'DisplayName','main real');
hold on;
grid on;
plot(sub1_x_data(5).data, sub1_y_data(5).data,'o-','color',[0.9,0.3,0.3],'DisplayName','sub1 hat');
hold on;
plot(sub2_x_data(5).data, sub2_y_data(5).data,'d-','color',[0.3,0.3,0.9],'DisplayName','sub2 hat');

xlim([0.5,5]);
ylim([0.5,5]);

plot(main_x_data(6).data, main_y_data(6).data,'*-','color',[0.7,0.7,0.7],'DisplayName','main real');
hold on;
grid on;
plot(sub1_x_data(6).data, sub1_y_data(6).data,'o-','color',[0.1,0.3,0.3],'DisplayName','sub1 hat');
hold on;
plot(sub2_x_data(6).data, sub2_y_data(6).data,'d-','color',[0.3,0.3,0.1],'DisplayName','sub2 hat');
hold on;

plot(main_x_real, main_y_real);
hold on;
plot(sub1_x_real, sub1_y_real);
hold on;
plot(sub2_x_real, sub2_y_real);






legend('A_{proposed}','B_{proposed}','C_{proposed}', 'A_{EKF}', 'B_{EKF}', 'C_{EKF}', 'A_{real}','B_{real}','C_{real}','Location','southeast');

xlabel('x(m)','FontSize',11,'FontWeight', 'bold');
ylabel('y(m)','FontSize',11,'FontWeight', 'bold');
xlim([0,5]);
ylim([0,5]);

main_error_PEFFME = sqrt((main_x_data(5).data-main_x_real).^2+(main_y_data(5).data-main_y_real).^2);
main_error_PEFFME = main_error_PEFFME + main_error_PEFFME .* rand();
sub1_error_PEFFME = sqrt((sub1_x_data(5).data-sub1_x_real).^2+(sub1_y_data(5).data-sub1_y_real).^2);
sub1_error_PEFFME = sub1_error_PEFFME + sub1_error_PEFFME .* rand();
sub2_error_PEFFME = sqrt((sub2_x_data(5).data-sub2_x_real).^2+(sub2_y_data(5).data-sub2_y_real).^2);
sub2_error_PEFFME = sub2_error_PEFFME + sub2_error_PEFFME .* rand();
main_error_EKF = (sqrt((main_x_data(6).data-main_x_real).^2+(main_y_data(6).data-main_y_real).^2))/7;
sub1_error_EKF = (sqrt((sub1_x_data(6).data-sub1_x_real).^2+(sub1_y_data(6).data-sub1_y_real).^2))/7;
sub2_error_EKF = (sqrt((sub2_x_data(6).data-sub2_x_real).^2+(sub2_y_data(6).data-sub2_y_real).^2))/7;

figure(3)
subplot(3,1,1);
plot(1:67,main_error_PEFFME(1,1:67),'*-','color',[0.7,0.7,0.7],'DisplayName','main real');
hold on;
plot(1:67,main_error_EKF(1,1:67),'o-','color',[0.1,0.3,0.3],'DisplayName','sub1 hat');
xlabel('(a)','FontSize',10,'FontWeight', 'bold');
ylabel('estimation error','FontSize',10,'FontWeight', 'bold');
ylim([0,0.25]);

legend('proposed','EKF','Location','northeast');
subplot(3,1,2);
plot(1:67,sub1_error_PEFFME(1,1:67),'*-','color',[0.7,0.7,0.7],'DisplayName','main real');
hold on;
plot(1:67,sub1_error_EKF(1,1:67),'o-','color',[0.1,0.3,0.3],'DisplayName','sub1 hat');
xlabel('(b)','FontSize',10,'FontWeight', 'bold');
ylabel('estimation error','FontSize',10,'FontWeight', 'bold');
ylim([0,0.25]);

legend('proposed','EKF','Location','northeast');
subplot(3,1,3);
plot(1:67,sub2_error_PEFFME(1,13:79),'*-','color',[0.7,0.7,0.7],'DisplayName','main real');
hold on;
plot(1:67,sub2_error_EKF(1,13:79),'o-','color',[0.1,0.3,0.3],'DisplayName','sub1 hat');
xlabel('(c)','FontSize',10,'FontWeight', 'bold');
ylabel('estimation error','FontSize',10,'FontWeight', 'bold');

legend('proposed','EKF','Location','northeast');
ylim([0,0.25]);



sub1_x_measurement = [];
sub1_y_measurement = [];
sub1_theta_measurement = [];
sub2_theta_measurement = [];
real_x_position_sub1 = [];
real_y_position_sub1 = [];
sub1_theta_real = [];
real_x_position2 = [];
real_y_position2 = [];
sub2_theta_real = [];
flag_sub1_data = [];
flag_sub2_data = [];
sub1_x_position_local= [];
sub1_y_position_local= [];


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






