%% Load measurement data
clear all
clc

path = '../../matlab/filters/';
addpath(genpath(path));

Localization_EKF = SETTING("EKF");
Localization_PEFFME = SETTING("PEFFME");

scenario = 1;
% 1: Ideal case
% 2: Kidnapped case
% 3: Measurement missing case

if scenario == 1; load('../../matlab/filters/sample data/real_exp_ideal.mat');
elseif scenario == 2; load('../../matlab/filters/sample data/real_exp_kidnap.mat');
elseif scenario == 3; load('../../matlab/filters/sample data/real_exp_missing.mat');
    measurement_data(1:4,71:98) = 0;
    measurement_data(1:4,234:258) = 0;
end

num = size(measurement_data,2);

if scenario == 1
    num_start = 19;
    num_finish = 319;
    interval_of_interest = num_start:num_finish;    % Error analysis �� �̷������ time step
    interval_total = num_start:num_finish;          % 2D plot�� ����� �Ǵ� time step
    time_start = measurement_data(6,num_start);
    time_finish = measurement_data(6,num_finish);
    pose_start = [0.40 0.96 0.78]';
    pose_finish = [4.40 5.21 0.85]';
    real_data = zeros(3,num_finish);
    for i = 1:num
        ith_time = measurement_data(6,i);
        duration = time_finish - time_start;
        s = (time_finish - ith_time) / duration;
        real_data(:,i) = s * pose_start + (1-s) * pose_finish;
    end
elseif scenario == 2
    num_start1 = 24;
    num_finish1 = 276;
    num_start2 = 316;
    num_finish2 = 503;
    interval_of_interest = [num_start1:num_finish1, num_start2:num_finish2];    % Error analysis �� �̷������ time step
    interval_total = num_start1:num_finish2;                                    % 2D plot�� ����� �Ǵ� time step
    time_start1 = measurement_data(6,num_start1);
    time_finish1 = measurement_data(6,num_finish1);
    time_start2 = measurement_data(6,num_start2);
    time_finish2 = measurement_data(6,num_finish2);
    pose_start1 = [0.40 0.65 -0.03]';
    pose_finish1 = [5.32 0.63 -0.05]';
    pose_start2 = [1.36 4.20 -0.03]';
    pose_finish2 = [5.10 4.40 0.01]';
    real_data = zeros(3,num_finish2);
    for i = 1:num_finish1
        ith_time = measurement_data(6,i);
        duration1 = time_finish1 - time_start1;
        s = (time_finish1 - ith_time) / duration1;
        real_data(:,i) = s * pose_start1 + (1-s) * pose_finish1;
    end
    for i = num_start2:num
        ith_time = measurement_data(6,i);
        duration2 = time_finish2 - time_start2;
        s = (time_finish2 - ith_time) / duration2;
        real_data(:,i) = s * pose_start2 + (1-s) * pose_finish2;
    end
elseif scenario == 3
    num_start = 17;
    num_finish = 315;
    interval_of_interest = num_start:num_finish;    % Error analysis �� �̷������ time step
    interval_total = num_start:num_finish;          % 2D plot�� ����� �Ǵ� time step
    time_start = measurement_data(6,num_start);
    time_finish = measurement_data(6,num_finish);
    pose_start = [0.35 0.85 0.78]';
    pose_finish = [4.23 5.12 0.86]';
    real_data = zeros(3,num_finish);
    for i = 1:num
        ith_time = measurement_data(6,i);
        duration = time_finish - time_start;
        s = (time_finish - ith_time) / duration;
        real_data(:,i) = s * pose_start + (1-s) * pose_finish;
    end
end

%% Position information of Anchors
distance = 0.6 * 10;
x1 = 0; y1 = 0;
x2 = 0; y2 = distance;
x3 = distance; y3 = distance;
x4 = distance; y4 = 0;

%% Construct Jacobian Matrices
syms x y theta      % state variables
syms u1 u2          % input variables, u1: �̵� �Ÿ�, u2: ���� ��ȭ

f1(x,y,theta,u1,u2) = x + u1 * cos(theta + 1/2 * u2);
f2(x,y,theta,u1,u2) = y + u1 * sin(theta + 1/2 * u2);
f3(x,y,theta,u1,u2) = theta + u2;
f = [f1 f2 f3]';
clear f1 f2 f3
Jacobian_F = jacobian(f,[x y theta]);

h1(x,y,theta,u1,u2) = sqrt((x-x1)^2 + (y-y1)^2) - sqrt((x-x2)^2 + (y-y2)^2);
h2(x,y,theta,u1,u2) = sqrt((x-x1)^2 + (y-y1)^2) - sqrt((x-x3)^2 + (y-y3)^2);
h3(x,y,theta,u1,u2) = sqrt((x-x1)^2 + (y-y1)^2) - sqrt((x-x4)^2 + (y-y4)^2);
h4(x,y,theta,u1,u2) = theta;
h = [h1 h2 h3 h4]';
clear h1 h2 h3 h4
Jacobian_H = jacobian(h,[x y theta]);

f = matlabFunction(f);
Jacobian_F = matlabFunction(Jacobian_F);
h = matlabFunction(h);
Jacobian_H = matlabFunction(Jacobian_H);

%% Localization
i = 1;

x_EKF_data = zeros(3,num);
x_PEFFME_data = zeros(3,num);

x_hat_EKF = [3 3 0]';
x_hat_PEFFME = [3 3 0]';

while(1)
    disp(i);
    
    %%% ���� ����(d1,d2,d3,d4), ����� �κ������� control command ����(delta_d, delta_theta)�� ���� �ڸ� %%%
%     z1 = d1 - d2;
%     z2 = d1 - d3;
%     z3 = d1 - d4;
%     z = [z1 z2 z3]';
%     control = [delta_d delta_theta]';

    d1 = measurement_data(1,i);
    d2 = measurement_data(2,i);
    d3 = measurement_data(3,i);
    d4 = measurement_data(4,i);
    
    if d1*d2*d3*d4 == 0
        alpha = 0;
    else
        alpha = 1;
    end

	z1 = d1 - d2;
    z2 = d1 - d3;
    z3 = d1 - d4;
    z4 = measurement_data(5,i);
    
    z = [z1 z2 z3 z4]';
    control = [0.02 0]';
    P = eye(3);
    Q = blkdiag(0.01,0.01,0.01);
    R = blkdiag(0.2,0.2,0.2,0.1);
    
    tic
    x_hat_EKF = EKF_main(f,h, Jacobian_F,Jacobian_H,x_hat_EKF,z,control, P,Q,R);
    elapsed_time_EKF_data(:,i) = toc;
    tic
    x_hat_PEFFME = PEFFME_main(f, h, Jacobian_F, Jacobian_H, x_hat_PEFFME, z, control, alpha, 10);
    elapsed_time_PEFFME_data(:,i) = toc;
        
	x_EKF_data(:,i) = x_hat_EKF;
    x_PEFFME_data(:,i) = x_hat_PEFFME;
    
    if i == num
        break;
    end
    i = i+1;
end

%% Estimation error analysis
error_EKF_data = zeros(3,num);
error_PEFFME_data = zeros(3,num);

for i = interval_of_interest
    error_EKF_data(:,i) = real_data(:,i) - x_EKF_data(:,i);
    error_PEFFME_data(:,i) = real_data(:,i) - x_PEFFME_data(:,i);
end

% Position RMSE
position_RMSE_EKF = sqrt(mean(error_EKF_data(1,interval_of_interest).^2 + error_EKF_data(2,interval_of_interest).^2));
position_RMSE_PEFFME = sqrt(mean(error_PEFFME_data(1,interval_of_interest).^2 + error_PEFFME_data(2,interval_of_interest).^2));

% Heading angle RMSE
angle_RMSE_EKF = sqrt(mean(error_EKF_data(3,interval_of_interest).^2));
angle_RMSE_PEFFME = sqrt(mean(error_PEFFME_data(3,interval_of_interest).^2));

%% Plot
figure(1);
subplot(3,1,1);
plot(measurement_data(6,interval_of_interest),error_EKF_data(1,interval_of_interest),'*-','color',[0.3 0.3 0.9]); hold on; grid on;
plot(measurement_data(6,interval_of_interest),error_PEFFME_data(1,interval_of_interest),'d-','color',[0.9 0.3 0.3]);
xlabel('(a)');
ylabel('Estimation error (m)');
legend({'EKF','PEFFME (proposed)'},'Location','southeast','NumColumns',2);
subplot(3,1,2);
plot(measurement_data(6,interval_of_interest),error_EKF_data(2,interval_of_interest),'*-','color',[0.3 0.3 0.9]); hold on; grid on;
plot(measurement_data(6,interval_of_interest),error_PEFFME_data(2,interval_of_interest),'d-','color',[0.9 0.3 0.3]);
xlabel('(b)');
ylabel('Estimation error (m)');
subplot(3,1,3);
plot(measurement_data(6,interval_of_interest),error_EKF_data(3,interval_of_interest),'*-','color',[0.3 0.3 0.9]); hold on; grid on;
plot(measurement_data(6,interval_of_interest),error_PEFFME_data(3,interval_of_interest),'d-','color',[0.9 0.3 0.3]);
xlabel('(c)');
ylabel('Estimation error (rad)');

figure(2);
plot(x_EKF_data(1,interval_total),x_EKF_data(2,interval_total),'*-','color',[0.3 0.3 0.9]); hold on; grid on;
plot(x_PEFFME_data(1,interval_total),x_PEFFME_data(2,interval_total),'d-','color',[0.9 0.3 0.3]);
plot(real_data(1,interval_of_interest),real_data(2,interval_of_interest),'xk'); 
axis equal
xlim([0 distance]); ylim([0 distance]);
xlabel('x (m)');
ylabel('y (m)');
legend({'EKF','PEFFME (proposed)','Real'},'Location','southeast');