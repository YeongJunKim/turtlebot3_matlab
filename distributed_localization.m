clc;
clear all;
close all;
%% setting
master_ip = ('localhost');
robot_ip = ["192.168.0.3", "192.168.0.4", "192.168.0.5"];       % add more...
robot_namespace =["/tb3_0", "/tb3_1", "/tb3_2"];                % add more...
time_interval = 0.2;

isdeltetimer = 1;


%% run
addpath('turtlebot3');
addpath('turtlebot3_ros');
addpath('turtlebot3_localization');
addpath(genpath('../matlab/filters'));

robots = turtlebot3;
robot_num = size(robot_ip, 2);                                  % same as size(robot_namespace, 2);

ros_deinit();
ros_init(master_ip);



for i = 1:robot_num
    robots(i) = turtlebot3; 
    turtlebot3_init(robots(i),robot_namespace(i));
end

%% delte all timer
if isdeltetimer == 1
    tmrList = timerfind()
    delete(tmrList);
    pause(2);
end
%% filter init
%% init filter
% Position information of Anchors


% distance = 0.6 * 10;
% x1 = 0; y1 = 0;
% x2 = 0; y2 = distance;
% x3 = distance; y3 = distance;
% x4 = distance; y4 = 0;
% syms x y theta
% syms u1 u2
% f1(x,y,theta,u1,u2) = x + u1 * cos(theta + 1/2 * u2);
% f2(x,y,theta,u1,u2) = y + u1 * sin(theta + 1/2 * u2);
% f3(x,y,theta,u1,u2) = theta + u2;
% f = [f1 f2 f3]';
% clear  f1 f2 f3
% jac_f = jacobian(f, [x, y, theta]);
% h1(x,y,theta,u1,u2) = sqrt((x-x1)^2 + (y-y1)^2) - sqrt((x-x2)^2 + (y-y2)^2);
% h2(x,y,theta,u1,u2) = sqrt((x-x1)^2 + (y-y1)^2) - sqrt((x-x3)^2 + (y-y3)^2);
% h3(x,y,theta,u1,u2) = sqrt((x-x1)^2 + (y-y1)^2) - sqrt((x-x4)^2 + (y-y4)^2);
% h4(x,y,theta,u1,u2) = theta;
% h= [h1 h2 h3 h4]';
% clear h1 h2 h3 h4
% jac_h = jacobian(h,[x y theta]);
% f = matlabFunction(f);
% jac_f = matlabFunction(jac_f);
% h = matlabFunction(h);
% jac_h = matlabFunction(jac_h);
% method = "PEFFME";       
% filters(1) = filtering;
% filtering_init(filters(1), method, f, h, jac_f, jac_h);
% filtering_set_h_size(filters(1), 5);
% filters(1).x_hat_pre = [3 3 0]';
% filters(2) = filtering;
% filters(3) = filtering;





tm = timer('BusyMode', 'drop', 'ExecutionMode', 'fixedRate', 'Period', time_interval, 'TimerFcn', {@timer_, robots});
start(tm);

