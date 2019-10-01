clc;
clear all;
close all;
%% setting
master_ip = ('localhost');
robot_ip = ["192.168.0.3", "192.168.0.4", "192.168.0.5"];       % add more...
robot_namespace =["/tb3_0", "/tb3_1", "/tb3_2"]; % add more...
time_interval = 0.5;

%% run
addpath('turtlebot3');
addpath('turtlebot3_ros');
addpath('turtlebot3_localization');
addpath(genpath('../matlab/filters'));

robots = turtlebot3;
robot_num = size(robot_ip, 2);

ros_deinit();
ros_init(master_ip);
for i = 1:robot_num
    robots(i) = turtlebot3; 
    turtlebot3_init(robots(i),robot_namespace(i));
end

variables.run = 1;

tm = timer('BusyMode', 'drop', 'ExecutionMode', 'fixedRate', 'Period', time_interval, 'TimerFcn', {@timer_, robots, variables});
start(tm);



