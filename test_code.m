clc;
clear all;
close all;
%% setting
master_ip = ('192.168.0.5');
robot_ip = ["192.168.0.3", "192.168.0.4", "192.168.0.5"];       % add more...
robot_namespace =["/tb3_0", "/tb3_1", "/tb3_2"];                % add more...
time_interval = 0.2;

isdeltetimer = 1;

tile_size = 20;

%% run
addpath('turtlebot3');
addpath('turtlebot3_ros');
addpath('turtlebot3_localization');
addpath(genpath('../matlab/filters'));

robots = turtlebot3;
robot_num = size(robot_ip, 2);                                  % same as size(robot_namespace, 2);

ros_deinit();
ros_init(master_ip);


%% robot init
for i = 1:robot_num
    robots(i) = turtlebot3; 
    turtlebot3_init(robots(i),robot_namespace(i));
end

filters = filter_init(tile_size, time_interval, robot_num);


%% delte all timer
if isdeltetimer == 1
    tmrList = timerfind()
    delete(tmrList);
    pause(2);
end

%% timer run
tm = timer('BusyMode', 'drop', 'ExecutionMode', 'fixedRate', 'Period', time_interval, 'TimerFcn', {@timer_, robots, filters});
start(tm);


