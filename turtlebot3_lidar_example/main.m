clear all
close all
delete all
clc

tmrList = timerfind()
delete(tmrList);
fprintf('ros start\r\n');

    
rosshutdown;
rosinit('http://192.168.0.5:11311/')
sub.turtlebot3_lidar = rossubscriber('/tb3_5/scan','BufferSize', 1);
% sub.turtlebot3_state = rossubscriber('/gazebo/model_states');

topic.lidar = receive(sub.turtlebot3_lidar);


sampling_time = 0.2;

struct.firstRun = 0;


test_timer = timer('Busymode', 'drop', 'ExecutionMode', 'fixedRate', 'Period', sampling_time, 'TimerFcn', {@timer10ms, sub, topic, struct});

start(test_timer);