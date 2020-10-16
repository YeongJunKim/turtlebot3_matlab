clear all
close all

global ROBOT


addpath('./../../matlab_utils');

delete_timer();
ros_deinit();
pause(3); % wait for rosshutdown.
rosinit('192.168.0.5');

turtlebot3_names = ["tb3a", "tb3b", "tb3c", "tb3d", "tb3e", "tb3f"];

ROBOT = turtlebot3;
turtlebot3_init(ROBOT, "tb3d");



tm = timer('BusyMode', 'drop', 'ExecutionMode', 'fixedRate', 'Period', 0.1, 'TimerFcn', {@timer_0});
start(tm);