global device

linux = 1;
window = 2;
device = linux;

clc;
clear all;
close all;

%% setting
master_ip = ('192.168.0.0');
robot_ip = ["192.168.0.0"];
robot_namespace = ["/tb3_0"];
robot = turtlebot3;

ros_init('localhost');

turtlebot3_init(robot, 'tb3_0');


N = 100;

for i=1:N
    disp(i);
    
    fprintf("real = %f\nimage = %f", real(robot.z), imag(robot.z));
    
end



function ros_init(ip)
    rosshutdown;
    rosinit(ip);
end
