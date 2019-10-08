%%%%%%%%%%%%%%%%%%%turtlebot_stop%%%%%%%%%%%5
rosshutdown;
% ip_address = ('192.168.0.101');
ip_address = ('192.168.1.46');

rosinit(ip_address);


[control_publisher, control_msg] = rospublisher('main/cmd_vel', 'geometry_msgs/Twist');
[control_publisher1, control_msg1] = rospublisher('sub1/cmd_vel', 'geometry_msgs/Twist');
[control_publisher2, control_msg2] = rospublisher('sub2/cmd_vel', 'geometry_msgs/Twist');

v = [0 0]';  %% turtlebot stop man~~~~~~~~~~~~~~~~~~~~~~~~~~~
            
linear_vel = v(1);    %%velocity update
angular_vel = v(2);
linear_vel1 = v(1);    %%velocity update
angular_vel1 = v(2);

linear_vel2 = v(1);    %%velocity update
angular_vel2 = v(2);



control_msg.Linear.X = linear_vel;
control_msg.Linear.Y = 0;
control_msg.Linear.Z = 0;

control_msg.Angular.X = 0;
control_msg.Angular.Y = 0;
control_msg.Angular.Z = angular_vel;
control_msg1.Linear.X = linear_vel1;
control_msg1.Linear.Y = 0;
control_msg1.Linear.Z = 0;

control_msg1.Angular.X = 0;
control_msg1.Angular.Y = 0;
control_msg1.Angular.Z = angular_vel1;

control_msg2.Linear.X = linear_vel2;
control_msg2.Linear.Y = 0;
control_msg2.Linear.Z = 0;

control_msg2.Angular.X = 0;
control_msg2.Angular.Y = 0;
control_msg2.Angular.Z = angular_vel2;

send(control_publisher,control_msg);
send(control_publisher1,control_msg1);
send(control_publisher2,control_msg2);
stop(test_timer)
delete(timerfindall)