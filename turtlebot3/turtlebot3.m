classdef turtlebot3 < handle
   properties 
       %% variable area
       %% TOPIC
       namespace = "";    
       
       sub_imu = [];
       sub_scan = [];
       sub_odom = [];
       
       sub_imu_data = [];
       sub_scan_data = [];
       
       pub_control = [];
       pub_control_msg = [];
       
       
   end
   methods
       %% function area
       function obj = turtlebot3_init(obj, namespace)
           %% args
           % namespace  : each robot's ROS namespace ex) (ROS_NAMESPACE=colsonbot roslaunch ~)
           obj.namespace = namespace;
           obj.sub_imu = rossubscriber(strcat(namespace,'/imu'));
           obj.sub_scan = rossubscriber(strcat(namespace,'/scan'));
           obj.sub_odom = rossubscriber(strcat(namespace,'/odom'));
           [obj.pub_control, obj.pub_control_msg] = rospublisher(strcat(namespace,'/cmd_vel'), 'geometry_msgs/Twist');
       end
       function r = turtlebot3_readTopicName(obj, topicName)
           if topicName == "imu"
               r = receive(obj.sub_imu);
           elseif topicName == "scan"
               r = receive(obj.sub_scan);
           else
               temp = rossubscriber(strcat(obj.namespace, "/",topicName));
               r = receive(temp);
           end
       end
   end
end