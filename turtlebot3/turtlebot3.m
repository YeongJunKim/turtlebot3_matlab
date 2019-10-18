classdef turtlebot3 < handle
   properties 
       %% variable area
       %% TOPIC
       namespace = "";    
       
       sub_imu = [];
       sub_scan = [];
       sub_odom = [];
       
       sub_added = [];
       sub_type = [];
       sub_data = [];
       addcount = 1;
       
       sub_imu_data;
       sub_scan_data;
       sub_odom_data;
       
       pub_control = [];
       pub_control_msg = [];
       
   end
   methods
       %% function area
       function obj = turtlebot3_init(obj, namespace)
           %% args
           % namespace  : each robot's ROS namespace ex) (ROS_NAMESPACE=colsonbot roslaunch ~)
           obj.namespace = namespace;
           obj.sub_imu = rossubscriber(strcat(namespace,'/imu'), {@sub_imu_callback, obj});
           obj.sub_scan = rossubscriber(strcat(namespace,'/scan'), {@sub_scan_callback, obj});
           obj.sub_odom = rossubscriber(strcat(namespace,'/odom'), {@sub_odom_callback, obj});
           [obj.pub_control, obj.pub_control_msg] = rospublisher(strcat(namespace,'/cmd_vel'), 'geometry_msgs/Twist');
       end
       
       function obj = turtlebot3_addSubscrier(obj, topic, callback)
           obj.sub_type(obj.addcount).data = rossubscriber(topic, {callback, obj});
           obj.sub_added(obj.addcount).data = obj.sub_type;
           obj.addcount = obj.addcount + 1;
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
       
       function r = turtlebot3_pubTopicName(obj, topicName)
           
       end
   end
end

%% callback
function sub_imu_callback(src, msg, obj)
    obj.sub_imu_data = msg;
end
function sub_scan_callback(src, msg, obj)
    obj.sub_scan_data = msg;
end
function sub_odom_callback(src, msg, obj)
    obj.sub_odom_data = msg;
end
