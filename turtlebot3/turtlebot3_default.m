% author: Yeong Jun Kim(colson)
% email: colson@korea.ac.kr || dud3722000@naver.com
% date: 2020-07-31
% For turtlebot3 experiment with Matlab,

% Usage :
% robot = turtlebot3
% namespace = ""
% robot = turtlebot3_default(namespace);
%


classdef turtlebot3_default < handle
    properties
        %% TOPIC
        
        % namespace dfault is "";
        namespace = "";
        % default subscriber
        sub_imu = []; sub_scan = []; sub_odom = []; sub_ahrsv1 = [];
        
        % sub_data
        sub_imu_data = []; sub_scan_data = []; sub_odom_data = []; sub_ahrsv1_data = []; lidar_data = [];
        
        % lidar
        lidar_rotation_angle = 90;
        lidar_accuracy = 0;
        lidar_seq = 0;
        lidar_xy = [];
        
        % additional subscriber
        sub_added = []; sub_type = []; sub_data = []; addcount = 1;
        
        %  publisher
        pub_control = []; pub_control_msg = [];
        
        %% edit application area
        x = 0;
        y = 0;
        
        initial_theta = 0;
        theta = 0;
        ahrsv1 = 0;
        % get from imu filtered data
        imu_initial_theta = 0;
        imu_theta = 0;
        
        % linear angular velocity
        u = [];
        v_l = 0;
        v_a = 0;
        
        run_time_info;
        
    end
    methods
        %% function area
        function obj = turtlebot3_default(namespace)
            % arguments
            % namespace  : each robot's ROS namespace ex) (ROS_NAMESPACE=colsonbot roslaunch ~)
            disp(namespace)
            %subscriber
            obj.namespace = namespace;
            obj.sub_imu = rossubscriber(strcat(namespace,'/imu'), {@sub_imu_callback, obj});
            obj.sub_scan = rossubscriber(strcat(namespace,'/scan'), {@sub_scan_callback, obj});
            obj.sub_odom = rossubscriber(strcat(namespace,'/odom'), {@sub_odom_callback, obj});
            obj.sub_ahrsv1 = rossubscriber(strcat(namespace,'/mw_ahrsv1/imu'), {@sub_ahrsv1_callback, obj});
            
            % publisher
            [obj.pub_control, obj.pub_control_msg] = rospublisher(strcat(namespace,'/cmd_vel'), 'geometry_msgs/Twist');
        end
        
        function obj = turtlebot3_addSubscriver(obj, topic, callback)
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
        %% application area
        
    end
end

%% callback
function sub_imu_callback(src, msg, obj)
obj.sub_imu_data = msg;
% obj.imu_theta = obj.sub_imu_data.Orientation.W;
% Need to convert quat to eul
% obj.sub_imu_data
% obj.sub_imu_data.LinearVelocity
imu = obj.sub_imu_data.Orientation;
temp = [imu.X imu.Y imu.Z imu.W];
temp = quat2eul(temp, 'ZYX');
obj.imu_theta = temp(3);
if obj.run_time_info == 1
    obj.imu_theta(end+1) = toc;
end
end
function sub_scan_callback(src, msg, obj)
obj.sub_scan_data = msg;
% if(obj.lidar_seq ~= 0)
    obj.lidar_seq = obj.lidar_seq + 1;
    obj.lidar_data(1:360) = obj.sub_scan_data.Ranges(:);
%     x = obj.lidar_data(:)' .* cos(deg2rad(1+obj.lidar_rotation_angle:obj.lidar_accuracy+obj.lidar_rotation_angle));
%     y = obj.lidar_data(:)' .* sin(deg2rad(1+obj.lidar_rotation_angle:obj.lidar_accuracy+obj.lidar_rotation_angle));
%     obj.lidar_xy(1,:) = x(:); obj.lidar_xy(2,:) = y(:);
%     if obj.run_time_info == 1
        obj.lidar_data(361) = toc;
%     end
%     disp(obj.namespace);
% end
end
function sub_odom_callback(src, msg, obj)
obj.sub_odom_data = msg;
odom = obj.sub_odom_data.Pose.Pose;
x_ = odom.Position.X;
y_ = odom.Position.Y;
obj.x = x_;
obj.y = y_;
obj.v_l = obj.sub_odom_data.Twist.Twist.Linear.X;
obj.v_a = obj.sub_odom_data.Twist.Twist.Angular.Z;
temp = [odom.Orientation.X, odom.Orientation.Y, odom.Orientation.Z, odom.Orientation.W];
temp = quat2eul(temp, 'ZYX');
obj.theta =  temp(3);
if obj.run_time_info == 1
    obj.v_l(end+1) = toc;
    obj.v_a(end+1) = toc;
    obj.theta(end+1) = toc;
end
end
function sub_ahrsv1_callback(src, msg, obj)
obj.sub_ahrsv1_data = msg;
ahrsv1 = obj.sub_ahrsv1_data.Orientation;
obj.ahrsv1 = ahrsv1.Z;
if obj.run_time_info == 1
    obj.ahrsv1(end+1) = toc;
end
end
