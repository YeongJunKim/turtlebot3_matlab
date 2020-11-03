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
        sub_imu = []; sub_scan = []; sub_odom = []; sub_ahrsv1 = []; sub_input = [];
        
        % sub_data
        sub_input_data = []; sub_imu_data = []; sub_scan_data = []; sub_odom_data = []; sub_ahrsv1_data = []; lidar_data = [];
        
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
        
        
        cmd_vel = [];
        
        run_time_info;
        
    end
    methods
        %% function area
        function obj = turtlebot3_default(namespace, iscallbcak, isknown)
            % arguments
            % namespace  : each robot's ROS namespace ex) (ROS_NAMESPACE=colsonbot roslaunch ~)
            disp(namespace)
            %subscriber
            obj.namespace = namespace;
            obj.cmd_vel = zeros(2,1);
            if iscallbcak == 1
                obj.sub_imu = rossubscriber(strcat(namespace,'/imu'), {@sub_imu_callback, obj});
                obj.sub_scan = rossubscriber(strcat(namespace,'/scan'), {@sub_scan_callback, obj});
                obj.sub_odom = rossubscriber(strcat(namespace,'/odom'), {@sub_odom_callback, obj});
                obj.sub_ahrsv1 = rossubscriber(strcat(namespace,'/mw_ahrsv1/imu'), {@sub_ahrsv1_callback, obj});
                obj.sub_input  = rossubscriber(strcat(namespace,'/cmd_vel'), {@sub_cmd_vel_callbcak, obj});
            else
                obj.sub_imu = rossubscriber(strcat(namespace,'/imu'));
                if isknown ~= 1
                    obj.sub_scan = rossubscriber(strcat(namespace,'/scan'));
                end
                obj.sub_odom = rossubscriber(strcat(namespace,'/odom'));
                obj.sub_ahrsv1 = rossubscriber(strcat(namespace,'/mw_ahrsv1/imu'));
                obj.sub_input  = rossubscriber(strcat(namespace,'/cmd_vel'), {@sub_cmd_vel_callbcak, obj});
            end
            
            % publisher
            [obj.pub_control, obj.pub_control_msg] = rospublisher(strcat(namespace,'/cmd_vel'), 'geometry_msgs/Twist');
        end
        
        function obj = turtlebot3_addSubscriver(obj, topic, callback, flag)
            disp("add subscriber");
            
            if flag == 1
                obj.sub_type(obj.addcount).data = rossubscriber(topic, {callback, obj});
            else
                obj.sub_type(obj.addcount).data = rossubscriber(topic);
            end
            %             if(callback ~= 0)
            %                 obj.sub_type(obj.addcount).data = rossubscriber(topic, {callback, obj});
            %             else
            %                 obj.sub_type(obj.addcount).data = rossubscriber(topic);
            %             end
            %
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

function sub_cmd_vel_callbcak(src, msg, obj)
sub_cmd_vel = msg;
v_l = sub_cmd_vel.Linear.X;
v_a = sub_cmd_vel.Angular.Z;
temp = [v_l v_a]';
obj.cmd_vel = temp;

end

function sub_imu_callback(src, msg, obj)
obj.sub_imu_data = msg;
imu = obj.sub_imu_data.Orientation;
temp = [imu.X imu.Y imu.Z imu.W];
temp = quat2eul(temp, 'ZYX');
obj.imu_theta = temp(3);
end
function sub_scan_callback(src, msg, obj)
obj.sub_scan_data = msg;
obj.lidar_seq = obj.lidar_seq + 1;
obj.lidar_data = obj.sub_scan_data.Ranges(:);
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
end
function sub_ahrsv1_callback(src, msg, obj)
obj.sub_ahrsv1_data = msg;
ahrsv1 = obj.sub_ahrsv1_data.Orientation;
obj.ahrsv1 = ahrsv1.Z;
end
