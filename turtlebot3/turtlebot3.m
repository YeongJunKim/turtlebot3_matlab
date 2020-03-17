%% KOREA UNIVERCITY
%% author : colson@korea.ac.kr(dud3722000@naver.com)

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
        
        max_vel = [];
        
        
        %% edit application area
        
        x = 0;
        y = 0;
        
        z; %complex number
        initial_theta = 0;
        theta = 0;
        
        traj = []; % trajectory of the robot
        trajmax = 0;
        trajcount = 0;
        
        measurement = [];
        measurementmax = 0;
        measurementcount = 0;
        
        lidar_data = [];
        lidar_rotation_angle = 90;
        lidar_accuracy = 0;
        lidar_seq = 0;
        lidar_xy = [];
        
        fig;
        n_o_f = 0;
        
        % get from imu filtered data
        imu_initial_theta = 0;
        imu_theta = 0;
        
        % get from ahrsv1
        ahrsv1_initial_theta = 0;
        ahrsv1_theta = 0;
        
        % linear angular velocity
        v_l = 0;
        v_a = 0;
        
    end
    methods
        %% function area
        function obj = turtlebot3_init(obj, namespace)
            %% args
            % namespace  : each robot's ROS namespace ex) (ROS_NAMESPACE=colsonbot roslaunch ~)
            %subscriber
            obj.namespace = namespace;
            obj.sub_imu = rossubscriber(strcat(namespace,'/imu'), {@sub_imu_callback, obj});
            obj.sub_scan = rossubscriber(strcat(namespace,'/scan'), {@sub_scan_callback, obj});
            obj.sub_odom = rossubscriber(strcat(namespace,'/odom'), {@sub_odom_callback, obj});
            
            % publisher
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
            if(topicName == "cmd_vel")
            else
            end
        end
        
        
        %% application area
        function r = turtlebot3_init_trajectory(obj, size)
            obj.traj = zeros(2, size);
            obj.trajmax = size;
            obj.trajcount = 1;
            r = "ok";
        end
        function r = turtlebot3_add_trajectory(obj, x, y)
            if(obj.trajcount ~= 0)
                if(obj.trajmax < obj.trajcount)
                    r = 0;
                else
                    obj.traj(:,obj.trajcount) = [x, y];
                    obj.trajcount = obj.trajcount + 1;
                    r = 1;
                end
            end
        end
        function r = turtlebot3_init_2Dmeasurement(obj, size1, size2)
            obj.measurement = zeros(size1, size2);
            obj.measurementmax = size2;
            obj.measurementcount = 1;
            r = "ok";
        end
        function r = turtlebot3_add_2Dmeasurement(obj, data)
            if(obj.measurementcount ~= 0)
                if(obj.measurementmax < obj.measurementcount)
                    r = 0;
                else
                    obj.measurement(:,obj.measurementcount) = data(:);
                    obj.measurementcount = obj.measurementcount + 1;
                    r = 1;
                end
            end
        end
        function r = turtlebot3_init_lidar(obj, accuracy)
           obj.lidar_data = zeros(1, accuracy); 
           obj.lidar_accuracy = accuracy;
           obj.lidar_xy = zeros(2, accuracy);
           obj.lidar_seq = 1;
            r = "ok";
        end
        function r = turtlebot3_init_figure(obj, fig_size)
            %% This method creat figures infinitly.
            % obj.fig = figure('Name',obj.namespace, "IntegerHandle", "off");
            %% Changed method is as follow.
            obj.fig = figure('Name',obj.namespace);
            r = "ok";
        end
        function r = turtlebot3_focusing_figure(obj)
            figure(obj.fig);
            r = "ok";
        end
        function r = turtlebot3_visualize_lidar(obj)
            figure(obj.fig);
            plot(obj.lidar_xy(1,:), obj.lidar_xy(2,:));
            xlim([-5 5]);
            ylim([-5 5]);
            r = "ok";
        end
        function r = turtlebot3_visualize_trajectory(obj)
            figure(obj.fig);
            plot(obj.traj(1,:), obj.traj(2,:));
            r = "ok";
        end
        function r = turtlebot3_lidar_clustering(obj, neighbors)
            
            r = "ok";
        end
        function r = turtlebot3_init_imu_theta(obj, theta)
           obj.imu_initial_theta = theta;
            r = "ok";
        end
        function r = turtlebot3_init_odom_theta(obj, theta)
            obj.initial_theta = theta;
            r = "ok";
        end
        function r = turtlebot3_init_ahrsv1_theta(obj, theta)
            obj.ahrsv1_initial_theta = theta;
            r = "ok";
        end
    end
end

%% callback
function sub_imu_callback(src, msg, obj)
obj.sub_imu_data = msg;
obj.sub_imu_data;
% obj.imu_theta = obj.sub_imu_data.Orientation.W;
imu = obj.sub_imu_data.Orientation;
temp = [imu.X imu.Y imu.Z imu.W];
temp = quat2eul(temp, 'ZYX');
obj.imu_theta = temp(3);

end
function sub_scan_callback(src, msg, obj)
obj.sub_scan_data = msg;
if(obj.lidar_seq ~= 0)
    obj.lidar_seq = obj.lidar_seq + 1;
    obj.lidar_data(:) = obj.sub_scan_data.Ranges(:);
    x = obj.lidar_data(:)' .* cos(deg2rad(1+obj.lidar_rotation_angle:obj.lidar_accuracy+obj.lidar_rotation_angle));
    y = obj.lidar_data(:)' .* sin(deg2rad(1+obj.lidar_rotation_angle:obj.lidar_accuracy+obj.lidar_rotation_angle));
    obj.lidar_xy(1,:) = x(:); obj.lidar_xy(2,:) = y(:);
end

end
function sub_odom_callback(src, msg, obj)
obj.sub_odom_data = msg;
odom = obj.sub_odom_data.Pose.Pose;
x_ = odom.Position.X;
y_ = odom.Position.Y;
z_ = x_ + 1i * y_;
obj.x = x_;
obj.y = y_;
obj.z = z_;
obj.v_l = obj.sub_odom_data.Twist.Twist.Linear.X;
obj.v_a = obj.sub_odom_data.Twist.Twist.Angular.Z;
temp = [odom.Orientation.X, odom.Orientation.Y, odom.Orientation.Z, odom.Orientation.W];
temp = quat2eul(temp, 'ZYX');
obj.theta =  temp(3);
end
