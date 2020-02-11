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
        
        
        
        
        %% edit application area
        x = 0;
        y = 0;
        
        z; %complex number
        theta;
        
        traj = []; % trajectory of the robot
        trajmax = 0;
        trajcount = 1;
        measurement = [];
        measurementmax = 0;
        measurementcount = 1;
        
    end
    methods
        %% function area
        function obj = turtlebot3_init(obj, namespace)
            %% args
            % namespace  : each robot's ROS namespace ex) (ROS_NAMESPACE=colsonbot roslaunch ~)
            %subscriber
            obj.namespace = namespace;
            %            obj.sub_imu = rossubscriber(strcat(namespace,'/imu'), {@sub_imu_callback, obj});
            %            obj.sub_scan = rossubscriber(strcat(namespace,'/scan'), {@sub_scan_callback, obj});
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
            
        end
        
        function r = turtlebot3_init_trajectory(obj, size)
            obj.traj = zeros(2, size);
            obj.trajmax = size;
            obj.trajcount = 1;
        end
        function r = turtlebot3_add_trajectory(obj, x, y)
            if(obj.trajmax < obj.trajcount)
                r = 0;
            else
                obj.traj(:,obj.trajcount) = [x, y];
                obj.trajcount = obj.trajcount + 1;
                r = 1;
            end
        end
        function r = turtlebot3_init_2Dmeasurement(obj, size1, size2)
           obj.measurement = zeros(size1, size2);
           obj.measurementmax = size2;
           obj.measurementcount = 1;
        end
        function r = turtlebot3_add_2Dmeasurement(obj, data)
            if(obj.measurementmax < obj.measurementcount)
                r = 0;
            else
                obj.measurement(:,obj.measurementcount) = data(:);
                obj.measurementcount = obj.measurementcount + 1;
                r = 1;
            end
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

odom = obj.sub_odom_data.Pose.Pose;

x_  = odom.Position.X;
y_ = odom.Position.Y;
z_ = x_ + 1i * y_;
obj.x = x_;
obj.y = y_;
obj.z = z_;

temp = [odom.Orientation.X, odom.Orientation.Y, odom.Orientation.Z, odom.Orientation.W];
temp = quat2eul(temp, 'ZYX');
obj.theta =  temp(3);
end
