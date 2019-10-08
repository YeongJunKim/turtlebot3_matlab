classdef turtle_bot < handle
    properties
        %publisher, subscriber
        %%%%%%for main %%%%%%%%%%%%%%%%
        %%%% 1. topic %%%%%            
        subscriber_main=[];
        message_main = [];
        distance_main = [];
        distance_msg_main = [];
        control_publisher_main = [];
        control_msg_main = [];
        imu_main = [];
        imu_msg_main = [];
        imu_data_main = [];
        serial_main = [];
        serial_msg_main = [];
        %%%% 2. imu %%%%
         imu_step_main = 1;
        save_imu_stack_main = 0;
        imu_calibration_value_main = 0;
        yaw_angle_main = 0; 
        %%%% 3. Lidar %%%%   
        lidar_main = [];
        raw_position_main = [];
        measurement_main = zeros(1,4);
        pre_measurement_main = zeros(1,4);
        %%%% 4. control %%%%
        control_main = [];
        angular_vel_main = 0;
        linear_vel_main = 0;
        
        %%%%%%for sub1 %%%%%%%%%%%%%%%%%%
        %%%% 1. topic %%%%
        subscriber_sub1=[];
        message_sub1 = [];
        distance_sub1 = [];
        distance_msg_sub1 = [];
        control_publisher_sub1 = [];
        control_msg_sub1 = [];
        imu_sub1 = [];
        imu_msg_sub1 = [];
        imu_data_sub1 = [];
        %%%% 2, imu %%%%
        imu_step_sub1 = 1;
        save_imu_stack_sub1 = 0;
        imu_calibration_value_sub1 = 0.0;
        yaw_angle_sub1 = 0; 
        %%%% 3. lidar %%%%
        lidar_sub1 = [];
        raw_position_sub1 = [];
        measurement_sub1 = zeros(1,3);
        global_position_sub1 = zeros(1,2);
        pre_measurement_sub1 = zeros(1,3);
        distance_measurement_sub1 = zeros(1,2);
        local_main_position_sub1=[];
        local_sub_position_sub1=[];
        predict_next_local_main_position_sub1=[];
        lidar_data_sub1 = [];
        lidar_x_position_sub1 = zeros(360,1);
        lidar_y_position_sub1 = zeros(360,1);
        %%%% 4. control %%%%
        control_state_PEFFME_sub1= zeros(3,1);
        control_sub1 = [];
        plot_measurement_sub1 = zeros(2,1);
        alpha_sub1 = 0;
        angular_vel_sub1 = 0;
        linear_vel_sub1 = 0;
        
        %%%%%%for sub2 %%%%%%%%%%%%%%%%%%
        %%%% 1. topic %%%%
        subscriber_sub2=[];
        message_sub2 = [];
        distance_sub2 = [];
        distance_msg_sub2 = [];
        control_publisher_sub2 = [];
        control_msg_sub2 = [];
        imu_sub2 = [];
        imu_msg_sub2 = [];
        imu_data_sub2 = [];
        %%%% 2. imu %%%%
        imu_step_sub2 = 1;
        save_imu_stack_sub2 = 0;
        imu_calibration_value_sub2 = -0.5;
        yaw_angle_sub2 = 0; 
        %%%% 3. Lidar %%%%
        lidar_sub2 = [];
        raw_position_sub2 = [];
        measurement_sub2 = zeros(1,3);
        global_position_sub2 = zeros(1,2);
        pre_measurement_sub2 = zeros(1,3);
        distance_measurement_sub2 = zeros(1,2);
        local_main_position_sub2=[];
        local_sub_position_sub2=[];
        predict_next_local_main_position_sub2=[];
        lidar_data_sub2 = [];
        lidar_x_position_sub2 = zeros(360,1);
        lidar_y_position_sub2 = zeros(360,1);
        %%%% 4. control %%%%
        control_state_PEFFME_sub2 = zeros(3,1);
        control_sub2 = [];
        plot_measurement_sub2 = zeros(2,1);
        alpha_sub2 = 0;
        angular_vel_sub2 = 0;
        linear_vel_sub2 = 0;
  
        %%%%%%%%% common variable %%%%%
        lim_range = 3;  %%% lidar detection range
        
        %%% filter  variable
         filter_input = [];
        f_main = [];
        h_main = [];
        f_sub = [];
        h_sub = [];
         flag = [];
        Jacobian_F_main = [];
        Jacobian_H_main = [];
        Jacobian_F_sub = [];
        Jacobian_H_sub = [];
        x_main_hat_PEFFME = zeros(3,1);
        x_sub1_hat_PEFFME = zeros(6,1);
        x_sub2_hat_PEFFME = zeros(6,1);
        x_main_hat_EKF = zeros(3,1);
        x_sub1_hat_EKF = zeros(6,1);
        x_sub2_hat_EKF = zeros(6,1);
        real_sub1_state = zeros(3,1);
        real_sub2_state = zeros(3,1);
        sampling_time = [];
        trajectory_step = 0;
        
        %%%%% control variable %%%%%%%
       
        xd = 0;
        yd = 0;  
        xd_dot = 0;
        yd_dot = 0;
        count = 1;
%         v_linear_max = 0.1;
%         v_angle_max = 0.3;
        v_linear_max = 0.22;
        v_angle_max = 1.82;
        %%%% figure variable %%%%%%%
        plot_figure = [];
        plot_figure2 = [];
        plot_figure_lidar_sub1 =[];
        plot_figure_lidar_sub2 =[];
        secondRun =[];
        tile_size = 0.6;
        tile_num = 9;
        pre_theta_sub1 =0;
        rotation_count_sub1 = 0;
        calibration_theta_sub1 = 0;
    end
    
    methods
        function obj = turtle_bot(ip, main_name, sub1_name, sub2_name, sampling_time)  % pub_name, param_fcn)
            rosshutdown;
            rosinit(ip);
            obj.sampling_time = sampling_time;
            obj.subscriber_main = rossubscriber(main_name);
            obj.subscriber_sub1 = rossubscriber(sub1_name);
            obj.subscriber_sub2 = rossubscriber(sub2_name);
%             obj.distance_main = rossubscriber('/tb3_0/scan');
%             obj.distance_sub1 = rossubscriber('/tb3_1/scan');
%             obj.distance_sub2 = rossubscriber('/tb3_2/scan');
%             obj.imu_main = rossubscriber('/tb3_0/imu');
%             obj.imu_sub1 = rossubscriber('/tb3_1/imu');
%             obj.imu_sub2 = rossubscriber('/tb3_2/imu');
%             [obj.control_publisher_main, obj.control_msg_main] = rospublisher('/tb3_0/cmd_vel', 'geometry_msgs/Twist');
%             [obj.control_publisher_sub1, obj.control_msg_sub1] = rospublisher('/tb3_1/cmd_vel', 'geometry_msgs/Twist');
%             [obj.control_publisher_sub2, obj.control_msg_sub2] = rospublisher('/tb3_2/cmd_vel', 'geometry_msgs/Twist');
            obj.distance_main = rossubscriber('/main/scan');
            obj.distance_sub1 = rossubscriber('/sub1/scan');
            obj.distance_sub2 = rossubscriber('/sub2/scan');
            obj.imu_main = rossubscriber('/main/imu');
            obj.imu_sub1 = rossubscriber('/sub1/imu');
            obj.imu_sub2 = rossubscriber('/sub2/imu');
            [obj.control_publisher_main, obj.control_msg_main] = rospublisher('/main/cmd_vel', 'geometry_msgs/Twist');
            [obj.control_publisher_sub1, obj.control_msg_sub1] = rospublisher('/sub1/cmd_vel', 'geometry_msgs/Twist');
            [obj.control_publisher_sub2, obj.control_msg_sub2] = rospublisher('/sub2/cmd_vel', 'geometry_msgs/Twist');
% %             
            %% filter initialization
            global initial_position_main initial_position_sub1 initial_position_sub2
            initial_position_main = [obj.tile_size*2, obj.tile_size*2]';
            initial_position_sub1 = [obj.tile_size*2, obj.tile_size*3]';
            initial_position_sub2 = [obj.tile_size*2 obj.tile_size*1]';
            obj.x_main_hat_PEFFME = [initial_position_main' 0]';
            obj.x_sub1_hat_PEFFME = [initial_position_main' 0 initial_position_sub1' 0]';
            obj.x_sub2_hat_PEFFME = [initial_position_main' 0 initial_position_sub2' 0]';
            obj.x_main_hat_EKF = [initial_position_main' 0]';
            obj.x_sub1_hat_EKF = [initial_position_main' 0 initial_position_sub1' 0]';
            obj.x_sub2_hat_EKF = [initial_position_main' 0 initial_position_sub2' 0]';
            
            %% Position informaiton of Anchors m   
            x1 = 0;                  y1 = 0;
            x2 = 0;                  y2 = obj.tile_size*obj.tile_num;
            x3 = obj.tile_size*obj.tile_num; y3 = obj.tile_size*obj.tile_num;
            x4 = obj.tile_size*obj.tile_num; y4 = 0;
            
            %% State and Measurement equations
            syms x y theta
            syms v_l v_theta
            syms x_sub y_sub theta_sub
            syms v_l_sub v_theta_sub
            dt = 1;
            % State of main robot
            obj.f_main.x(x,y,theta,v_l,v_theta) = x + v_l * cos(theta) * dt;
            obj.f_main.y(x,y,theta,v_l,v_theta) = y + v_l * sin(theta) * dt;
            obj.f_main.theta(x,y,theta,v_l,v_theta) = theta + v_theta * dt;
            obj.f_main = [obj.f_main.x obj.f_main.y obj.f_main.theta]';
            obj.Jacobian_F_main = jacobian(obj.f_main,[x,y,theta]);
            obj.f_main = matlabFunction(obj.f_main);
            obj.Jacobian_F_main = matlabFunction(obj.Jacobian_F_main);
            
            % State of sub robots
            obj.f_sub.x_main(x,y,theta,x_sub,y_sub,theta_sub,v_l,v_theta,v_l_sub,v_theta_sub) = x + v_l * cos(theta) * dt;
            obj.f_sub.y_main(x,y,theta,x_sub,y_sub,theta_sub,v_l,v_theta,v_l_sub,v_theta_sub) = y + v_l * sin(theta) * dt;
            obj.f_sub.theta_main(x,y,theta,x_sub,y_sub,theta_sub,v_l,v_theta,v_l_sub,v_theta_sub) = theta + v_theta * dt;
            obj.f_sub.x_sub(x,y,theta,x_sub,y_sub,theta_sub,v_l,v_theta,v_l_sub,v_theta_sub) = x_sub + v_l_sub * cos(theta_sub) * dt;
            obj.f_sub.y_sub(x,y,theta,x_sub,y_sub,theta_sub,v_l,v_theta,v_l_sub,v_theta_sub) = y_sub + v_l_sub * sin(theta_sub) * dt;
            obj.f_sub.theta_sub(x,y,theta,x_sub,y_sub,theta_sub,v_l,v_theta,v_l_sub,v_theta_sub) = theta_sub + v_theta_sub * dt;
            obj.f_sub = [obj.f_sub.x_main obj.f_sub.y_main obj.f_sub.theta_main obj.f_sub.x_sub obj.f_sub.y_sub obj.f_sub.theta_sub]';
            obj.Jacobian_F_sub = jacobian(obj.f_sub,[x,y,theta,x_sub,y_sub,theta_sub]);
            obj.f_sub = matlabFunction(obj.f_sub);
            obj.Jacobian_F_sub = matlabFunction(obj.Jacobian_F_sub);
            
            % Measurement of main robot
            obj.h_main.dist1(x,y,theta,v_l,v_theta) = sqrt((x-x1)^2 + (y-y1)^2);
            obj.h_main.dist2(x,y,theta,v_l,v_theta) = sqrt((x-x2)^2 + (y-y2)^2);
            obj.h_main.dist3(x,y,theta,v_l,v_theta) = sqrt((x-x3)^2 + (y-y3)^2);
            obj.h_main.dist4(x,y,theta,v_l,v_theta) = sqrt((x-x4)^2 + (y-y4)^2);
            obj.h_main.theta(x,y,theta,v_l,v_theta) = theta;
            obj.h_main = [obj.h_main.dist1 obj.h_main.dist2 obj.h_main.dist3 obj.h_main.dist4 obj.h_main.theta]';
            obj.Jacobian_H_main = jacobian(obj.h_main,[x,y,theta]);
            obj.h_main = matlabFunction(obj.h_main);
            obj.Jacobian_H_main = matlabFunction(obj.Jacobian_H_main);
            
            % Measurement of sub robots
            obj.h_sub.x_main(x,y,theta,x_sub,y_sub,theta_sub,v_l,v_theta,v_l_sub,v_theta_sub) = x;
            obj.h_sub.y_main(x,y,theta,x_sub,y_sub,theta_sub,v_l,v_theta,v_l_sub,v_theta_sub) = y;
            obj.h_sub.theta_main(x,y,theta,x_sub,y_sub,theta_sub,v_l,v_theta,v_l_sub,v_theta_sub) = theta;
            obj.h_sub.d_x(x,y,theta,x_sub,y_sub,theta_sub,v_l,v_theta,v_l_sub,v_theta_sub) = x - x_sub;
            obj.h_sub.d_y(x,y,theta,x_sub,y_sub,theta_sub,v_l,v_theta,v_l_sub,v_theta_sub) = y - y_sub;
            obj.h_sub.theta(x,y,theta,x_sub,y_sub,theta_sub,v_l,v_theta,v_l_sub,v_theta_sub) = theta_sub;
            obj.h_sub = [obj.h_sub.x_main obj.h_sub.y_main obj.h_sub.theta_main obj.h_sub.d_x obj.h_sub.d_y obj.h_sub.theta]';
            obj.Jacobian_H_sub = jacobian(obj.h_sub,[x,y,theta,x_sub,y_sub,theta_sub]);
            obj.h_sub = matlabFunction(obj.h_sub);
            obj.Jacobian_H_sub = matlabFunction(obj.Jacobian_H_sub);

        end
        
        function initialization_imu_fcn(obj)
            if obj.imu_step_main <= 10
                obj.save_imu_stack_main = obj.imu_data_main(3) + obj.save_imu_stack_main;
            else
                obj.imu_calibration_value_main = obj.save_imu_stack_main / 10;
            end
            
            if obj.imu_step_sub1 <= 10
                obj.save_imu_stack_sub1 = obj.imu_data_sub1(3) + obj.save_imu_stack_sub1;
            else
                obj.imu_calibration_value_sub1 = obj.save_imu_stack_sub1 / 10;
            end
            if obj.imu_step_sub2 <= 10
                obj.save_imu_stack_sub2 = obj.imu_data_sub2(3) + obj.save_imu_stack_sub2;
            else
                obj.imu_calibration_value_sub2 = obj.save_imu_stack_sub2 / 10;
            end
            
        end       
        %% receive distance data between Achor1,2,3,4 and main / receive imu data of main, sub1 and sub2
        function perception_fcn(obj)
            
            obj.serial_main = rossubscriber('/read_waffle');
            obj.serial_msg_main = receive(obj.serial_main);
            obj.raw_position_main = extractBetween(obj.serial_msg_main.Data,"a 1"," b");
            obj.measurement_main = cell2mat(obj.raw_position_main);
            
            if ischar(obj.measurement_main)
                obj.measurement_main = str2num(obj.measurement_main);
                obj.pre_measurement_main = obj.measurement_main;
            else
                obj.measurement_main =  obj.pre_measurement_main;
            end
            
            if size(obj.measurement_main)<4
                obj.measurement_main = [0 0 0 0];
            end
            
            obj.measurement_main = (1/1000)*obj.measurement_main;
%             
            obj.message_main= receive(obj.subscriber_main);
            obj.message_sub1 = receive(obj.subscriber_sub1);
            obj.message_sub2 = receive(obj.subscriber_sub2); 
            %%%%%% main_imu %%%%%%%%%%%%%%%%%%
            obj.imu_msg_main = obj.message_main.Pose.Pose;
            ex_imu_data_main = [obj.imu_msg_main.Orientation.X, obj.imu_msg_main.Orientation.Y, obj.imu_msg_main.Orientation.Z, obj.imu_msg_main.Orientation.W];
            obj.imu_data_main = quat2eul(ex_imu_data_main, 'ZYX');    %*360/2/pi;
            obj.yaw_angle_main = obj.imu_data_main(3) - obj.imu_calibration_value_main;
            disp(obj.yaw_angle_main)
            %%%%% sub1_imu %%%%%%%%%%%%%%%%%%%%%%%%
            obj.imu_msg_sub1 = obj.message_sub1.Pose.Pose;
            ex_imu_data_sub1 = [obj.imu_msg_sub1.Orientation.X, obj.imu_msg_sub1.Orientation.Y, obj.imu_msg_sub1.Orientation.Z, obj.imu_msg_sub1.Orientation.W];
            obj.imu_data_sub1 = quat2eul(ex_imu_data_sub1, 'ZYX');    %*360/2/pi;
            obj.yaw_angle_sub1 = obj.imu_data_sub1(3) - obj.imu_calibration_value_sub1;
            
            %%%%% sub2_imu %%%%%%%%%%%%%%%%%%%%%%%%
            obj.imu_msg_sub2 = obj.message_sub2.Pose.Pose;
            ex_imu_data_sub2 = [obj.imu_msg_sub2.Orientation.X, obj.imu_msg_sub2.Orientation.Y, obj.imu_msg_sub2.Orientation.Z, obj.imu_msg_sub2.Orientation.W];
            obj.imu_data_sub2 = quat2eul(ex_imu_data_sub2, 'ZYX');    %*360/2/pi;
            obj.yaw_angle_sub2 = obj.imu_data_sub2(3) - obj.imu_calibration_value_sub2;
    
        end
        
        
        %% localization for main, sub1, sub2 
        function localization_main(obj)
            
            
            d1_main = obj.measurement_main(1);
            d2_main = obj.measurement_main(2);
            d3_main = obj.measurement_main(3);
            d4_main = obj.measurement_main(4); %%global heading angle
            theta_main = obj.yaw_angle_main;
            z_main = [d1_main, d2_main, d3_main, d4_main, theta_main]';
            
            if obj.measurement_main(1)*obj.measurement_main(2)*obj.measurement_main(3)*obj.measurement_main(4) == 0
                alpha = 0;
            else
                alpha = 1;
            end   
            
%             obj.x_main_hat_PEFFME(1,1) = obj.message_main.Pose.Pose.Position.X;
%             obj.x_main_hat_PEFFME(2,1) = obj.message_main.Pose.Pose.Position.Y;
%             obj.x_main_hat_PEFFME(3,1) = obj.yaw_angle_main;
                      
            obj.real_sub1_state(1,1) = obj.message_sub1.Pose.Pose.Position.X;
            obj.real_sub1_state(2,1) = obj.message_sub1.Pose.Pose.Position.Y;
            obj.real_sub1_state(3,1) = obj.yaw_angle_sub1;
            
            obj.real_sub2_state(1,1) = obj.message_sub2.Pose.Pose.Position.X;
            obj.real_sub2_state(2,1) = obj.message_sub2.Pose.Pose.Position.Y;
            obj.real_sub2_state(3,1) = obj.yaw_angle_sub2;

            obj.control_main = [obj.linear_vel_main*obj.sampling_time obj.angular_vel_main*obj.sampling_time]';
            obj.x_main_hat_PEFFME = Localization_main_PEFFME(obj.f_main,obj.Jacobian_F_main,obj.h_main,obj.Jacobian_H_main,obj.x_main_hat_PEFFME,z_main,obj.control_main,alpha);
            
        end  %% using RTLS ( using the distance between main tag and Anchor
        
        function lidar_localization_sub1(obj)
        
            obj.control_sub1 = [obj.linear_vel_sub1*obj.sampling_time obj.angular_vel_sub1*obj.sampling_time]';
            obj.x_sub1_hat_PEFFME(1:3,1) = obj.x_main_hat_PEFFME;
            oneto360 = 1:360;
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% sub1 lidar process %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            lidar_sub_1 = receive(obj.distance_sub1);
            obj.lidar_data_sub1(:,1) = lidar_sub_1.Ranges.*cos(oneto360*pi/180)';
            obj.lidar_data_sub1(:,2) = lidar_sub_1.Ranges.*sin(oneto360*pi/180)';
            obj.lidar_x_position_sub1(:,1) = lidar_sub_1.Ranges.*cos(oneto360*pi/180)';
            obj.lidar_y_position_sub1(:,1) = lidar_sub_1.Ranges.*sin(oneto360*pi/180)';
            
            for i = 1:360
                if (obj.lidar_data_sub1(i,1)>obj.lim_range) || (obj.lidar_data_sub1(i,1)<-obj.lim_range)
                    obj.lidar_data_sub1(i,1) =0;
                    obj.lidar_data_sub1(i,2) =0;
                end
                if (obj.lidar_data_sub1(i,2)>obj.lim_range) || (obj.lidar_data_sub1(i,2)<-obj.lim_range)
                    obj.lidar_data_sub1(i,1) =0;
                    obj.lidar_data_sub1(i,2) =0;
                end
            end
             
            
             [obj.measurement_sub1, obj.local_main_position_sub1, obj.local_sub_position_sub1, obj.predict_next_local_main_position_sub1, obj.alpha_sub1] = lidar_clustering_sub1(obj.lidar_data_sub1, obj.yaw_angle_sub1,obj.f_sub,obj.h_sub,obj.x_sub1_hat_PEFFME, [obj.control_main' obj.control_sub1']');
            obj.plot_measurement_sub1 = [obj.x_main_hat_PEFFME(1)-obj.measurement_sub1(1) obj.x_main_hat_PEFFME(2)-obj.measurement_sub1(2)]';
            
            if obj.measurement_sub1(1,3)-obj.pre_theta_sub1< -1 %&& obj.measurement_sub1(1,3)-obj.pre_theta_sub>0
                obj.rotation_count_sub1 = obj.rotation_count_sub1 +1;
            elseif obj.measurement_sub1(1,3)-obj.pre_theta_sub1 > 1
                obj.rotation_count_sub1 = obj.rotation_count_sub1 -1;
            end
            obj.calibration_theta_sub1 = obj.measurement_sub1(1,3) + 2*pi*obj.rotation_count_sub1;
            %             obj.measurement_sub1(1,3)= wrapTo2Pi(obj.measurement_sub1(1,3));
%             z_sub1_PEFFME = [obj.x_main_hat_PEFFME' obj.measurement_sub1]'; %%% Get sub 1 measurement
            z_sub1_PEFFME = [obj.x_main_hat_PEFFME' obj.measurement_sub1(1,1:2) obj.calibration_theta_sub1]';
            obj.x_sub1_hat_PEFFME = Localization_sub1_PEFFME(obj.f_sub,obj.Jacobian_F_sub,obj.h_sub,obj.Jacobian_H_sub,obj.x_sub1_hat_PEFFME,z_sub1_PEFFME,[obj.control_main' obj.control_sub1']',obj.alpha_sub1); %%% Get sub1 state hat
%             obj.x_sub1_hat_PEFFME(6,1)= wrapToPi(obj.x_sub1_hat_PEFFME(6,1));
            obj.pre_theta_sub1 = obj.measurement_sub1(1,3);
%                         obj.x_sub1_hat_PEFFME(4:6,1) = obj.real_sub1_state;
%                         obj.x_sub1_hat_PEFFME(6,1) = obj.real_sub1_state(3,1);
        end    %% using sub1 robot Lidar data ( clustering and processing )
        
        function lidar_localization_sub2(obj)
        
            obj.control_sub2 = [obj.linear_vel_sub2*obj.sampling_time obj.angular_vel_sub2*obj.sampling_time]';
            obj.x_sub2_hat_PEFFME(1:3,1) = obj.x_main_hat_PEFFME;
            oneto360 = 1:360;
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% sub1 lidar process %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            lidar_sub_2 = receive(obj.distance_sub2);
            
            obj.lidar_data_sub2(:,1) = lidar_sub_2.Ranges.*cos(oneto360*pi/180)';
            obj.lidar_data_sub2(:,2) = lidar_sub_2.Ranges.*sin(oneto360*pi/180)';
            obj.lidar_x_position_sub2(:,1) = lidar_sub_2.Ranges.*cos(oneto360*pi/180)';
            obj.lidar_y_position_sub2(:,1) = lidar_sub_2.Ranges.*sin(oneto360*pi/180)';

            for i = 1:360
                if (obj.lidar_data_sub2(i,1)>obj.lim_range) || (obj.lidar_data_sub2(i,1)<-obj.lim_range)
                    obj.lidar_data_sub2(i,1) =0;
                    obj.lidar_data_sub2(i,2) =0;
                end
                if (obj.lidar_data_sub2(i,2)>obj.lim_range) || (obj.lidar_data_sub2(i,2)<-obj.lim_range)
                    obj.lidar_data_sub2(i,1) =0;
                    obj.lidar_data_sub2(i,2) =0;
                end
            end
            
           [obj.measurement_sub2, obj.local_main_position_sub2, obj.local_sub_position_sub2, obj.predict_next_local_main_position_sub2, obj.alpha_sub2] = lidar_clustering_sub2(obj.lidar_data_sub2, obj.yaw_angle_sub2,obj.f_sub,obj.h_sub,obj.x_sub2_hat_PEFFME, [obj.control_main' obj.control_sub2']');
             obj.plot_measurement_sub2 = [obj.x_main_hat_PEFFME(1)-obj.measurement_sub2(1) obj.x_main_hat_PEFFME(2)-obj.measurement_sub2(2)]'; 
            z_sub2_PEFFME = [obj.x_main_hat_PEFFME' obj.measurement_sub2]'; %%% Get sub 1 measurement
            obj.x_sub2_hat_PEFFME = Localization_sub2_PEFFME(obj.f_sub,obj.Jacobian_F_sub,obj.h_sub,obj.Jacobian_H_sub,obj.x_sub2_hat_PEFFME,z_sub2_PEFFME,[obj.control_main' obj.control_sub2']',obj.alpha_sub2); %%% Get sub2 state hat
%             obj.x_sub2_hat_PEFFME(4:6,1) = obj.real_sub2_state;
        end    %% using sub2 robot Lidar data ( clustering and processing )
        
        %% control part
        function control_fcn(obj)
            
             %% refernece position for kinematic %%%
%              x_ref = [obj.tile_size*6 obj.tile_size*6 0]';
%              x_ref2 = [obj.tile_size*6 obj.tile_size*8 0]';
%              x_ref3 = [obj.tile_size*4 obj.tile_size*6 0]';
            
            circle_velocity = 0.7;
% %             %% reference position for flocking %%
            obj.xd = obj.tile_size*2*cos((pi/10)*(circle_velocity*obj.trajectory_step))+obj.tile_size*4;
            obj.yd = obj.tile_size*2*sin((pi/10)*(circle_velocity*obj.trajectory_step))+obj.tile_size*4;
            obj.xd_dot = -obj.tile_size*2*(pi/10)*sin((pi/10)*(circle_velocity*obj.trajectory_step));
            obj.yd_dot = obj.tile_size*2*(pi/10)*cos((pi/10)*(circle_velocity*obj.trajectory_step));

%             obj.xd = obj.tile_size*6;
%             obj.yd = obj.tile_size*6;
%             obj.xd_dot = 0;
%             obj.yd_dot = 0;

            x_ref = [obj.xd obj.yd 0]';              
            x_ref_dot = [obj.xd_dot obj.yd_dot 0]';
%             x_ref2 = [obj.xd obj.yd-0.5 0]'; 
            obj.control_state_PEFFME_sub1 = obj.x_sub1_hat_PEFFME(4:6,1);
            obj.control_state_PEFFME_sub2 = obj.x_sub2_hat_PEFFME(4:6,1);
            
            
            obj.trajectory_step = obj.trajectory_step + obj.sampling_time;
            
           
            %%%%%%%%%%%%%%%% choose control input %%%%%%%%%%%%%%%%
            v_flocking = Flocking_control(obj.x_main_hat_PEFFME,obj.control_state_PEFFME_sub1, x_ref, x_ref_dot);
%             v_flocking = Flocking_control_three(obj.x_main_hat_PEFFME,obj.control_state_PEFFME_sub1, obj.control_state_PEFFME_sub2, x_ref, x_ref_dot);
           
%             v_kinematic = Kinematic(obj.x_main_hat_PEFFME,x_ref);
%             v_kinematic2 = Kinematic2(obj.control_state_PEFFME_sub1,x_ref2);
%             v_kinematic3 = Kinematic3(obj.control_state_PEFFME_sub2,x_ref3);
            
%             if abs(obj.yaw_angle_main)<(3.14/2)
%                 constant_linear_input =0.0;
%                 constant_angular_input = 1;
%             else
%                 constant_linear_input =0.1;
%                 constant_angular_input = 0.00;
% %             end
            
           
            %%%%%%%%%%%%%%control for main%%%%%%%%%%%%%%%%            
%             obj.linear_vel_main = v_kinematic(1,1);    %%velocity update
%             obj.angular_vel_main = v_kinematic(2,1);
%             
             obj.linear_vel_main = v_flocking(1,1);    %%velocity update
             obj.angular_vel_main = v_flocking(2,1);
%             obj.linear_vel_main = constant_linear_input;    %%velocity update
%             obj.angular_vel_main = constant_angular_input;
%             obj.linear_vel_main = 0;    %%velocity update
%             obj.angular_vel_main = 0;
             if obj.linear_vel_main > obj.v_linear_max
                obj.linear_vel_main= obj.v_linear_max;
            elseif obj.linear_vel_main < -1*obj.v_linear_max
                obj.linear_vel_main= -1*obj.v_linear_max;
                
            end
            
            if obj.angular_vel_main > obj.v_angle_max
                obj.angular_vel_main = obj.v_angle_max;
            elseif obj.angular_vel_main < -1*obj.v_angle_max
                obj.angular_vel_main = -1*obj.v_angle_max;
            end
            obj.control_msg_main.Linear.X = obj.linear_vel_main;
            obj.control_msg_main.Linear.Y = 0;
            obj.control_msg_main.Linear.Z = 0;
            
            obj.control_msg_main.Angular.X = 0;
            obj.control_msg_main.Angular.Y = 0;
            obj.control_msg_main.Angular.Z = obj.angular_vel_main;
     
            send(obj.control_publisher_main,obj.control_msg_main);
            
            %%%%%%%%%%%%%%control for sub1 %%%%%%%%%%%%%%%%
%             obj.linear_vel_sub1 = v_kinematic2(1,1);    
%             obj.angular_vel_sub1 = v_kinematic2(2,1);
            obj.linear_vel_sub1 = v_flocking(3,1);    
            obj.angular_vel_sub1 = v_flocking(4,1);
%             obj.linear_vel_sub1 = constant_linear_input;    
%             obj.angular_vel_sub1 = constant_angular_input;
%             obj.linear_vel_sub1 = 0;    %%velocity update
%             obj.angular_vel_sub1 = 0;

             if obj.linear_vel_sub1 > obj.v_linear_max
                obj.linear_vel_sub1=obj.v_linear_max;
            elseif obj.linear_vel_sub1 < -1*obj.v_linear_max
                obj.linear_vel_sub1= -1*obj.v_linear_max;
            end
            
            if obj.angular_vel_sub1 > obj.v_angle_max
                obj.angular_vel_sub1= obj.v_angle_max;
            elseif obj.angular_vel_sub1 < -1*obj.v_angle_max
                obj.angular_vel_sub1 = -1*obj.v_angle_max;
            end
            obj.control_msg_sub1.Linear.X = obj.linear_vel_sub1;
            obj.control_msg_sub1.Linear.Y = 0;
            obj.control_msg_sub1.Linear.Z = 0;
           
            obj.control_msg_sub1.Angular.X = 0;
            obj.control_msg_sub1.Angular.Y = 0;
            obj.control_msg_sub1.Angular.Z = obj.angular_vel_sub1; 

            send(obj.control_publisher_sub1,obj.control_msg_sub1);
            
            %%%%%%%%%%%%%%control for sub2 %%%%%%%%%%%%%%%%
%             obj.linear_vel_sub2 = v_kinematic3(1,1);    
%             obj.angular_vel_sub2 = v_kinematic3(2,1);
%             obj.linear_vel_sub2 = v_flocking(5,1);    
%             obj.angular_vel_sub2 = v_flocking(6,1);
%             obj.linear_vel_sub2 = constant_linear_input;   
%             obj.angular_vel_sub2 = constant_angular_input;
%             obj.linear_vel_sub2 = 0;    
%             obj.angular_vel_sub2 = 0;

           if obj.linear_vel_sub2 > obj.v_linear_max
                obj.linear_vel_sub2=obj.v_linear_max;
            elseif obj.linear_vel_sub2 < -1*obj.v_linear_max
                obj.linear_vel_sub2= -1*obj.v_linear_max;
            end
            
            if obj.angular_vel_sub2 > obj.v_angle_max
                obj.angular_vel_sub2= obj.v_angle_max;
            elseif obj.angular_vel_sub2 < -1*obj.v_angle_max
                obj.angular_vel_sub2 = -1*obj.v_angle_max;
            end
            
            obj.control_msg_sub2.Linear.X = obj.linear_vel_sub2;
            
            obj.control_msg_sub2.Linear.Y = 0;
            obj.control_msg_sub2.Linear.Z = 0;
            
            obj.control_msg_sub2.Angular.X = 0;
            obj.control_msg_sub2.Angular.Y = 0;
            obj.control_msg_sub2.Angular.Z = obj.angular_vel_sub2; 

            send(obj.control_publisher_sub2,obj.control_msg_sub2);
            
       
        end   %% making control signal ( constant input or flockinig control)
        
        %% making plot
        function plot_fcn(obj)
%             plot_limit1 = obj.tile_size*obj.tile_num;            
%             obj.plot_figure = figure(1);
%             head = scatter(obj.x_main_hat_PEFFME(1),obj.x_main_hat_PEFFME(2),'filled','MarkerFaceColor','r','MarkerEdgeColor','r'); hold on; grid on;
%             head2 = scatter(obj.x_sub1_hat_PEFFME(4),obj.x_sub1_hat_PEFFME(5),'filled','MarkerFaceColor','b','MarkerEdgeColor','b');
% %             head3 = scatter(obj.x_sub2_hat_PEFFME(4),obj.x_sub2_hat_PEFFME(5),'filled','MarkerFaceColor','g','MarkerEdgeColor','b');
%             refer = scatter(obj.xd,obj.yd,'filled','MarkerFaceColor','g','MarkerEdgeColor','g');
%             xlim([0 plot_limit1])
%             ylim([0 plot_limit1])
%             drawnow;
%             delete(head);
%             delete(head2);
% %             delete(head3);
%             
%             delete(refer);
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%% lidar plot %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            plot_limit2 = 4;
            box_edge = plot_limit2/10;
            main_box_sub1 = [obj.local_main_position_sub1(1,1)-(box_edge/2), obj.local_main_position_sub1(2,1)-(box_edge/2),box_edge,box_edge];
            
            obj.plot_figure_lidar_sub1 = figure(2);
            scatter(0,0,'filled', 'MarkerFaceColor', 'r')
            hold on;
            heading = quiver(0,0,0.5,0, 'LineWidth', 1.8, 'MaxHeadSize', 3, 'ShowArrowHead', 'on');
            hold on
            grid on
            point_cloud = plot(obj.lidar_data_sub1(:,1),obj.lidar_data_sub1(:,2),'*');
            main_robot=plot(obj.local_main_position_sub1(1,1),obj.local_main_position_sub1(2,1),'ro');
            predict_position=plot(obj.predict_next_local_main_position_sub1(1,1),obj.predict_next_local_main_position_sub1(2,1),'bd');
            legend([heading, point_cloud,main_robot,predict_position], 'heading angle', 'lidar data', 'main robot', 'predict position');
            rectangle('Position',main_box_sub1,'EdgeColor','r');
                                    
            hold off;
            
            xlim([-plot_limit2 plot_limit2])
            ylim([-plot_limit2 plot_limit2])
            drawnow;
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
%             %%%%%%%%%%%%%%%%%%%%%%%%%%%%% lidar plot %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%             plot_limit3 = 4;
%             box_edge2 = plot_limit3/10;
%             main_box_sub2 = [obj.local_main_position_sub2(1,1)-(box_edge2/2), obj.local_main_position_sub2(2,1)-(box_edge2/2),box_edge2,box_edge2];
%            
%             obj.plot_figure_lidar_sub2 = figure(3);
%             scatter(0,0,'filled', 'MarkerFaceColor', 'r')
%             hold on;
%             heading2 = quiver(0,0,0.5,0, 'LineWidth', 1.8, 'MaxHeadSize', 3, 'ShowArrowHead', 'on');
%             hold on
%             grid on
%             point_cloud2 = plot(obj.lidar_data_sub2(:,1),obj.lidar_data_sub2(:,2),'*');
%             main_robot2=plot(obj.local_main_position_sub2(1,1),obj.local_main_position_sub2(2,1),'ro');
%             predict_position2=plot(obj.predict_next_local_main_position_sub2(1,1),obj.predict_next_local_main_position_sub2(2,1),'bd');
%             legend([heading2, point_cloud2,main_robot2,predict_position2], 'heading angle', 'lidar data', 'main robot', 'predict position');
%             rectangle('Position',main_box_sub2,'EdgeColor','r');                         
%             hold off;
%             
%             xlim([-plot_limit3 plot_limit3])
%             ylim([-plot_limit3 plot_limit3])
%             drawnow;
            %%%%%%%%%        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        end
        

    end
end



function y_correct = Correction(y)
persistent temp firstRun
if isempty(firstRun)
    temp = zeros(size(y));
    firstRun = 1;
end

y_correct = y;
zero_index = find(~y);
y_correct(zero_index) = temp(zero_index);
temp = y_correct;

end

