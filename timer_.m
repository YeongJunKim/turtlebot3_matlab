function timer_(obj, event, robots)
    persistent step;
    persistent firstRun;
        
    persistent var;
    persistent f h jac_f jac_h;
    
    if isempty(firstRun)
       step = 0; 
              
       %% init task
       % additional subscriber
       for i = 1:size(robots)
           var.sub_odom_data(i).data = robots(i).sub_odom_data;
       end
       for i = 1:size(robots)
           var.sub_imu_data(i).data = robots(i).sub_imu_data;
       end
       for i = 2:size(robots)
           var.sub_scan_data(i).data = robots(i).sub_scan_data;
       end
       var.sub_measurement_data = [];
       var.sub_measurement_past = [];
       var.sub_measurement_now = [];
       var.sub_measurement_raw = [];
       var.data = [];
       
       turtlebot3_addSubscrier(robots(1),'/tb3_0/read', @measurement_data);
       pause(2);
       
       %% init filter
       
       
       
       
       firstRun = 1;
    end
    
    %% get data
    tic;
    for i = 1:size(robots,2)
       var.sub_odom_data(i).data = robots(i).sub_odom_data;
    end
    for i = 1:size(robots,2)
       var.sub_imu_data(i).data = robots(i).sub_imu_data;
    end
    for i = 2:size(robots,2)
       var.sub_scan_data(i).data = robots(i).sub_scan_data;
    end
    
    var.sub_measurement_data = robots(1).sub_data(1).data.Data;
    var.sub_measurement_raw = extractBetween(var.sub_measurement_data,"a 1 "," b");
    var.sub_measurement_now = cell2mat(var.sub_measurement_raw(1));
    if ischar(var.sub_measurement_now)
        var.sub_measurement_now = str2num(var.sub_measurement_now);
        var.sub_measurement_past = var.sub_measurement_now;
    else
        var.sub_measurement_now = var.sub_measurement_past;
    end
    
    if size(var.sub_measurement_now) ~= 4
       var.sub_measurement_now = [0 0 0 0];
    end
    var.sub_measurement_now = (0.001)*var.sub_measurement_now;
    disp(var.sub_measurement_now);    
    
    for i = 1:size(robots,2)
        var.data(i).odom = var.sub_odom_data(i).data.Pose.Pose;
        imu = [var.data(i).odom.Orientation.X, var.data(i).odom.Orientation.Y, var.data(i).odom.Orientation.Z, var.data(i).odom.Orientation.W];
        var.data(i).imu = quat2eul(imu, 'ZYX');
        yaw = var.data(i).imu(3); % TODO calibration
        var.data(i).yaw = wrapToPi(yaw);
        disp(i)
        disp(var.data(i))
    end
    toc;
    tic;
    %% localization
    
    toc;
    
    step = step + 1;
end

function calc_measurement_data(data)


end
function measurement_data(src, msg, obj)
%      disp('measurement data');
%        disp(var.sub_measurement_data.data);
    obj.sub_data(1).data = msg;
%     disp(obj.sub_data(1).data.Data);
end
function measurement_data1(src, msg, arg, var)
%      disp('measurement data');
       var.sub_measurement_data.data = msg;
       disp(var.sub_measurement_data.data);
       
       raw_data = msg.Data;
       raw_data = extractBetween(raw_data,"a 1"," b");
       data = cell2mat(raw_data);
       
       if ischar(data)
          data = str2num(data);
          var.sub_measurement_data = data;
          var.sub_measurement_pre_data = data;
       else
          var.sub_measurement_data = var.sub_measurement_pre_data;
       end
       
       if size(var.sub_measurement_data) < 4
          var.measurement_data = [0 0 0 0]; 
       end
       
       var.measurement_data = (1/1000)*var.measurement_data; 
end