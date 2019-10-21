function timer_(obj, event, robots)
    persistent step;
    persistent firstRun;
    global var;
    
    if isempty(firstRun)
       step = 0; 
              
       %% init task
       var.sub_odom_data = [];
       var.sub_imu_data = [];
       var.sub_scan_data = [];
       var.sub_measurement_data = [];
       var.sub_measurement_past = [];
       var.sub_measurement_now = [];
       var.sub_measurement_raw = [];
       var.data = [];
       
       for i = 1:size(robots)
           var.sub_odom_data(i).data = robots(i).sub_odom_data;
       end
       for i = 1:size(robots)
           var.sub_imu_data(i).data = robots(i).sub_imu_data;
       end
       for i = 2:size(robots)
           var.sub_scan_data(i).data = robots(i).sub_scan_data;
       end
       
       % additional subscriber
       pause(1);
       turtlebot3_addSubscrier(robots(1),'/read', @measurement_data);
       pause(1);
       

       firstRun = 1;
    end    
    step = step + 1;
    disp(step)
    if step > 10
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
        toc;
        %% localization

        toc;
    end
end












function measurement_data(src, msg, obj)
    obj.sub_data(1).data = msg;
    var.sub_measurement_data = msg.Data;
    var.sub_measurement_raw = extractBetween(var.sub_measurement_data,"a 1 "," b");
    if size(var.sub_measurement_raw, 1) == 0
        var.sub_measurement_raw = cell(1,1);
        var.sub_measurement_raw(1) = {'0 0 0 0'};
    end
    
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
%     disp(var.sub_measurement_now);  
end