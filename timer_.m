function timer_(obj, event, robots, filters)
    persistent step;
    persistent firstRun;
    global var;
    
    if isempty(firstRun)
       step = 0; 
       
       %%       
       
       %% data saving ?
       var.data_save = 1;
       
       %% is it end?
       var.endflag = 0;
              
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
       
       
       %% user variables
       var.x_hat_save = [];
       var.y_measurement_save = [];
       
       
       %% additional subscriber
       pause(1);
       turtlebot3_addSubscrier(robots(1),'/read', @measurement_data);
       pause(1);
       firstRun = 1;
    end    
    
    
    %% TASK
    %% 
    step = step + 1;
    disp(step)
    %% 
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
           var.sub_scan_data(i).data = robots(i).sub_scan_data.Ranges;
        end
        var.sub_measurement_data = robots(1).sub_data(1).data;
        toc;
        %% localization
        % add your task here
        
        % add yout task end
        toc;
    end
    if step > 20
       var.endflag = 1; 
    end
    if var.endflag == 1
       timer_end(var.data_save, var); 
    end
end










function timer_end(data_save, data)
    if data_save == 1
       date = string(datetime('now'));
       date = erase(date,":");
       delimiter = "-";
       naming = "distributed localization";
       date =  (date, delimiter, naming);
       save(date, 'data');
       disp("data saved!");
    else
       disp("no data saving mode!");
    end
    rosshutdown
    tmrList = timerfind();
    delete(tmrList);
end

function measurement_data(src, msg, obj)
    
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
    disp(var.sub_measurement_now);  
    obj.sub_data(1).data = var.sub_measurement_now;
end