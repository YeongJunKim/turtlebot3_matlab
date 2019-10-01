function timer_(obj, event, robots, variables)
    persistent step;
    persistent firstRun;
    persistent imudata;
    persistent scandata;
    
    if isempty(firstRun)
       step = 0; 
       firstRun = 1;
       
       imudata = [];
       scandata = [];
       
       %% init task
       
       
       
       
    end
    
    
    %% do task
    
    %% get data
    for i = 1:3
       variables.sub_imu_data = turtlebot3_readTopicName(robots(i),"imu");
    end
    for i = 2:3
       variables.sub_scan_data = turtlebot3_readTopicName(robots(i),"scan"); 
    end
    sub_serial = turtlebot3_readTopicName(robots(1),"read_waffle");
    variables.measurement_raw = extractBetween(sub_serial.Data," 1"," b");
    variables.measurement_data = cell2mat(measurement_raw);
    
    if ischar(measurement_data)
       variables.measurement_data = str2num(measurement_data);
       variables.measurement_pre_data = measurement_data;
    else
       variables.measurement_data = measurement_pre_data;
    end
    
    if size(measurement_data) < 4
       variables.measurement_data = [0 0 0 0]; 
    end
       variables.measurement_data = (1/1000)*measurement_data;
    %%
    

    step = step + 1
end