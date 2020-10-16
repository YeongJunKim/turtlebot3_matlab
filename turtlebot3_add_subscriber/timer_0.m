function timer_0(obj, event)

global ROBOT
persistent firstrun
persistent step

if isempty(firstrun)
    step = 1;
    firstrun = 1;
    
    
    figure(1);
    ax = axes;
    
    turtlebot3_addSubscrier(ROBOT, '/tb3a/read',@measurement);
    fprintf("timer initialized\n");
end

fprintf("step = %d\n", step);

plot(ROBOT.sub_scan_data);

fprintf("distance measurement = [%d, %d, %d, %d] \r\n", ROBOT.sub_data(1).data(1), ROBOT.sub_data(1).data(2), ROBOT.sub_data(1).data(3), ROBOT.sub_data(1).data(4));

step = step + 1;

end

function measurement(src, msg, obj)
sub_measurement_data = msg.Data;
sub_measurement_raw = extractBetween(sub_measurement_data,"a 1 "," b");
if size(sub_measurement_raw, 1) == 0
    sub_measurement_raw = cell(1,1);
    sub_measurement_raw(1) = {'0 0 0 0'};
end
sub_measurement_now = cell2mat(sub_measurement_raw(1));
if ischar(sub_measurement_now)
    sub_measurement_now = str2num(sub_measurement_now);
    sub_measurement_past = sub_measurement_now;
else
    sub_measurement_now = sub_measurement_past;
end
if size(sub_measurement_now) ~= 4
    sub_measurement_now = [0 0 0 0];
end
sub_measurement_now = (0.001)*sub_measurement_now;
%     disp(sub_measurement_now);
obj.sub_data(1).data = sub_measurement_now;
end