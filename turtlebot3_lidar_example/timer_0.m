function timer_0(obj, event)

global ROBOT
persistent firstrun
persistent step

if isempty(firstrun)
    step = 1;
    firstrun = 1;
    
    
    figure(1);
    ax = axes;
    
    fprintf("timer initialized\n");
end

fprintf("step = %d\n", step);

plot(ROBOT.sub_scan_data);


step = step + 1;
end