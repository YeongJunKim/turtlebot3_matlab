function timer10ms(obj, event, sub, topic, struct)


if struct.firstRun == 0
    struct.firstRun = 1;
    
end
%txt1 = ' event occurred at ';

%event_type = event.Type;
%event_time = datestr(event.Data.time);

%msg = [event_type txt1 event_time];
%disp(msg)


topic.lidar = receive(sub.turtlebot3_lidar)

%disp(topic.lidar.Ranges)

drawnow;
draw_plot = zeros(360);

xlim([-5 5])
ylim([-5 5]) 
scatter(0,0,'filled', 'MarkerFaceColor', 'r')
   drawnow;
   
for i = 1:5:360
   %draw_lidar = scatter(topic.lidar.Ranges(i)*cos(deg2rad(i)) * 100, topic.lidar.Ranges(i)*sin(deg2rad(i)) * 100, 'filled', 'MarkerFaceColor', 'r')
   
   rotate = zeros(2,1);
   
   rotate(1) = topic.lidar.Ranges(i)*cos(deg2rad(i));
   rotate(2) = topic.lidar.Ranges(i)*sin(deg2rad(i));
   
   res = rot(1.57,rotate);
   
   %draw_plot(i) = plot(topic.lidar.Ranges(i)*cos(deg2rad(i)) , topic.lidar.Ranges(i)*sin(deg2rad(i)), '-c*');
   draw_plot(i) = plot(res(1), res(2), '-c*');
   hold on;
   drawnow;
end
delete(draw_plot);
%delete(draw_lidar)


end

function res = rot(rad,state)
mat = zeros(2,2);
mat = [cos(rad), -1*sin(rad);
    sin(rad), cos(rad)];
res = mat * state;
end