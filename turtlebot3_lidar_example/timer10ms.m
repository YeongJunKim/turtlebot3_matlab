function timer10ms(obj, event, sub, topic, struct)

if struct.firstRun == 0
    struct.firstRun = 1;
    rotate = zeros(360,2);
end
%txt1 = ' event occurred at ';

%event_type = event.Type;
%event_time = datestr(event.Data.time);

%msg = [event_type txt1 event_time];
%disp(msg)

% fprintf('timer\r\n')

topic.lidar = receive(sub.turtlebot3_lidar);

%disp(topic.lidar.Ranges)

%draw_plot = zeros(360);

oneto360 = 1:360;

rotate(:,1) = topic.lidar.Ranges.*cos(oneto360*pi/180)';
rotate(:,2) = topic.lidar.Ranges.*sin(oneto360*pi/180)';

scatter(0,0,'filled', 'MarkerFaceColor', 'r')
hold on;
heading = quiver(0,0,0.5,0, 'LineWidth', 1.8, 'MaxHeadSize', 3, 'ShowArrowHead', 'on');
hold on;

point_cloud = plot(rotate(:,1),rotate(:,2),'c*');
legend([heading, point_cloud], 'heading angle', 'points');
hold off;
xlim([-5 5])
ylim([-5 5]) 
drawnow;

% delete(draw_plot);
   
% for i = 1:5:360
%    %draw_lidar = scatter(topic.lidar.Ranges(i)*cos(deg2rad(i)) * 100, topic.lidar.Ranges(i)*sin(deg2rad(i)) * 100, 'filled', 'MarkerFaceColor', 'r')
%    
%    rotate = zeros(2,1);
%    
%    rotate(1) = topic.lidar.Ranges(i)*cos(deg2rad(i));
%    rotate(2) = topic.lidar.Ranges(i)*sin(deg2rad(i));
%    
%    res = rot(1.57,rotate);
%    
%    %draw_plot(i) = plot(topic.lidar.Ranges(i)*cos(deg2rad(i)) , topic.lidar.Ranges(i)*sin(deg2rad(i)), '-c*');
%    draw_plot(i) = plot(res(1), res(2), '-c*');
%    hold on;
%    drawnow;
% end

% delete(draw_lidar)

end
