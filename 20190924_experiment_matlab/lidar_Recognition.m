function obstacle_position = lidar_Recognition(centroid_array, init_obstacle_position, max_number)


distance_array = zeros(max_number,1);
for i=1:max_number
    distance_array(i)= sqrt((centroid_array(i,1)-init_obstacle_position(1))^2+(centroid_array(i,2)-init_obstacle_position(2))^2);
end

min_distance = min(distance_array);

centroid_number = find(distance_array==min_distance);

obstacle_position = [centroid_array(centroid_number,1),centroid_array(centroid_number,2)]';


end

