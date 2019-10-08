function [sub_measurement, local_main_position_sub1, local_sub_position_sub1, predict_next_local_main_position, flag_sub] = lidar_clustering_sub1(lidar_data_sub1, yaw_angle_sub1,f,h,x_sub_hat,control_commends )
 persistent firstRun 
 persistent local_main_position local_sub_position
 persistent state_previous
 persistent success_count
 persistent k
 global initial_position_main initial_position_sub1 initial_position_sub2
 global data
 
 heading_angle_sub1 = yaw_angle_sub1;
 tile_size = 1;
 X = lidar_data_sub1;
 Y = pdist(X);
 Z = linkage(Y,'single');
 c = cluster(Z,'cutoff',0.1,'criterion','distance');
 cluster_max_number = max(c);
 centroid_array = zeros(cluster_max_number,2);
 
 for i=1:cluster_max_number
     ins = find(c==i);
     centroid_array(i,:)=[mean(X(ins,1)),mean(X(ins,2))];
 end
 
 if isempty(firstRun)
     pre_heading_angle_sub1 = yaw_angle_sub1;
%      init_local_main_position = [tile_size*-1,tile_size*2]';
     init_local_main_position = initial_position_main-initial_position_sub1;
%      init_local_sub_position = [tile_size*(0),tile_size*(-2)]';
     init_local_sub_position = initial_position_sub2 -initial_position_sub1;
     local_main_position = init_local_main_position;
     local_sub_position = init_local_sub_position;
     firstRun=1;
     success_count = 1;
     k=1;
 end
  
 
 state_previous = x_sub_hat;
%    predict_next_local_main_position = [state_previous(1)-state_previous(4), state_previous(2)-state_previous(5)]';
%    predict_next_local_main_position = local_main_position;
 
 arguments = num2cell([state_previous' control_commends']);
 f_hat=f(arguments{:});
 arguments2 = num2cell([f_hat' control_commends']);
 h_hat = h(arguments2{:});
 
 heading_angle_sub1= wrapToPi(heading_angle_sub1);
 rotate_matrix = [ cos(-heading_angle_sub1) -sin(-heading_angle_sub1); sin(-heading_angle_sub1) cos(-heading_angle_sub1)];
 
    
    predict_next_local_main_position = [h_hat(4,1),h_hat(5,1)]';
    predict_next_local_main_position = rotate_matrix*predict_next_local_main_position;
% 
% 
 predict_next_local_sub_position = rotate_matrix*local_sub_position;
%  
 local_main_position = lidar_Recognition(centroid_array, predict_next_local_main_position, cluster_max_number);
%  local_sub_position = lidar_Recognition(centroid_array, predict_next_local_sub_position, cluster_max_number);
%  
%    
 if local_main_position(1,1)==0 && local_main_position(2,1)==0
     local_main_position = predict_next_local_main_position;
     flag_sub =0;
     disp('fail')
     success_count = 1;
 else
     flag_sub = 1;
     success_count=success_count+1;
     disp('success')
 end
%  
%  if 0==mod(success_count,1)
%    predict_next_local_main_position=local_main_position ;
% end
 data.local_measurement_sub1(:,k)= local_main_position;
 k = k+1;
 sub_measurement_distance = inv(rotate_matrix)*local_main_position; 
%  sub_measurement = [sub_measurement_distance' h_hat(6,1)];
 sub_measurement = [sub_measurement_distance' heading_angle_sub1];
 local_main_position_sub1 = local_main_position;
 local_sub_position_sub1 = local_sub_position;
%  pre_heading_angle_sub1 = yaw_angle_sub1; 
%  pre_predict_next_local_main_position=predict_next_local_main_position;
end

