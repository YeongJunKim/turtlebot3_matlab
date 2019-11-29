


now_pos = [];
now_pos(1).data = [pose1(1) , pose1(2) * 1i];
now_pos(2).data = [pose2(1) , pose2(2) * 1i];
now_pos(3).data = [pose3(1) , pose3(2) * 1i];
now_pos(1).value = 0;
now_pos(2).value = 0;
now_pos(3).value = 0;
now_value_vector= zeros(robot_num, 1);
now_weight = zeros(robot_num, robot_num);

dataToValue(now_pos);

function r = dataToValue(data)
    data(1).value = data(1).data(1) + data(1).data(2);
    data(2).value = data(2).data(1) + data(2).data(2);
    data(3).value = data(3).data(1) + data(3).data(2);
end