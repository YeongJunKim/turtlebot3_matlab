%% Matlab에서 허수 i(루트 -1) 입력하는 법
% -> 1i 로 써도 되고 1j 도 됨
% -> 계수가 붙는 경우 2i 이렇게 써도 되고 2 * 1i 이렇게 해도 됩니다
% -> 그냥 i로만 쓰면 i 라는 변수를 쓰게되는 경우(특히 반복문에서) 변수 겹치면서 실수할 수 있으니깐 되도록 위에처럼 쓰세요
% -> 옳은 예시: 2+3i, 2+3j, 2+3*1i, 2+3*1j
% -> ㅈ망할 수 있는 예시: 2+3*i

%% 상황 설명 간략히
% 일단 우리의 경우에 맞춰 로봇이 3대인 상황(n=3)을 가정했음 (원하면 대수 늘려도 됨)
% 따라서 i = 1,2,3 이고, i = 1 이 follower, i = 2,3 이 leader
% 1773쪽부터 나오는 Rigid formation control 이 가능하려면, 로봇이 3대일 경우 모든 edge가 서로 연결되어 있어야 하는 듯함
% 그러니까 다 연결되어 있다고 가정하고 해보세요
% 님들이 해야할 것: 행렬 K 를 design
% 힌트: 구글에 eigenvalue assignment, pole placement 등을 쳐보세요 (뭐든 쳐보세요)
% 추가로 하면 좋은 것: 로봇 대수 늘리고 다양한 형태의 그래프에 대해서도 해보기

%% Initialize
dt = 0.1;           % Sampling time, 단위: 초
robot_num = 3;      % 로봇 개수

alpha = 1;          % 식 (12)
k = [1 1 1]';       % 식 (13)

pose1 = [1 -5]';
pose2 = [-3 0]';
pose3 = [4 1]';     % 처음 로봇들 위치 x,y 좌표
z = [pose1(1) + 1i*pose1(2) , pose2(1) + 1i*pose2(2) , pose3(1) + 1i*pose3(2)]';    % Aggregate state
target_pose1 = [0 -3]';
target_pose2 = [-2 1]';
target_pose3 = [2 1]';      % 목표 x,y 좌표
z_target = [target_pose1(1) + 1i*target_pose1(2) , target_pose2(1) + 1i*target_pose2(2) , target_pose3(1) + 1i*target_pose3(2)]';

now_pos = [];
now_pos(1).data = [pose1(1) + pose1(2) * 1i];
now_pos(2).data = [pose2(1) + pose2(2) * 1i];
now_pos(3).data = [pose3(1) + pose3(2) * 1i];

appended_data = [];
appended_data.pose(1).data = now_pos(1).data;
appended_data.pose(2).data = now_pos(2).data;
appended_data.pose(3).data = now_pos(3).data;
appended_data.weight = zeros(robot_num, robot_num);

%% Simulation
N = 1;
for i = 1:N
    % weight matrix
    weight = zeros(robot_num, robot_num);
    
    % Determine weight values (식 (5)나 그 위에있는 내용 참고)
    for i = 1:robot_num
        j = i - 1;
        k = i + 1;
        if(j == 0)
           j = robot_num;
        end
        if(k == robot_num + 1)
           k = 1;
        end
        disp(j);
        disp(k);
        random_weight = (rand()*10 + rand()*10*1i);
        weight(i, j) = random_weight*(now_pos(j).data - now_pos(i).data);
        random_weight = (rand()*10 + rand()*10*1i);
        weight(i, k) = random_weight*(now_pos(k).data - now_pos(k).data);
        weight = weight + weight.';
        
    end
    
    % Determine L (Laplacian matrix)
    
    
    % Design K
    
    
%     z_dot = ?
%     z = z + z_dot * dt;
%     
%     figure(1);
%     plot(real(z(1)),imag(z(1)),'ro'); hold on;
%     plot(real(z(2)),imag(z(2)),'bo');
%     plot(real(z(3)),imag(z(3)),'go'); hold off;
%     drawnow;
    
end