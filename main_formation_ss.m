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

clear all
close all

%% Initialize
dt = 0.1;           % Sampling time, 단위: 초
num_robot = 3;      % 로봇 개수

alpha = 0.05;       % 식 (12)

pose1 = [1 -5]';
pose2 = [-3 0]';
pose3 = [4 1]';     % 처음 로봇들 위치 x,y 좌표
z = [pose1(1) + 1i*pose1(2) ; pose2(1) + 1i*pose2(2) ; pose3(1) + 1i*pose3(2)];    % Aggregate state
target_pose1 = [0 -3]';
target_pose2 = [-2 1]';
target_pose3 = [2 1]';      % 목표 x,y 좌표
z_target = [target_pose1(1) + 1i*target_pose1(2) ; target_pose2(1) + 1i*target_pose2(2) ; target_pose3(1) + 1i*target_pose3(2)];

figure(1);
ax = axes;
p1_target = plot(ax,real(z_target(1)),imag(z_target(1)),'r*'); hold on; grid on;
p2_target = plot(ax,real(z_target(2)),imag(z_target(2)),'b*');
p3_target = plot(ax,real(z_target(3)),imag(z_target(3)),'g*');
p1 = plot(ax,real(z(1)),imag(z(1)),'ro');
p2 = plot(ax,real(z(2)),imag(z(2)),'bo');
p3 = plot(ax,real(z(3)),imag(z(3)),'go');
ax.XLim = [-5 6];
ax.YLim = [-7 4];
axis equal
legend(ax,{'target1','target2','target3','robot1(follower)','robot2','robot3'},'Location','southeast');

%% Simulation
N = 100;
for iteration = 1:N
    %% Determine weight values (식 (5)나 그 위에있는 내용 참고)
    L = zeros(num_robot,num_robot);
    for i = 1:num_robot
        for j = 1:num_robot-1
            if j == i
                continue;
            end
            for k = j+1:num_robot
                if k == i
                    continue;
                end
                L(i,j) = L(i,j) + z_target(k) - z_target(i);
                L(i,k) = L(i,k) + z_target(i) - z_target(j);
            end
        end
        L(i,i) = - sum(L(i,:));
    end
%     L(num_robot-1,:) = -L(num_robot-1,:);
%     L(num_robot,:) = -L(num_robot,:);
            
    %% Design K
    poles = [5];                        % n-2 개의 poles
    [U_svd,S_svd,V_svd] = svd(L);       % Singular value decomposition
    S_sqrt = sqrt(S_svd);
    U = U_svd(:,1:num_robot-2) * S_sqrt(1:num_robot-2,1:num_robot-2)';
    V = S_sqrt(1:num_robot-2,1:num_robot-2) * V_svd(:,1:num_robot-2)';
    
    temp = place(zeros(num_robot-2),V(:,1:num_robot-2),poles);
    temp = temp / U(1:num_robot-2,:);
    K = [temp zeros(num_robot-2,2) ; zeros(2,num_robot-2) eye(2)];
%     eig(K*L)
    
    %% Control
    z_dot = zeros(num_robot,1);
    for i = 1:num_robot-2       % Followers
        for j = 1:num_robot
            if j == i
                continue;
            end
            z_dot(i) = z_dot(i) + K(i,i) * L(i,j) * (z(j) - z(i));
%             z_dot(i) = z_dot(i) + K(i,i) * (z(j) - z(i));
        end
    end
    d_bar = norm(z_target(num_robot) - z_target(num_robot-1));
    diff_leaders = z(num_robot) - z(num_robot-1);
    z_dot(num_robot-1) = alpha * diff_leaders * (norm(diff_leaders)^2 - d_bar^2);
    z_dot(num_robot) = -z_dot(num_robot-1);
    
	z = z + z_dot * dt;
%     z_dot
    
    %% Plot
    p1.XData = real(z(1));
    p1.YData = imag(z(1));
    p2.XData = real(z(2));
    p2.YData = imag(z(2));
    p3.XData = real(z(3));
    p3.YData = imag(z(3));
    pause(0.1);
    drawnow;
    
end