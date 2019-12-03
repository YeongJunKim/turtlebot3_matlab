%% Matlab���� ��� i(��Ʈ -1) �Է��ϴ� ��
% -> 1i �� �ᵵ �ǰ� 1j �� ��
% -> ����� �ٴ� ��� 2i �̷��� �ᵵ �ǰ� 2 * 1i �̷��� �ص� �˴ϴ�
% -> �׳� i�θ� ���� i ��� ������ ���ԵǴ� ���(Ư�� �ݺ�������) ���� ��ġ�鼭 �Ǽ��� �� �����ϱ� �ǵ��� ����ó�� ������
% -> ���� ����: 2+3i, 2+3j, 2+3*1i, 2+3*1j
% -> ������ �� �ִ� ����: 2+3*i

%% ��Ȳ ���� ������
% �ϴ� �츮�� ��쿡 ���� �κ��� 3���� ��Ȳ(n=3)�� �������� (���ϸ� ��� �÷��� ��)
% ���� i = 1,2,3 �̰�, i = 1 �� follower, i = 2,3 �� leader
% 1773�ʺ��� ������ Rigid formation control �� �����Ϸ���, �κ��� 3���� ��� ��� edge�� ���� ����Ǿ� �־�� �ϴ� ����
% �׷��ϱ� �� ����Ǿ� �ִٰ� �����ϰ� �غ�����
% �Ե��� �ؾ��� ��: ��� K �� design
% ��Ʈ: ���ۿ� eigenvalue assignment, pole placement ���� �ĺ����� (���� �ĺ�����)
% �߰��� �ϸ� ���� ��: �κ� ��� �ø��� �پ��� ������ �׷����� ���ؼ��� �غ���

clear all
close all

%% Initialize
dt = 0.1;           % Sampling time, ����: ��
num_robot = 3;      % �κ� ����

alpha = 0.05;       % �� (12)

pose1 = [1 -5]';
pose2 = [-3 0]';
pose3 = [4 1]';     % ó�� �κ��� ��ġ x,y ��ǥ
z = [pose1(1) + 1i*pose1(2) ; pose2(1) + 1i*pose2(2) ; pose3(1) + 1i*pose3(2)];    % Aggregate state
target_pose1 = [0 -3]';
target_pose2 = [-2 1]';
target_pose3 = [2 1]';      % ��ǥ x,y ��ǥ
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
    %% Determine weight values (�� (5)�� �� �����ִ� ���� ����)
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
    poles = [5];                        % n-2 ���� poles
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