%% plot_multi.m - 多智能体编队避障终极可视化脚本 (支持多障碍物)
% 包含了 3D 静态轨迹、编队误差曲线、最小避障距离曲线以及 3D 动态动画

% =========================================================================
% 1. 极其鲁棒的数据提取 (Try-Catch 终极防护，兼容各种 Simulink 版本)
% =========================================================================
if exist('P_data', 'var')
    raw_data = P_data;
elseif exist('out', 'var')
    try
        raw_data = out.P_data;
    catch
        try
            raw_data = out.get('P_data');
        catch
            error('未找到 P_data！请检查 Simulink 里的 To Workspace 模块变量名是否为 P_data。');
        end
    end
else
    error('未找到 P_data 变量，请确保仿真已成功运行且输出了数据。');
end

% 判断数据格式并提取 Data 和 Time
if isa(raw_data, 'timeseries') || (isstruct(raw_data) && isfield(raw_data, 'Data'))
    P_data_matrix = raw_data.Data;
    time = raw_data.Time;
else
    % 如果是纯 Array 格式
    P_data_matrix = raw_data;
    got_time = false;
    if exist('out', 'var')
        try
            time = out.tout;
            got_time = true;
        catch
        end
    end
    if ~got_time
        dt = 0.001; 
        time = (0:size(P_data_matrix, 3)-1) * dt; 
    end
end

% 确保 time 是行向量
if iscolumn(time)
    time = time';
end

% 维度自适应：确保数据格式为 (3 x N x T)
if size(P_data_matrix, 1) == length(time) && ndims(P_data_matrix) == 3
    P_data_matrix = permute(P_data_matrix, [2, 3, 1]);
end

num_agents = size(P_data_matrix, 2);
num_followers = num_agents - 1;

% 定义颜色: 领航者(1)为红色，跟随者为其他配色
colors = lines(num_agents);
colors(1,:) = [0, 0, 0]; 

% =========================================================================
% 图 1: 三维静态轨迹图 (带 3D 多障碍物光影渲染)
% =========================================================================
figure('Name', '3D Trajectories & Obstacles', 'Position', [100, 100, 800, 600]);
hold on; grid on; view(3);
title('多无人机编队 3D 飞行轨迹', 'FontSize', 14, 'FontWeight', 'bold');
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');

% 绘制终点 (绿色五角星)
plot3(q_goal(1), q_goal(2), q_goal(3), 'p', 'MarkerSize', 18, 'MarkerFaceColor', 'g', 'MarkerEdgeColor', 'k');

% 循环绘制多障碍物球体
[X_sph, Y_sph, Z_sph] = sphere(50);
num_obs = size(obs_center, 2);
for k = 1:num_obs
    % 1. 内部实体球 (深灰色不透明)
    surf(obs_center(1,k) + obs_radius(k) * X_sph, ...
         obs_center(2,k) + obs_radius(k) * Y_sph, ...
         obs_center(3,k) + obs_radius(k) * Z_sph, ...
         'FaceColor', [0.2 0.2 0.2], 'EdgeColor', 'none', 'FaceAlpha', 1, 'HandleVisibility', 'off');
    % 2. 外部警戒圈 (橘色半透明)
    surf(obs_center(1,k) + obs_rho0(k) * X_sph, ...
         obs_center(2,k) + obs_rho0(k) * Y_sph, ...
         obs_center(3,k) + obs_rho0(k) * Z_sph, ...
         'FaceColor', [1 0.5 0], 'EdgeColor', 'none', 'FaceAlpha', 0.2, 'HandleVisibility', 'off');
end

% 打光与材质渲染 (打造真实 3D 质感)
camlight('headlight');    
lighting gouraud;         
material shiny;           

% 绘制所有无人机轨迹
for i = 1:num_agents
    p_traj = squeeze(P_data_matrix(:, i, :)); % 提取第 i 架飞机的 3 x T 轨迹
    plot3(p_traj(1,:), p_traj(2,:), p_traj(3,:), 'Color', colors(i,:), 'LineWidth', 1.5, ...
          'DisplayName', ['Node ', num2str(i-1)]);
    % 起点与终点打点
    plot3(p_traj(1,1), p_traj(2,1), p_traj(3,1), 'o', 'MarkerFaceColor', colors(i,:), 'HandleVisibility', 'off');
    plot3(p_traj(1,end), p_traj(2,end), p_traj(3,end), 's', 'MarkerFaceColor', colors(i,:), 'HandleVisibility', 'off');
end
legend('Location', 'best');


% =========================================================================
% 图 2: 三轴编队跟踪误差曲线 (相对编队槽位的偏差)
% =========================================================================
figure('Name', 'Formation Tracking Errors', 'Position', [950, 50, 600, 800]);
sgtitle('跟随者相对领航者的编队跟踪误差', 'FontSize', 15, 'FontWeight', 'bold');

P_leader = squeeze(P_data_matrix(:, 1, :)); % 领航者轨迹 3 x T
axis_titles = {'X轴误差 (m)', 'Y轴误差 (m)', 'Z轴误差 (m)'};

for axis_idx = 1:3
    subplot(3, 1, axis_idx); hold on; grid on;
    title(axis_titles{axis_idx});
    if axis_idx == 3, xlabel('Time (s)'); end
    
    for j = 1:num_followers
        agent_idx = j + 1; % 跟随者在矩阵中从第 2 列开始
        P_follower = squeeze(P_data_matrix(:, agent_idx, :));
        
        % 计算误差: p_leader - p_follower + delta_j (期望偏移量)
        err = P_leader - P_follower + delta(:, j);
        
        plot(time, err(axis_idx, :), 'Color', colors(agent_idx,:), 'LineWidth', 1.5, ...
             'DisplayName', ['Follower ', num2str(j)]);
    end
    if axis_idx == 1, legend('Location', 'best', 'NumColumns', 2); end
end


% =========================================================================
% 图 3: 无人机与最近障碍物中心的距离曲线
% =========================================================================
figure('Name', 'Distance to Nearest Obstacle', 'Position', [100, 400, 800, 300]);
hold on; grid on;
title('无人机与最近障碍物中心距离曲线', 'FontSize', 14, 'FontWeight', 'bold');
xlabel('Time (s)'); ylabel('Distance (m)');

for i = 1:num_agents
    p_traj = squeeze(P_data_matrix(:, i, :)); % 3 x T
    dist_to_obs = zeros(1, length(time));
    
    % 遍历每个时间步，寻找距离当前位置最近的障碍物
    for t_idx = 1:length(time)
        curr_p = p_traj(:, t_idx);
        % 计算到所有障碍物的距离
        dists = sqrt(sum((obs_center - curr_p).^2, 1));
        % 取最小值作为"当前面临的最危险距离"
        dist_to_obs(t_idx) = min(dists);
    end
    
    plot(time, dist_to_obs, 'Color', colors(i,:), 'LineWidth', 1.5, ...
         'DisplayName', ['Node ', num2str(i-1)]);
end

% 绘制警戒线与实体线 (参考第 1 个障碍物的半径设置即可)
yline(obs_rho0(1), 'r--', 'LineWidth', 2, 'DisplayName', '警戒圈半径 (参考)');
yline(obs_radius(1), 'k-', 'LineWidth', 2, 'DisplayName', '物理实体半径 (参考)');
legend('Location', 'best', 'NumColumns', 3);


% =========================================================================
% 图 4: 动态三维轨迹动画
% =========================================================================
figure('Name', '3D Dynamic Animation', 'Position', [950, 400, 800, 600]);
hold on; grid on; view(3);
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');

% 🚨 极其鲁棒的坐标轴锁定 (自动过滤 NaN 和 Inf 爆炸数据)
x_valid = P_data_matrix(1,:,:); x_valid = x_valid(isfinite(x_valid));
y_valid = P_data_matrix(2,:,:); y_valid = y_valid(isfinite(y_valid));
z_valid = P_data_matrix(3,:,:); z_valid = z_valid(isfinite(z_valid));

if isempty(x_valid)
    warning('⚠️ 轨迹数据无效，采用默认坐标轴范围。');
    min_x = -10; max_x = 10; min_y = -10; max_y = 10; min_z = -10; max_z = 10;
else
    min_x = min(x_valid) - 5; max_x = max(x_valid) + 5;
    min_y = min(y_valid) - 5; max_y = max(y_valid) + 5;
    min_z = min(z_valid) - 5; max_z = max(z_valid) + 5;
end
axis([min_x, max_x, min_y, max_y, min_z, max_z]);

% 绘制静态环境 (目标点与多障碍物)
plot3(q_goal(1), q_goal(2), q_goal(3), 'p', 'MarkerSize', 18, 'MarkerFaceColor', 'g');
for k = 1:num_obs
    surf(obs_center(1,k) + obs_radius(k) * X_sph, obs_center(2,k) + obs_radius(k) * Y_sph, obs_center(3,k) + obs_radius(k) * Z_sph, ...
         'FaceColor', [0.2 0.2 0.2], 'EdgeColor', 'none', 'FaceAlpha', 1);
    surf(obs_center(1,k) + obs_rho0(k) * X_sph, obs_center(2,k) + obs_rho0(k) * Y_sph, obs_center(3,k) + obs_rho0(k) * Z_sph, ...
         'FaceColor', [1 0.5 0], 'EdgeColor', 'none', 'FaceAlpha', 0.2);
end
camlight('headlight'); lighting gouraud; material shiny;

% 初始化飞机实体与拖尾
h_pos = zeros(1, num_agents);
h_traj = zeros(1, num_agents);
for i = 1:num_agents
    msize = 6; mstyle = '--'; lwidth = 1.5;
    if i == 1, msize = 10; mstyle = '-'; lwidth = 2.5; end % 领航者加粗加大
    
    h_pos(i) = plot3(P_data_matrix(1,i,1), P_data_matrix(2,i,1), P_data_matrix(3,i,1), 'o', ...
                     'MarkerSize', msize, 'MarkerFaceColor', colors(i,:), 'MarkerEdgeColor', 'k');
    h_traj(i) = plot3(P_data_matrix(1,i,1), P_data_matrix(2,i,1), P_data_matrix(3,i,1), mstyle, ...
                      'Color', colors(i,:), 'LineWidth', lwidth);
end

h_title = title('多智能体协同避障与编队动态过程', 'FontSize', 15, 'FontWeight', 'bold');

% 动画播放 (帧率控制，提取约 400 帧保证流畅度)
num_steps = length(time);
frame_step = round(num_steps / 400); 
if frame_step < 1, frame_step = 1; end

for t = 1:frame_step:num_steps
    set(h_title, 'String', sprintf('编队飞行中... Time: %.2f s', time(t)));
    for i = 1:num_agents
        % 更新当前机头位置
        set(h_pos(i), 'XData', P_data_matrix(1,i,t), 'YData', P_data_matrix(2,i,t), 'ZData', P_data_matrix(3,i,t));
        % 更新历史轨迹
        set(h_traj(i), 'XData', squeeze(P_data_matrix(1,i,1:t)), 'YData', squeeze(P_data_matrix(2,i,1:t)), 'ZData', squeeze(P_data_matrix(3,i,1:t)));
    end
    drawnow; 
end

% 确保最后一帧对齐终点
for i = 1:num_agents
    set(h_pos(i), 'XData', P_data_matrix(1,i,end), 'YData', P_data_matrix(2,i,end), 'ZData', P_data_matrix(3,i,end));
    set(h_traj(i), 'XData', squeeze(P_data_matrix(1,i,:)), 'YData', squeeze(P_data_matrix(2,i,:)), 'ZData', squeeze(P_data_matrix(3,i,:)));
end
set(h_title, 'String', sprintf('✅ 抵达终点！(总耗时: %.2f s)', time(end)));