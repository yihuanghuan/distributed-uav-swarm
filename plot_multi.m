%% plot_multi.m - 多智能体编队避障终极可视化脚本 (带经纬线地球仪多障碍物版)
% 包含了 3D 静态轨迹、编队误差曲线、最小避障距离曲线以及 3D 动态动画

% =========================================================================
% 1. 极其鲁棒的数据提取 (Try-Catch 终极防护)
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

if isa(raw_data, 'timeseries') || (isstruct(raw_data) && isfield(raw_data, 'Data'))
    P_data_matrix = raw_data.Data;
    time = raw_data.Time;
else
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

if iscolumn(time), time = time'; end
if size(P_data_matrix, 1) == length(time) && ndims(P_data_matrix) == 3
    P_data_matrix = permute(P_data_matrix, [2, 3, 1]);
end

num_agents = size(P_data_matrix, 2);
num_followers = num_agents - 1;
colors = lines(num_agents);
colors(1,:) = [1, 0, 0]; 

% =========================================================================
% ★ 核心美化：自定义渐变色池 (绿 -> 黄 -> 橙)
% =========================================================================
c_green  = [0.2, 0.8, 0.2];
c_yellow = [1.0, 0.9, 0.1];
c_orange = [1.0, 0.5, 0.0];
custom_cmap = [linspace(c_green(1), c_yellow(1), 32)', linspace(c_green(2), c_yellow(2), 32)', linspace(c_green(3), c_yellow(3), 32)';
               linspace(c_yellow(1), c_orange(1), 32)', linspace(c_yellow(2), c_orange(2), 32)', linspace(c_yellow(3), c_orange(3), 32)'];

% =========================================================================
% 图 1: 三维静态轨迹图 (带 3D 多障碍物地球仪渲染)
% =========================================================================
figure('Name', '3D Trajectories & Obstacles', 'Position', [100, 100, 800, 600]);
hold on; grid on; view(3);
title('多无人机编队 3D 飞行轨迹', 'FontSize', 14, 'FontWeight', 'bold');
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');

% 绘制终点
plot3(q_goal(1), q_goal(2), q_goal(3), 'p', 'MarkerSize', 18, 'MarkerFaceColor', 'g', 'MarkerEdgeColor', 'k');

% 循环绘制多障碍物球体
[X_sph, Y_sph, Z_sph] = sphere(40); % 生成 40x40 经纬度网格
num_obs = size(obs_center, 2);
for k = 1:num_obs
    X_in = obs_center(1,k) + obs_radius(k) * X_sph;
    Y_in = obs_center(2,k) + obs_radius(k) * Y_sph;
    Z_in = obs_center(3,k) + obs_radius(k) * Z_sph;
    
    X_out = obs_center(1,k) + obs_rho0(k) * X_sph;
    Y_out = obs_center(2,k) + obs_rho0(k) * Y_sph;
    Z_out = obs_center(3,k) + obs_rho0(k) * Z_sph;
    
    % 1. 内部物理实体球 (采用黄橙绿渐变色与深灰色经纬线)
    surf(X_in, Y_in, Z_in, Z_in, ...
         'FaceAlpha', 0.9, 'EdgeColor', [0.3 0.3 0.3], 'LineWidth', 0.5, 'HandleVisibility', 'off');
         
    % 2. 外部警戒圈 (保持为极淡的纯色半透明保护罩，不加条纹以免视觉杂乱)
    surf(X_out, Y_out, Z_out, ...
         'FaceColor', [1 0.5 0], 'EdgeColor', 'none', 'FaceAlpha', 0.15, 'HandleVisibility', 'off');
end
colormap(gca, custom_cmap); % 激活渐变色

camlight('headlight'); lighting gouraud; material shiny;           

for i = 1:num_agents
    p_traj = squeeze(P_data_matrix(:, i, :)); 
    plot3(p_traj(1,:), p_traj(2,:), p_traj(3,:), 'Color', colors(i,:), 'LineWidth', 1.5, ...
          'DisplayName', ['Node ', num2str(i-1)]);
    plot3(p_traj(1,1), p_traj(2,1), p_traj(3,1), 'o', 'MarkerFaceColor', colors(i,:), 'HandleVisibility', 'off');
    plot3(p_traj(1,end), p_traj(2,end), p_traj(3,end), 's', 'MarkerFaceColor', colors(i,:), 'HandleVisibility', 'off');
end
legend('Location', 'best');

% =========================================================================
% 图 2: 三轴编队跟踪误差曲线
% =========================================================================
figure('Name', 'Formation Tracking Errors', 'Position', [950, 50, 600, 800]);
sgtitle('跟随者相对领航者的编队跟踪误差', 'FontSize', 15, 'FontWeight', 'bold');

P_leader = squeeze(P_data_matrix(:, 1, :)); 
axis_titles = {'X轴误差 (m)', 'Y轴误差 (m)', 'Z轴误差 (m)'};

for axis_idx = 1:3
    subplot(3, 1, axis_idx); hold on; grid on;
    title(axis_titles{axis_idx});
    if axis_idx == 3, xlabel('Time (s)'); end
    
    for j = 1:num_followers
        agent_idx = j + 1; 
        P_follower = squeeze(P_data_matrix(:, agent_idx, :));
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
    p_traj = squeeze(P_data_matrix(:, i, :)); 
    dist_to_obs = zeros(1, length(time));
    for t_idx = 1:length(time)
        curr_p = p_traj(:, t_idx);
        dists = sqrt(sum((obs_center - curr_p).^2, 1));
        dist_to_obs(t_idx) = min(dists);
    end
    plot(time, dist_to_obs, 'Color', colors(i,:), 'LineWidth', 1.5, ...
         'DisplayName', ['Node ', num2str(i-1)]);
end

yline(obs_rho0(1), 'r--', 'LineWidth', 2, 'DisplayName', '警戒圈半径 (参考)');
yline(obs_radius(1), 'k-', 'LineWidth', 2, 'DisplayName', '物理实体半径 (参考)');
legend('Location', 'best', 'NumColumns', 3);

% =========================================================================
% 图 4: 动态三维轨迹动画
% =========================================================================
figure('Name', '3D Dynamic Animation', 'Position', [950, 400, 800, 600]);
hold on; grid on; view(3);
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');

x_valid = P_data_matrix(1,:,:); x_valid = x_valid(isfinite(x_valid));
y_valid = P_data_matrix(2,:,:); y_valid = y_valid(isfinite(y_valid));
z_valid = P_data_matrix(3,:,:); z_valid = z_valid(isfinite(z_valid));

if isempty(x_valid)
    min_x = -10; max_x = 10; min_y = -10; max_y = 10; min_z = -10; max_z = 10;
else
    min_x = min(x_valid) - 5; max_x = max(x_valid) + 5;
    min_y = min(y_valid) - 5; max_y = max(y_valid) + 5;
    min_z = min(z_valid) - 5; max_z = max(z_valid) + 5;
end
axis([min_x, max_x, min_y, max_y, min_z, max_z]);

plot3(q_goal(1), q_goal(2), q_goal(3), 'p', 'MarkerSize', 18, 'MarkerFaceColor', 'g');
for k = 1:num_obs
    X_in = obs_center(1,k) + obs_radius(k) * X_sph;
    Y_in = obs_center(2,k) + obs_radius(k) * Y_sph;
    Z_in = obs_center(3,k) + obs_radius(k) * Z_sph;
    
    X_out = obs_center(1,k) + obs_rho0(k) * X_sph;
    Y_out = obs_center(2,k) + obs_rho0(k) * Y_sph;
    Z_out = obs_center(3,k) + obs_rho0(k) * Z_sph;

    % 动画中的内部渐变色经纬线实体球
    surf(X_in, Y_in, Z_in, Z_in, ...
         'FaceAlpha', 0.9, 'EdgeColor', [0.3 0.3 0.3], 'LineWidth', 0.5);
    % 动画中的外部淡色半透明警戒圈
    surf(X_out, Y_out, Z_out, ...
         'FaceColor', [1 0.5 0], 'EdgeColor', 'none', 'FaceAlpha', 0.15);
end
colormap(gca, custom_cmap); % 激活渐变色
camlight('headlight'); lighting gouraud; material shiny;

h_pos = zeros(1, num_agents);
h_traj = zeros(1, num_agents);
for i = 1:num_agents
    msize = 6; mstyle = '--'; lwidth = 1.5;
    if i == 1, msize = 10; mstyle = '-'; lwidth = 2.5; end 
    h_pos(i) = plot3(P_data_matrix(1,i,1), P_data_matrix(2,i,1), P_data_matrix(3,i,1), 'o', ...
                     'MarkerSize', msize, 'MarkerFaceColor', colors(i,:), 'MarkerEdgeColor', 'k');
    h_traj(i) = plot3(P_data_matrix(1,i,1), P_data_matrix(2,i,1), P_data_matrix(3,i,1), mstyle, ...
                      'Color', colors(i,:), 'LineWidth', lwidth);
end

h_title = title('多智能体协同避障与编队动态过程', 'FontSize', 15, 'FontWeight', 'bold');

num_steps = length(time);
frame_step = round(num_steps / 400); 
if frame_step < 1, frame_step = 1; end

for t = 1:frame_step:num_steps
    set(h_title, 'String', sprintf('编队飞行中... Time: %.2f s', time(t)));
    for i = 1:num_agents
        set(h_pos(i), 'XData', P_data_matrix(1,i,t), 'YData', P_data_matrix(2,i,t), 'ZData', P_data_matrix(3,i,t));
        set(h_traj(i), 'XData', squeeze(P_data_matrix(1,i,1:t)), 'YData', squeeze(P_data_matrix(2,i,1:t)), 'ZData', squeeze(P_data_matrix(3,i,1:t)));
    end
    drawnow; 
end

for i = 1:num_agents
    set(h_pos(i), 'XData', P_data_matrix(1,i,end), 'YData', P_data_matrix(2,i,end), 'ZData', P_data_matrix(3,i,end));
    set(h_traj(i), 'XData', squeeze(P_data_matrix(1,i,:)), 'YData', squeeze(P_data_matrix(2,i,:)), 'ZData', squeeze(P_data_matrix(3,i,:)));
end
set(h_title, 'String', sprintf('✅ 抵达终点！(总耗时: %.2f s)', time(end)));