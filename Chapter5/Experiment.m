clear; clc; close all;

addpath('Auxiliary/');

qrData = importdata('CA_210928_152943.txt');

for i = 1: length(qrData.colheaders)
    eval([eval(['qrData.colheaders{', num2str(i),'}']), '= qrData.data(:, ', num2str(i), ');']);
end
CNT = max(No)+1;
for i = 2: length(qrData.colheaders)
    %   0~48,  4~52,
    eval(['temp =', eval(['qrData.colheaders{', num2str(i),'}']),'(state >= int32(''4''));']);
    eval([eval(['qrData.colheaders{', num2str(i),'}']), '=reshape(temp, ',num2str(CNT),', []);']);
end

% tNow = tNow-tNow(1);
tNow = (1:length(tNow))*0.05;
vX   = (realX(:, 2:end) - realX(:, 1:end-1))/0.05; vX = [vX, vX(:, end)];
vY   = (realY(:, 2:end) - realY(:, 1:end-1))/0.05; vY = [vY, vY(:, end)];
vZ   = (realZ(:, 2:end) - realZ(:, 1:end-1))/0.05; vZ = [vZ, vZ(:, end)];
aX   = (vX(:, 2:end) - vX(:, 1:end-1))/0.05;  aX = [aX, aX(:, end)];
aY   = (vY(:, 2:end) - vY(:, 1:end-1))/0.05;  aY = [aY, aY(:, end)];
aZ   = (vZ(:, 2:end) - vZ(:, 1:end-1))/0.05;  aZ = [aZ, aZ(:, end)];
a_uresX = (uresX(:, 2:end) - uresX(:, 1:end-1)) / 0.05; a_uresX = [a_uresX, a_uresX(:, end)];
a_uresY = (uresY(:, 2:end) - uresY(:, 1:end-1)) / 0.05; a_uresY = [a_uresY, a_uresY(:, end)];
a_uresZ = (uresZ(:, 2:end) - uresZ(:, 1:end-1)) / 0.05; a_uresZ = [a_uresZ, a_uresZ(:, end)];

% set range
T   = 8.0;
state = reshape(state, CNT, []);
state = state(1, state(1, :)>=int32('4'));
index = 1:size(state, 2);
original_idx = index(state==int32('7'));
reshaped_idx = index(state==int32('8'));
% original_idx = original_idx(1, 1:(T/0.05));

try
    original_idx = original_idx(1, (1:(T/0.05))+300);
catch
    original_idx = [];
end

% reshaped_idx = reshaped_idx(1, (1:(T*1.3/0.05))+600);
try
    reshaped_idx = reshaped_idx(1, (1:(T*1.0/0.05))+000);
catch
    reshaped_idx = [];
end
t_original = tNow(1, original_idx)-min(tNow(1, original_idx));
t_reshape  = tNow(1, reshaped_idx)-min(tNow(1, reshaped_idx));

%% figures
% figure(1); hold on; set(gcf, 'position', [800,10,800, 500], 'color', 'w');
% % x-axis
% subplot(2, 1, 1); hold on; grid minor; axis equal;
% i = 1; plot3(t_original, realX(i, original_idx), realY(i, original_idx), 'k-', 'displayname', 'Agent_1 RP', 'linewidth', 1);
% i = 2; plot3(t_original, realX(i, original_idx), realY(i, original_idx), 'k:', 'displayname', 'Agent_2 RP', 'linewidth', 1);
% i = 3; plot3(t_original, realX(i, original_idx), realY(i, original_idx), 'k--', 'displayname', 'Agent_3 RP', 'linewidth', 1);
% xlabel('time (sec)');
% ylabel('[p]_1 (m)')
% zlabel('[p]_2 (m)')
% handle_f1_legend = legend('location', 'northeastoutside');
% set(handle_f1_legend, 'position', [0.8056    0.7190    0.1925    0.2260]);
% set(gca, 'fontsize', 18, 'position', [0.12, 0.57, 0.63, 0.44]);
% view(-20, 15);
% view(90, 0);
% 
% subplot(2, 1, 2); hold on; set(gca, 'fontsize', 16); grid minor; axis equal;
% i = 1; plot3(t_reshape, realX(i, reshaped_idx), realY(i, reshaped_idx), 'b-', 'displayname', 'Agent_1 RPRF', 'linewidth', 1);
% i = 2; plot3(t_reshape, realX(i, reshaped_idx), realY(i, reshaped_idx), 'b:', 'displayname', 'Agent_2 RPRF', 'linewidth', 1);
% i = 3; plot3(t_reshape, realX(i, reshaped_idx), realY(i, reshaped_idx), 'b--', 'displayname', 'Agent_3 RPRF', 'linewidth', 1);
% xlabel('time (sec)');
% ylabel('[p]_1 (m)')
% zlabel('[p]_2 (m)')
% handle_f2_legend = legend;
% set(handle_f2_legend, 'position', [0.7687    0.2070    0.2287    0.2260]);
% set(gca, 'fontsize', 18, 'position', [0.12, 0.12, 0.63, 0.44]);
% view(-20, 15);
% 
% view(90, 0);

% figure(4); 
% set(gcf, 'position', [10, 10, 800, 500], 'color', 'w');
% 
% subplot(2,1,1); hold on; 
% i = 1; plot(t_reshape, vecnorm([a_uresX(i, original_idx); a_uresY(i, original_idx)]), 'b-', 'displayname', 'Agent 1', 'linewidth', 1);
% i = 2; plot(t_reshape, vecnorm([a_uresX(i, original_idx); a_uresY(i, original_idx)]), 'b:', 'displayname', 'Agent 2', 'linewidth', 1);
% i = 3; plot(t_reshape, vecnorm([a_uresX(i, original_idx); a_uresY(i, original_idx)]), 'b--', 'displayname', 'Agent 3', 'linewidth', 1);
% handle_f41_legend = legend;
% set(handle_f41_legend, 'location', 'northwest', 'position', [0.7942    0.7513    0.1787    0.2080]);
% set(gca, 'fontsize', 18);
% grid minor;
% axis([0, T, 0, 20]);
% xlabel('time (sec)');
% ylabel({'derivative', '|v^{*d}_i|~(m/s^2)'})
% 
% subplot(2,1,2); hold on; 
% i = 1; plot(t_original, vecnorm([a_uresX(i, reshaped_idx); a_uresY(i, reshaped_idx)]), 'b-', 'displayname', 'Agent 1', 'linewidth', 1);
% i = 2; plot(t_original, vecnorm([a_uresX(i, reshaped_idx); a_uresY(i, reshaped_idx)]), 'b:', 'displayname', 'Agent 2', 'linewidth', 1);
% i = 3; plot(t_original, vecnorm([a_uresX(i, reshaped_idx); a_uresY(i, reshaped_idx)]), 'b--', 'displayname', 'Agent 3', 'linewidth', 1);
% handle_f42_legend = legend;
% set(handle_f42_legend, 'location', 'northwest', 'position', [0.7604    0.3433    0.2112    0.2080]);
% grid minor;
% axis([0, T, 0, 20]);
% 
% set(gca, 'fontsize', 18);
% xlabel('time (sec)');
% ylabel({'derivative', '|v^{*d}_i|~(m/s^2)'})


figure(6); 
set(gcf, 'position', [10, 10, 800, 400], 'color', 'w');

subplot(2,1,1); hold on; 
i = 1; plot(t_original, vecnorm([uresX(i, original_idx); uresY(i, original_idx)]), 'k-', 'displayname', 'agent 1', 'linewidth', 2);
i = 2; plot(t_original, vecnorm([uresX(i, original_idx); uresY(i, original_idx)]), 'k:', 'displayname', 'agent 2', 'linewidth', 2);
i = 3; plot(t_original, vecnorm([uresX(i, original_idx); uresY(i, original_idx)]), 'k--', 'displayname', 'agent 3', 'linewidth', 2);
handle_f41_legend = legend;
% set(handle_f41_legend, 'location', 'northwest', 'position', [0.7942    0.7513    0.1787    0.2080]);
set(handle_f41_legend, 'location', 'northeastoutside');
set(gca, 'fontsize', 17, 'box', 'on', 'position', [0.16, 0.65, 0.65, 0.33]);
grid minor;
axis([0, T, 0, 1.5]);
% xlabel('time (sec)');
ylabel({'RP', '|v^{*}_i| (m/s)'})

subplot(2,1,2); hold on; 
i = 1; plot(t_reshape, vecnorm([uresX(i, reshaped_idx); uresY(i, reshaped_idx)]), 'b-', 'displayname', 'agent 1', 'linewidth', 2);
i = 2; plot(t_reshape, vecnorm([uresX(i, reshaped_idx); uresY(i, reshaped_idx)]), 'b:', 'displayname', 'agent 2', 'linewidth', 2);
i = 3; plot(t_reshape, vecnorm([uresX(i, reshaped_idx); uresY(i, reshaped_idx)]), 'b--', 'displayname', 'agent 3', 'linewidth', 2);
% handle_f42_legend = legend('Orientation','horizontal', 'location', 'northwest');
% set(handle_f42_legend, 'location', 'northwest', 'position', [0.7604    0.3433    0.2112    0.2080]);
handle_f42_legend = legend;
set(handle_f42_legend, 'location', 'northeastoutside');
% set(handle_f42_legend, 'location', 'northwest');
grid minor;
axis([0, T, 0, 1.5]);
set(gca, 'fontsize', 17, 'box', 'on', 'position', [0.16, 0.2, 0.65, 0.33]);
xlabel('time (sec)');
ylabel({'RPRF', '|v^{*}_i| (m/s)'})



% the velocity
% figure(41); clf; hold on; 
% % set(gcf, 'position', [10, 10, 800, 300], 'color', 'w');
% plot(t_original, [vecnorm([uresX(1, original_idx); uresY(1, original_idx)]);vecnorm([uresX(2, original_idx); uresY(2, original_idx)]);vecnorm([uresX(3, original_idx); uresY(3, original_idx)])], ...
%     'k:*', 'linewidth', 2, 'displayname', 'RF');
% plot(t_reshape, [vecnorm([uresX(1, reshaped_idx); uresY(1, reshaped_idx)]);vecnorm([uresX(2, reshaped_idx); uresY(2, reshaped_idx)]);vecnorm([uresX(3, reshaped_idx); uresY(3, reshaped_idx)])], ...
%     'b-*', 'linewidth', 2, 'displayname', 'RFRF');
% handle_f4_legend = legend;
% % set(handle_f4_legend, 'location', 'southwest','Orientation','horizontal');
% set(handle_f4_legend, 'location', 'northwest');
% grid minor;
% set(gca, 'fontsize', 16);
% axis([0, T, 0, 2]);
% xlabel('time (sec)');
% ylabel({'maximal derivative', '$\max_{i=1,2,3} |v^{*d}_i|~(m/s^2)$'}, 'Interpreter','latex')



figure(5); clf; hold on; box on;
set(gcf, 'position', [10, 10, 800, 350], 'color', 'w');
set(gca, 'fontsize', 16);
xlim([0, 8]);
ylim([0, 1.4]);
grid on;
xlabel('$t$ (s)', 'Interpreter','latex');
ylabel('$min_{i\neq j}$ $|p_i-p_j|$ (m)', 'Interpreter','latex')
min_original_d = min_dist(realX(:, original_idx), realY(:, original_idx));
min_reshape_d  = min_dist(realX(:, reshaped_idx), realY(:, reshaped_idx));
plot(t_original, min_original_d, 'k-', 'LineWidth', 2)
plot(t_reshape,  min_reshape_d, 'b--', 'LineWidth', 2);
plot(t_original, 0.2*ones(size(t_original)), 'k:', 'LineWidth', 2, 'HandleVisibility', 'off')
plot(t_reshape,  0.2*ones(size(t_original)), 'k:', 'LineWidth', 2, 'HandleVisibility', 'off');
legend('RP', 'RPRF');

figure(7);
set(gcf, 'position', [10, 10, 800, 430], 'color', 'w');
subplot(1,2,1); hold on; axis equal;
original_sparse_idx = original_idx(1:6:size(original_idx, 2));
N = size(original_sparse_idx, 2);
i = 3; H1_3 = draw_agents([realX(i, original_sparse_idx); realY(i, original_sparse_idx)], [vX(i, original_sparse_idx); vY(i, original_sparse_idx)], [linspace(1, 0, N); linspace(1, 0, N); ones(1, N)]);
i = 2; H1_2 = draw_agents([realX(i, original_sparse_idx); realY(i, original_sparse_idx)], [vX(i, original_sparse_idx); vY(i, original_sparse_idx)], [linspace(1, 0, N); ones(1, N); linspace(1, 0, N)]);
i = 1; H1_1 = draw_agents([realX(i, original_sparse_idx); realY(i, original_sparse_idx)], [vX(i, original_sparse_idx); vY(i, original_sparse_idx)], [ones(1, N); linspace(1, 0, N); linspace(1, 0, N)]);
handle_f7l1 = legend([H1_1.cir(end), H1_2.cir(end), H1_3.cir(end)], 'agent 1', 'agent 2', 'agent 3', 'location', 'northwest');
xlabel('[p]_1 (m)');
ylabel('[p]_2 (m)');
title('RP');
% set(handle_f7l1, 'position', [0.1333    0.6778    0.1450    0.2133]);
set(gca, 'fontsize', 17, 'box', 'on', 'position', [0.08, 0.2, 0.44, 0.70]);
axis(1.6*[-1,1,-1,1]);
grid on;

subplot(1,2,2); hold on; axis equal;
reshaped_sparse_idx = reshaped_idx(1:6:size(reshaped_idx, 2));
N = size(reshaped_sparse_idx, 2);
i = 3; H2_3 = draw_agents([realX(i, reshaped_sparse_idx); realY(i, reshaped_sparse_idx)], [vX(i, reshaped_sparse_idx); vY(i, reshaped_sparse_idx)], [linspace(1, 0, N); linspace(1, 0, N); ones(1, N)]);
i = 2; H2_2 = draw_agents([realX(i, reshaped_sparse_idx); realY(i, reshaped_sparse_idx)], [vX(i, reshaped_sparse_idx); vY(i, reshaped_sparse_idx)], [linspace(1, 0, N); ones(1, N); linspace(1, 0, N)]);
i = 1; H2_1 = draw_agents([realX(i, reshaped_sparse_idx); realY(i, reshaped_sparse_idx)], [vX(i, reshaped_sparse_idx); vY(i, reshaped_sparse_idx)], [ones(1, N); linspace(1, 0, N); linspace(1, 0, N)]);
handle_f7l2 =  legend([H2_1.cir(end), H2_2.cir(end), H2_3.cir(end)], 'agent 1', 'agent 2', 'agent 3', 'location', 'northwest');
xlabel('[p]_1 (m)')
ylabel('[p]_2 (m)')
title('RPRF');
set(gca, 'fontsize', 17, 'box', 'on', 'position', [0.58, 0.2, 0.44, 0.70]);
% set(handle_f7l2, 'position', [0.5734    0.6778    0.1450    0.2133]);
axis(1.6*[-1,1,-1,1]);
grid on;


function [qrData, filename] = getlastlog(path)
fileFolder=fullfile(path);
dirOutput=dir(fullfile(fileFolder,'*.txt'));
[~, idx] = max([dirOutput.datenum]);
filename = dirOutput(idx).name;
last_log_path = [path, filename];
qrData = importdata(last_log_path);
fprintf('[LOG] I read %s\n', last_log_path);
end

function [qrData] = openlogs(path, names)
N = length(names);
for i = 1:N
    tmp = importdata([path, char(names(i))]);
    if(i == 1)
        qrData = tmp;
    else
        qrData.data = [qrData.data; tmp.data];
    end
end
end

function d = min_dist(x, y)
    % x, y: N×T
    assert(isequal(size(x), size(y)), 'x and y must have the same size.');

    N = size(x, 1);
    T = size(x, 2);

    K = N*(N-1)/2;
    dist = zeros(K, T);

    count = 0;
    for i = 1:N-1
        for j = i+1:N
            count = count + 1;

            dx = x(i,:) - x(j,:);
            dy = y(i,:) - y(j,:);

            % 1×T, 每个采样时刻的距离
            dist(count,:) = sqrt(dx.^2 + dy.^2);
            % 或者：dist(count,:) = vecnorm([dx; dy], 2, 1);
        end
    end

    % 1×T, 每时刻最小两两距离
    d = min(dist, [], 1);
end
function draw_position(realX,realY,realZ,aimX,aimY,aimZ, index)
figure(1); set(gcf, 'position', [0,0, 900, 300]);
i = 1; subplot(1,3,i); hold on;  view(2);axis equal; axis([-1,1,-1,1]*1.2); grid minor;
H.x   = animatedline('color', 'k', 'linestyle', '-', 'linewidth', 2);
H.x_r = animatedline('color', 'r', 'linestyle', '-', 'linewidth', 2);
i = 2; subplot(1,3,i); hold on;  view(2);axis equal; axis([-1,1,-1,1]*1.2); grid minor;
H.y   = animatedline('color', 'k', 'linestyle', '--', 'linewidth', 2);
H.y_r = animatedline('color', 'r', 'linestyle', '--', 'linewidth', 2);
i = 3; subplot(1,3,i); hold on;  view(2);axis equal; axis([-1,1,-1,1]*1.2); grid minor;
H.z   = animatedline('color', 'k', 'linestyle', ':', 'linewidth', 2);
H.z_r = animatedline('color', 'r', 'linestyle', ':', 'linewidth', 2);
cntDraw = size(realX(i, index), 2);
for i = 1:cntDraw
    addpoints(H.x,    realX(1, i), realY(1, i));
    addpoints(H.x_r,   aimX(1, i),  aimY(1, i));
    
    addpoints(H.y,    realX(2, i), realY(2, i));
    addpoints(H.y_r,   aimX(2, i),  aimY(2, i));
    
    addpoints(H.z,    realX(3, i), realY(3, i));
    addpoints(H.z_r,   aimX(3, i),  aimY(3, i));
%     drawnow;
end
end


% draw_position(realX, realY, realZ, aimX, aimY, aimZ, original_idx);
% figure(2); hold on;
% min_d = min_dist(realX(:, original_flag), realY(:, original_flag));
% plot(tNow(1, original_flag)-min(tNow(1, original_flag)), min_d, 'k', 'displayname', 'RP');
% min_d = min_dist(realX(:, reshaped_flag), realY(:, reshaped_flag));
% plot(tNow(1, reshaped_flag)-min(tNow(1, reshaped_flag)), min_d, 'b', 'displayname', 'RPRF');
% xlabel('t (sec)');
% xlabel('distance (m)');
% grid minor;


% figure(3); hold on; 
% line_styles = {'-', '--', ':'};
% i = 1; plot(t_original, vecnorm([a_uresX(i, original_idx); a_uresY(i, original_idx)]), ['k', line_styles{i}], 'linewidth', 2);
% i = 2; plot(t_original, vecnorm([a_uresX(i, original_idx); a_uresY(i, original_idx)]), ['k', line_styles{i}], 'linewidth', 2);
% i = 3; plot(t_original, vecnorm([a_uresX(i, original_idx); a_uresY(i, original_idx)]), ['k', line_styles{i}], 'linewidth', 2);
% i = 1; plot(t_reshape, vecnorm([a_uresX(i, reshaped_idx); a_uresY(i, reshaped_idx)]), ['b', line_styles{i}], 'linewidth', 2);
% i = 2; plot(t_reshape, vecnorm([a_uresX(i, reshaped_idx); a_uresY(i, reshaped_idx)]), ['b', line_styles{i}], 'linewidth', 2);
% i = 3; plot(t_reshape, vecnorm([a_uresX(i, reshaped_idx); a_uresY(i, reshaped_idx)]), ['b', line_styles{i}], 'linewidth', 2);


function handles = draw_agents(pos, vel, clr)
% pos \in \mathbb{R}^{2\times N}
% vel \in \mathbb{R}^{2\times N}
% clr \in \mathbb{R}^{3\times N}
N   = size(pos, 2);
Ds  = 0.2;
D   = 0.2;
cir = Ds*[cos(linspace(0, 2*pi, 40)); sin(linspace(0, 2*pi, 40))];
dirAngle = pi*5/6;
dir = D *[[cos(0); sin(0)], [cos(dirAngle); sin(dirAngle)], [cos(dirAngle); -sin(dirAngle)], [cos(0); sin(0)]];

for i = 1:N
    theta = atan2(vel(2, i), vel(1, i));
    rotate = [cos(theta), -sin(theta); sin(theta), cos(theta)];
    dir_tmp = rotate*dir;
    if(i == N)
        handles.cir(i) = patch('xdata', pos(1, i)+cir(1, :), 'ydata', pos(2, i)+cir(2, :), 'facecolor', clr(:, i), 'facealpha', 1.0, 'handlevisibility', 'on');
    else
        handles.cir(i) = patch('xdata', pos(1, i)+cir(1, :), 'ydata', pos(2, i)+cir(2, :), 'facecolor', clr(:, i), 'facealpha', 1.0, 'handlevisibility', 'off');
    end
%     handles.dir(i) = patch('xdata', pos(1, i)+dir_tmp(1, :), 'ydata', pos(2, i)+dir_tmp(2, :), 'facecolor', clr(:, i));
end

end

function fillwindows(ax)
outerpos = ax.OuterPosition;
ti = ax.TightInset; 
left = outerpos(1) + ti(1);
bottom = outerpos(2) + ti(2);
ax_width = outerpos(3) - ti(1) - ti(3);
ax_height = outerpos(4) - ti(2) - ti(4);
ax.Position = [left bottom ax_width ax_height];
end