close all; clear all; clc;

addpath('Auxiliary/');

% 1. set the grids
range      = [-5.2, 1.7, -2.05, 2.05];
gca_pos    = [0.02,0.05,0.87,0.9];
zoom_range = [-0.10, 0.10, -0.10, 0.10]*3.0;
cnt        = 69*3+1;
% cnt        = 6+1;
grid_size  = abs([range(2) - range(1), range(4) - range(3)]) / cnt;
[X, Y]     = meshgrid(linspace(range(1), range(2), cnt), linspace(range(3), range(4), cnt));
cnt_x2     = 20;
x2_bar     = 1.0;
x2_list    = x2_bar*[cos(linspace(0,2*pi,cnt_x2)); sin(linspace(0,2*pi,cnt_x2))];

% 2. set the obstacles
p_o(:, 1) = [0;  1];
p_o(:, 2) = [0; -1];
R         = 0.99;
Radius    = 0.50;

p_o_circ  = [cos(linspace(0,2*pi,1000)); sin(linspace(0,2*pi,1000))];

[norm_original, norm_reshaped] = norm_of_solution_compare_new(X, Y, p_o, 1.0, exp(-R), [1; 0]);
% [norm_original, norm_reshaped] = norm_of_solution_compare_new_mex(X, Y, p_o, 1.0, exp(-R), [1; 0]);
% [norm_original, norm_reshaped] = norm_of_solution_compare_mex(X, Y, p_o, 1.0, D, [1; 0]);
norm_original(norm_original<0) = nan;
norm_reshaped(norm_reshaped<0) = nan;

% 3. set the parameters
figure(1); clf;
set(gcf, 'Position', [10, 10, 600, 300], 'color', 'w');
clrs = [[0.0, 0.0, 1.0]; [0.0, 0.0, 1.0]];
% axes(ax(1)); 
% subplot(1,2,1); 
axis equal; hold on; box on; grid off;
set(gca, 'FontSize', 16, 'XAxisLocation', 'Origin', 'YAxisLocation', 'origin', 'layer', 'top', 'xcolor', 'k', 'ycolor', 'k', 'LineWidth', 0.75, 'FontWeight', 'normal', 'position', gca_pos);
[H_original, h_colorbar] = plotColoredGrid_compare(X, Y, min(norm_original, 1), clrs(1, :), grid_size, 1.0, true);
h_colorbar.Label.String = {'Value of $|\rho_c(x_1)|$', '(darker = larger)'};
h_colorbar.Label.Interpreter = 'latex';
for i_o = 1:size(p_o, 2)
    H_obs1(i_o) = patch(p_o(1, i_o)+Radius*p_o_circ(1, :), p_o(2, i_o)+Radius*p_o_circ(2, :), 2*ones(size(p_o_circ(2, :))), 'w', 'facealpha', 1.0, 'handlevisibility', 'off');
    H_obs1_plot(i_o) = plot3(p_o(1, i_o)+R*p_o_circ(1, :), p_o(2, i_o)+R*p_o_circ(2, :), 2*ones(size(p_o_circ(2, :))), 'k', 'linewidth', 1);
    % H_obs1_hatch(i_o) = hatchfill(H_obs1(i_o),'single',45,10, [1,1,1]);
    H_obs1(i_o).FaceColor = 'w';
    H_obs1(i_o).FaceAlpha = 1.0;
    H_obs1(i_o).ZData     = 2*ones(size(H_obs1(i_o).XData));
    % H_obs1_hatch(i_o).ZData = 2*ones(size(H_obs1_hatch(i_o).XData));
end

%
plot3(p_o(1, :), p_o(2, :), 3*ones(size(p_o(1,:))), 'k', 'Marker', '.', 'markersize', 20, 'LineStyle', 'none');
text( p_o(1, 1)+0.1, p_o(2, 1), 3,                      '$o_1$ ', 'Interpreter', 'latex', 'HorizontalAlignment','right', 'FontSize', 20);
text( p_o(1, 2)+0.1, p_o(2, 2), 3,                      '$o_2$ ', 'Interpreter', 'latex', 'HorizontalAlignment','right', 'FontSize', 20);
xticks(-2:1:2)
yticks(-2:1:2)

xlabel('$[x_1]_1$', 'Interpreter','latex');
ylabel('$[x_1]_2$', 'Interpreter','latex');
% title('original');
axis(range)



hAxes = gca;                    % 获取当前坐标轴的句柄
hNewAxes = copyobj(hAxes, gcf); % 复制坐标轴
% fig1_zoom_range = drawRectangles(hAxes, zoom_range, 2);
% quiver3(zoom_range(1)-0.01, zoom_range(3)-0.01, 2, -1.05, -0.4, 2, 1.0, 'filled', 'LineWidth', 2, 'MaxHeadSize', 0.5, 'color', 'b');
axes(hNewAxes); grid off;
axis(zoom_range); xticks([]); yticks([]);
set(hNewAxes, 'Position', [0.005, 0.1, 0.5, 0.8], 'fontsize',14, 'xtick', []);    % 设置新坐标轴的位置


figure(2); clf;
set(gcf, 'Position', [10, 10, 600, 300], 'color', 'w');
axis equal; hold on; box on; grid off;
set(gca, 'FontSize', 16, 'XAxisLocation', 'Origin', 'YAxisLocation', 'origin', 'layer', 'top', 'xcolor', 'k', 'ycolor', 'k', 'LineWidth', 0.75, 'FontWeight', 'normal', 'position', gca_pos);
[H_reshaped, H_reshaped_colorbar] = plotColoredGrid_compare(X, Y, min(norm_reshaped, 1), clrs(2, :), grid_size, 0.7, true);
H_reshaped_colorbar.Label.String = {'Value of $|\rho_L(x_1)|$', '(darker = larger)'};
H_reshaped_colorbar.Label.Interpreter = 'latex';
% H_HOCBF.DisplayName = sprintf('reshaped');
for i_o = 1:size(p_o, 2)
    H_obs2(i_o) = patch(p_o(1, i_o)+Radius*p_o_circ(1, :), p_o(2, i_o)+Radius*p_o_circ(2, :), 2*ones(size(p_o_circ(2, :))), 'w', 'facealpha', 1.0, 'handlevisibility', 'off');
    H_obs2_plot(i_o) = plot3(p_o(1, i_o)+R*p_o_circ(1, :), p_o(2, i_o)+R*p_o_circ(2, :), 2*ones(size(p_o_circ(2, :))), 'k', 'linewidth', 1);
    % H_obs2_hatch(i_o) = hatchfill(H_obs2(i_o),'single',45,10, [1,1,1]);
    H_obs2(i_o).FaceColor = 'w';
    H_obs2(i_o).FaceAlpha = 1.0;
    H_obs2(i_o).ZData     = 2*ones(size(H_obs2(i_o).XData));
    % H_obs2_hatch(i_o).ZData = 2*ones(size(H_obs2_hatch(i_o).XData));
end
plot3(p_o(1, :), p_o(2, :), 3*ones(size(p_o(1,:))), 'k', 'Marker', '.', 'markersize', 20, 'LineStyle', 'none');
text( p_o(1, 1)+0.1, p_o(2, 1), 3,                      '$o_1$ ', 'Interpreter', 'latex', 'HorizontalAlignment','right', 'FontSize', 20);
text( p_o(1, 2)+0.1, p_o(2, 2), 3,                      '$o_2$ ', 'Interpreter', 'latex', 'HorizontalAlignment','right', 'FontSize', 20);
xticks(-2:1:2)
yticks(-2:1:2)

xlabel('$[x_1]_1$', 'Interpreter','latex');
ylabel('$[x_1]_2$', 'Interpreter','latex');
% title('reshaped');
axis(range)

hAxes2 = gca;                    % 获取当前坐标轴的句柄
hNewAxes2 = copyobj(hAxes2, gcf); % 复制坐标轴
% fig2_zoom_range = drawRectangles(hAxes2, zoom_range, 2);
% quiver3(zoom_range(1)-0.01, zoom_range(3)-0.01, 2, -1.05, -0.4, 2, 1.0, 'filled', 'LineWidth', 2, 'MaxHeadSize', 0.5, 'color', 'b');
axes(hNewAxes2); grid off;
axis(zoom_range); xticks([]); yticks([]);
set(hNewAxes2, 'Position', [0.005, 0.1, 0.5, 0.8], 'fontsize',14, 'xtick', []);    % 设置新坐标轴的位置



