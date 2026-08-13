close all; clear all; clc;
addpath('Auxiliary/');
H = draw_graphic('E240325_221647.mat', '-', '$k_{\alpha} = 8.0$',   'k');
H = draw_graphic('E240325_221943.mat', '--', '$k_{\alpha} = 12.0$', 'b');
H = draw_graphic('E240325_221839.mat', ':', '$k_{\alpha} = 16.0$',  'r');


figure(4)
setAxesFull(gcf, 0.1, 0.01, 0.2, 0.02, 0.00)


function H = draw_graphic(log_path, line_style, name, clr)
load(log_path);
H = graphic([], record_x(1), record_u(1), line_style, name, clr);
for i = 1:record_cnt
    if(record_x(i).t_now-H.t0 >= 21)
        return;
    end
    H = graphic(H, record_x(i), record_u(i), line_style, name, clr);
    drawnow limitrate;
end
end

function H = graphic(H, x, u, line_style, name, clr)
T = 21;
if(isempty(H))
    figure(1);
    hold on; axis equal;  grid on; box on;
    view(0, 0);
    axis([-3.5, 3.5, -4, 4, -0.1, 3.2])

    ylabel('Z');
    xlabel('$[x_1]_1$', 'Interpreter', 'latex');
    zlabel('$[x_1]_2$', 'Interpreter', 'latex');

    set(gcf, 'position', [0, 50, 800, 450], 'color', 'w');
    set(gca, 'fontsize', 16);

    H.fig3d_vtol      = vtol_show(-x.p(1), x.p(2), x.p(3), x.angle(1), x.angle(2), x.angle(3), []);
    H.fig3d_vtol_traj = animatedline('Color', clr, 'DisplayName', 'Trajectory, $k_{\gamma_2}$=?', 'LineWidth', 2.0, 'LineStyle', line_style, 'displayname', name);
    H.fig3d_obstacles_real = obstacle_show_2D_hatchfill([], u.safety_ctrl.obs_cell, u.safety_ctrl.r+u.safety_ctrl.margin, [0.8, 0.8, 0.8], 1.0, 1.0);
    legend('Location', 'northwest', 'Interpreter','latex');

    figure(3);
    hold on; grid on; box on;
    set(gcf, 'position', [800, 50, 800, 400], 'color', 'w');
    H.fig3_pz_ref = animatedline('Color', 'k',  'LineStyle', '--', 'LineWidth', 2.0, 'displayname', 'pz ref');
    H.fig3_pz     = animatedline('Color', 'k',  'LineStyle', '-', 'LineWidth', 2.0, 'displayname', 'pz real');
    H.fig3_az_ref = animatedline('Color', 'b',  'LineStyle', '-', 'LineWidth', 1.0, 'displayname', 'az');

    H.fig3_vz_ref = animatedline('Color', 'r',  'LineStyle', ':', 'LineWidth', 2.0, 'displayname', 'vz ref');
    H.fig3_vz     = animatedline('Color', 'r',  'LineStyle', '-', 'LineWidth', 2.0, 'displayname', 'vz');

    figure(4);
    hold on; grid on; box on;
    xlabel('Time (sec)', 'Interpreter', 'latex');
    ylabel('Distance (m)', 'Interpreter', 'latex');
    set(gca, 'fontsize', 16, 'XAxisLocation', 'origin');
    set(gcf, 'position', [800, 50, 800, 300], 'color', 'w');
    axis([0, T, 0.15, 0.55]);
    H.fig4_min_dist = animatedline('Color', 'k',  'LineStyle', line_style, 'LineWidth', 1.5, 'displayname', name, 'color', clr);
    plot([0,T], [u.safety_ctrl.r+u.safety_ctrl.margin, u.safety_ctrl.r+u.safety_ctrl.margin], 'k-', 'HandleVisibility', 'off');
    text(T, 0.35, 'Desired Safe Distance ', 'FontSize', 16, 'HorizontalAlignment','right','VerticalAlignment','top');
    legend('Location', 'northeast', 'Interpreter','latex');


    figure(5);
    set(gcf, 'position', [850, 10, 800, 350], 'color', 'w');

    subplot(2, 1, 1); hold on; grid on; box on;
    set(gca, 'fontsize', 16);
    axis([0, T, -2, 2]);
    ylabel('$[x^*_2]_1$ m/s', 'Interpreter', 'latex');
    H.fig5_xref2_1 = animatedline('Color', 'k',  'LineStyle', line_style, 'LineWidth', 1.5, 'displayname', name, 'color', clr);
    legend('Location', 'southeast', 'Interpreter','latex');

    subplot(2, 1, 2); hold on; grid on; box on;
    set(gca, 'fontsize', 16);
    xlabel('$t (s)$', 'Interpreter', 'latex');
    ylabel('$[x^*_2]_2$ (m/s)', 'Interpreter', 'latex');
    H.fig5_xref2_2 = animatedline('Color', 'k',  'LineStyle', line_style, 'LineWidth', 1.5, 'displayname', name, 'color', clr);
    legend('Location', 'southeast', 'Interpreter','latex');
    axis([0, T, -1.2, 1.2]);
    H.drawtime = x.t_now;
    H.t0 = x.t_now;



else
    if(H.drawtime == x.t_now)
        return;
    end
    H.drawtime = x.t_now-H.t0;
    H.fig3d_vtol = vtol_show(-x.p(1), x.p(2), x.p(3), x.angle(1), x.angle(2), x.angle(3), H.fig3d_vtol);
    addpoints(H.fig3d_vtol_traj, -x.p(1), x.p(2), x.p(3));

    addpoints(H.fig3_pz_ref, x.t_now-H.t0, u.pz_ref);
    addpoints(H.fig3_pz,     x.t_now-H.t0, x.p(3));
    addpoints(H.fig3_az_ref, x.t_now-H.t0, u.az_ref/9.8);

    addpoints(H.fig3_vz,     x.t_now-H.t0, x.v(3));
    addpoints(H.fig3_vz_ref, x.t_now-H.t0, u.vz_ref);

    addpoints(H.fig4_min_dist, x.t_now-H.t0, min([u.safety_ctrl.h1; u.safety_ctrl.h2]+u.safety_ctrl.r));

    addpoints(H.fig5_xref2_1, x.t_now-H.t0, u.x2_ref(1));
    addpoints(H.fig5_xref2_2, x.t_now-H.t0, u.x2_ref(2));
end
drawnow limitrate;
end


function H = vtol_show(x, y, z, roll, pitch, yaw, H)
R = eul2rotm([yaw, pitch, roll], "ZYX")';
p = [x, y, z];

if(isempty(H))
    H.vertices = [ 0.2205	 0.2055	0.1865	0.1305	0.1125	0.1175	0.1725	0.1855	0.1715	0.1135	0.00650	-0.1255	-0.1145	-0.0955	-0.0715	-0.0925	-0.2225	-0.2225	-0.0925	-0.0715	-0.0955	-0.1145	-0.1255	0.00650	0.1135	0.1715	0.1855	0.1725	0.1175	0.1125	0.1305	0.1865	0.2055	0.2205	0.2205	0.2055	0.1865	0.1305	0.1125	0.1175	0.1725	0.1855	0.1715	0.1135	0.00650	-0.1255	-0.1145	-0.0955	-0.0715	-0.0925	-0.2225	-0.2225	-0.0925	-0.0715	-0.0955	-0.1145	-0.1255	0.00650	0.1135	0.1715	0.1855	0.1725	0.1175	0.1125	0.1305	0.1865	0.2055	0.2205;
                      0	-0.0370	-0.0430	-0.0630	-0.0890	-0.109	-0.110	-0.128	-0.146	-0.146	-0.317	-0.349	-0.317	-0.260	-0.0830	-0.0240	0	0	0.0240	0.0830	0.260	0.317	0.349	0.317	0.146	0.146	0.128	0.110	0.109	0.0890	0.0630	0.0430	0.0370	0	0	-0.0370	-0.0430	-0.0630	-0.0890	-0.109	-0.110	-0.128	-0.146	-0.146	-0.317	-0.349	-0.317	-0.260	-0.0830	-0.0240	0	0	0.0240	0.0830	0.260	0.317	0.349	0.317	0.146	0.146	0.128	0.110	0.109	0.0890	0.0630	0.0430	0.0370	0;
                 -0.0443	-0.0428	-0.0409	-0.0353	-0.0335	-0.0340	-0.0395	-0.0408	-0.0394	-0.0336	-0.0229	-0.00970	-0.0108	-0.0127	-0.0151	-0.0130	0	0	-0.0130	-0.0151	-0.0127	-0.0108	-0.00970	-0.0229	-0.0336	-0.0394	-0.0408	-0.0395	-0.0340	-0.0335	-0.0353	-0.0409	-0.0428	-0.0443	-0.0886	-0.0856	-0.0818	-0.0706	-0.0670	-0.0680	-0.0790	-0.0816	-0.0788	-0.0672	-0.0458	-0.0194	-0.0216	-0.0254	-0.0302	-0.0260	0	0	-0.0260	-0.0302	-0.0254	-0.0216	-0.0194	-0.0458	-0.0672	-0.0788	-0.0816	-0.0790	-0.0680	-0.0670	-0.0706	-0.0818	-0.0856	-0.0886]';
    H.cnt = size(H.vertices, 1);
    H.cnt_half = H.cnt / 2;
    H.face_index1 = [1:H.cnt_half; (H.cnt_half+1):H.cnt];
    H.face_index2 = mod((1:H.cnt_half)'+[0, 1, 1, 0]-1, H.cnt_half) + 1 + [0,0,H.cnt_half,H.cnt_half];
    rotated_vertices = H.vertices*R + p;
    H.handle_surface1 = patch('Faces', H.face_index1, 'Vertices', rotated_vertices, 'FaceColor', 'c', 'handlevisibility', 'off');
    H.handle_surface2 = patch('Faces', H.face_index2, 'Vertices', rotated_vertices, 'FaceColor', 'c', 'handlevisibility', 'off');

    radius = 100 / 1000;
    motor_angle_bias = -pi/20;
    H.circle = (radius*[cos(motor_angle_bias), 0, -sin(motor_angle_bias); 0, 1, 0; sin(motor_angle_bias), 0, cos(motor_angle_bias)]*[cos(linspace(0, 2*pi, 20)); sin(linspace(0, 2*pi, 20)); zeros(1, 20)])';
    H.motor_position = H.vertices([8, 17, 27], :) + [zeros(3,1), [0.045; 0; 0], zeros(3,1)]';

    motor = (H.circle + H.motor_position(1, :))*R + p;
    H.handle_motor1 = patch(motor(:, 1), motor(:, 2), motor(:, 3), 'k-', 'linewidth', 2, 'handlevisibility', 'off', 'FaceAlpha', 0.2);
    motor = (H.circle + H.motor_position(2, :))*R + p;
    H.handle_motor2 = patch(motor(:, 1), motor(:, 2), motor(:, 3), 'k-', 'linewidth', 2, 'handlevisibility', 'off', 'FaceAlpha', 0.2);
    motor = (H.circle + H.motor_position(3, :))*R + p;
    H.handle_motor3 = patch(motor(:, 1), motor(:, 2), motor(:, 3), 'k-', 'linewidth', 2, 'handlevisibility', 'off', 'FaceAlpha', 0.2);
else
    rotated_vertices  = H.vertices*R + p;
    H.handle_surface1.Vertices = rotated_vertices;
    H.handle_surface2.Vertices = rotated_vertices;

    motor = (H.circle + H.motor_position(1, :))*R + p;
    H.handle_motor1.XData = motor(:, 1);
    H.handle_motor1.YData = motor(:, 2);
    H.handle_motor1.ZData = motor(:, 3);

    motor = (H.circle + H.motor_position(2, :))*R + p;
    H.handle_motor2.XData = motor(:, 1);
    H.handle_motor2.YData = motor(:, 2);
    H.handle_motor2.ZData = motor(:, 3);

    motor = (H.circle + H.motor_position(3, :))*R + p;
    H.handle_motor3.XData = motor(:, 1);
    H.handle_motor3.YData = motor(:, 2);
    H.handle_motor3.ZData = motor(:, 3);
end
end

function H = obstacle_show_2D(H, obs_cell, r, clr, layer, facealpha)
if(~isempty(H) && isequal(obs_cell, H.obs_cell))
    return;
end

for i = 1:size(obs_cell, 2)
    obs = obs_cell{i};
    angle          = zeros(1, size(obs,2));
    angle(1:end-1) = atan2(obs(2, 2:end)-obs(2, 1:end-1), obs(1, 2:end)-obs(1, 1:end-1));
    angle(end)     = angle(1);
    angle = angle+pi/2;
    angle = atan2(sin(angle), cos(angle));

    item_cnt = 20;
    line_pos = zeros(2, item_cnt*4);
    line_pos_record = zeros(2, item_cnt*4*(size(obs, 2)-1));
    for j = 1:size(obs, 2)-1
        line_pos(:, 1) = obs(:,j) + r(i)*[cos(angle(j)); sin(angle(j))];
        line_pos(:, item_cnt*0+(1:item_cnt)) = line_pos(:, 1) + [linspace(0, obs(1, j+1)-obs(1, j), item_cnt); linspace(0, obs(2, j+1)-obs(2, j), item_cnt)];
        angle_segments = linspace(angle(j), angle(j)-pi, item_cnt);
        line_pos(:, item_cnt*1+(1:item_cnt)) = obs(:, j+1) + r(i)*[cos(angle_segments); sin(angle_segments)];
        line_pos(:, item_cnt*2+(1:item_cnt)) = line_pos(:, item_cnt*2) + [linspace(0, obs(1, j)-obs(1, j+1), item_cnt); linspace(0, obs(2, j)-obs(2, j+1), item_cnt)];
        angle_segments = linspace(angle(j)+pi, angle(j), item_cnt);
        line_pos(:, item_cnt*3+(1:item_cnt)) = obs(:, j) + r(i)*[cos(angle_segments); sin(angle_segments)];
        line_pos_record(:, (j-1)*item_cnt*4 + (1:item_cnt*4)) = line_pos;

        H.handle_patch{i}.patch(j) = patch(line_pos(1, :), layer*ones(1, size(line_pos,2)), line_pos(2, :), clr, 'handlevisibility', 'off', 'linestyle', 'none', 'facealpha', facealpha);
    end
    H.handle_line(i)  = plot3(obs(1, :), layer*ones(size(obs(1, :))), obs(2, :), 'k', 'linewidth', 2, 'marker', '.', 'linestyle', '-', 'HandleVisibility', 'off');
end

end

function H = obstacle_show_2D_hatchfill(H, obs_cell, r, clr, layer, facealpha)
if(~isempty(H) && isequal(obs_cell, H.obs_cell))
    return;
end

for i = 1:size(obs_cell, 2)
    obs = obs_cell{i};
    angle          = zeros(1, size(obs,2));
    angle(1:end-1) = atan2(obs(2, 2:end)-obs(2, 1:end-1), obs(1, 2:end)-obs(1, 1:end-1));
    angle(end)     = angle(1);
    angle = angle+pi/2;
    angle = atan2(sin(angle), cos(angle));

    item_cnt = 20;
    line_pos = zeros(2, item_cnt*4);
    line_pos_record = zeros(2, item_cnt*4*(size(obs, 2)-1));
    for j = 1:size(obs, 2)-1
        line_pos(:, 1) = obs(:,j) + r(i)*[cos(angle(j)); sin(angle(j))];
        line_pos(:, item_cnt*0+(1:item_cnt)) = line_pos(:, 1) + [linspace(0, obs(1, j+1)-obs(1, j), item_cnt); linspace(0, obs(2, j+1)-obs(2, j), item_cnt)];
        angle_segments = linspace(angle(j), angle(j)-pi, item_cnt);
        line_pos(:, item_cnt*1+(1:item_cnt)) = obs(:, j+1) + r(i)*[cos(angle_segments); sin(angle_segments)];
        line_pos(:, item_cnt*2+(1:item_cnt)) = line_pos(:, item_cnt*2) + [linspace(0, obs(1, j)-obs(1, j+1), item_cnt); linspace(0, obs(2, j)-obs(2, j+1), item_cnt)];
        angle_segments = linspace(angle(j)+pi, angle(j), item_cnt);
        line_pos(:, item_cnt*3+(1:item_cnt)) = obs(:, j) + r(i)*[cos(angle_segments); sin(angle_segments)];
        line_pos_record(:, (j-1)*item_cnt*4 + (1:item_cnt*4)) = line_pos;

        H.handle_patch{i}.patch(j) = patch(line_pos(1, :), layer*ones(1, size(line_pos,2)), line_pos(2, :), 'w', 'handlevisibility', 'off', 'linestyle', 'none', 'facealpha', 1.0);
        H.handle_patch{i}.patch(j) = patch(line_pos(1, :), line_pos(2, :), clr, 'handlevisibility', 'off', 'linestyle', 'none', 'facealpha', facealpha);
        H.handle_patch{i}.hacthfill(j) = hatchfill(H.handle_patch{i}.patch(j), 'single', 45, 5, [1.0, 1.0, 1.0]);
        H.handle_patch{i}.hacthfill(j).HandleVisibility = 'off';
        H.handle_patch{i}.hacthfill(j).ZData = H.handle_patch{i}.hacthfill(j).YData;
        H.handle_patch{i}.hacthfill(j).YData = layer*ones(1, size(H.handle_patch{i}.hacthfill(j).YData, 2));
    end
    H.handle_line(i)  = plot3(line_pos(1, :), layer*ones(1, size(line_pos, 2)), line_pos(2, :), 'k', 'linewidth', 0.5, 'marker', 'none', 'linestyle', '-', 'HandleVisibility', 'off');
end

end
