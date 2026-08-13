close all; clear all; clc;
addpath('Auxiliary/');

load('experiment_matlab_2024-12-18-17-37-49.mat');
H = graphic([]);

T = 0.1;
k = 1;
for t = 0:T:22
    feedback = feedback_res{k};
    my_slam = my_slam_res{k};
    [~, state] = my_slam.slamAlg.scansAndPoses(end);
    p_global = [state(1); state(2)];
    theta_global = state(3);
    k= k + 1;
    H = graphic(H, my_slam);
end

[~, states] = my_slam.slamAlg.scansAndPoses;

figure(11); clf;
set(gcf, 'position', [50,400,1600,700], 'color', 'w');
set(gca, 'fontsize', 24, 'LineWidth', 2, 'XAxisLocation', 'origin', 'YAxisLocation', 'origin');
hold on; box off;
axis equal; grid off;
plot(states(:, 1), states(:, 2), 'linewidth', 5, 'color', 'b');
xlabel('$[p]_1$ (m)', 'Interpreter','latex');
ylabel_handle = ylabel('$[p]_2$ (m)', 'Interpreter','latex');
ylabel_handle.Units = 'normalized';
ylabel_handle.Position = [0.05, 0.9, 0];
ylim([-1, 1]);
xlim([0,5]);
yticks(-0.5:0.5:0.5);

function H = graphic(H, my_slam)
persistent theta_last;

if(isempty(H))
    figure(1); clf;
    camup([1 0 0])
    set(gcf, 'position', [50,50,800,700], 'color', 'w');
    set(gca, 'fontsize', 16);
    hold on; axis equal;
    grid on;
    axis([-1,1,-1,1]*5);
    xticks(-5:5); yticks(-5:5);
    set(gca, 'XAxisLocation','origin', 'YAxisLocation','origin');
    title('Real-time Laser Scan Display - Top View');
    xlabel('[R^Tp]_1 or [v]_1');
    ylabel('[R^Tp]_2 or [v]_2');
    H.fig1_robot          = graphic_plant_2D([], [0; 0], [1, 0; 0, 1], 0.26, 'g');
    H.fig1_v_n       = quiver(0, 0, 1,0, 1.0, 'Color', 'k', 'LineWidth', 2, 'MaxHeadSize', 2);
    H.fig1_laserscan = scatter(0, 0, 1, 'Color', 'r');
    H.fig1_path_deadsolve = scatter(0, 0, 10, "filled", 'Color', 'b');
    H.fig1_v_deadsolve = quiver(0, 0, 1,0, 1.0, 'Color', 'r', 'LineWidth', 2, 'MaxHeadSize', 2);
    H.fig1_v_safe      = quiver(0, 0, 1,0, 1.0, 'Color', 'b', 'LineWidth', 2, 'MaxHeadSize', 2);
    H.fig1  = gcf;
    H.axes1 = gca;

    figure(2); clf;
    set(gcf, 'position', [50,800,500,500], 'color', 'w');
    set(gca, 'fontsize', 16);
    hold on;
    title('SLAM');
    axis equal; grid on;
    H.fig2_SLAM = gcf;
    H.axes2_SLAM = gca;

    H.fig2_pointcloud = scatter(0, 0, 1, 'Color', 'k');
    H.fig2_poses      = plot(0, 0, 'b.-', 'markersize', 4, 'LineWidth', 2);


    figure(3); clf;
    set(gcf, 'position', [1050,50,800,550], 'color', 'w');

    subplot(2,1,1);
    hold on;  box on; grid on;
    set(gca, 'fontsize', 16);
    ylabel({'Longitudinal', 'velocity (m/s)'}, 'Interpreter','latex');
    xlim([0, 22]);
    H.fig3_vx_safe     = animatedline('Color', 'b', 'LineWidth', 2, 'LineStyle', '-', 'displayname', '$[v]_1$');
    H.fig3_vx_original = animatedline('Color', 'b', 'LineWidth', 2, 'LineStyle', '--', 'displayname', '$[v_n]_1$');
    H.fig3_legend = legend('Interpreter', 'latex');
    H.fig3_legend.AutoUpdate = "off";

    subplot(2,1,2);
    hold on;  box on; grid on;
    set(gca, 'fontsize', 16);
    ylabel({'Lateral', 'velocity (m/s)'}, 'Interpreter','latex');
    xlabel('t (sec)', 'Interpreter','latex');
    xlim([0, 22]);
    H.fig3_vy_safe     = animatedline('Color', 'k', 'LineWidth', 2, 'LineStyle', '-', 'displayname', '$[v]_2$');
    H.fig3_vy_original = animatedline('Color', 'k', 'LineWidth', 2, 'LineStyle', '--', 'displayname', '$[v_n]_2$');
    H.fig3_legend = legend('Interpreter', 'latex');
    H.fig3_legend.AutoUpdate = "off";

    figure(4); clf;
    set(gcf, 'position', [1050,650,800, 400], 'color', 'w');

    subplot(2,1,1);
    hold on;  box on; grid on;
    set(gca, 'fontsize', 16, 'position', [0.15,0.7,0.78,0.28]);
    xlim([0, 22]);

    H.fig4_theta = animatedline('Color', 'b', 'LineWidth', 2, 'LineStyle', '-', 'displayname', 'atan2$(-[R]_{1,2},[R]_{1,1})$');
    ylabel('Angle (rad)');
    H.fig5_legend1 = legend('Interpreter', 'latex');
    H.fig5_legend1.AutoUpdate = "off";

    subplot(2,1,2);
    hold on;  box on; grid on;
    set(gca, 'fontsize', 16, 'position', [0.15,0.16,0.78,0.44]);
    xlim([0, 22]);
    ylabel({'Angular Velocity', '(rad/s)'});
    H.fig4_vw     = animatedline('Color', 'k', 'LineWidth', 2, 'LineStyle', '-', 'displayname', '$[\Omega]_{2,1}$');
    H.fig4_vw_ref = animatedline('Color', 'k', 'LineWidth', 2, 'LineStyle', ':', 'displayname', 'Reference of $[\Omega]_{2,1}$');
    xlabel('t (sec)');

    H.fig5_legend = legend('Interpreter', 'latex');
    H.fig5_legend.AutoUpdate = "off";

    figure(6); clf;
    set(gcf, 'position', [1050,700,800, 350], 'color', 'w');
    set(gca, 'fontsize', 16);
    hold on;  box on; grid on;
    H.fig6_dist = animatedline('Color', 'k', 'LineWidth', 2, 'LineStyle', '-', 'displayname', 'Minimum Distance');
    H.fig6_dist_expect = plot([0, 22], [0.26, 0.26], 'Color', [0.5,0.5,0.5], 'LineWidth', 2, 'LineStyle', '--', 'displayname', 'Expected Safety Distance');
    xlabel('t (sec)');
    ylabel('Distance (m)');
    ylim([0, 1]);
    xlim([0, 22]);
    H.fig6_legend = legend('Interpreter', 'latex');
    H.fig6_legend.AutoUpdate = "off";
else
    feedback = evalin("base", 'feedback');
    p_global = evalin("base", 'p_global');
    theta_global = evalin("base", 'theta_global');
    t = evalin("base", 't');
    if(isempty(theta_last))
        theta_last = ones(5, 1)*theta_global;
    end
    theta_global = theta_last(end) + atan2(sin(theta_global - theta_last(end)), cos(theta_global - theta_last(end)));
    vw_real      = (theta_global - theta_last(1)) / (0.1*length(theta_last));
    theta_last   = [theta_last(2:end, 1); theta_global(end)];

    H.fig1_robot = graphic_plant_2D(H.fig1_robot, [0; 0], [cos(theta_global), -sin(theta_global); sin(theta_global), cos(theta_global)], 0.26, 'g');
    H.laserscan.XData = my_slam.xy(1, :);
    H.laserscan.YData = my_slam.xy(2, :);
    H.fig1_v_n.UData  = feedback.cmd.vx_local;
    H.fig1_v_n.VData  = feedback.cmd.vy_local;
    H.fig1_v_safe.UData  = feedback.car.vx_safe_local;
    H.fig1_v_safe.VData  = feedback.car.vy_safe_local;


    H.fig1_laserscan.XData      = my_slam.position(1, :);
    H.fig1_laserscan.YData      = my_slam.position(2, :);
    H.fig1_path_deadsolve.XData = feedback.deadlock_resolving.path(1, :);
    H.fig1_path_deadsolve.YData = feedback.deadlock_resolving.path(2, :);
    H.fig1_v_deadsolve.UData    = feedback.deadlock_resolving.vx_local;
    H.fig1_v_deadsolve.VData    = feedback.deadlock_resolving.vy_local;


    addpoints(H.fig3_vx_safe              , t, feedback.car.vx_safe_local);
    addpoints(H.fig3_vy_safe              , t, feedback.car.vy_safe_local);
    addpoints(H.fig3_vx_original          , t, feedback.car.vx_ref_local);
    addpoints(H.fig3_vy_original          , t, feedback.car.vy_ref_local);


    addpoints(H.fig4_theta , t, theta_global);


    addpoints(H.fig4_vw    , t, vw_real);
    addpoints(H.fig4_vw_ref, t, pi*feedback.cmd.vw_local);

    addpoints(H.fig6_dist  , t, feedback.min_dist);


    plotSLAMMap(my_slam.slamAlg, H.fig2_pointcloud, H.fig2_poses);
end

drawnow limitrate;

end



function plotSLAMMap(slamAlg, handle_pointcloud, handle_poses)

    mapResolution = 20;
    maxLidarRange = 8;

    [scans, optimizedPoses] = scansAndPoses(slamAlg);


    handle_poses.XData = optimizedPoses(:,1);
    handle_poses.YData = optimizedPoses(:,2);

    allScanPoints = [];

    for i = 1:numel(scans)
        scanXY = scans{i}.Cartesian;

        R = [cos(optimizedPoses(i,3)) -sin(optimizedPoses(i,3));
             sin(optimizedPoses(i,3))  cos(optimizedPoses(i,3))];
        scanPoints = (R * scanXY')' + optimizedPoses(i,1:2);

        allScanPoints = [allScanPoints; scanPoints];
    end

    handle_pointcloud.XData = allScanPoints(:,1);
    handle_pointcloud.YData = allScanPoints(:,2);

end



function H = graphic_plant_2D(H, p, R, radius, clr)

if(isempty(H))
    H.dir = [1, -1/1.7937, -1/1.7937; 0, 0.4151, -0.4151]*0.618*radius;
    H.cir = [cos(linspace(0,2*pi,20)); sin(linspace(0,2*pi,20))]*radius;

    H.robot_cir  = patch(H.cir(1, :), H.cir(2, :), clr, 'HandleVisibility', 'off', 'FaceAlpha', 0.2);
    H.robot_dir  = patch(H.dir(1, :), H.dir(2, :), clr, 'HandleVisibility', 'off', 'FaceAlpha', 0.5);
    H.robot_path = animatedline(p(1), p(2), 'Color', clr, 'linewidth', 2, 'Linestyle', ':', 'HandleVisibility', 'off');
end

H.robot_cir.XData = p(1) + H.cir(1, :);
H.robot_cir.YData = p(2) + H.cir(2, :);

dir_realtime = R*H.dir;
H.robot_dir.XData = p(1) + dir_realtime(1, :);
H.robot_dir.YData = p(2) + dir_realtime(2, :);

addpoints(H.robot_path, p(1), p(2));

end
