close all; clear all; clc;
addpath('Auxiliary/');
load('E250809_174230.mat');




x_struct  = [record_data.x];
run_state = [record_data.run_state];

mask = (run_state > 11 & run_state < 100);
t_max = 0;
styles = {'k-', 'b:'};
display_name = {'UC', 'PC'};
for i = 1:2
    if(i == 1)
        mask  = (run_state == 12);
        if(sum(mask) == 0)
            continue;
        end
    else
        mask  = (run_state == 14);
        if(sum(mask) == 0)
            continue;
        end
    end
    t         = [record_data.t_now];
    v_set     = [record_data.v];
    v_c       = [record_data.v_c];
    p         = [x_struct.p];
    v_real    = [x_struct.v];

    t         = t         (:, mask);
    v_set     = v_set     (:, mask);
    v_c       = v_c       (:, mask);
    p         = p         (:, mask);
    v_real    = v_real    (:, mask);
    t = t - min(t);
    t_max = max(t_max, max(t));

    figure(1);
    if(i == 1)
        set(gcf, 'position', [100, 100, 900, 500], 'color', 'w');
        hold on; axis equal; grid on; box on; view(2);
        set(gca, 'FontSize', 16);
        xlim([-2,2]); ylim([-1.0, 1.0]);
        xlabel('[p]_1 (m)');
        ylabel('[p]_2 (m)');
        zlabel('[p]_3 (m)');
        legend;
    end


    plot3(p(1,:), p(2,:), p(3,:), styles{i}, 'LineWidth', 2, 'DisplayName', display_name{i});


    theta = linspace(0, 2*pi, 30);
    circle_x = cos(theta);
    circle_y = sin(theta);

    H_obs1_ds = patch(obs.o(1, 1) + obs.Ds * circle_x, obs.o(2, 1) + obs.Ds * circle_y, -1*ones(size(circle_y)), 'c', 'FaceAlpha', 0.1, 'EdgeColor', 'b', 'LineStyle', 'none', 'LineWidth', 2, 'HandleVisibility', 'off');
    H_obs1 = patch(obs.o(1, 1) + obs.D * circle_x, obs.o(2, 1) + obs.D * circle_y, 'w', 'FaceAlpha', 1.0, 'EdgeColor', 'k', 'LineWidth', 2, 'HandleVisibility', 'off');
    H_obs1_hatch = hatchfill(H_obs1, 'single', 45, 10, [1, 1, 1]);
    H_obs1.FaceColor = 'w';
    H_obs1_hatch.HandleVisibility = 'off';


    H_obs2_ds = patch(obs.o(1, 2) + obs.Ds * circle_x, obs.o(2, 2) + obs.Ds * circle_y, -1*ones(size(circle_y)), 'c', 'FaceAlpha', 0.1, 'EdgeColor', 'b', 'LineStyle', 'none', 'LineWidth', 2, 'HandleVisibility', 'off');
    H_obs2 = patch(obs.o(1, 2) + obs.D * circle_x, obs.o(2, 2) + obs.D * circle_y, 'w', 'FaceAlpha', 1.0, 'EdgeColor', 'k', 'LineWidth', 2, 'HandleVisibility', 'off');
    H_obs2_hatch = hatchfill(H_obs2, 'single', 45, 10, [1, 1, 1]);
    H_obs2.FaceColor = 'w';
    H_obs2_hatch.HandleVisibility = 'off';



    figure(3);
    set(gcf, 'position', [50, 50, 900, 600], 'color', 'w');
    v_tilde_norm    = vecnorm(v_set(1:2, :) - v_real(1:2, :));
    dist_min        = min(vecnorm(p(1:2, :) - obs.o(:, 1)), vecnorm(p(1:2, :) - obs.o(:, 2)));
    Dini_v_set_norm = vecnorm((v_set(1:2,2:end) - v_set(1:2,1:end-1))/0.1);
    Dini_v_set_norm = [Dini_v_set_norm, Dini_v_set_norm(end)];
    subplot(3,1,1);
    if(i == 1)
        hold on; box on; grid on;
        set(gca, 'FontSize', 16);
        legend;
        ylabel('$|\tilde{v}|$', 'Interpreter','latex');
    end
    xlim([0, t_max]);
    plot(t,    v_tilde_norm,  styles{i}, 'LineWidth', 2, 'DisplayName', display_name{i});
    subplot(3,1,2);
    if(i == 1)
        hold on; box on; grid on;
        set(gca, 'FontSize', 16);
        legend;
        ylabel('$\min_i |p-o_i|-D$', 'Interpreter','latex');
    end
    xlim([0, t_max]);
    plot(t,        dist_min-obs.D,  styles{i}, 'LineWidth', 2, 'DisplayName', display_name{i});
    subplot(3,1,3);
    if(i == 1)
        hold on; box on; grid on;
        set(gca, 'FontSize', 16);
        legend;
        xlabel('t (s)');
        ylabel('$|D^+ v^*|$', 'Interpreter','latex');
    end
    xlim([0, t_max]);
    plot(t, Dini_v_set_norm,  styles{i}, 'LineWidth', 2, 'DisplayName', display_name{i});

end



