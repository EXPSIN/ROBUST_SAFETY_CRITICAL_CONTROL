clear all; clc; close all;
addpath('Auxiliary/');
figure(1);
hold on;
axis equal;
axis off;
view(135,15);

set(gcf, 'color', 'w');

plot3DAxes(2)
fimplicit3(@(x,y,z)(0*x+0*y+z), [-1 1 -1 1 -1 1]*1.5, 'EdgeColor','none','FaceAlpha',.3, 'FaceColor', 'k');
fimplicit3(@(x,y,z)((1.5 - x.^2 - y.^2)-z), [-1 1 -1 1 -1 1]*2,'EdgeColor','none','FaceAlpha',.5);

H = animatedline('Color','b', 'LineStyle', '-', 'LineWidth', 2);
H_dash = animatedline('Color','b', 'LineStyle', ':', 'LineWidth', 2);
p = [-0.5; 0.5];

for i = 1:100
    v_r = [1; 0];

    [h, dh_dx] = h_fun(p(1),p(2));
    A = -dh_dx;
    b = h;

    if(A * v_r <= b)
        v = v_r;
    else
        v = (eye(2) - A'/(A*A')*A)*v_r + A'/(A*A')*b;
    end
    d = [1; 0]*0.0;
    p = p + (v+d)*0.05;

    addpoints(H, p(1), p(2), h);
    addpoints(H_dash, p(1), p(2));
    drawnow limitrate;
end


H = animatedline('Color','r', 'LineStyle', '-', 'LineWidth', 2);
H_dash = animatedline('Color','r', 'LineStyle', ':', 'LineWidth', 2);
p = [-0.5; 0.5];

for i = 1:100
    v_r = [1; 0];

    [h, dh_dx] = h_fun(p(1),p(2));
    A = -dh_dx;
    b = h;

    if(A * v_r <= b)
        v = v_r;
    else
        v = (eye(2) - A'/(A*A')*A)*v_r + A'/(A*A')*b;
    end

    d = [sin(i*0.05*2*pi/20); cos(i*0.05*2*pi/20)]*0.2;
    p = p + (v+d)*0.05;

    addpoints(H, p(1), p(2), h);
    addpoints(H_dash, p(1), p(2));
    drawnow limitrate;
end

function [h, dh_dx] = h_fun(x,y)
h     = 1.5 - x.^2 - y.^2;
dh_dx = -[2.*x, 2.*y];
end

function plot3DAxes(length)

    quiver3(0, 0, 0, length, 0, 0, 'k', 'LineWidth', 2, 'MaxHeadSize', 0.5);

    hold on;
    quiver3(0, 0, 0, 0, length, 0, 'k', 'LineWidth', 2, 'MaxHeadSize', 0.5);

    quiver3(0, 0, 0, 0, 0, length, 'k', 'LineWidth', 2, 'MaxHeadSize', 0.5);
end
