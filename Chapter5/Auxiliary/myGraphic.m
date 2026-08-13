function H = myGraphic(H, p)
nao = size(p, 2);
na  = 1;
no  = nao - na;
Ds  = 0.68;
Do  = 0.6;
drawnow limitrate;
if(isempty(H))
    figure(1); clf;
    axis equal;
    hold on;
    grid on;
    set(gcf, 'position', [0,0, 800, 500 ], 'color', 'w');
    set(gca, 'fontsize', 16);

    for iO = (na+1):(na+no)
        H.shapeO(iO) = patch(p(1,iO)+Ds*sin(linspace(0, 2*pi, 30)), p(2,iO)+Ds*cos(linspace(0, 2*pi, 30)), 'c', 'facealpha', 0.2);
        H.realO(iO)  = patch(p(1,iO)+Do*sin(linspace(0, 2*pi, 30)), p(2,iO)+Do*cos(linspace(0, 2*pi, 30)), 'c', 'facealpha', 0.9);
    end
    H.posO = plot(p(1, na+1:end), p(2, na+1:end), 'k.', 'markersize', 5, 'linestyle', 'none');

    for iA = 1:na
        HandleVisibility  = 'off';
        H.trajA(iA) = animatedline('color', 'r', 'HandleVisibility', HandleVisibility, 'linewidth', 1.5, 'linestyle', '-', 'marker', 'none');
    end
end
