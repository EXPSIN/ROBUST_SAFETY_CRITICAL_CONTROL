clear all
clc
close all

o1=[0.1;0];
o2=[-0.1;0];


times=1500;
timesc=7000;
step=0.0001;

q=zeros(2,times);
q(:,1)=[0;0];
v=zeros(2,times);

a=zeros(2,times);
opt = optimoptions('quadprog', 'Display','off');
m1=1;
m2=1;
L1=0.1;
L2=0.3;

ds=0.096;
dr=0.004;
vc=[0;10];

qc=zeros(2,times);
qc(:,1)=[0;0];
vc=zeros(2,times);
ac=zeros(2,times);
Kv=0.013;
cM=0.8;
dh=3.4975e-5;
basis_cnt = 21;
c0=cos(2*pi/basis_cnt);




[M, C, N] = MCN(q(:,1), v(:,1), m1,m2,L1,L2);

l1=(o1-q(:,1))/norm(o1-q(:,1));
l2=(o2-q(:,1))/norm(o2-q(:,1));
h1(1)=norm(o1-q(:,1))-ds;
h2(1)=norm(o2-q(:,1))-ds;
A=[l1';l2'];
b=[max(6*h1(1),6*c0*h1(1))-dr;max(6*h2(1),6*c0*h2(1))-dr];
H=[2,0;0,2];
f=[0;-10];
vstar=quadprog(H,f,A,b,[], [], [], [], [0;0], opt);
a(:,1)=-40*inv(M)*(v(:,1)-vstar);




lc = [cos(linspace(0, 2*pi, basis_cnt+1)); sin(linspace(0, 2*pi, basis_cnt+1))];
lc = lc(:, 1:end-1);
badd=ones(basis_cnt,1);
badd=badd*(cos(pi/basis_cnt))*(30-dr);


Ac=lc';
m=1;
while m<=basis_cnt
    bc(m,1)= min( h1+max((-Kv*(  l1'*lc(:,m)-sqrt(2-2*cM)  )/(6*c0))+dh,0),h2+max((-Kv*(  l2'*lc(:,m)-sqrt(2-2*cM)  )/(6*c0))+dh,0));
    bc(m,1)=min(6*bc(m,1),c0*(30-dr));
    m=m+1;
end
m=1;
H=[2,0;0,2];
f=[0;-10];

vstarc=quadprog(H,f,Ac,bc,[], [], [], [], [0;0], opt);
ac(:,1)=-40*inv(M)*(vc(:,1)-vstarc);




k=1;
while k<=times


    q(:,k+1)=q(:,k)+step*v(:,k);
    v(:,k+1)=v(:,k)+step*a(:,k);

    l1=(o1-q(:,k+1))/norm(o1-q(:,k+1));
    l2=(o2-q(:,k+1))/norm(o2-q(:,k+1));
    h1(k+1)=norm(o1-q(:,k+1))-ds;
    h2(k+1)=norm(o2-q(:,k+1))-ds;
    A=[l1';l2';lc'];
    b=[h1(k+1)-dr;h2(k+1)-dr;badd];
    H=[2,0;0,2];
    f=[0;-10];
    vstar=quadprog(H,f,A,b,[], [], [], [], [0;0], opt);
    vstar_r(:,k)=vstar;
    [M, C, N] = MCN(q(:,k+1), v(:,k+1), m1,m2,L1,L2);
    a(:,k+1)=-40*inv(M)*(v(:,k+1)-vstar);



    k=k+1;

end
k=1;
while k<=timesc





    qc(:,k+1)=qc(:,k)+step*vc(:,k);
    vc(:,k+1)=vc(:,k)+step*ac(:,k);

    l1=(o1-qc(:,k+1))/norm(o1-qc(:,k+1));
    l2=(o2-qc(:,k+1))/norm(o2-qc(:,k+1));
    h1c(k+1)=norm(o1-qc(:,k+1))-ds;
    h2c(k+1)=norm(o2-qc(:,k+1))-ds;

    q_ = qc(:, k+1);
    l1=(o1-q_)/norm(o1-q_);
    l2=(o2-q_)/norm(o2-q_);
    h1=norm(o1-q_)-ds;
    h2=norm(o2-q_)-ds;
    A=[l1';l2'];
    b=[max(6*h1,6*c0*h1)-dr;max(6*h2,6*c0*h2)-dr];


    m=1;
    while m<=basis_cnt
        bc(m,1)= min( h1c(k+1)+max((-Kv*(  l1'*lc(:,m)-sqrt(2-2*cM)  )/(6*c0))+dh,0),h2c(k+1)+max((-Kv*(  l2'*lc(:,m)-sqrt(2-2*cM)  )/(6*c0))+dh,0));
        bc(m,1)=min(6*bc(m,1),c0*(30-dr));
        m=m+1;
    end
    m=1;
    H=[2,0;0,2];
    f=[0;-10];

    vstarc=quadprog(H,f,Ac,bc,[], [], [], [], [0;0], opt);
    vstarc_r(:,k)=vstarc;
    [M, C, N] = MCN(qc(:,k+1), vc(:,k+1), m1,m2,L1,L2);
    ac(:,k+1)=-40*inv(M)*(vc(:,k+1)-vstarc);



    k=k+1;
end

figure(1)
hold on;
axis equal;
box on;
set(gcf, 'Position', [0 0 1000 400], 'Color', 'w');
xlim([-0.06 0.06]);
ylim([-0.01 0.05]);
xticks([-0.06:0.02:0.06]);
set(gca, 'fontsize', 24);




ho1_patch = patch(o1(1)+ds*cos(0:0.01*pi:2*pi),o1(2)+ds*sin(0:0.01*pi:2*pi), 'k', 'handlevisibility', 'off');
ho2_patch = patch(o2(1)+ds*cos(0:0.01*pi:2*pi),o2(2)+ds*sin(0:0.01*pi:2*pi), 'k', 'handlevisibility', 'off');
ho1_patchfill = hatchfill(ho1_patch, 'single', 45, 5, [1.0, 1.0, 1.0]);
ho2_patchfill = hatchfill(ho2_patch, 'single', 45, 5, [1.0, 1.0, 1.0]);
ho1_patchfill.HandleVisibility = 'off';
ho2_patchfill.HandleVisibility = 'off';
ho1_patchfill.LineWidth = 0.1;
ho2_patchfill.LineWidth = 0.1;

plot(q(1,:),q(2,:),'LineWidth',2,'LineStyle','--','Color','r', 'displayname', 'Normal QP');
plot(qc(1,:),qc(2,:),'LineWidth',2,'Color','b', 'displayname', 'Reshaped QP');
legend('Interpreter','latex','FontSize',24);



figure(2); clf;
hold on;
axis equal;
box on;
xlim([-0.06 0.06]);
ylim([-0.05 0.15]);
set(gcf, 'Position', [0 0 1000 400], 'Color', 'w');
set(gca, 'fontsize', 24);
range = [-0.1505, 0.1505, -0.03, 0.10];
axis(range);


ho1_r_patch = patch(o1(1)+0.1*cos(0:0.01*pi:2*pi), o1(2)+0.1*sin(0:0.01*pi:2*pi), 'c', 'handlevisibility', 'off', 'facealpha', 0.5, 'linestyle','none');
ho2_r_patch = patch(o2(1)+0.1*cos(0:0.01*pi:2*pi), o2(2)+0.1*sin(0:0.01*pi:2*pi), 'c', 'handlevisibility', 'off', 'facealpha', 0.5, 'linestyle','none');



ho1_patch = patch(o1(1)+ds*cos(0:0.01*pi:2*pi),o1(2)+ds*sin(0:0.01*pi:2*pi), 'k', 'handlevisibility', 'off');
ho2_patch = patch(o2(1)+ds*cos(0:0.01*pi:2*pi),o2(2)+ds*sin(0:0.01*pi:2*pi), 'k', 'handlevisibility', 'off');
ho1_patchfill = hatchfill(ho1_patch, 'single', 45, 5, [1.0, 1.0, 1.0]);
ho2_patchfill = hatchfill(ho2_patch, 'single', 45, 5, [1.0, 1.0, 1.0]);
ho1_patchfill.HandleVisibility = 'off';
ho2_patchfill.HandleVisibility = 'off';
ho1_patchfill.LineWidth = 0.1;
ho2_patchfill.LineWidth = 0.1;

legend('Interpreter','latex','Fontsize',24);

H_feasibleRegionFull = half_space_2D([], A, b, range*1.0, 'k', 'domain', 'Noraml Feasible Set', q_);
set(H_feasibleRegionFull.handle_space, 'facealpha', 0.1,  'linewidth', 1.0, 'EdgeColor', 0.5*ones(1,3));
H_feasibleRegion     = half_space_2D([], Ac, bc/1.1, range*1.0, 'c', 'domain', 'Reshaped Feasible Set', q_);
set(H_feasibleRegion.handle_space, 'facealpha', 0.5,  'linewidth', 1.0, 'EdgeColor', 0.5*ones(1,3));

H_fig2_basis_or = quiver(ones(size(A,1),1)*q_(1), 1.0*ones(size(A,1),1)*q_(2), A(:,1).*b(:), A(:,2).*b(:), 1.1, 'Color','k', 'LineWidth', 2, 'MaxHeadSize',0.3, 'HandleVisibility', 'off');
H_fig2_basis = quiver(ones(size(Ac,1),1)*qc(1,end), 1.0*ones(size(Ac,1),1)*qc(2,end), Ac(:,1).*bc(:)/1.1, Ac(:,2).*bc(:)/1.1, 1.1, 'Color','b', 'LineWidth', 2, 'MaxHeadSize',0.3, 'HandleVisibility', 'off');




















function [M, C, N] = MCN(q, v, m1,m2,L1,L2)

M=[m2*L2*L2+2*L2*m2*L1*cos(deg2rad(q(2)))+L1*L1*(m1+m2),m2*L2*L2+L2*m2*L1*cos(deg2rad(q(2)));m2*L2*L2+L2*m2*L1*cos(deg2rad(q(2))),L2*L2*m2];
C=[-2*m2*L1*L2*sin(deg2rad(q(2)))*v(2),-m2*L1*L2*sin(deg2rad(q(2)))*v(2);m2*L1*L2*sin(deg2rad(q(2)))*v(1),0];
N=[(m1+m2)*L1*9.8*cos(deg2rad(q(1)))+m2*L2*9.8*cos(deg2rad(q(1)+q(2)));m2*L2*9.8*cos(deg2rad(q(1)+q(2)))];

end



