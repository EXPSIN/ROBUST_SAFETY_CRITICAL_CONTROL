clear all
clc
close all


root_dir = fileparts(mfilename('fullpath'));
func_folder = 'Auxiliary';
addpath(genpath(fullfile(root_dir,func_folder)));

o1=[0.1;0];
o2=[-0.1;0];

times=300;
timesc=14000;
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
c0=cos(2*pi/21);




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




lc=[cos(0:2*pi/21:40*pi/21);sin(0:2*pi/21:40*pi/21)];
badd=ones(21,1);
badd=badd*(cos(pi/21))*(30-dr);


Ac=lc';
m=1;
while m<=21
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



    m=1;
    while m<=21
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
xlim([-0.06 0.06]);
ylim([-0.05 0.15]);

set(gcf, 'position', [50,100,800, 400], 'color', 'w');
set(gca, 'fontsize', 18);
xlabel('$[q]_1$ (rad)', 'Interpreter','latex');
ylabel('$[q]_2$ (rad)', 'Interpreter','latex');
axis([-0.17, 0.17, -0.03, 0.15])

ho1 = plot(o1(1)+ds*cos(0:0.01*pi:2*pi),o1(2)+ds*sin(0:0.01*pi:2*pi),'k','LineWidth',2, 'handlevisibility', 'off');
ho2 = plot(o2(1)+ds*cos(0:0.01*pi:2*pi),o2(2)+ds*sin(0:0.01*pi:2*pi),'k','LineWidth',2, 'handlevisibility', 'off');
ho1_patch = patch(ho1.XData, ho1.YData, 'k', 'handlevisibility', 'off');
ho2_patch = patch(ho2.XData, ho2.YData, 'k', 'handlevisibility', 'off');
ho1_patchfill = hatchfill(ho1_patch, 'single', 45, 5, [1.0, 1.0, 1.0]);
ho2_patchfill = hatchfill(ho2_patch, 'single', 45, 5, [1.0, 1.0, 1.0]);
ho1_patchfill.HandleVisibility = 'off';
ho2_patchfill.HandleVisibility = 'off';

plot(q(1,:),q(2,:),'LineWidth',2,'LineStyle','--','Color','r', 'displayname', 'Usual');
plot(qc(1,:),qc(2,:),'LineWidth',2,'Color','b', 'displayname', 'Reshaped');
legend('Interpreter','latex');
























function [M, C, N] = MCN(q, v, m1,m2,L1,L2)

M=[m2*L2*L2+2*L2*m2*L1*cos(deg2rad(q(2)))+L1*L1*(m1+m2),m2*L2*L2+L2*m2*L1*cos(deg2rad(q(2)));m2*L2*L2+L2*m2*L1*cos(deg2rad(q(2))),L2*L2*m2];
C=[-2*m2*L1*L2*sin(deg2rad(q(2)))*v(2),-m2*L1*L2*sin(deg2rad(q(2)))*v(2);m2*L1*L2*sin(deg2rad(q(2)))*v(1),0];
N=[(m1+m2)*L1*9.8*cos(deg2rad(q(1)))+m2*L2*9.8*cos(deg2rad(q(1)+q(2)));m2*L2*9.8*cos(deg2rad(q(1)+q(2)))];

end



