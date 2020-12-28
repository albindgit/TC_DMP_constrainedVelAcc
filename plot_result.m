close all


figure('Name','Velocity')

subplot(2,1,1)
plot(res{1}.s,res{1}.ref_vel(1,:),'b-','LineWidth',1.5), hold on
plot(res{2}.s,res{2}.ref_vel(1,:),'k--','LineWidth',1.5)
plot(res{3}.s,res{3}.ref_vel(1,:),'m-.','LineWidth',1.5)
plot([0 sim_params.s_v_max_online],[sim_params.v_max_online(1) sim_params.v_max_online(1)],'r:','LineWidth',1)
plot([0 sim_params.s_v_max_online],-[sim_params.v_max_online(1) sim_params.v_max_online(1)],'r:','LineWidth',1)
yticks([])
xticks([round(min([res{1}.s res{2}.s res{3}.s]),2) sim_params.s_v_max_online 0.9 1])
xlim([min([res{1}.s res{2}.s res{3}.s]) 1])
xlabel('$s$', 'Interpreter','latex','Fontsize',30)
ylim([min([res{2}.ref_vel(1,:) res{2}.ref_vel(2,:)]) max([res{2}.ref_vel(1,:) res{2}.ref_vel(2,:)])])
ylabel('$\dot{y}_1$', 'Interpreter','latex','Fontsize',30)
set(gca,'XDir','reverse')
set(gca,'FontSize',16)

subplot(2,1,2)
plot(res{1}.s,res{1}.ref_vel(2,:),'b-','LineWidth',1.5), hold on
plot(res{2}.s,res{2}.ref_vel(2,:),'k--','LineWidth',1.5)
plot(res{3}.s,res{3}.ref_vel(2,:),'m-.','LineWidth',1.5)
plot([0 sim_params.s_v_max_online],[sim_params.v_max_online(2) sim_params.v_max_online(2)],'r:','LineWidth',1)
plot([0 sim_params.s_v_max_online],-[sim_params.v_max_online(2) sim_params.v_max_online(2)],'r:','LineWidth',1)
yticks([-sim_params.v_max_online(2) 0 sim_params.v_max_online(2)])
set(gca, 'YTickLabel', {'-v_{max}','0','v_{max}'},'FontSize',16);
xticks([round(min([res{1}.s res{2}.s res{3}.s]),2) sim_params.s_v_max_online 0.9 1])
xlim([min([res{1}.s res{2}.s res{3}.s]) 1])
xlabel('$s$', 'Interpreter','latex','Fontsize',30)
ylim([min([res{2}.ref_vel(1,:) res{2}.ref_vel(2,:)]) max([res{2}.ref_vel(1,:) res{2}.ref_vel(2,:)])])
ylabel('$\dot{y}_2$', 'Interpreter','latex','Fontsize',30)
set(gca,'XDir','reverse')

figure('Name','Acceleration')
subplot(2,1,1)
plot(res{1}.s,res{1}.ref_acc(1,:),'b-','LineWidth',1.5), hold on
plot(res{2}.s,res{2}.ref_acc(1,:),'k--','LineWidth',1.5)
plot(res{3}.s,res{3}.ref_acc(1,:),'m-.','LineWidth',1.5)
plot([res{3}.s(1) res{3}.s(end)],[sim_params.a_max(1) sim_params.a_max(1)],'r:','LineWidth',1)
plot([res{3}.s(1) res{3}.s(end)],-[sim_params.a_max(1) sim_params.a_max(1)],'r:','LineWidth',1)
plot([res{1}.s(1) res{1}.s(end)],[sim_params.a_max(1) sim_params.a_max(1)],'r:','LineWidth',1)
plot([res{1}.s(1) res{1}.s(end)],-[sim_params.a_max(1) sim_params.a_max(1)],'r:','LineWidth',1)
xticks([round(min([res{1}.s res{2}.s res{3}.s]),2) sim_params.s_v_max_online 0.9 1])
yticks([-sim_params.a_max(1) 0 sim_params.a_max(1)])
set(gca, 'YTickLabel', {'-a_{max}','0','a_{max}'},'FontSize',16);
xlim([min([res{1}.s res{2}.s res{3}.s]) 1])
xlabel('$s$', 'Interpreter','latex','Fontsize',30)
ylim([min([res{2}.ref_acc(1,:) res{2}.ref_acc(2,:)]) max([res{2}.ref_acc(1,:) res{2}.ref_acc(2,:)])])
ylabel('$\ddot{y}_1$', 'Interpreter','latex','Fontsize',30)
set(gca,'XDir','reverse')

subplot(2,1,2)
plot(res{1}.s,res{1}.ref_acc(2,:),'b-','LineWidth',1.5), hold on
plot(res{2}.s,res{2}.ref_acc(2,:),'k--','LineWidth',1.5)
plot(res{3}.s,res{3}.ref_acc(2,:),'m-.','LineWidth',1.5)
plot([res{3}.s(1) res{3}.s(end)],[sim_params.a_max(2) sim_params.a_max(2)],'r:','LineWidth',1)
plot([res{3}.s(1) res{3}.s(end)],-[sim_params.a_max(2) sim_params.a_max(2)],'r:','LineWidth',1)
plot([res{1}.s(1) res{1}.s(end)],[sim_params.a_max(2) sim_params.a_max(2)],'r:','LineWidth',1)
plot([res{1}.s(1) res{1}.s(end)],-[sim_params.a_max(2) sim_params.a_max(2)],'r:','LineWidth',1)
xticks([round(min([res{1}.s res{2}.s res{3}.s]),2) sim_params.s_v_max_online 0.9 1])
yticks([-sim_params.a_max(2) 0 sim_params.a_max(2)])
set(gca, 'YTickLabel', {'-a_{max}','0','a_{max}'},'FontSize',16);
xlim([min([res{1}.s res{2}.s res{3}.s]) 1])
xlabel('$s$', 'Interpreter','latex','Fontsize',30)
ylim([min([res{2}.ref_acc(1,:) res{2}.ref_acc(2,:)]) max([res{2}.ref_acc(1,:) res{2}.ref_acc(2,:)])])
ylabel('$\ddot{y}_2$', 'Interpreter','latex','Fontsize',30)
set(gca,'XDir','reverse')

% Path
figure('Name','Path')
clf
tmp = 200;
plot(res{1}.ref_pos(1,1:tmp:end),res{1}.ref_pos(2,1:tmp:end),'g*','LineWidth',8), hold on
plot(res{1}.sys_pos(1,:),res{1}.sys_pos(2,:),'b-','LineWidth',2.5)
plot(res{2}.sys_pos(1,:),res{2}.sys_pos(2,:),'k--','LineWidth',2.5)
plot(res{3}.sys_pos(1,:),res{3}.sys_pos(2,:),'m-.','LineWidth',2.5)
xlabel('$\xi_{1}$', 'Interpreter','latex','Fontsize',30)
ylabel('$\xi_{2}$', 'Interpreter','latex','Fontsize',30)
xticks([])
yticks([])

% Tau
figure('Name','Tau')
plot(res{1}.s,res{1}.tau,'b','LineWidth',2.5), hold on
plot(res{2}.s,res{2}.tau,'k--','LineWidth',2.5)
plot(res{3}.s,res{3}.tau,'m-.','LineWidth',2.5)
xticks([round(min([res{1}.s res{2}.s res{3}.s]),2) sim_params.s_v_max_online 0.9 1])
yticks([])
set(gca,'FontSize',16)
xlim([min([res{1}.s res{2}.s res{3}.s]) 1])
ylim([0.9*dmp.nominal_tau 1.05*max(res{1}.tau)])
xlabel('$s$', 'Interpreter','latex','Fontsize',30)
ylabel('$\tau$', 'Interpreter','latex','Fontsize',30)
set(gca,'XDir','reverse')


% s
figure('Name','s')
plot(res{1}.t,res{1}.s,'b','LineWidth',2.5), hold on
plot(res{2}.t,res{2}.s,'k--','LineWidth',2.5)
plot(res{3}.t,res{3}.s,'m-.','LineWidth',2.5)
yticks([round(min([res{1}.s res{2}.s res{3}.s]),2) sim_params.s_v_max_online 0.9 1])
set(gca,'FontSize',20)
xlim([0 1.05*max([res{1}.t(end) res{2}.t(end) res{3}.t(end)])])
ylim([0.9*res{1}.s(end) 1.05])
xlabel('$t$', 'Interpreter','latex','Fontsize',40)
ylabel('$s$', 'Interpreter','latex','Fontsize',40)

