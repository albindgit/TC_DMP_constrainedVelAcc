clearvars
clc
close all 

% Create demonstration trajectory
load('trajectories/g')
% load('trajectories/omega')
% load('trajectories/z')
% load('trajectories/s')
% load('trajectories/psi')

% Nominal trajectory functions
dmp_params.D = 20;
dmp_params.K = dmp_params.D^2/4;
dmp_params.n_kernel = 100;
dmp_params.alpha_s = 1;
dmp = DMP(dmp_params,demo_traj);

% Simulation setup
sim_params.dt = 1/1000;
sim_params.T_max = 100;
sim_params.kv = 10;
sim_params.kp = sim_params.kv^2/4;

nominalTraj = dmp.rollout(sim_params.dt);
sim_params.a_max = 0.5*[max(abs(nominalTraj.acc(1,:))); max(abs(nominalTraj.acc(2,:)))];
sim_params.v_max = inf*[1 1]';
sim_params.s_v_max_online = 0.65;
sim_params.v_max_online = [inf;0.15];
% sim_params.v_max_online = inf*[1 1]';

% Scaling parameters
tc_params.nominal_tau = dmp.nominal_tau;
tc_params.eps = 1e-3;
tc_params.gamma_nominal = 1;
tc_params.gamma_a = 0.5;
tc_params.a_max = sim_params.a_max; 
tc_params.v_max = sim_params.v_max;

% Create temporal coupling objects
tc = {};
tc{1} = TemporalCoupling(tc_params);
tc{2} = NoTemporalCoupling(tc_params);
tc{3} = TemporalCouplingNoPotentials(tc_params);

% Simulate
nSim = length(tc);
res = cell(length(tc),1);
for d = 1:nSim
    disp(['Running simulation ' num2str(d) '/' num2str(length(tc)) '...']);
    res{d} = run_simulation(dmp,tc{d},sim_params);   
end

% Plot result
plot_result
