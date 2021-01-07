function res = run_simulation(dmp,tmp_couple,params)

% Parameters
dt = params.dt;
T_max = params.T_max;
kp = params.kp;
kv = params.kv;
a_max = params.a_max;
v_max = params.v_max;
s_v_max_online = params.s_v_max_online;
v_max_online = params.v_max_online;

% Initialize
dmp = dmp.init();
t = 0;
k = 1;
sys.pos = dmp.ref_pos();
sys.vel = 0*sys.pos;
sys.acc = 0*sys.pos;

while t < T_max
    % Store values
    res.t(k) = t;
    res.s(k) = dmp.s();
    res.ref_pos(:,k) = dmp.ref_pos();
    res.ref_vel(:,k) = dmp.ref_vel();
    res.ref_acc(:,k) = dmp.ref_acc();
    res.sys_pos(:,k) = sys.pos;
    res.sys_vel(:,k) = sys.vel;
    res.sys_acc(:,k) = sys.acc;
    res.tau(k) = dmp.tau;

    % Final position convergence condition
    if norm(dmp.ref_pos()-dmp.g) < 1e-2
        break;
    end

    % Perform one time step
    k = k + 1;
    t = t + dt;
    
    % Add velocity limit online
    if dmp.s() < s_v_max_online
        v_max = v_max_online;
        tmp_couple.v_max= v_max_online;
    end

    % Step trajectory generator
    tau_dot = tmp_couple.tau_dot(dmp,dt);
    dmp = dmp.step(tau_dot,dt); 

    % Controller
    sys_acc_ctrl = dmp.ref_acc() + kv*(dmp.ref_vel()-sys.vel) + kp*(dmp.ref_pos()-sys.pos);
    % Saturate acceleration
    sys_acc_ctrl = min(max(sys_acc_ctrl,-a_max),a_max);
    % Velocity integration
    vel_prev = sys.vel;
    sys.vel = sys.vel + sys_acc_ctrl*dt;
    % Saturate velocity
    sys.vel = min(max(sys.vel,-v_max),v_max);
    sys.acc = (sys.vel-vel_prev)/dt;
    % Position integration
    sys.pos = sys.pos + sys.vel*dt;

end