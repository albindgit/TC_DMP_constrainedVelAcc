classdef DMP
    properties
        % State
        x % [z; y; s]
        tau
        tauDot
        % Settings
        K
        D
        alpha_s
        n_kernel
        n
        g
        y0
        nominal_tau
        x0
        mu
        std_dev
        w
        demo_traj
    end
    
    methods
        function obj = DMP(dmp_params,demo_traj)
            obj.K = dmp_params.K;
            obj.D = dmp_params.D;
            obj.alpha_s = dmp_params.alpha_s;
            obj.n_kernel = dmp_params.n_kernel;
            obj.demo_traj = demo_traj;
            obj = obj.fit();
        end
        
        function obj = init(obj)
            obj.x = obj.x0;
            obj.tau = obj.nominal_tau;
            obj.tauDot = 0;
        end
        
        function obj = step(obj,tauDot,dt)
            xDot = [obj.h();obj.z();-obj.s()]/obj.tau;
            obj.x = obj.x + xDot*dt;
            obj.tau = obj.tau + tauDot*dt;
            obj.tauDot = tauDot;
        end
        
        function ref_pos_val = ref_pos(obj)
            ref_pos_val = obj.y();
        end
        
        function ref_pos_val = ref_vel(obj)
            ref_pos_val = obj.z()/obj.tau;
        end
        
        function ref_pos_val = ref_acc(obj)
            ref_pos_val = (obj.h() - obj.tauDot*obj.z()) / obj.tau^2;
        end
                
        function yVal = y(obj)
            yVal = obj.x(obj.n+1:2*obj.n);
        end
        
        function zVal = z(obj)
            zVal = obj.x(1:obj.n);
        end
        
        function sVal = s(obj)
            sVal = obj.x(end);
        end
        
        function hVal = h(obj)
            psi = exp(-1./(2*obj.std_dev.^2).*(obj.s()-obj.mu).^2);
            hVal = obj.K*(obj.g-obj.y()) - obj.D*obj.z() + obj.w*psi' ./ max(sum(psi),1e-8) .* (obj.g-obj.y0)*obj.s();
        end
        
        function traj = rollout(obj,dt)
            dmp = obj.init();
            traj.t = 0;
            traj.pos = dmp.y();
            traj.vel = dmp.z()/dmp.nominal_tau;
            traj.acc = dmp.h()/dmp.nominal_tau^2;
            traj.s = dmp.s();
            while norm(abs(dmp.y()-dmp.g)) > 1e-2
                dmp = dmp.step(0,dt);
                traj.t = [traj.t traj.t(end)+dt];
                traj.pos = [traj.pos dmp.ref_pos()];
                traj.vel = [traj.vel dmp.ref_vel()];
                traj.acc = [traj.acc dmp.ref_acc()];
                traj.s = [traj.s dmp.s()];
            end
        end
       
       function obj = fit(obj)

            psi = @(s,c,d) exp(-1./(2*d.^2).*(s-c).^2); % Basis function
            obj.n = size(obj.demo_traj.pos,1);
            obj.y0 = obj.demo_traj.pos(:,1);
            obj.g = obj.demo_traj.pos(:,end);
            obj.x0 = [zeros(obj.n,1); obj.y0; 1];
            obj.nominal_tau = obj.demo_traj.t(end);

            lastKernel = 1*obj.nominal_tau; % Time of last kernel center
            ct = (0:lastKernel/(obj.n_kernel-1):lastKernel); % Time means of basis functions
            obj.mu = exp(-obj.alpha_s/obj.nominal_tau*ct); % Phase means of basis functions
            obj.std_dev  = abs((diff(obj.mu)));
            obj.std_dev  = [obj.std_dev obj.std_dev(end)]; % Standard deviation of basis functions

            svec = exp(-obj.alpha_s/obj.nominal_tau*obj.demo_traj.t);
            obj.w = zeros(obj.n,obj.n_kernel);
            for varNr = 1:obj.n
                if abs(obj.g(varNr)-obj.y0(varNr)) < 0.001
                    continue;
                end
                fGain = svec*(obj.g(varNr)-obj.y0(varNr));
                f_target = obj.nominal_tau^2*obj.demo_traj.acc(varNr,:) - obj.K*(obj.g(varNr)-obj.demo_traj.pos(varNr,:)) + obj.D*obj.nominal_tau*obj.demo_traj.vel(varNr,:);
                for i = 1:obj.n_kernel
                    Gamma_i = diag(psi(svec,obj.mu(i),obj.std_dev(i)));
                    if fGain*Gamma_i*fGain' < 1e-8
                        obj.w(varNr,i) = 0;
                    else
                        obj.w(varNr,i) = fGain*Gamma_i*f_target'/(fGain*Gamma_i*fGain');
                    end
                end
            end
       end
        
    end
end

