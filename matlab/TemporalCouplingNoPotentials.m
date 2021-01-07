classdef TemporalCouplingNoPotentials
    properties
        % Settings
        v_max
        a_max
        nominal_tau
        gamma_nominal
    end
    
    methods
        function obj = TemporalCouplingNoPotentials(params)
            obj.nominal_tau = params.nominal_tau;
            obj.v_max = params.v_max;
            obj.a_max = params.a_max;
            obj.gamma_nominal = params.gamma_nominal;
        end
      
        function tau_dot_val = tau_dot(obj,dmp,dt)
            % DMP prediction step
            dmp_next = dmp.step(0,dt);
            
            % Formulate matrices
            [A,B,C,D] = obj.tc_mtrcs(dmp);
            [A_next,~,C_next,~] = obj.tc_mtrcs(dmp_next);

            % Compute bounds
            tau_dot_min_a = obj.tau_dot_min_a(A,B,C,dmp.tau);
            tau_dot_max_a = obj.tau_dot_max_a(A,B,C,dmp.tau);
            tau_dot_min_v = obj.tau_dot_min_v(A_next,D,dmp.tau,dt);
            tau_dot_min_f = obj.tau_dot_min_f(A_next,B,C_next,dmp.tau,dt);
            tau_dot_min_nominal = obj.tau_dot_min_nominal(dmp.tau,dt);
            
            % Base update law
            tau_dot_val = obj.gamma_nominal*(obj.nominal_tau-dmp.tau);
            
            % Saturate
            tau_dot_min = max([tau_dot_min_a,tau_dot_min_v,tau_dot_min_f,tau_dot_min_nominal]);
            if tau_dot_val > tau_dot_max_a
                tau_dot_val = tau_dot_max_a;
            end
            if tau_dot_val < tau_dot_min
                tau_dot_val = tau_dot_min;
            end
        end
        
        function [A,B,C,D] = tc_mtrcs(obj,dmp)
            A = [-dmp.z(); 
                dmp.z()];
            B = [-obj.a_max;
                 -obj.a_max];
            C = [dmp.h();
                 -dmp.h()];
            D = [-obj.v_max;
                 -obj.v_max];
        end
        
        function minVal = tau_dot_min_a(obj,A,B,C,tau)
            i = A < 0;
            minVal = max(-(B(i)*tau^2+C(i))./A(i));
        end
        
        function maxVal = tau_dot_max_a(obj,A,B,C,tau)
            i = A > 0;
            maxVal = min(-(B(i)*tau^2+C(i))./A(i));
        end
        
        function minVal = tau_dot_min_f(obj,A,B,C,tau,dt)
            Ais = A(A<0);
            Ajs = A(A>0);
            Bis = B(A<0);
            Bjs = B(A>0);
            Cis = C(A<0);
            Cjs = C(A>0);
            tauNextMin = -inf;
            for i=1:length(Ais)
                for j = 1:length(Ajs)
                    num = Cis(i)*abs(Ajs(j))+Cjs(j)*abs(Ais(i));
                    den = abs(Bis(i)*Ajs(j))+abs(Bjs(j)*Ais(i));
                    if num > 0
                        if sqrt(num/den) > tauNextMin
                            tauNextMin = sqrt(num/den);
                        end
                    end
                end
            end
            minVal = (tauNextMin-tau)/dt;
        end
        
        function minVal = tau_dot_min_v(obj,A,D,tau,dt)
            tauNextMax = max(-A./D);
            minVal = (tauNextMax-tau)/dt;
        end
        
        function minVal = tau_dot_min_nominal(obj,tau,dt)
            minVal = (obj.nominal_tau-tau)/dt;
        end

    end
end

