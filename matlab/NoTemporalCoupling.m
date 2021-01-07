classdef NoTemporalCoupling
    properties
       v_max
       a_max
    end
    
    methods
        
        function obj = NoTemporalCoupling(params)
            obj.v_max = params.v_max;
            obj.a_max = params.a_max;
        end
        
        function tau_dot_val = tau_dot(obj,dmp,dt)
            tau_dot_val = 0;
        end
    end
end

