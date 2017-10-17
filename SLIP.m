classdef SLIP < handle
    %SLIP Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        s
        v
        nQ = 6;
        nU = 2;
        start_state
        end_state
        pos_idx
        vel_idx
        u_idx
        mu = 2.0;
        modes = [];
    end
    
    methods
        
        %Load the SLIP library and initialize the model and visual
        function obj = SLIP(enableVisuals)
            if libisloaded('libslip')
                unloadlibrary('libslip')
            end
            loadlibrary('libslip','src/slip.h');
            
            obj.s = calllib('libslip','init');
            if(enableVisuals == 1)
                obj.v = calllib('libslip', 'vis_init');
            end
           
        end
        
        function [left_foot, right_foot] = get_foot_pos(obj, q, qd)
            
            state = obj.blank_state();
            state.q = q;
            state.qd = qd;
            state = libpointer('state_t', state);
            calllib('libslip', 'forward', obj.s, state);
            temp = state.Value;
            cpos = temp.cpos;
            cvel = temp.cvel;
            left_foot = [cpos(1:2);cvel(1:2)];
            right_foot = [cpos(3:4);cvel(3:4)];
            
        end
        function add_mode(obj, num_pts, time, mode_type)
            offset = 0;
            if (~isempty(obj.modes))
                offset = obj.modes(end).idx(end);
            end
            mode = TrajMode(obj.nQ, obj.nU, obj.nC, obj.DOF, num_pts, time/num_pts, offset, mode_type);
            obj.modes = [obj.modes; mode]; 
        end
        
        function vec = initialize_vec(obj)
           totalN = 0;
           for i = 1:1:length(obj.modes)
               totalN = totalN + obj.modes(i).N;
           end
           
           s0 = obj.modes(1).start_state;
           sN = obj.modes(end).end_state;
           delq = sN.q - s0.q;
           delqd = sN.qd - s0.qd;
           delu = sN.u - s0.u;
           delf = sN.f - s0.f;
           
           new_start = obj.blank_state();
           new_end = obj.end_state();
           
           vec = [];
           prevN = 0;
           for i = 1:1:length(obj.modes)
               new_start.q = s0.q + (prevN/totalN)*delq;
               new_end.q = new_start.q + (obj.modes(i).N/totalN)*delq;
               new_start.qd = s0.qd + (prevN/totalN)*delqd;
               new_end.qd = new_start.qd + (obj.modes(i).N/totalN)*delqd;
               new_start.u = s0.u + (prevN/totalN)*delu;
               new_end.u = new_start.u + (obj.modes(i).N/totalN)*delu;
               new_start.f = s0.f + (prevN/totalN)*delf;
               new_end.f = new_start.f + (obj.modes(i).N/totalN)*delf;
               prevN = prevN + obj.modes(i).N;
                
               vec = [vec; obj.modes(i).interpolate_vec(new_start, new_end)];
           end
           
        end
        
        
        function [qdd, cpos, cvel, cacc, Jc_left, Jc_right, M] = dynamics(obj, state)
            state = libpointer('state_t', state);
            calllib('libslip', 'forward', obj.s, state);
            temp = state.Value;
            qdd = temp.qdd;
            cpos = temp.cpos;
            cvel = temp.cvel;
            cacc = temp.cacc;
            Jc_left = reshape(temp.Jc(1:obj.DOF*obj.nQ),  obj.nQ, obj.DOF);
            Jc_right = reshape(temp.Jc(obj.DOF*obj.nQ+1:end), obj.nQ, obj.DOF);
            M = reshape(temp.M, obj.nQ, obj.nQ);
        end
        
        function set_start_end(obj, s0, sN, s0idx, sNidx)
           obj.modes(1).set_start_end(s0, sN, s0idx, sNidx); %wont use end
           obj.modes(end).set_start_end(s0, sN, s0idx, sNidx); %wont use start
        end
                
%         function delX = get_start_constraint(obj, pos, vel)
%             delX = [obj.start_state.q' - pos; obj.start_state.qd' - vel];
%             delX = delX([1,2,3,4,7,10:end],1);
%         end
%         
%         function delX = get_end_constraint(obj, pos, vel)
%             delX = [obj.end_state.q' - pos; obj.end_state.qd' - vel];
%             delX = delX([1,2,3,4,7,10:end],1);
%         end
        
        function [lb, ub] = get_state_limits(obj)
            bounds.lb = zeros(1, obj.nQ);
            bounds.ub = zeros(1,obj.nQ);
            bounds = libpointer('pos_limits_t', bounds);
            calllib('libslip', 'get_joint_limits', bounds);
            lb = bounds.Value.lb;
            ub = bounds.Value.ub;
        end
        
        function [lb, ub] = get_motor_limits(obj)
            bounds.lb = zeros(1, obj.nU);
            bounds.ub = zeros(1,obj.nU);
            bounds = libpointer('motor_limits_t', bounds);
            calllib('libslip', 'get_motor_limits', bounds);
            lb = bounds.Value.lb;
            ub = bounds.Value.ub;
        end
        
        function state = blank_state(obj)
           state.q = zeros(1, obj.nQ);
           state.qd = state.q;
           state.qdd = state.q;
           state.f = zeros(1, obj.nC*obj.DOF);
           state.u = zeros(1, obj.nU);
           state.Jc = zeros(1, obj.nC*obj.nQ*obj.DOF);
           state.M = zeros(1, obj.nQ*obj.nQ);
           state.cvel = state.f;
           state.cpos = state.f;
        end
        
        function state = get_stationary_state(obj, root_z)
            state = obj.blank_state();
            state.q(2) = root_z;
            state = libpointer('state_t', state);
            
            ret = calllib('libslip', 'get_stationary_init', obj.s, state);
        
            if (ret)
                fprintf('couldnt set root pos');
                %return some kind of error
            end
            
            state = state.Value;
        end
        
        function state = run_forward(obj, s, h)
           s = libpointer('state_t', s);
           calllib('libslip', 'run_forward', obj.s, s, h);
           
           state = s.Value;
        end
        
        function [lb, ub] = get_bounds(obj)
            
            [q_lb, q_ub] = obj.get_state_limits();
            [u_lb, u_ub] = obj.get_motor_limits();
            
            lb = [];
            ub = [];
            
            for i = 1:1:length(obj.modes)
                N = obj.modes(i).N;

                % Lower bound the simulation time at zero seconds, and bound the
                lb = [lb; repmat(q_lb', N, 1); ones(N * obj.nQ, 1) * -5; repmat(u_lb', N, 1)];
                ub = [ub; repmat(q_ub', N, 1); ones(N * obj.nQ, 1) * 5; repmat(u_ub', N, 1)];
            end
        end
        
        function rendered = draw(obj)
            rendered = calllib('libslip', 'vis_draw', obj.v, obj.s);
        end
        
        function close(obj)
            calllib('libslip','vis_close',obj.v)
        end

    end
    
end

