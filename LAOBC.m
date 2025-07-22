classdef LAOBC < handle
    
    properties (GetAccess = public, SetAccess = private)
        nx
        nu
        Ts                      % Samplig time (if Ts=0 => discrete time model)
        x0bar                   % Parameter: arrival cost
        x_k                     % State estimated at time k
        x_r
        x_rp1
        U1_kp1
        U2_kp1
        D2_k
        e_k
        k
        K1
        K2
        l1
        l2
        u                       % Controls
        ULUBounds
        u_lb
        u_ub
        dULUBounds
        du_lb
        du_ub
        W
        optVar_lb
        optVar_ub
        nlp_constraints_lb
        nlp_constraints_ub
        solver
        optInitCondition
        current_parameters
        w
        vehicle
    end
    
    methods
        function obj = LAOBC(nx,nu,x0bar,Ts,W,Uconstraints,dUconstraints,vehicle)
            %
            obj.nx          = nx;
            obj.nu          = nu;
            obj.Ts          = Ts;            
            obj.x0bar       = x0bar;
            obj.x_rp1       = obj.x0bar;
            obj.U1_kp1      = zeros(obj.nx,1);
            obj.U2_kp1      = zeros(obj.nx,1);
            obj.D2_k        = zeros(obj.nx,1);
            obj.e_k         = zeros(obj.nx,1);
            obj.u           = zeros(obj.nu,1);
            obj.W           = casadi.DM(W);
            obj.ULUBounds   = Uconstraints;
            obj.dULUBounds  = dUconstraints;
            obj.vehicle     = vehicle;
            obj.l1          = 1;
            obj.l2          = 1;
            %
            if(~isempty(obj.ULUBounds))
                obj.u_lb   = obj.ULUBounds(:,1);
                obj.u_ub   = obj.ULUBounds(:,2);
            else
                obj.u_lb   = -casadi.DM.inf(obj.nu);
                obj.u_ub   = casadi.DM.inf(obj.nu);    
            end
            %
            if(~isempty(obj.dULUBounds))
                obj.du_lb  = obj.dULUBounds(:,1);
                obj.du_ub  = obj.dULUBounds(:,2);
            else
                obj.du_lb  = -casadi.DM.inf(obj.nu);
                obj.du_ub  = casadi.DM.inf(obj.nu);    
            end
            %            
            W                       = casadi.MX.sym('W',obj.nx, obj.nx);
            w                       = casadi.MX.sym('w',obj.nx);
            u                       = casadi.MX.sym('u',obj.nu);
            u_past                  = casadi.MX.sym('u',obj.nu);
            b                       = casadi.MX.sym('b',obj.nx);
            theta                   = casadi.MX.sym('theta',1);
            %
            u_constraints           = [{u(1)-b(1)-w(4)}, {u(2)-b(2)-w(1)*cos(theta)-w(2)*sin(theta)}, {u(3)-b(3)+w(2)*cos(theta)-w(1)*sin(theta)}, {u(4)-b(4)-w(3)}, {u-u_past}];
            u_constraints_lb        = [zeros(obj.nu,1); obj.du_lb];
            u_constraints_ub        = [zeros(obj.nu,1); obj.du_ub];
            %
            nlp_constraints         = u_constraints(:)';
            obj.nlp_constraints_lb  = [u_constraints_lb(:)];
            obj.nlp_constraints_ub  = [u_constraints_ub(:)];
            %
            J = mtimes(mtimes(w', W), w);
            %
            optVar                  = [w(:)', u(:)'];
            obj.optVar_lb           = [-casadi.DM.inf(obj.nx); obj.u_lb];
            obj.optVar_ub           = [casadi.DM.inf(obj.nx); obj.u_ub];
            %
            optParam                = {vec(W), b, theta, u_past};
            % Create an NLP solver ________________________________________
            problem     = struct('f', J, ...
                                 'x', vertcat(optVar{:}), ...
                                 'g', vertcat(nlp_constraints{:}), ...
                                 'p', vertcat(optParam{:}));
            % listo of parameters to be tuned: https://casadi.sourceforge.net/v1.4.0beta/api/html/dd/df1/classCasADi_1_1IpoptSolver.html
            nlpoptions                                  = struct;
            nlpoptions.ipopt.max_iter                   = 2000;  %2000
            nlpoptions.ipopt.print_level                = 3;
            nlpoptions.print_time                       = 0;
            nlpoptions.ipopt.acceptable_tol             = 1e-8;
            nlpoptions.ipopt.acceptable_obj_change_tol  = 1e-6;
%             nlpoptions.ipopt.max_cpu_time               = 1e-3;
            obj.solver = casadi.nlpsol('solver', 'ipopt', problem, nlpoptions);   
            %
%             qpoptions                           = struct;
%             qpoptions.ad_weight                 = 0;
%             qpoptions.ad_weight_sp              = 0;
%             qpoptions.compiler                  = 'no';
%             qpoptions.gather_stats              = 0;
%             qpoptions.jac_penalty               = -1;
%             qpoptions.jit                       = 1;
%             qpoptions.verbose                   = 0;
%             obj.solver                          = casadi.qpsol('solver', 'qpoases', problem, qpoptions);
        end
        
        function set_x0bar(obj, x0bar)
           obj.x0bar = x0bar; 
        end

        function set_gains(obj, k, K1, K2)
           obj.k    = k;
           obj.K1   = K1;
           obj.K2   = K2;
        end

        function set_forgetting_factors(obj,l1,l2)
            obj.l1 = l1;
            obj.l2 = l2;
        end
                               
        function updateState(obj,x)
            obj.x_k = x;
        end

        function updateReference(obj,x_rp1)
            obj.x_r     = obj.x_rp1;
            obj.x_rp1   = x_rp1;    
        end
                            
        function solve(obj)
            % Update errors
            obj.e_k = obj.x_r - obj.x_k;
            % Update U1
            obj.U1_kp1 = obj.U1_kp1 * obj.l1 + obj.Ts * obj.e_k;
            % Update U2
            obj.U2_kp1 = obj.U2_kp1 * obj.l2 + obj.Ts * obj.U1_kp1;
            % Update D2_k
            obj.D2_k = obj.x_rp1 - obj.k.*obj.e_k + obj.K1.*obj.U1_kp1 + obj.K2.*obj.U2_kp1 - obj.x_k;
            % Compute controls
%             {vec(W), b, theta, u_past};
            obj.current_parameters = vertcat(vec(obj.W),...
             [obj.D2_k(4)/obj.Ts; (obj.D2_k(1)/obj.Ts)*cos(obj.x_k(4))+(obj.D2_k(2)/obj.Ts)*sin(obj.x_k(4)); (obj.D2_k(1)/obj.Ts)*sin(obj.x_k(4))-(obj.D2_k(2)/obj.Ts)*cos(obj.x_k(4)); obj.D2_k(3)/obj.Ts],...
             obj.x_k(4), obj.u);
            
                        
            sol = obj.solver('x0' ,obj.optInitCondition, ...
                             'lbx',obj.optVar_lb, ...
                             'ubx',obj.optVar_ub, ...
                             'lbg',obj.nlp_constraints_lb, ...
                             'ubg',obj.nlp_constraints_ub, ...
                             'p'  ,obj.current_parameters);
            
            obj.optInitCondition = sol.x;
            x_opt                = full(sol.x);
            %
            obj.w = x_opt(1:4)';
            obj.u = x_opt(5:end)';
        end                        

    end
    
end

