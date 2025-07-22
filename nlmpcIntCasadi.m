classdef nlmpcIntCasadi < handle
    
    properties (GetAccess = public, SetAccess = private)
        Nc                      % Horinzon length
        F                       % Evolution of the system (x_{k+1})
        nx                      % Dimension of the vector state
        nu                      % Dimension of the input
        Ts                      % Samplig time (if Ts=0 => discrete time model)
        Q                       % Matrix weight of process noise
        R                       % Matrix weight of residual estimation
        Qf                      % Arrival cost weight matrix
        xf_lb
        xf_ub
        x_lb
        x_ub
        u_lb
        u_ub
        du_lb
        du_ub
        x0bar                   % Parameter: arrival cost
        xref                     % Parameter: reence to follow
        uref                    % Set point for the control action
        Xtraj                   % State trajectory
        Utraj                   % Inputs calculateds        
        u_k
        X0                      % Initial condition for solver
        z                       % integral mode: accumulated error
        offsetZ
        Ztraj
        
        Xk
        Uk
        u_last

        optVar_lb
        optVar_ub
        nlp_constraints_lb
        nlp_constraints_ub

        optInitCondition
        
        control_constraints
        control_constraints_lb
        control_constraints_ub
                
        current_parameters
        
        solver
    end
    
    methods
        function obj = nlmpcIntCasadi(Nc,F,mtxs,dims,constraints,Ts)
            obj.Nc          = Nc;            
            obj.nx          = dims.nq;
            obj.nu          = dims.nu;               % valid options: 'x', 'y'
            obj.Q           = casadi.DM(mtxs.Q);
            obj.Qf          = casadi.DM(mtxs.Qf);
            obj.R           = casadi.DM(mtxs.R);
            obj.Ts          = Ts;
            obj.Utraj       = zeros(obj.nu,obj.Nc);
            obj.F           = F;
            obj.u_last      = zeros(obj.nu,1);
            obj.optInitCondition = [];
            obj.xref        = zeros(obj.nx,1);
            obj.z           = 0;
            obj.offsetZ     = obj.nx*obj.Nc+obj.nu*obj.Nc;
            obj.Ztraj       = zeros(1,obj.Nc);
            
            if(~isempty(constraints.Xf))
                obj.xf_lb   = constraints.Xf(:,1);
                obj.xf_ub   = constraints.Xf(:,2);
            else
                obj.xf_lb   = -casadi.DM.inf(obj.nx);
                obj.xf_ub   = casadi.DM.inf(obj.nx);
            end
            
            if(~isempty(constraints.X))
                obj.x_lb    = constraints.X(:,1);
                obj.x_ub    = constraints.X(:,2);
            else
                obj.x_lb    = -casadi.DM.inf(obj.nx);
                obj.x_ub    = casadi.DM.inf(obj.nx);
            end
            
            if(~isempty(constraints.U))
                obj.u_lb    = constraints.U(:,1);
                obj.u_ub    = constraints.U(:,2);
            else
                obj.u_lb    = -casadi.DM.inf(obj.nu);
                obj.u_ub    = casadi.DM.inf(obj.nu);    
            end
            
            if(~isempty(constraints.dU))
                obj.du_lb   = constraints.dU(:,1);
                obj.du_ub   = constraints.dU(:,2);
            else
                obj.du_lb   = -casadi.DM.inf(obj.nu);
                obj.du_ub   = casadi.DM.inf(obj.nu);
            end
            
            % Optimisation variables
            Xk              = {};%casadi.MX.sym('Xi',obj.nx,obj.Nc);
            Zk              = {};%integral mode
            Uk              = {};%casadi.MX.sym('Ui',obj.nu,obj.Nc);            
            % Parameters
            U_last          = casadi.MX.sym('lastU',obj.nu);
            X0              = casadi.MX.sym('X0',obj.nx);
            Z0              = casadi.MX.sym('Z0',1);
            Xref            = casadi.MX.sym('xref',obj.nx);
            Q               = casadi.MX.sym('Q',obj.nx,obj.nx);
            Qf              = casadi.MX.sym('Qf',obj.nx,obj.nx);
            R               = casadi.MX.sym('R',obj.nu,obj.nu);                        
            %            
            for i=1:obj.Nc
                Xk  = [Xk {casadi.MX.sym(['X_{k+' num2str(i),'}'], obj.nx)}];
                Zk  = [Zk {casadi.MX.sym(['Z_{k+' num2str(i),'}'], 1)}];
                Uk  = [Uk {casadi.MX.sym(['U_{k+' num2str(i),'}'], obj.nu)}];
            end
            %
            optVar          = {Xk{:},Uk{:},Zk{:}};
            optParam        = {X0,Xref,vec(Q),vec(Qf),vec(R),U_last,Z0};
            %
            if obj.Nc==1
                obj.optVar_lb   = [ obj.xf_lb; obj.u_lb; -inf];
                obj.optVar_ub   = [ obj.xf_ub; obj.u_ub; inf];
            else
                obj.optVar_lb   = [ repmat(obj.x_lb,obj.Nc-1,1); obj.xf_lb; repmat(obj.u_lb,obj.Nc,1); -inf(obj.Nc,1)];
                obj.optVar_ub   = [ repmat(obj.x_ub,obj.Nc-1,1); obj.xf_ub; repmat(obj.u_ub,obj.Nc,1); inf(obj.Nc,1)];
            end
            %
            state_constraints       = {};
            state_constraints_lb    = casadi.DM.zeros(obj.nx*(obj.Nc));
            state_constraints_ub    = casadi.DM.zeros(obj.nx*(obj.Nc));
            %
            intMode_constraints     = {};
            intMode_constraints_lb  = casadi.DM.zeros(obj.Nc);
            intMode_constraints_ub  = casadi.DM.zeros(obj.Nc);
            %
            deltaU_constraints      = {Uk{1} - U_last};
            deltaU_constraints_lb   = repmat(obj.du_lb,obj.Nc,1);
            deltaU_constraints_ub   = repmat(obj.du_ub,obj.Nc,1);
            %
            J = 0;%casadi.DM(0);
            for i=1:obj.Nc
                if i==1
                    Fk = obj.F(X0,Uk{i});
                    z  = Z0 + obj.Ts*( cos(Xref(4)) * (Xk{i}(1)-Xref(1)) - sin(Xref(4)) * (Xk{i}(2)-Xref(2)) );
                else
                    Fk = obj.F(Xk{i-1},Uk{i});
                    z  = Zk{i-1} + obj.Ts*( cos(Xref(4)) * (Xk{i}(1)-Xref(1)) - sin(Xref(4)) * (Xk{i}(2)-Xref(2)) );
                end                
                %
                state_constraints    = [state_constraints {Xk{i} - Fk}];
                intMode_constraints  = [intMode_constraints {Zk{i} - z}];
                % Rate of change in control actions constraints -----------
                if i>1
                    deltaU_constraints = [deltaU_constraints {Uk{i} - Uk{i-1}}];             
                end
                % Compute cost --------------------------------------------
                if i==obj.Nc
                    J  = J + mtimes(mtimes((Xk{i}-Xref)',Qf),(Xk{i}-Xref)) + mtimes(mtimes((Uk{i})',R),(Uk{i})) + mtimes(Zk{i},Zk{i});
                else
                    J  = J + mtimes(mtimes((Xk{i}-Xref)',Q),(Xk{i}-Xref)) + mtimes(mtimes((Uk{i})',R),(Uk{i})) + mtimes(Zk{i},Zk{i});
                end
            end                
            % All states constraints, including the multiple shooting _____
            nlp_constraints         = {state_constraints{:} deltaU_constraints{:} intMode_constraints{:}};
            obj.nlp_constraints_lb  = [state_constraints_lb(:); deltaU_constraints_lb(:); intMode_constraints_lb(:)];
            obj.nlp_constraints_ub  = [state_constraints_ub(:); deltaU_constraints_ub(:); intMode_constraints_ub(:)];            
            % Create an NLP solver ________________________________________
            problem     = struct('f', J, ...
                                  'x', vertcat(optVar{:}), ...
                                  'g', vertcat(nlp_constraints{:}), ...
                                  'p', vertcat(optParam{:}));
            % ipopt solver
            nlpoptions                          = struct;
            nlpoptions.ipopt.max_iter           = 2000;
            nlpoptions.ipopt.print_level        = 0;%0,3
            nlpoptions.print_time               = 0;
            nlpoptions.ipopt.acceptable_tol     = 1e-8;
            nlpoptions.ipopt.acceptable_obj_change_tol = 1e-6;
            obj.solver                          = casadi.nlpsol('solver', 'ipopt', problem, nlpoptions);
            % qpoases solver
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
        
        function setx0(obj,x)
            obj.x0bar = x;
        end

        function setRef(obj,xref)
            obj.xref = xref;
        end

        function solve(obj)
            %Accumulate the error
            obj.z = obj.z + obj.Ts * ( cos(obj.xref(4))*(obj.x0bar(1)-obj.xref(1)) - sin(obj.xref(4))*(obj.x0bar(2)-obj.xref(2)) );
            % optParam = {X0,vec(Q),vec(Qf),vec(R),U_last};   
            obj.current_parameters = vertcat(obj.x0bar, obj.xref, vec(obj.Q), vec(obj.Qf), vec(obj.R), obj.u_last, obj.z);
                        
            sol = obj.solver('x0' ,obj.optInitCondition, ...
                             'lbx',obj.optVar_lb, ...
                             'ubx',obj.optVar_ub, ...
                             'lbg',obj.nlp_constraints_lb, ...
                             'ubg',obj.nlp_constraints_ub, ...
                             'p'  ,obj.current_parameters);
            
            obj.optInitCondition = sol.x;
            x_opt                = full(sol.x);
            %
            obj.Xtraj   = reshape(x_opt(1:obj.Nc*obj.nx),obj.nx,obj.Nc);
            obj.Utraj   = reshape(x_opt(obj.Nc*obj.nx+1:obj.offsetZ),obj.nu,obj.Nc);
            obj.Ztraj   = reshape(x_opt(obj.offsetZ+1:end),1,obj.Nc);
            %
            obj.u_k     = obj.Utraj(:,1);
            obj.u_last  = obj.u_k;
        end                        

    end
    
end

