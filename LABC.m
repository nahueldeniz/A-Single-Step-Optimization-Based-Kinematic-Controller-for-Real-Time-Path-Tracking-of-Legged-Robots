classdef LABC < handle
    
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
        u                       % Controls
        ULUBounds
        u_lb
        u_ub
    end
    
    methods
        function obj = LABC(nx,nu,x0bar,Ts,Uconstraints)
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
            obj.u           = zeros(obj.nx,1);
            obj.ULUBounds   = Uconstraints;
            %
            if(~isempty(obj.ULUBounds))
                obj.u_lb   = obj.ULUBounds(:,1);
                obj.u_ub   = obj.ULUBounds(:,2);
            else
                obj.u_lb   = -inf(4);
                obj.u_ub   = inf(4);    
            end
        end
        
        function set_x0bar(obj, x0bar)
           obj.x0bar = x0bar; 
        end

        function set_gains(obj, k, K1, K2)
           obj.k    = k;
           obj.K1   = K1;
           obj.K2   = K2;
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
            obj.U1_kp1 = obj.U1_kp1 + obj.Ts * obj.e_k;
            % Update U2
            obj.U2_kp1 = obj.U2_kp1 + obj.Ts * obj.U1_kp1;
            % Update D2_k
            obj.D2_k = obj.x_rp1 - obj.k.*obj.e_k + obj.K1.*obj.U1_kp1 + obj.K2.*obj.U2_kp1 - obj.x_k;
            % Compute controls
            obj.u = [   obj.D2_k(4)/obj.Ts
                        (obj.D2_k(1)/obj.Ts) * cos(obj.x_k(4)) + (obj.D2_k(2)/obj.Ts) * sin(obj.x_k(4))
                        (obj.D2_k(1)/obj.Ts) * sin(obj.x_k(4)) - (obj.D2_k(2)/obj.Ts) * cos(obj.x_k(4))
                        obj.D2_k(3)/obj.Ts ];
            % Saturate controls
            for i=1:4
                if sign(obj.u(i)) > 0
                    obj.u(i) = min([obj.u(i),obj.u_ub(i)]);
                elseif sign(obj.u(i)) < 0
                    obj.u(i) = max([obj.u(i),obj.u_lb(i)]);
                end
            end
        end                        

    end
    
end

