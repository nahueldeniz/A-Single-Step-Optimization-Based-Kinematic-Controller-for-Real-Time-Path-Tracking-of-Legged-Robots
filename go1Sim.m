% #########################################################################
% 
% Estimation and Control for Legged Robots
% Author: Nestor. N. Deniz - 2022
%
% #########################################################################
% TODO:
%
% 
% #########################################################################


function S = go1Sim()
    clear all; clc;
    import casadi.*
    % ---------------------------------------------------------------------
    S = init();
    % Init ROS's things ---------------------------------------------------
    if S.config.dataOnline == true; figure('units','normalized','outerposition',[0 0 1 1]); hold on;
        plot(S.path.coordinates(1,:),S.path.coordinates(2,:),'b','LineWidth',5); grid on; 
%         xlim([-1 10]); ylim([-15 35]); daspect([1 1 1]); 
        dim = [.2 .5 .3 .3];
%         ann = annotation('textbox',dim,'String','','verticalalignment','top','horizontalalignment','left','FitBoxToText','on','Interpreter','Latex','FontSize',30);
    end
    if ~S.config.SIM == true; S = ROS(S); end
    for num_batch = 1:S.config.NUM_BATCHS
        S.data.numBatch = num_batch;
        for num_sims = 1:S.config.NUM_SIMS
            % Set Vels
            S = setVel(S, num_sims);
            %
            S = call_init_functions(S);        
            %
            S.config.iters = 1;
            while (S.config.time(end) < S.config.tf) && ~(S.path.reach_end_mhempc)
                [flag,S] = check_end_condition(S);
                if flag; break; else
                    S = callSimulator(S);                
                    % Perform real-time iteration _____________________________
                    tic;
                    S.exec_time.t_tot   = [S.exec_time.t_tot, [S.exec_time.t_mhe(end); S.exec_time.t_pfa(end); max([S.exec_time.t_mpc(end),S.exec_time.t_labc(end),S.exec_time.t_laobc(end)]); S.exec_time.t_mis(end)]];
                    S.exec_time.t_acum  = sum(S.exec_time.t_tot(:,end));
                    while (((S.exec_time.t_acum + toc) < S.config.Ts) && ~S.config.SIM); end
                    % 
                end          
                % #############################################################
                S.config.time   = [S.config.time, S.config.time(end)+S.config.Ts];
                S.config.iters  = S.config.iters+1;
                a               = [S.data.numBatch, num_sims, S.config.iters]
                % -------------------------------------------------------------
                if (S.config.dataOnline == true) plot_legged(S, S.data.xest(:,end), 1); end
                S.exec_time.t_mis = [S.exec_time.t_mis, toc];
                % -------------------------------------------------------------
            end
            if ~S.config.SIM
                S = write_ctrlGo1(S, 0, 0, 0, 0);
            end
            % -----------------------------------------------------------------
            S.data.performance.xsim{(num_batch-1)*S.config.NUM_SIMS+num_sims}       = S.data.xsim;
            S.data.performance.ysim{(num_batch-1)*S.config.NUM_SIMS+num_sims}       = S.data.ysim_mhempc;
            S.data.performance.xest{(num_batch-1)*S.config.NUM_SIMS+num_sims}       = S.data.xest;
            S.data.performance.compBurden{(num_batch-1)*S.config.NUM_SIMS+num_sims} = S.exec_time.t_tot;
            if (S.config.SIM==true)
                S.data.performance.deviation{(num_batch-1)*S.config.NUM_SIMS+num_sims} = computeDeviation(S);
            end
            S.data.performance.controls{(num_batch-1)*S.config.NUM_SIMS+num_sims}   = S.data.Ctrls;
            S.data.performance.laobc_wsl{(num_batch-1)*S.config.NUM_SIMS+num_sims}  = S.algorithms.LAOBC.w_slack;
        end
    end
end

function S = build_setup(S)
    % SIMULATION PARAMETERS ===============================================
    S.ROS.IMU_ZEROING_VEC2  = 1.26;
    % Solver ______________________________________________________________
    S.config.solver         = 'acado'; % options: 'casadi', 'acado'
    S.config.controller     = 'mpcCasadi'; % 'laobc'
    % Simulation or field experiment (Unitree) ____________________________
    S.config.SIM            = true;
    %
    S.config.verbose        = true;
    S.config.calcMtxConvCoord = true; if ~S.config.calcMtxConvCoord; warning('WARNING!!! The matrix for correcting x-y coordinates is not being computed...'); end
    S.config.t0             = 0;                                        % initial time of sym. [Seg]
    S.config.tf             = 120;%1000;                                     % final time of sym. [Seg]
    S.config.Ts             = 0.05;                                     % sampling period, 0=>discrete-time system
    S.config.N              = [];                                       % number of steps of sym.
    S.config.same_seed      = true;
    S.config.EXPORT         = false; if ~S.config.EXPORT; warning('WARNING!!! MHE and MPC were not compiled...'); end
    S.config.iters          = 0;
    S.config.time           = 0;
    S.config.NUM_SIMS       = 1;
    S.config.NUM_BATCHS     = 1;
    if S.config.NUM_SIMS > 1
        S.config.same_seed = false;
    end
    S.config.updtAC         = false;
    % Definition of some global parameters
    S.config.outputs        = 1:4;
    %
    S.config.Nc             = 10;                                       % Lenght of the control horizon
    S.config.Ne             = 10;                                       % Lenght of the estimation window    
    S.config.Uconstraints   = [-pi/2, pi/2      % w
                                -0.5, 0.5       % v_f
                                -0.5, 0.5       % v_l
                                -0.5, 0.5];     % v_z
    
    S.config.dUconstraints  = S.config.Ts.*[-pi/2, pi/2     % (rad/s2)
                                            -0.25,  0.25    % (m/s2)
                                            -0.25,  0.25    % (m/s2)
                                            -0.25,  0.25];  % (m/s2)
    % Noise amplitudes ____________________________________________________
    S.config.noise          = 'gaussian'; % 'gaussian' or 'uniform'
    S.config.noise_lvl      = 0.*[0.1; 0.1; 0.02; 3.5*pi/180]; % measurement noise amplitude: [x; y; z; theta]
    S.config.procDisturb    = 0.*[0.01; 0.01; 0; 1*pi/180];
    % Reference velocities ________________________________________________
    S.config.w0             = 0;
    S.config.vf             = 0.35;%/S.config.Ts;%0.1;
    S.config.vl             = 0;
    S.config.vz             = 0;    
    S.config.uref           = [S.config.w0; S.config.vf; S.config.vl; S.config.vz];
    S.config.thetaRef       = [];%pi/4;
    S.config.thetaRefFixPoint = [];%[7; 4];
    % Distance to last target for finishing navigation and control ________
    S.config.dist_last_tgt  = 0.25;                                    % Distance in meters to next coordinate    
    % Plot data online during experiments. USeful for debbuging purposes __
    S.config.dataOnline     = true;
    % Save workspace flag _________________________________________________
    S.config.save_workspace = false;
    % Structure for indexing the states and controls ______________________
    S.config.X              = {};
    S.config.U              = {};
    S.config.X.x            = 1;
    S.config.X.y            = 2;
    S.config.X.z            = 3;
    S.config.X.theta        = 4;
    S.config.U.w0           = 1;
    S.config.U.vf           = 2;
    S.config.U.vl           = 3;
    S.config.U.vz           = 4;    
end

function S = setVel(S, num_sims)
    if S.config.NUM_SIMS > 1
        if num_sims>=1 && num_sims<=10
            S.config.vf     = 0.1;
            S.config.uref   = [S.config.w0; S.config.vf; S.config.vl; S.config.vz];
        elseif num_sims>=11 && num_sims<=20
            S.config.vf     = 0.2;
            S.config.uref   = [S.config.w0; S.config.vf; S.config.vl; S.config.vz];
        elseif num_sims>=21 && num_sims<=30
            S.config.vf     = 0.3;
            S.config.uref   = [S.config.w0; S.config.vf; S.config.vl; S.config.vz];
        elseif num_sims>=31 && num_sims<=40
            S.config.vf     = 0.4;
            S.config.uref   = [S.config.w0; S.config.vf; S.config.vl; S.config.vz];
        elseif num_sims>=41 && num_sims<=50
            S.config.vf     = 0.5;
            S.config.uref   = [S.config.w0; S.config.vf; S.config.vl; S.config.vz];
        elseif num_sims>=51 && num_sims<=60
            S.config.vf     = 0.6;
            S.config.uref   = [S.config.w0; S.config.vf; S.config.vl; S.config.vz];
        elseif num_sims>=61 && num_sims<=70
            S.config.vf     = 0.7;
            S.config.uref   = [S.config.w0; S.config.vf; S.config.vl; S.config.vz];
        elseif num_sims>=71 && num_sims<=80
            S.config.vf     = 0.8;
            S.config.uref   = [S.config.w0; S.config.vf; S.config.vl; S.config.vz];
        elseif num_sims>=81 && num_sims<=90
            S.config.vf     = 0.9;
            S.config.uref   = [S.config.w0; S.config.vf; S.config.vl; S.config.vz];
        else
            S.config.vf     = 1;
            S.config.uref   = [S.config.w0; S.config.vf; S.config.vl; S.config.vz];
        end
    end
end

function devPath = computeDeviation(S)
    % #####################################################################
    % Deviation at ime k
    devPath       = [];
    % for k=1:length(S.path.references)
    %     dist_aux    = (S.path.references(:,k)-S.data.xsim(:,k));
    %     devPath     = [devPath, dist_aux];
    % end
end

function S = callSimulator(S)
    % Solve estimation problem ________________________________
    S = call_MHE(S);
    % Path-tracking problem ***********************************        
    S = call_PFA2(S);
    % Solve control problem ___________________________________
    S = callController(S);
    % Apply controls to the Husky _____________________________
    S = callSystem(S);
    % Update measurements and inputs to the MHE _______________
    S = update_measurements(S);
end

function S = callSystem(S)
    if S.config.SIM == true
        S.simulation_input.x = S.data.xsim(:,end);
        states             = integrate_legged(S.simulation_input);
        S.data.xsim = [S.data.xsim, states.value + diag(S.config.procDisturb)*randn(S.system.nq,1)];
    else
        S = write_ctrlGo1(S, S.algorithms.mpc.legged_w0, S.algorithms.mpc.legged_vf, S.algorithms.mpc.legged_vl, S.algorithms.mpc.legged_vz);
    end
end

function S = callController(S)
    if strcmp(S.config.controller,'mpc')
        S = call_MPC(S);
    elseif strcmp(S.config.controller,'mpcCasadi')
        S = call_MPCCasadi(S);
    elseif strcmp(S.config.controller,'mpcIntCasadi')
        S = call_MPCintCasadi(S);
    elseif strcmp(S.config.controller,'labc')
        S = call_LABC(S);
    elseif strcmp(S.config.controller,'laobc')
        S = call_LAOBC(S);
    elseif strcmp(S.config.controller,'labsat')
        S = call_LABC_SAT(S);
    else
        fprintf('No existe el controlador seleccionado!!!\n\n');
        return;
    end
end

function plot_legged(S, qk, scale, clr)
    if nargin ==4
        clrHb = clr;
        clrHr = clr;        
        noRef = true;
    else
        clrHb = 'b';
        clrHr = 'k';        
        noRef = false;
    end
    %
    if ~noRef
        plot(S.path.coordinates(1,:),S.path.coordinates(2,:),'b','LineWidth',2); grid on; hold on;
        xmin = min(S.path.coordinates(1,:));
        xmax = max(S.path.coordinates(1,:));
        ymin = min(S.path.coordinates(2,:));
        ymax = max(S.path.coordinates(2,:));

        margin = 1;

        xlim([xmin-margin, xmax+margin]); ylim([ymin-margin, ymax+margin]); 
        daspect([1 1 1]);
    end    
    % Dimensions of the vehice
    widthH      = 0.42;
    lengthH     = 0.990;
    widthHr     = 0.125;
    lengthHr    = 0.33;
    % Husky's Wheel
    Praux  = [-widthHr/2, -widthHr/2, widthHr/2, widthHr/2; -lengthHr/2, lengthHr/2, lengthHr/2, -lengthHr/2].*scale;    
    %
    Phaux  = [-widthH/2, -widthH/2, widthH/2, widthH/2; -lengthH/2, lengthH/2, lengthH/2, -lengthH/2].*scale;
    Prh    = [];
    Ph     = [];
    % Rotate the Husky and wheels according to their attitude
    theta  = qk(S.config.X.theta);
    R      = [cos(theta-pi/2), -sin(theta-pi/2); sin(theta-pi/2), cos(theta-pi/2)];
    for i=1:4
        Prh = [Prh, R*Praux(:,i)];
        Ph  = [Ph, R*Phaux(:,i)];
    end
    % Husky's coordinates   
    P0c     = [qk(S.config.X.x); qk(S.config.X.y)];
    % Husky's wheels coordinates
    Xrc     = -(widthH/2 + widthHr/2);
    Yrc     = (widthH/2 + widthHr/2);
    %
    Prh1    = Prh + repmat(P0c,1,4) + repmat([cos(theta-pi/2), -sin(theta-pi/2); sin(theta-pi/2), cos(theta-pi/2)] * [-Xrc; -Yrc],1,4);
    Prh1    = [Prh1, Prh1(:,1)];
    Prh2    = Prh + repmat(P0c,1,4) + repmat([cos(theta-pi/2), -sin(theta-pi/2); sin(theta-pi/2), cos(theta-pi/2)] * [-Xrc; Yrc],1,4);
    Prh2    = [Prh2, Prh2(:,1)];
    Prh3    = Prh + repmat(P0c,1,4) + repmat([cos(theta-pi/2), -sin(theta-pi/2); sin(theta-pi/2), cos(theta-pi/2)] * [Xrc; Yrc],1,4);
    Prh3    = [Prh3, Prh3(:,1)];
    Prh4    = Prh + repmat(P0c,1,4) + repmat([cos(theta-pi/2), -sin(theta-pi/2); sin(theta-pi/2), cos(theta-pi/2)] * [Xrc; -Yrc],1,4);
    Prh4    = [Prh4, Prh4(:,1)];
    %
    Phusky  = Ph + repmat(P0c,1,4);
    Phusky  = [Phusky, Phusky(:,1)];
    % plt    
    patch(Phusky(1,:),Phusky(2,:),clrHb);
    patch(Prh1(1,:),Prh1(2,:),clrHr);
    patch(Prh2(1,:),Prh2(2,:),clrHr);
    patch(Prh3(1,:),Prh3(2,:),clrHr);
    patch(Prh4(1,:),Prh4(2,:),clrHr);
    %
    if ~ noRef
        plot(S.path.references(S.config.X.x,end),S.path.references(S.config.X.y,end),'r+','markersize',20)
        plot(S.path.references(S.config.X.x,end),S.path.references(S.config.X.y,end),'ro','markersize',20)
        %
        if ~isempty(S.algorithms.mpcCasadi.Xtraj)
            plot(S.algorithms.mpcCasadi.Xtraj(1,:),S.algorithms.mpcCasadi.Xtraj(2,:),'r','LineWidth',2)
            plot(S.algorithms.mpcCasadi.Xtraj(1,1),S.algorithms.mpcCasadi.Xtraj(2,1),'ro','markersize',20)
        end
    end
    %
    if ~noRef
        hold off;
    end
    %
    drawnow limitrate
    
end

function S = call_init_functions(S)
    S = reserve_temp_memory(S);
    S = gen_init_conditions(S);
    S = create_algorithms(S);
    S = init_flags_and_counters(S);
end

function S = compute_tras_rot_mtx(S)
    sdpvar a11 a12 a21 a22;
    sdpvar x1bar y1bar x2bar y2bar;
    % Besides the reference point, two more are needed to obtaint the local
    % reference frame
    % AZOTEA AC3E *********************************************************
    % Coordinates of point (x, 0)
    S.ROS.local_coord_1.lat     = -33.034213;       % hand coded value, measurement from rtk
    S.ROS.local_coord_1.long    = -71.592168;   % hand coded value, measurement from rtk
    % CANCHA DE FUTBOL ****************************************************
%     S.ROS.local_coord_1.lat     = -33.03517;       % hand coded value, measurement from rtk
%     S.ROS.local_coord_1.long    = -71.594195;   % hand coded value, measurement from rtk
    % *********************************************************************
    [x1, y1]                    = latlon2xy(S.ROS.local_coord_1.lat, S.ROS.local_coord_1.long, S.ROS.LAT0, S.ROS.LON0);
    x1                          = x1*1000;
    y1                          = y1*1000;
    x                           = norm([x1 y1]);
    % AZOTEA AC3E *********************************************************
    % Coordinates of point (0, y)
    S.ROS.local_coord_2.lat     = -33.034088;        % hand coded value, measurement from rtk
    S.ROS.local_coord_2.long    = -71.59211333;         % hand coded value, measurement from rtk
    % CANCHA DE FUTBOL ****************************************************
%     S.ROS.local_coord_2.lat     = -33.034403333;        % hand coded value, measurement from rtk
%     S.ROS.local_coord_2.long    = -71.594075;         % hand coded value, measurement from rtk
    % *********************************************************************
    [x2, y2]                    = latlon2xy(S.ROS.local_coord_2.lat, S.ROS.local_coord_2.long, S.ROS.LAT0, S.ROS.LON0);
    x2                          = x2*1000;
    y2                          = y2*1000;
    y                           = norm([x2 y2]);
    % With the "origin" and a point alongside each axe, compute the mtx
    A                           = [a11 a12; a21 a22];
    v                           = [x1; y1; x2; y2];
    b                           = [x1bar; y1bar; x2bar; y2bar];
    Constraints                 = [[A zeros(2); zeros(2) A]*v - b == zeros(4,1); x1bar*x2bar + y1bar*y2bar == 0];
    %
    obj                         = (x1bar - x)^2 + (y2bar - y)^2;
    % 
    optimize(Constraints, obj);
    %
    S.ROS.Mtx                   = value(A);
end

function S = update_measurements(S)
    tic;
    if S.config.SIM == true
        if strcmp(S.config.noise,'gaussian')
            noise = randn(S.system.ny,1).*S.config.noise_lvl;
        else
            noise = (2*rand(S.system.ny,1)-1).*S.config.noise_lvl;
        end
        S.data.ysim_mhempc = [S.data.ysim_mhempc, S.data.xsim(S.config.outputs,end) + noise];
    else
        S                  = read_sensors(S);
        S                  = measurements_vector(S);
        S.data.ysim_mhempc = [S.data.ysim_mhempc, S.ROS.sensors.measurement];
    end
    S = update_sensorData_MHE(S);
    S.exec_time.t_sensors = [S.exec_time.t_sensors, toc];
end

function [flag,S] = check_end_condition(S)
    if (size(S.data.xest,2) == 1) || isempty(S.controller.ref)
        flag = false;
    else       
        d0totgt = norm(S.data.xest(S.config.X.x:S.config.X.y,end) - S.controller.ref(S.config.X.x:S.config.X.y));      
        flag = (S.algorithms.ctrl.last_tgt && (d0totgt < S.config.dist_last_tgt)) || sum(sum(isnan(S.data.xest)));

%         if S.algorithms.ctrl.last_tgt == true
%             S.config.countToFinish = S.config.countToFinish + 1;
%         end
% 
%         if S.config.countToFinish >= S.config.valToFinish
%             flag = true;
%         else
%             flag = false;
%         end
        
%         v1 = [S.data.xest(2*S.config.num_trailers+2)-S.data.xest(2*S.config.num_trailers+4), S.data.xest(2*S.config.num_trailers+3)-S.data.xest(2*S.config.num_trailers+5)];
%         v2 = [S.data.xest(2*S.config.num_trailers+2)-S.controller.ref(2*S.config.num_trailers+2), S.data.xest(2*S.config.num_trailers+3)-S.controller.ref(2*S.config.num_trailers+3)];
%         v1dotv2 = v1*v2';
% 
%         if (v1dotv2 > 0) && S.algorithms.ctrl.last_tgt
%             flag = flag || true;
%         end
    end
end

function S = ROS(S)
    S = init_ROS(S);
    S = create_obj_sens(S);
    S = create_obj_vel(S);    
end

function S = init_ROS(S)
    % Init coordinates of my local 2D plane
    S = get_reference(S);
    
    if S.config.calcMtxConvCoord == true
        S = compute_tras_rot_mtx(S);
    else
        S.ROS.Mtx = eye(2);
    end
    
    S.ROS.sensors.measurements = [];
    % Create and init Husky's velocities
    S.algorithms.mpc.legged_vf = 0;
    S.algorithms.mpc.legged_w0 = 0;
    % Here some instructions need to be followed from matlab's terminal
    rosshutdown;
    % input('use el comando roscore desde terminal...\n')
    %
    rosshutdown;
    rosinit;
    %
    % fprintf('roslaunch RTK.launch\nroslaunch vectornav vectornav.launch\nroslaunch vectornav vectornav2.launch\nroslaunch imu_3dm_gx4 imu.launch\nroslaunch Encoders.launch\nroslaunch husky_base base.launch\npython speedholder.py\nroslaunch VP12 lidar.launch\n')
    % aux1 = input('Presione enter...\n');
end

function S = create_obj_sens(S)
    sub_rtk         = rossubscriber('/fix_holder');
    % sub_vec1        = rossubscriber('/vectornav/IMU');
    % sub_vec1_vel    = rossubscriber('/imu_INS');
    sub_vec2        = rossubscriber('/vectornav2/IMU');
    sub_vec2_vel    = rossubscriber('/imu_INS2');
    % sub_encoders    = rossubscriber('/enc');
    % sub_micro1      = rossubscriber('/imu/pose');
    % sub_lidar       = rossubscriber('/angles');
    %
    S.ROS.rtk            = sub_rtk;
    S.ROS.vectornav2     = sub_vec2;
    % S.ROS.vectornav2     = sub_vec2;
    S.ROS.vectornav2_vel = sub_vec2_vel;
    % S.ROS.microstrain    = sub_micro1;
    % S.ROS.IncEncoders    = sub_encoders;
    % S.ROS.betasFromLidar = sub_lidar;

    % COrection factor for unwraping phase on real-time
    S.ROS.pose.correction_vec1              = 0;      % This value should be substracted to every pose measurement
    S.ROS.sensors.vectornav_euler_vec1      = [];
    S.ROS.sensors.vectornav_euler_vec1_Old  = 0;     % Value for computinng the difference. If it is bigger in absolute value than pi, a correction is needed
    %
    S.ROS.pose.correction_vec2              = 0;
    S.ROS.sensors.vectornav_euler_vec2      = [];
    S.ROS.sensors.vectornav_euler_vec2_Old  = 0;
    %
    S.ROS.pose.correction_micro1            = 0;      % This value should be substracted to every pose measurement
    S.ROS.sensors.microstrain_euler_micro1    = [];
    S.ROS.sensors.microstrain_euler_micro1_Old  = 0;
end

function S = create_obj_vel(S)
    [pub,msg]       = rospublisher('/cmd_vel_aux','geometry_msgs/Twist');
    %
    S.ROS.CTRL.pub       = pub;
    S.ROS.CTRL.msg       = msg;
end

function S = get_reference(S)
    % Coordiantes of the origin of our local reference 2D-axes x-y. Values
    % charged by hand before carry out the experiments.
    % AZOTEA AC3E *********************************************************
    S.ROS.LAT0   = -33.034115;
    S.ROS.LON0  = -71.592205;
    % CANCHA DE FUTBOL ****************************************************
%     S.ROS.LAT0   = -33.03453;
%     S.ROS.LON0  = -71.594498;
    %
end

function S = read_rtk(S)
    S.ROS.sensors.gpsrtk = receive(S.ROS.rtk);
    
    if isnan(S.ROS.sensors.gpsrtk.Latitude) || isnan(S.ROS.sensors.gpsrtk.Longitude)
        S = write_ctrlGo1(S, 0, 0, 0, 0);
        while isnan(S.ROS.sensors.gpsrtk.Latitude) || isnan(S.ROS.sensors.gpsrtk.Longitude)
            fprintf('NO GPS SIGNAL...')
            pause(0.2);
            S.ROS.sensors.gpsrtk = receive(S.ROS.rtk);
        end
    end
    % Convert lat &v lon to x-y coordinated in a tangential plane ro
    % earth's surface (tangent to my reference point)
    [x,y]   = latlon2xy(S.ROS.sensors.gpsrtk.Latitude,S.ROS.sensors.gpsrtk.Longitude,S.ROS.LAT0,S.ROS.LON0);
    % Convert Km to m
    x_raw   = x*1000;
    y_raw   = y*1000;
    % Adjust to my local 2D plane through the rotation matrix
    xy_cor  = S.ROS.Mtx * [x_raw; y_raw];
    % The, store values to be used later
    S.ROS.sensors.rtk.xN = xy_cor(1);
    S.ROS.sensors.rtk.yN = xy_cor(2);
end

function S = write_ctrlGo1(S, w0, vf, vl, vz)
    tic;
    S.ROS.CTRL.msg.Linear.X    = vf;
    S.ROS.CTRL.msg.Linear.Y    = -vl;   %% TAKE INTO ACCOUNT THE CHANGE OF SIGN HERE!!!!!!!!!!!!!!
    S.ROS.CTRL.msg.Linear.Z    = vz;
    S.ROS.CTRL.msg.Angular.Z   = w0;
    send(S.ROS.CTRL.pub,S.ROS.CTRL.msg);
    S.exec_time.t_ctrl = [S.exec_time.t_ctrl, toc];
end

function S = read_vectornav(S) % measure theta0 and the speeds
    % pose
    S.ROS.sensors.vectornav             = receive(S.ROS.vectornav1);
    quat                                = S.ROS.sensors.vectornav.Orientation;
    S.ROS.sensors.vectornav_euler_vec1  = quat2eul([quat.X,quat.Y,quat.Z,quat.W]);    
    % Unwrap pahse from online data _______________________________________
    if (S.ROS.sensors.vectornav_euler_vec1(3)-S.ROS.sensors.vectornav_euler_vec1_Old) >= pi
        S.ROS.pose.correction_vec1 = S.ROS.pose.correction_vec1 + 2*pi;
    elseif (S.ROS.sensors.vectornav_euler_vec1(3)-S.ROS.sensors.vectornav_euler_vec1_Old) <= -pi
        S.ROS.pose.correction_vec1 = S.ROS.pose.correction_vec1 - 2*pi;         
    end         
    %
    S.ROS.sensors.vectornav_euler_vec1_Old = S.ROS.sensors.vectornav_euler_vec1(3);    
    % No compute the attitude angle in my reference frama -----------------
    S.ROS.sensors.vectornav_theta0 = -S.ROS.sensors.vectornav_euler_vec1(3) + S.ROS.IMU_ZEROING_VEC1 + S.ROS.pose.correction_vec1;
    % Measure the speed ---------------------------------------------------
    S.ROS.sensors.vectornav_vel     = receive(S.ROS.vectornav1_vel);
    S.ROS.sensors.vectornav_NedVelX = S.ROS.sensors.vectornav_vel.Data(1);
    S.ROS.sensors.vectornav_NedVelY = S.ROS.sensors.vectornav_vel.Data(2);
    S.ROS.sensors.vectornav_u0      = sqrt(S.ROS.sensors.vectornav_NedVelX^2 + S.ROS.sensors.vectornav_NedVelY^2);
    S.ROS.sensors.vectornav_w0      = -1 * S.ROS.sensors.vectornav.AngularVelocity.Z;
end

function S = read_vectornav2(S) % measure theta2
    % pose
    S.ROS.sensors.vectornav2            = receive(S.ROS.vectornav2);
    quat                                = S.ROS.sensors.vectornav2.Orientation;
    S.ROS.sensors.vectornav_euler_vec2  = quat2eul([quat.X,quat.Y,quat.Z,quat.W]);    
    % Unwrap pahse from online data _______________________________________
    if (S.ROS.sensors.vectornav_euler_vec2(3)-S.ROS.sensors.vectornav_euler_vec2_Old) >= pi
        S.ROS.pose.correction_vec2 = S.ROS.pose.correction_vec2 + 2*pi;
    elseif (S.ROS.sensors.vectornav_euler_vec2(3)-S.ROS.sensors.vectornav_euler_vec2_Old) <= -pi
        S.ROS.pose.correction_vec2 = S.ROS.pose.correction_vec2 - 2*pi;         
    end         
    %
    S.ROS.sensors.vectornav_euler_vec2_Old = S.ROS.sensors.vectornav_euler_vec2(3);
    % No compute the attitude angle in my reference frame -----------------
    S.ROS.sensors.vectornav_theta2 = -S.ROS.sensors.vectornav_euler_vec2(3) + S.ROS.IMU_ZEROING_VEC2 + S.ROS.pose.correction_vec2;
end

function S = read_microstrain(S)
    S.ROS.sensors.microstrain   = receive(S.ROS.microstrain);    
    quat                        = S.ROS.sensors.microstrain.Pose.Orientation;
    S.ROS.sensors.microstrain_euler_micro1 = quat2eul([quat.X,quat.Y,quat.Z,quat.W]);
    % Unwrap pahse from online data _______________________________________
    if (S.ROS.sensors.microstrain_euler_micro1(3)-S.ROS.sensors.microstrain_euler_micro1_Old) >= pi
        S.ROS.pose.correction_micro1 = S.ROS.pose.correction_micro1 + 2*pi;
    elseif (S.ROS.sensors.microstrain_euler_micro1(3)-S.ROS.sensors.microstrain_euler_micro1_Old) <= -pi
        S.ROS.pose.correction_micro1 = S.ROS.pose.correction_micro1 - 2*pi;         
    end         
    %
    S.ROS.sensors.microstrain_euler_micro1_Old = S.ROS.sensors.microstrain_euler_micro1(3);
    % No compute the attitude angle in my reference frame -----------------
    S.ROS.sensors.microstrain_theta1 = -S.ROS.sensors.microstrain_euler_micro1(3) + S.ROS.IMU_ZEROING_MIC1 + S.ROS.pose.correction_micro1;
end

function S = read_sensors(S)
    % S = read_vectornav(S);
    S = read_vectornav2(S);
    % S = read_microstrain(S);
    % S = read_encoders(S);
    S = read_rtk(S);
    % S = read_betas_lidar(S);
end

function S = measurements_vector(S)
    S.ROS.sensors.measurement   = [S.ROS.sensors.rtk.xN; S.ROS.sensors.rtk.yN; 0.29; S.ROS.sensors.vectornav_theta2];
    S.ROS.sensors.measurements  = [S.ROS.sensors.measurements, S.ROS.sensors.measurement];
end

function S = call_MHE(S)
    tic;
%     S.data.xest                     = [S.data.xest, S.ROS.sensors.measurement];
    %                                                 
    if S.config.iters > 1
        state_aux = S.algorithms.mhe.output_MHE.x(2,:)';
    else 
        state_aux = S.init_condition.x0bar; 
    end
    S.algorithms.mhe.output_MHE     = acado_MHEstep(S.algorithms.mhe.input_MHE);
    S.data.xest                     = [S.data.xest, S.algorithms.mhe.output_MHE.x(end,:)'];
    S.algorithms.mhe.input_MHE.x    = S.algorithms.mhe.output_MHE.x;
    %
    state_aux                       = state_aux - S.algorithms.mhe.output_MHE.x(1,:)';
    S.algorithms.mhe.input_MHE.xAC  = S.algorithms.mhe.output_MHE.x(2,:)';
    % Update arrival-cost parameters (manually) ___________________
    if S.config.updtAC == true
        if S.config.iters > S.config.Ne+1
            S                               = updateAC(S,state_aux, S.algorithms.mhe.output_MHE.x(2,S.config.outputs)-S.algorithms.mhe.input_MHE.y(1,1:end));
            S.algorithms.mhe.input_MHE.SAC  = eye(size(S.algorithms.mhe.P)) / (S.algorithms.mhe.P);
            S.algorithms.mhe.input_MHE.WL   = chol(S.algorithms.mhe.input_MHE.SAC, 'lower');
        end
    end            
    %
    S.exec_time.t_mhe               = [S.exec_time.t_mhe, toc];
end

function S = call_PFA2(S)
    tic;

    disAcum = 0;
    kMax    = length(S.path.coordinates)-1;
    for i=S.path.acumIndx+1:kMax
        disAcum = disAcum + norm(S.path.coordinates(:,i)-S.path.coordinates(:,i-1));
        if (disAcum/S.config.Ts >= S.config.vf) || (i==kMax)
            S.path.acumIndx     = i;
            if i==kMax
                S.path.last_tgt = true;
            end
            break;
        end        
    end    

    if isempty(i)
        i = kMax;
    end

    if isempty(S.config.thetaRefFixPoint)
        dx0       = S.path.coordinates(1,i) - S.path.coordinates(1,i-1);
        dy0       = S.path.coordinates(2,i) - S.path.coordinates(2,i-1);
        new_angle = atan2c(dx0, dy0, S.angle0_old);
    else
        dx0       = S.config.thetaRefFixPoint(1) - S.path.coordinates(1,i-1);
        dy0       = S.config.thetaRefFixPoint(2) - S.path.coordinates(2,i-1);
        new_angle = atan2c(dx0, dy0, S.angle0_old);
    end    

    S.controller.ref    = [S.path.coordinates(:,i); 0.29; new_angle ];    
    S.angle0_old        = new_angle;
    
    if S.path.last_tgt
        S.algorithms.ctrl.last_tgt = 1;
        if isempty(i)
            S.controller.ref    = [S.path.coordinates(:,end); 0.29; new_angle ];
        end
    end

%     if isempty(S.path.uref)
%         S.config.uref = [S.config.w0; S.config.vf; S.config.vl; S.config.vz];
%     else
%         S.config.uref = S.path.uref;
%     end
    S.exec_time.t_pfa = [S.exec_time.t_pfa, toc];
end

function S = call_MPC(S)
    tic;
    S.algorithms.mpc.input_MPC.y   = [repmat(S.controller.ref', S.config.Nc,1), repmat(S.config.uref',S.config.Nc,1)];    
    S.algorithms.mpc.input_MPC.yN  = S.controller.ref';
    try
        S.path.references       = [S.path.references, S.controller.ref];
    catch
        a=1
    end
    % Solve control problem _______________________________________________
    S.algorithms.mpc.input_MPC.x0  = S.data.xest(:,end)';
    if S.config.iters > S.config.Ne+1        
        S.algorithms.mpc.output_MPC = acado_MPCstep(S.algorithms.mpc.input_MPC);
        if isnan(S.algorithms.mpc.output_MPC.u(1,:))
            S.algorithms.mpc.output_MPC.u = zeros(size(S.algorithms.mpc.output_MPC.u));
        end             
    else
        S.algorithms.mpc.output_MPC.u(1,:) = [0, 0, 0, 0];
    end
    S.exec_time.t_mpc              = [S.exec_time.t_mpc, toc];
    %
    S.algorithms.mpc.controls_MPC  = [S.algorithms.mpc.controls_MPC, S.algorithms.mpc.output_MPC.u(1,:)'];
    % To the last estimated velocties, add the last acceleration computed _
    S.algorithms.mpc.controls_MPC_w0 = [S.algorithms.mpc.controls_MPC_w0, S.algorithms.mpc.controls_MPC(S.config.U.w0,end)];
    S.algorithms.mpc.controls_MPC_vf = [S.algorithms.mpc.controls_MPC_vf, S.algorithms.mpc.controls_MPC(S.config.U.vf,end)];
    S.algorithms.mpc.controls_MPC_vl = [S.algorithms.mpc.controls_MPC_vl, S.algorithms.mpc.controls_MPC(S.config.U.vl,end)];
    S.algorithms.mpc.controls_MPC_vz = [S.algorithms.mpc.controls_MPC_vz, S.algorithms.mpc.controls_MPC(S.config.U.vz,end)];
    % Velocities to apply to the legged robot _____________________________
    S.algorithms.mpc.legged_w0       = S.algorithms.mpc.controls_MPC_w0(end);    
    S.algorithms.mpc.legged_vf       = S.algorithms.mpc.controls_MPC_vf(end);    
    S.algorithms.mpc.legged_vz       = S.algorithms.mpc.controls_MPC_vz(end);
    %
    if isnan(S.algorithms.mpc.output_MPC.u(1,:))
        S.algorithms.mpc.input_MPC.u   = zeros(size(S.algorithms.mpc.output_MPC.u)); 
    else
        S.algorithms.mpc.input_MPC.x   = S.algorithms.mpc.output_MPC.x;
        S.algorithms.mpc.input_MPC.u   = S.algorithms.mpc.output_MPC.u; 
    end        
    % Apply controls
    S.simulation_input.u = S.algorithms.mpc.controls_MPC(:,end);
    S.data.Ctrls = [S.data.Ctrls, S.simulation_input.u];
end

function S = call_MPCCasadi(S)
    tic;
    %
    setRef(S.algorithms.mpcCasadi,S.controller.ref);
    setx0(S.algorithms.mpcCasadi,S.data.xest(:,end));    

    try
        S.path.references       = [S.path.references, S.controller.ref];
    catch
        a=1
    end
    % Solve control problem _______________________________________________
    if S.config.iters > S.config.Ne+1        
        solve(S.algorithms.mpcCasadi);
    else
        S.algorithms.mpc.output_MPC.u(1,:) = [0, 0, 0, 0];
    end
    S.exec_time.t_mpc               = [S.exec_time.t_mpc, toc];
    %
    S.algorithms.mpc.controls_MPC  = [S.algorithms.mpc.controls_MPC, S.algorithms.mpcCasadi.u_k];
    % To the last estimated velocties, add the last acceleration computed _
    S.algorithms.mpc.controls_MPC_w0 = [S.algorithms.mpc.controls_MPC_w0, S.algorithms.mpc.controls_MPC(S.config.U.w0,end)];
    S.algorithms.mpc.controls_MPC_vf = [S.algorithms.mpc.controls_MPC_vf, S.algorithms.mpc.controls_MPC(S.config.U.vf,end)];
    S.algorithms.mpc.controls_MPC_vl = [S.algorithms.mpc.controls_MPC_vl, S.algorithms.mpc.controls_MPC(S.config.U.vl,end)];
    S.algorithms.mpc.controls_MPC_vz = [S.algorithms.mpc.controls_MPC_vz, S.algorithms.mpc.controls_MPC(S.config.U.vz,end)];
    % Velocities to apply to the legged robot _____________________________
    S.algorithms.mpc.legged_w0       = S.algorithms.mpc.controls_MPC_w0(end);
    S.algorithms.mpc.legged_vf       = S.algorithms.mpc.controls_MPC_vf(end);
    S.algorithms.mpc.legged_vz       = S.algorithms.mpc.controls_MPC_vz(end);    
    % Apply controls
    S.simulation_input.u            = S.algorithms.mpc.controls_MPC(:,end);
    S.data.Ctrls                    = [S.data.Ctrls, S.simulation_input.u];
end

function S = call_MPCintCasadi(S)
    tic;
    %
    setRef(S.algorithms.mpcIntCasadi,S.controller.ref);
    setx0(S.algorithms.mpcIntCasadi,S.data.xest(:,end));    

    try
        S.path.references       = [S.path.references, S.controller.ref];
    catch
        a=1
    end
    % Solve control problem _______________________________________________
    if S.config.iters > S.config.Ne+1        
        solve(S.algorithms.mpcIntCasadi);
    else
        S.algorithms.mpc.output_MPC.u(1,:) = [0, 0, 0, 0];
    end
    S.exec_time.t_mpc               = [S.exec_time.t_mpc, toc];
    %
    S.algorithms.mpc.controls_MPC  = [S.algorithms.mpc.controls_MPC, S.algorithms.mpcIntCasadi.u_k];
    % To the last estimated velocties, add the last acceleration computed _
    S.algorithms.mpc.controls_MPC_w0 = [S.algorithms.mpc.controls_MPC_w0, S.algorithms.mpc.controls_MPC(S.config.U.w0,end)];
    S.algorithms.mpc.controls_MPC_vf = [S.algorithms.mpc.controls_MPC_vf, S.algorithms.mpc.controls_MPC(S.config.U.vf,end)];
    S.algorithms.mpc.controls_MPC_vl = [S.algorithms.mpc.controls_MPC_vl, S.algorithms.mpc.controls_MPC(S.config.U.vl,end)];
    S.algorithms.mpc.controls_MPC_vz = [S.algorithms.mpc.controls_MPC_vz, S.algorithms.mpc.controls_MPC(S.config.U.vz,end)];
    % Velocities to apply to the legged robot _____________________________
    S.algorithms.mpc.legged_w0       = S.algorithms.mpc.controls_MPC_w0(end);
    S.algorithms.mpc.legged_vf       = S.algorithms.mpc.controls_MPC_vf(end);
    S.algorithms.mpc.legged_vl       = S.algorithms.mpc.controls_MPC_vl(end);
    S.algorithms.mpc.legged_vz       = S.algorithms.mpc.controls_MPC_vz(end);
    % Apply controls
    S.simulation_input.u            = S.algorithms.mpc.controls_MPC(:,end);
    S.data.Ctrls                    = [S.data.Ctrls, S.simulation_input.u];
end

function S = call_LABC(S)
    tic;
    try
        S.path.references       = [S.path.references, S.controller.ref];
    catch
        a=1
    end
    updateReference(S.algorithms.LABC.alg,S.controller.ref);
    updateState(S.algorithms.LABC.alg,S.data.xest(:,end));
    solve(S.algorithms.LABC.alg);
    %
    S.algorithms.mpc.controls_MPC_w0 = [S.algorithms.mpc.controls_MPC_w0, S.algorithms.LABC.alg.u(S.config.U.w0,end)];
    S.algorithms.mpc.controls_MPC_vf = [S.algorithms.mpc.controls_MPC_vf, S.algorithms.LABC.alg.u(S.config.U.vf,end)];
    S.algorithms.mpc.controls_MPC_vl = [S.algorithms.mpc.controls_MPC_vl, S.algorithms.LABC.alg.u(S.config.U.vl,end)];
    S.algorithms.mpc.controls_MPC_vz = [S.algorithms.mpc.controls_MPC_vz, S.algorithms.LABC.alg.u(S.config.U.vz,end)];

    S.exec_time.t_labc = [S.exec_time.t_labc, toc];
    %
    S.algorithms.LABC.controls_LABC = [S.algorithms.LABC.controls_LABC, S.algorithms.LABC.alg.u];
    %    
    % Apply controls
    S.simulation_input.u = S.algorithms.LABC.controls_LABC(:,end);
    S.data.Ctrls = [S.data.Ctrls, S.simulation_input.u];
end

function S = call_LABC_SAT(S)
    tic;
    try
    S.path.references       = [S.path.references, S.controller.ref];
    catch
        a=1
    end
    updateReference(S.algorithms.LABC_SAT.alg,S.controller.ref);
    updateState(S.algorithms.LABC_SAT.alg,S.data.xest(:,end));
    solve(S.algorithms.LABC_SAT.alg);
    S.exec_time.t_labc = [S.exec_time.t_labc, toc];
    %
    S.algorithms.LABC_SAT.controls_LABC_SAT = [S.algorithms.LABC_SAT.controls_LABC_SAT, S.algorithms.LABC_SAT.alg.u];
    %    
    % Apply controls
    S.simulation_input.u = S.algorithms.LABC_SAT.controls_LABC_SAT(:,end);
    S.data.Ctrls = [S.data.Ctrls, S.simulation_input.u];
end

function S = call_LAOBC(S)
    tic;
    try
    S.path.references       = [S.path.references, S.controller.ref];
    catch
        a=1
    end
    updateReference(S.algorithms.LAOBC.alg,S.controller.ref);
    updateState(S.algorithms.LAOBC.alg,S.data.xest(:,end));
    solve(S.algorithms.LAOBC.alg);
    S.exec_time.t_laobc = [S.exec_time.t_laobc, toc];
    %
    S.algorithms.mpc.controls_MPC_w0 = [S.algorithms.mpc.controls_MPC_w0, S.algorithms.LAOBC.alg.u(S.config.U.w0,end)];
    S.algorithms.mpc.controls_MPC_vf = [S.algorithms.mpc.controls_MPC_vf, S.algorithms.LAOBC.alg.u(S.config.U.vf,end)];
    S.algorithms.mpc.controls_MPC_vl = [S.algorithms.mpc.controls_MPC_vl, S.algorithms.LAOBC.alg.u(S.config.U.vl,end)];
    S.algorithms.mpc.controls_MPC_vz = [S.algorithms.mpc.controls_MPC_vz, S.algorithms.LAOBC.alg.u(S.config.U.vz,end)];
    % Velocities to apply to the legged robot _____________________________
    S.algorithms.mpc.legged_w0       = S.algorithms.mpc.controls_MPC_w0(end);
    S.algorithms.mpc.legged_vf       = S.algorithms.mpc.controls_MPC_vf(end);
    S.algorithms.mpc.legged_vl       = S.algorithms.mpc.controls_MPC_vl(end);
    S.algorithms.mpc.legged_vz       = S.algorithms.mpc.controls_MPC_vz(end);
    %
    S.algorithms.LAOBC.controls_LAOBC = [S.algorithms.LAOBC.controls_LAOBC, S.algorithms.LAOBC.alg.u];    
    % Apply controls
    S.simulation_input.u = S.algorithms.LAOBC.controls_LAOBC(:,end);
    S.data.Ctrls = [S.data.Ctrls, S.simulation_input.u];
    %
    S.algorithms.LAOBC.w_slack = [S.algorithms.LAOBC.w_slack, S.algorithms.LAOBC.alg.w];
end

function S = update_sensorData_MHE(S)
    S.algorithms.mhe.input_MHE.y   = double([S.algorithms.mhe.input_MHE.y(2:end,:); S.algorithms.mhe.input_MHE.yN]);
    S.algorithms.mhe.input_MHE.yN  = double(S.data.ysim_mhempc(:,end)');
    %
    S.algorithms.mhe.input_MHE.u   = [S.algorithms.mhe.output_MHE.u(2:end,:); S.algorithms.mpc.controls_MPC(:,end)'];
end

function S = gen_path(S,type)
    if strcmp(type,'infinity')
        a = 1.75;
        c = 4;
        b = 1;
        t = 0:0.0001:2*pi;
        x = (a*sqrt(2).*cos(t))./(sin(t).^2+1);
        y = (c*sqrt(2).*cos(t).*sin(t))./(sin(t).^2 + b);
        %
        S.path.coorection_x = 6.5;  % correction for the field experiemnts in order to fit the path in my locala reference frame
        S.path.coorection_y = 4.5;
        S.path.coordinates  = [x+S.path.coorection_x;y+S.path.coorection_y];
        
    elseif strcmp(type,'rectangular')
        t = 0:0.0001:2*pi;
        p = 2.5;
        q = 2;

        x = p.*(sqrt(cos(t).*cos(t)).*cos(t) + sqrt(sin(t).*sin(t)).*sin(t));
        y = q.*(sqrt(cos(t).*cos(t)).*cos(t) - sqrt(sin(t).*sin(t)).*sin(t));

        S.path.coorection_x = p+4;  % correction for the field experiemnts in order to fit the path in my locala reference frame
        S.path.coorection_y = q+2.5;
        S.path.coordinates  = [x+S.path.coorection_x;y+S.path.coorection_y];
        
    elseif strcmp(type,'circular')
        a = 2;
        b = 2;
%         b = 0.4;
        t = 0:0.0001:2*pi;        
        x = a*cos(t);
        y = b*sin(t);
        %
        S.path.coorection_x = 5.5;
        S.path.coorection_y = 4.5;
        S.path.coordinates  = [x+S.path.coorection_x;y+S.path.coorection_y];
     elseif strcmp(type,'rect')
        x = 0:0.0001:100;
        y = zeros(1,length(x));
        S.path.coordinates  = [x;y];
    end
end

function S = gen_obstacle(S,x,y,r)
    S.path.obstacles = [S.path.obstacles; [x, y, r]];
end

function S = gen_dynamic_and_solvers(S)
    % System's dimension __________________________________________________        
    S.system.nu      = 4;                                                              % input dimension: [w0,v0]
    S.system.ny      = 4;      % output dimension: [beta_i,theta_0,xy_0]
    S.system.nv      = S.system.ny;    
    S.system.nq      = 4;
    % System's parameters -------------------------------------------------        

    % System's variables  -------------------------------------------------
    % Since ACADO does not allows to incorporate directly rate of change
    % constraints, I've add the controls w0 and v0 as additional states and
    % define as controls the variaton of w0 and v0 and add constraints on
    % these new inputs. Proper differential equations were added too.
    Control w0 vf vl vz;
    % ACADO FORMULATION *******************************************
    DifferentialState x y z theta;    
    % Model to be used with the MHE -------------------------------
    f_mhe = [   dot(x) - vf*cos(theta) - vl*sin(theta) == 0; 
                dot(y) - vf*sin(theta) + vl*cos(theta) == 0;
                                        dot(z) - vz    == 0;
                                      dot(theta) - w0  == 0 ];
    % Model to be used with the MPC -------------------------------
    f_mpc = [   dot(x) - vf*cos(theta) - vl*sin(theta) == 0; 
                dot(y) - vf*sin(theta) + vl*cos(theta) == 0;
                                        dot(z) - vz    == 0;
                                      dot(theta) - w0  == 0 ];
    % True model --------------------------------------------------    
        F = [   dot(x) - vf*cos(theta) - vl*sin(theta) == 0; 
                dot(y) - vf*sin(theta) + vl*cos(theta) == 0;
                                        dot(z) - vz    == 0;
                                      dot(theta) - w0  == 0 ];
    % Model formulated with Casadi
    S.dynamic.q     = casadi.MX.sym('q',S.system.nq);        
    S.dynamic.u     = casadi.MX.sym('u',S.system.nu);        
    S.dynamic.f_rhs = [S.dynamic.u(2)*cos(S.dynamic.q(4)) + S.dynamic.u(3)*sin(S.dynamic.q(4));...
                       S.dynamic.u(2)*sin(S.dynamic.q(4)) - S.dynamic.u(3)*cos(S.dynamic.q(4));...
                       S.dynamic.u(4);...
                       S.dynamic.u(1) ];
    % Generate CASADI functions and integrators ***************************
    S.dynamic.f     = casadi.Function('f_rhs', {S.dynamic.q,S.dynamic.u}, {S.dynamic.f_rhs});    
    opts            = struct('main',true,'mex',true);
    S.dynamic.f.generate('f.c',opts)
    mex f.c -largeArrayDims;
    % RK4 -----------------------------------------------------------------
    k1              = S.dynamic.f(S.dynamic.q, S.dynamic.u);
    k2              = S.dynamic.f(S.dynamic.q + S.config.Ts / 2 * k1, S.dynamic.u);
    k3              = S.dynamic.f(S.dynamic.q + S.config.Ts / 2 * k2, S.dynamic.u);
    k4              = S.dynamic.f(S.dynamic.q + S.config.Ts * k3, S.dynamic.u);
    x_rk4           = S.dynamic.q + S.config.Ts / 6 * (k1 + 2 * k2 + 2 * k3 + k4);
    S.dynamic.F     = casadi.Function('F', {S.dynamic.q, S.dynamic.u}, {x_rk4});
%         S.dynamic.F     = casadi.Function('F', {S.dynamic.q, S.dynamic.u}, {S.dynamic.q + S.dynamic.f(S.dynamic.q, S.dynamic.u)*S.config.Ts});
    S.dynamic.F.generate('F.c',opts);
%         mex F.c -largeArrayDims;
    % Output of the system ------------------------------------------------
    S.dynamic.h_rhs  = S.dynamic.q(S.config.outputs);
    S.dynamic.h      = casadi.Function('h', {S.dynamic.q}, {S.dynamic.h_rhs});       
    %
    % Generate ACADO integrator *******************************************
    acadoSet('problemname', 'EXP');
    EXP = acado.SIMexport( S.config.Ts );
    EXP.setModel(F);    % USE TRUE MODEL HERE
    EXP.set( 'INTEGRATOR_TYPE',            'INT_IRK_GL4' );%INT_IRK_GL4
    EXP.set( 'NUM_INTEGRATOR_STEPS',        5     );%10
    EXP.set( 'DISCRETIZATION_TYPE',        'MULTIPLE_SHOOTING'      );
    if S.config.EXPORT
        EXP.exportCode( 'export_EXP' );
        cd export_EXP
        make_acado_integrator('../integrate_legged')
        cd ..
    end
    % =========================== ESTIMATOR ===============================
    acadoSet('problemname', 'mhe');
    ocpmhe  = acado.OCP( 0.0, S.config.Ne*S.config.Ts, S.config.Ne );
    meas    = [x, y, z, theta];
    Wmhe    = acado.BMatrix(eye(S.system.nq));
    WNmhe   = acado.BMatrix(eye(S.system.nq));
    %
    ocpmhe.minimizeLSQ( Wmhe, meas );
    ocpmhe.minimizeLSQEndTerm( WNmhe, meas );
    ocpmhe.setModel(f_mhe);
    %
    mhe     = acado.OCPexport( ocpmhe );
    mhe.set( 'HESSIAN_APPROXIMATION',       'GAUSS_NEWTON'      );
    mhe.set( 'LEVENBERG_MARQUARDT',          1e-12              );%1e-6
    mhe.set( 'DISCRETIZATION_TYPE',         'MULTIPLE_SHOOTING' );
    mhe.set( 'SPARSE_QP_SOLUTION',          'CONDENSING'        );
    mhe.set( 'HOTSTART_QP',                 'YES'               );
    mhe.set( 'INTEGRATOR_TYPE',             'INT_IRK_GL4'       );%INT_BDF
    mhe.set( 'NUM_INTEGRATOR_STEPS',         (S.config.Ne+1)  );
    mhe.set( 'QP_SOLVER',                   'QP_QPOASES'    	);
    mhe.set( 'FIX_INITIAL_STATE',           'NO'                );
    mhe.set( 'CG_USE_ARRIVAL_COST',         'NO'               );
    % Export estimator
    if S.config.EXPORT
        mhe.exportCode( 'export_MHE' );    
        global ACADO_; %#ok<*TLEV,*REDEF>
        copyfile([ACADO_.pwd '/../../external_packages/qpoases'], 'export_MHE/qpoases')    
        cd export_MHE
        make_acado_solver('../acado_MHEstep')   % function created, takes as input a structure with parameters
        cd ..
    end
    S.algorithms.mhe.input_MHE = struct;
    % ========================== CONTROLLER ===============================
    acadoSet('problemname', 'mpc');
    ocpmpc = acado.OCP( 0.0, S.config.Nc*S.config.Ts, S.config.Nc );
    h     = [x, y, z, theta, w0, vf, vl, vz];
    hN    = [x, y, z, theta];
    Wmpc  = acado.BMatrix(eye(length(h)));
    WNmpc = acado.BMatrix(eye(length(hN)));
    %
    ocpmpc.minimizeLSQ( Wmpc, h );
    ocpmpc.minimizeLSQEndTerm( WNmpc, hN );
    ocpmpc.subjectTo( S.config.Uconstraints(1,1) <= w0 <= S.config.Uconstraints(1,2) );
    ocpmpc.subjectTo( S.config.Uconstraints(2,1) <= vf <= S.config.Uconstraints(2,2) );
    ocpmpc.subjectTo( S.config.Uconstraints(3,1) <= vl <= S.config.Uconstraints(3,2) );
    ocpmpc.subjectTo( S.config.Uconstraints(4,1) <= vz <= S.config.Uconstraints(4,2) );
    ocpmpc.setModel(f_mpc);   % use uncertain model here
    %
    mpc = acado.OCPexport( ocpmpc );
    mpc.set( 'HESSIAN_APPROXIMATION',       'GAUSS_NEWTON'      );
    mpc.set( 'LEVENBERG_MARQUARDT',         1e-12               );
    mpc.set( 'DISCRETIZATION_TYPE',         'MULTIPLE_SHOOTING' );
    mpc.set( 'SPARSE_QP_SOLUTION',          'CONDENSING'        ); % FULL_CONDENSING_N2
    mhe.set( 'HOTSTART_QP',                 'YES'               );
    mpc.set( 'INTEGRATOR_TYPE',             'INT_IRK_GL4'     ); % INT_EX_EULER, INT_IRK_GL4, INT_IRK_RIIA5,INT_RK78, INT_RK12 INT_RK23, INT_RK45, INT_RK78, INT_BDF. More details gere: https://acado.sourceforge.net/doc/html/da/d05/classIntegrator.html
    mpc.set( 'NUM_INTEGRATOR_STEPS',        (S.config.Nc+1)   );
    mpc.set( 'QP_SOLVER',                   'QP_QPOASES'    	);
    %
    if S.config.EXPORT
        mpc.exportCode( 'export_MPC' );    
        global ACADO_; %#ok<*TLEV>
        copyfile([ACADO_.pwd '/../../external_packages/qpoases'], 'export_MPC/qpoases')    
        cd export_MPC
        make_acado_solver('../acado_MPCstep')
        cd ..
    end
    %
    S.algorithms.mpc.input_MPC = struct;
end

function S = updateAC(S,q,v)
    S.algorithms.mhe.Mk     = exp( -((q' * S.algorithms.mhe.P * q) * ( (v * v' ) / S.algorithms.mhe.B )) -1e-3);
    S.algorithms.mhe.alphak = 1 - S.algorithms.mhe.Mk;
    l                       = real(eig(S.algorithms.mhe.P));
    L                       = max(l);
    S.algorithms.mhe.Wk     = S.algorithms.mhe.P * ( eye(S.system.nq) - ( (q * q') * S.algorithms.mhe.P) ./ (1 + q' * q * L) );
    traceWoverAlphak        = trace(S.algorithms.mhe.Wk) / S.algorithms.mhe.alphak;
    if traceWoverAlphak >= S.algorithms.mhe.A
        S.algorithms.mhe.P = S.algorithms.mhe.Wk .* (S.algorithms.mhe.alphak)^(1/S.system.nq);
    else
        S.algorithms.mhe.P = S.algorithms.mhe.Wk;
    end
end

function S = init_mhe(S)
    % PARAMETERS MHE
    S.algorithms.mhe.input_MHE.x      = repmat(S.init_condition.x0bar',S.config.Ne+1,1);
    S.algorithms.mhe.input_MHE.u      = zeros(S.config.Ne,S.system.nu);
    
    S.algorithms.mhe.input_MHE.y      = double([repmat(S.init_condition.x0bar(S.config.outputs)',S.config.Ne,1)]);  % measurement of (available) states and inputs
    S.algorithms.mhe.input_MHE.yN     = double(S.init_condition.x0bar(S.config.outputs)');         % last node: measurement only of (available) states
    % weights for field experiments with the G2T
    if S.config.SIM == false % G2T
        S.algorithms.mhe.input_MHE.W  = eye(4);
        S.algorithms.mhe.input_MHE.WN = eye(4);
    else
        S.algorithms.mhe.input_MHE.W  = diag([3 3 1 2]);
        S.algorithms.mhe.input_MHE.WN = diag([3 3 1 2]);
    end
   

    S.algorithms.mhe.input_MHE.shifting.strategy = 1;
%     arrival-cost parameters: https://sourceforge.net/p/acado/discussion/general/thread/a090a6b6/?limit=25#e44b
%     AC implementation: A real-time algorithm for moving horizon state and parameter estimation        
    S = init_MHE_AC(S);    
end

function S = init_MHE_AC(S)
    S.algorithms.mhe.B                = 1e3;
    S.algorithms.mhe.C                = 1e1;
    S.algorithms.mhe.A                = S.algorithms.mhe.C;
    S.algorithms.mhe.P                = S.algorithms.mhe.C.*eye(4);
    %
    S.algorithms.mhe.input_MHE.xAC    = S.init_condition.x0bar;
    S.algorithms.mhe.input_MHE.SAC    = inv(S.algorithms.mhe.P);
    S.algorithms.mhe.input_MHE.WL     = chol(S.algorithms.mhe.input_MHE.SAC);
end

function S = init_mpc(S)
    % PARAMETERS MPC
    Xref                               = [0 0 0.29 0];
    Uref                               = S.config.uref';
    S.algorithms.mpc.input_MPC.x       = repmat(S.init_condition.x0bar',S.config.Nc+1,1);
    
    S.algorithms.mpc.output_MPC.x      = S.algorithms.mpc.input_MPC.x;

    S.algorithms.mpc.Uref              = repmat(Uref,S.config.Nc,1);
    S.algorithms.mpc.input_MPC.u       = zeros(size(S.algorithms.mpc.Uref));
    
    S.algorithms.mpc.output_MPC.u      = S.algorithms.mpc.input_MPC.u;

    S.algorithms.mpc.input_MPC.y       = repmat(Xref,S.config.Nc,1);
    S.algorithms.mpc.input_MPC.yN      = Xref;
    
    if S.config.SIM == false
        S.algorithms.mpc.input_MPC.W   =    diag([10 10 0.001 5 1 1  1 1]);
        S.algorithms.mpc.input_MPC.WN  = 5.*diag([10 10 0.001 5]);
    else
        S.algorithms.mpc.input_MPC.W   = diag([1 1 1 1 0 0 0 0]);
        S.algorithms.mpc.input_MPC.WN  = S.algorithms.mpc.input_MPC.W(1:S.system.nq,1:S.system.nq);
    end    

    S.algorithms.ctrl.last_tgt          = 0;
    S.algorithms.mpc.input_MPC.shifting.strategy = 1;   
    S.algorithms.mpc.controls_MPC      = zeros(S.system.nu,1);
    %
    S.algorithms.mpc.controls_MPC_w0   = 0;
    S.algorithms.mpc.controls_MPC_vf   = 0;
    S.algorithms.mpc.controls_MPC_vl   = 0;
    S.algorithms.mpc.controls_MPC_vz   = 0;
    %
    S.algorithms.mpc.legged_w0          = 0;
    S.algorithms.mpc.legged_vf         = 0;
    S.algorithms.mpc.legged_vl         = 0;
    S.algorithms.mpc.legged_vz         = 0;
end

function S = init_mpcCasadi(S)
    % PARAMETERS MPC
    % constraints
    constraints     = struct;
    constraints.X   = [];
    constraints.Xf  = [];
    constraints.U   = S.config.Uconstraints;
    constraints.dU  = S.config.dUconstraints;
    % Wieghting matrices
    mtxs            = struct;
    mtxs.Q          = diag([10 10 0.001 5]);
    mtxs.Qf         = mtxs.Q;
    mtxs.R          = diag([0.1 0.1 0.1 0.1]);
    % Systems dimension
    dims            = struct;
    dims.nq         = S.system.nq;
    dims.nu         = S.system.nu;

%     S.algorithms.mpcCasadi = nlmpcCasadi(S.config.Nc,S.dynamic.F,mtxs,dims,constraints,S.config.Ts);
    S.algorithms.mpcCasadi = nlmpcCasadi(S.config.Nc,S.dynamic.F,mtxs,dims,constraints,S.config.Ts);
    setx0(S.algorithms.mpcCasadi,S.init_condition.x0bar);

    S.algorithms.ctrl.last_tgt          = 0;
    S.algorithms.mpc.controls_MPC       = zeros(S.system.nu,1);
    %
    S.algorithms.mpc.controls_MPC_w0    = 0;
    S.algorithms.mpc.controls_MPC_vf    = 0;
    S.algorithms.mpc.controls_MPC_vl    = 0;
    S.algorithms.mpc.controls_MPC_vz    = 0;
    %
    S.algorithms.mpc.legged_w0          = 0;
    S.algorithms.mpc.legged_vf          = 0;
    S.algorithms.mpc.legged_vl          = 0;
    S.algorithms.mpc.legged_vz          = 0;
end

function S = init_mpcIntCasadi(S)
    % PARAMETERS MPC
    % constraints
    constraintsInt     = struct;
    constraintsInt.X   = [];
    constraintsInt.Xf  = [];
    constraintsInt.U   = S.config.Uconstraints;
    constraintsInt.dU  = S.config.dUconstraints;
    % Wieghting matrices
    mtxsInt            = struct;
    mtxsInt.Q          = diag([2 2 0.001 1]);
    mtxsInt.Qf         = mtxsInt.Q;
    mtxsInt.R          = diag([1 1 1 1]);
    % Systems dimension
    dimsInt            = struct;
    dimsInt.nq         = S.system.nq;
    dimsInt.nu         = S.system.nu;

%     S.algorithms.mpcCasadi = nlmpcCasadi(S.config.Nc,S.dynamic.F,mtxs,dims,constraints,S.config.Ts);
    S.algorithms.mpcIntCasadi = nlmpcIntCasadi(S.config.Nc,S.dynamic.F,mtxsInt,dimsInt,constraintsInt,S.config.Ts);
    setx0(S.algorithms.mpcIntCasadi,[S.init_condition.x0bar;0]);   % integral state starts from 0.

    S.algorithms.ctrl.last_tgt          = 0;
    S.algorithms.mpc.controls_MPC       = zeros(S.system.nu,1);
    %
    S.algorithms.mpc.controls_MPC_w0    = 0;
    S.algorithms.mpc.controls_MPC_vf    = 0;
    S.algorithms.mpc.controls_MPC_vl    = 0;
    S.algorithms.mpc.controls_MPC_vz    = 0;
    %
    S.algorithms.mpc.legged_w0          = 0;
    S.algorithms.mpc.legged_vf          = 0;
    S.algorithms.mpc.legged_vl          = 0;
    S.algorithms.mpc.legged_vz          = 0;
end

function S = init_LABC(S)
    S.algorithms.LABC.controls_LABC = zeros(S.system.nu,1);
    S.algorithms.LABC.alg           = LABC(S.system.nq,S.system.nu,S.init_condition.x0bar,S.config.Ts,[]);
    S.algorithms.LABC.k             = 0.794;
    S.algorithms.LABC.K1            = 0.02;
    S.algorithms.LABC.K2            = 0.0031;
    set_gains(S.algorithms.LABC.alg, S.algorithms.LABC.k, S.algorithms.LABC.K1, S.algorithms.LABC.K2)

    S.algorithms.ctrl.last_tgt          = 0;
end

function S = init_LABC_SAT(S)
    S.algorithms.LABC_SAT.controls_LABC_SAT = zeros(S.system.nu,1);
    S.algorithms.LABC_SAT.alg           = LABC(S.system.nq,S.system.nu,S.init_condition.x0bar,S.config.Ts,S.config.Uconstraints);
    S.algorithms.LABC_SAT.k             = 0.794;
    S.algorithms.LABC_SAT.K1            = 0.02;
    S.algorithms.LABC_SAT.K2            = 0.0031;
    set_gains(S.algorithms.LABC_SAT.alg, S.algorithms.LABC_SAT.k, S.algorithms.LABC_SAT.K1, S.algorithms.LABC_SAT.K2)

    S.algorithms.ctrl.last_tgt          = 0;
end

function S = init_LAOBC(S)
    S.algorithms.LAOBC.controls_LAOBC = zeros(S.system.nu,1);    
    S.algorithms.W                   = diag([1 1 1 1]);%eye(S.system.nq);
    S.algorithms.LAOBC.alg           = LAOBC(S.system.nq,S.system.nu,S.init_condition.x0bar,S.config.Ts,S.algorithms.W,S.config.Uconstraints,S.config.dUconstraints,'go1');
    S.algorithms.LAOBC.k             = 0.9802;    % 0.794;
    S.algorithms.LAOBC.K1            = 0.01;     % 0.02;   % K1 should be chosen as K1 = (1-k)/Ts
    S.algorithms.LAOBC.K2            = 0.01;%0.1;       % 0.0031;    
    S.algorithms.LAOBC.l1            = 0.47;     % forgetting factor of fisrt integral part
    S.algorithms.LAOBC.l2            = 0.53;     % forgetting factor of second integral part
    set_gains(S.algorithms.LAOBC.alg, S.algorithms.LAOBC.k, S.algorithms.LAOBC.K1, S.algorithms.LAOBC.K2)
    set_forgetting_factors(S.algorithms.LAOBC.alg,S.algorithms.LAOBC.l1,S.algorithms.LAOBC.l2)

    S.algorithms.ctrl.last_tgt       = 0;
end

function S = init()
    % init fields of the structure
    S                 = struct;
    S.config          = struct;
    S.algorithms      = struct;
    S.noises          = struct;
    S.init_condition  = struct;
    S.path            = struct;
    S.obstacles       = struct;
    S.data            = struct;    
    S.plots           = struct;
    S.acado           = struct;
    S.acado.system    = struct;
    S.acado.states    = struct;
    S.acado.controls  = struct;
    S.exec_time       = struct;
    %
    S                 = reserve_notemp_memory(S);
    S                 = build_setup(S);
    S                 = gen_dynamic_and_solvers(S);            
    S                 = gen_path(S,'infinity');    
    S.path.obstacles  = [];                                           % No obstacles           
    S.numOfNaNs       = 0;
% %     sim = gen_obstacle(sim,5,15,1);
% %     sim = gen_obstacle(sim,13,33,2);
% %     sim = gen_obstacle(sim,24,8,2.5);
%     %        
%     S                 = gen_xend(S);                                % come back to starting point
%     S                 = gen_constraints(S);

end

function S = gen_init_conditions(S)
    S           = gen_x0(S);
    S           = gen_x0bar(S);
    %
    S           = init_mhe(S);
    S           = init_mpc(S);
    S           = init_LABC(S);
    S           = init_LAOBC(S);
    S           = init_LABC_SAT(S);
    S           = init_mpcCasadi(S);
    S           = init_mpcIntCasadi(S);
end

function S = create_algorithms(S)
    % Init MHE and MPC ****************************************************
    S = init_mhe(S);
    S = init_mpc(S);
end

function S = compute_init_reference(S)
    new_pos_tgtXY0      = S.path.coordinates(:,2);
    dx0                 = S.path.coordinates(1,2)-S.path.coordinates(1,1);
    dy0                 = S.path.coordinates(2,2)-S.path.coordinates(2,1);        
    new_angle_tgtXY0    = atan2c(dx0, dy0, 0);    
    %    
    z_pos               = 0.29;
    S.path.tgt = [new_pos_tgtXY0; z_pos; new_angle_tgtXY0];
    % Compute intial coeffcients of the rect ______________________________
    ux0                  = S.path.coordinates(1,2) - S.path.coordinates(1,1);
    uy0                  = S.path.coordinates(2,2) - S.path.coordinates(2,1);
    if uy0~=0
        vx0              = 1;
        vy0              = -ux0/uy0;
    else
        vy0              = 1;
        vx0              = 0;
    end                        
    v_norm0              = norm([vx0, vy0]);
    vx0                  = vx0/v_norm0;
    vy0                  = vy0/v_norm0;
    A0                   = vy0;
    B0                   = -1*vx0;
    C0                   = S.path.coordinates(2,2)*vx0 - S.path.coordinates(1,2)*vy0;
    norm_AB0             = sqrt(A0^2 + B0^2);
    %
    uxN                  = S.path.coordinates(1,2) - S.path.coordinates(1,1);
    uyN                  = S.path.coordinates(2,2) - S.path.coordinates(2,1);    
    if uyN~=0
        vxN              = 1;
        vyN              = -uxN/uyN;
    else
        vyN              = 1;
        vxN              = 0;
    end                        
    v_normN              = norm([vxN, vyN]);
    vxN                  = vxN/v_normN;
    vyN                  = vyN/v_normN;
    AN                   = vyN;
    BN                   = -1*vxN;
    CN                   = S.path.coordinates(2,2)*vx0 - S.path.coordinates(1,2)*vy0;
    norm_ABN             = sqrt(AN^2 + BN^2);
    %
    S.path.rect.mhe_mpc.A0 = A0;
    S.path.rect.mhe_mpc.B0 = B0;
    S.path.rect.mhe_mpc.C0 = C0;
    S.path.rect.mhe_mpc.norm_AB0 = norm_AB0;
    S.path.rect.mhe_mpc.ux0 = ux0;
    S.path.rect.mhe_mpc.uy0 = uy0;
    %
    S.path.rect.mhe_mpc.AN = AN;
    S.path.rect.mhe_mpc.BN = BN;
    S.path.rect.mhe_mpc.CN = CN;
    S.path.rect.mhe_mpc.norm_ABN = norm_ABN;
    S.path.rect.mhe_mpc.uxN = uxN;
    S.path.rect.mhe_mpc.uyN = uyN;

    S.path.tg_rect.A0        = A0;
    S.path.tg_rect.B0        = B0;
    S.path.tg_rect.C0        = C0;
    S.path.tg_rect.norm_AB0  = norm_AB0;
    S.path.tg_rect.ux0       = ux0;
    S.path.tg_rect.uy0       = uy0;
    %
    S.path.tg_rect.AN        = AN;
    S.path.tg_rect.BN        = BN;
    S.path.tg_rect.CN        = CN;
    S.path.tg_rect.norm_ABN  = norm_ABN;
    S.path.tg_rect.uxN       = uxN;
    S.path.tg_rect.uyN       = uyN;
end

function S = init_flags_and_counters(S)
    S.path.reach_end_mhempc       = false;    
    %
    S.config.time     = 0;
    S.config.iters    = 0;
end

function S = reserve_notemp_memory(S)
    S.data.mhempc.performance           = struct;
    S.data.mhempc.performance.Psi_e     = [];
    S.data.mhempc.performance.Psi_e_vec = [];
    S.data.mhempc.performance.maxPsi_e  = [];
    S.data.mhempc.performance.Psi_u     = [];    
    S.data.mhempc.performance.Psi_u_vec = [];    
    S.data.mhempc.performance.maxPsi_u  = [];    
    S.data.mhempc.performance.time_tot  = [];
    S.data.mhempc.performance.est_err   = [];
    %    
    S.data.numBatch                     = 1;
end

function S = reserve_temp_memory(S)
    % Variables: states, outputs, etc. ____________________________________
    S.path.acumIndx               = 1;
    %
    S.data.xsim                   = [];
    S.data.ysim_mhempc            = [];
    %
    S.data.slip                   = [];
    S.data.slipest_mhempc         = [];
    %
    S.data.meas_noise             = [];
    %
    S.data.test                   = [];
    S.algorithms.LAOBC.w_slack    = [];
    %                        
    S.data.xest                   = [];    
    S.data.Ctrls                  = [];
    %                        
    S.data.time_mhempc            = [];    
    %
    S                             = compute_init_reference(S);
    S.path.references             = S.path.tgt;    
    %
    S.path.reach_end_mhempc       = false;        
    %
    S.path.indices_tgt            = [];
    %
    S.path.vel_tgt                = [];
    S.path.angle_tgt              = [];
    %
    % auxiliar indice shared by all algorithms
    S.path.indx_alg               = [1;1];
    S.path.uref                   = [];
    S.path.last_tgt               = 0;
    %   
    S.exec_time.t_mhe             = [];
    S.exec_time.t_pfa             = [];
    S.exec_time.t_mpc             = -inf;
    S.exec_time.t_labc            = -inf;
    S.exec_time.t_laobc           = -inf;
    S.exec_time.t_sensors         = [];
    S.exec_time.t_ctrl            = 0;
    S.exec_time.t_tot             = [];
    S.exec_time.t_acum            = 0;
    S.exec_time.t_mis             = 0;
    %
    S.debug.flags_mhe             = [];
    S.debug.flags_mpc             = [];
    S.debug.flags_index           = [];
    %
    S.init_flag.mhe               = false;
    S.init_flag.mpc               = false;
    S.init_flag.pfa               = false;
    %
    S.simulation_input.u          = zeros(S.system.nu,1);
    S.simulation_input.x          = zeros(S.system.nq,1);
    %
    S.config.countToFinish        = 0;
    S.config.valToFinish          = 2/S.config.Ts;
end

function S = gen_x0(S)
    % DO NOT FORGET CHECKING FEASIBILITY OF INITIAL CONDITION!!!
    Dx                  = S.path.coordinates(1,2)-S.path.coordinates(1,1);
    Dy                  = S.path.coordinates(2,2)-S.path.coordinates(2,1);
    theta0              = atan2(Dy,Dx);    
    xy_0                = S.path.coordinates(:,1);
    z_pos               = 0.29;
    %    
    S.init_condition.x0       = [ xy_0; z_pos; theta0 ] + [0.5*randn; 0.5*randn; 0; randn];
    %
    S.data.xsim(:,1)   = S.init_condition.x0;%[S.init_condition.x0; [1;1]];    
end

function S = gen_x0bar(S)
    %       
    if S.config.SIM
        S.init_condition.x0bar = S.init_condition.x0 + S.config.noise_lvl.*randn(size(S.config.noise_lvl));
    else    
        S                      = read_sensors(S);
        S                      = measurements_vector(S);

        S.init_condition.x0bar = [ S.ROS.sensors.rtk.xN; 
                                   S.ROS.sensors.rtk.yN; 
                                   0.29;
                                   S.ROS.sensors.vectornav_theta2 ];            

    end    
% ####    
S.data.xest     = S.init_condition.x0bar;
S.angle0_old    = S.init_condition.x0bar(end);
% ####
end
