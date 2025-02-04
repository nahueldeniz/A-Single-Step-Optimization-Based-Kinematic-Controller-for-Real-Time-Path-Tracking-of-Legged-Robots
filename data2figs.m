
clear all;

labcB                    = struct;
labcB.traj.circ          = {};
labcB.traj.circ.dev      = [];
labcB.traj.circ.cb       = [];
labcB.traj.circ.ce       = [];
labcB.traj.square        = {};
labcB.traj.square.dev    = [];
labcB.traj.square.cb     = [];
labcB.traj.square.ce     = [];
labcB.traj.infty         = {};
labcB.traj.infty.dev     = [];
labcB.traj.infty.cb      = [];
labcB.traj.infty.ce      = [];

laobcB                   = struct;
laobcB.traj.circ         = {};
laobcB.traj.circ.dev     = [];
laobcB.traj.circ.cb      = [];
laobcB.traj.circ.ce      = [];
laobcB.traj.square       = {};
laobcB.traj.square.dev   = [];
laobcB.traj.square.cb    = [];
laobcB.traj.square.ce    = [];
laobcB.traj.infty        = {};
laobcB.traj.infty.dev    = [];
laobcB.traj.infty.cb     = [];
laobcB.traj.infty.ce     = [];

laobcAccelB                   = struct;
laobcAccelB.traj.circ         = {};
laobcAccelB.traj.circ.dev     = [];
laobcAccelB.traj.circ.cb      = [];
laobcAccelB.traj.circ.ce      = [];
laobcAccelB.traj.square       = {};
laobcAccelB.traj.square.dev   = [];
laobcAccelB.traj.square.cb    = [];
laobcAccelB.traj.square.ce    = [];
laobcAccelB.traj.infty        = {};
laobcAccelB.traj.infty.dev    = [];
laobcAccelB.traj.infty.cb     = [];
laobcAccelB.traj.infty.ce     = [];

mpc1B                    = struct;
mpc1B.traj.circ          = {};
mpc1B.traj.circ.dev      = [];
mpc1B.traj.circ.cb       = [];
mpc1B.traj.circ.ce       = [];
mpc1B.traj.square        = {};
mpc1B.traj.square.dev    = [];
mpc1B.traj.square.cb     = [];
mpc1B.traj.square.ce     = [];
mpc1B.traj.infty         = {};
mpc1B.traj.infty.dev     = [];
mpc1B.traj.infty.cb      = [];
mpc1B.traj.infty.ce      = [];

mpc2B                    = struct;
mpc2B.traj.circ          = {};
mpc2B.traj.circ.dev      = [];
mpc2B.traj.circ.cb       = [];
mpc2B.traj.circ.ce       = [];
mpc2B.traj.square        = {};
mpc2B.traj.square.dev    = [];
mpc2B.traj.square.cb     = [];
mpc2B.traj.square.ce     = [];
mpc2B.traj.infty         = {};
mpc2B.traj.infty.dev     = [];
mpc2B.traj.infty.cb      = [];
mpc2B.traj.infty.ce      = [];

mpc3B                    = struct;
mpc3B.traj.circ          = {};
mpc3B.traj.circ.dev      = [];
mpc3B.traj.circ.cb       = [];
mpc3B.traj.circ.ce       = [];
mpc3B.traj.square        = {};
mpc3B.traj.square.dev    = [];
mpc3B.traj.square.cb     = [];
mpc3B.traj.square.ce     = [];
mpc3B.traj.infty         = {};
mpc3B.traj.infty.dev     = [];
mpc3B.traj.infty.cb      = [];
mpc3B.traj.infty.ce      = [];


precision = '%0.2g';
precision_ce = '%0.4g';
exps = [9:12];
warning('off');
for countExps=1:size(exps,1)
    tableDispCirc           = {};
    tableDispSqua           = {repmat(' & ',10,1)};
    tableDispInft           = {repmat(' & ',10,1)};
    
    for counterI=exps(countExps,:)        
        filename = strcat(['/home/nahuel/Dropbox/PosDoc/AC3E/Go1 MPC/Simulaciones/sims_HUFA/labc_', num2str(counterI),'.mat']);
        clear S;
        load(filename);
        for jj=1:10
            meanDev = [];
            compBur = [];
            ctrlEff = [];
            for kk=1:10
                meanDev = [meanDev, mean(S.data.performance.deviation{(jj-1)*10+kk}(:))];
                compBur = [compBur, mean(S.data.performance.compBurden{(jj-1)*10+kk}(:))/S.config.Ts];
                ctrlEff = [ctrlEff, mean(sqrt(sum(S.data.performance.controls{(jj-1)*10+kk}(1,:).^2)) + sqrt(sum(S.data.performance.controls{(jj-1)*10+kk}(2,:).^2)) + sqrt(sum(S.data.performance.controls{(jj-1)*10+kk}(3,:).^2)))];            
            end
            if counterI==1 || counterI==2 || counterI==3 || counterI==4
                labcB.traj.circ.dev      = [labcB.traj.circ.dev; mean(meanDev)];
                labcB.traj.circ.cb       = [labcB.traj.circ.cb; mean(compBur)];
                labcB.traj.circ.ce       = [labcB.traj.circ.ce; mean(ctrlEff)];
            elseif counterI==5 || counterI==6 || counterI==7 || counterI==8
                labcB.traj.square.dev    = [labcB.traj.square.dev; mean(meanDev)];
                labcB.traj.square.cb     = [labcB.traj.square.cb; mean(compBur)];
                labcB.traj.square.ce     = [labcB.traj.square.ce; mean(ctrlEff)];
            else
                labcB.traj.infty.dev     = [labcB.traj.infty.dev; mean(meanDev)];
                labcB.traj.infty.cb      = [labcB.traj.infty.cb; mean(compBur)];
                labcB.traj.infty.ce      = [labcB.traj.infty.ce; mean(ctrlEff)];
            end
            a=1;
        end
%         if counterI==exps(countExps,1)
%             tableDispCirc = {tableDispCirc{:},  num2str(labcB.traj.circ.dev,precision), repmat(' & ',10,1), num2str(labcB.traj.circ.cb,precision), repmat(' & ',10,1), num2str(labcB.traj.circ.ce,precision_ce)};
%         elseif counterI==exps(countExps,2)
%             tableDispSqua = {tableDispSqua{:}, num2str(labcB.traj.square.dev,precision), repmat(' & ',10,1), num2str(labcB.traj.square.cb,precision), repmat(' & ',10,1), num2str(labcB.traj.square.ce,precision_ce), repmat(' & ',10,1)};
%         else
%             tableDispInft = {tableDispInft{:}, num2str(labcB.traj.infty.dev,precision), repmat(' & ',10,1), num2str(labcB.traj.infty.cb,precision), repmat(' & ',10,1), num2str(labcB.traj.infty.ce,precision_ce), repmat(' & ',10,1)};
%         end
        %
        filename = strcat(['/home/nahuel/Dropbox/PosDoc/AC3E/Go1 MPC/Simulaciones/sims_HUFA/laobc_', num2str(counterI),'.mat']);
        clear S;
        load(filename);
        for jj=1:10
            meanDev = [];
            compBur = [];
            ctrlEff = [];
            for kk=1:10
                meanDev = [meanDev, mean(S.data.performance.deviation{(jj-1)*10+kk}(:))];
                compBur = [compBur, mean(S.data.performance.compBurden{(jj-1)*10+kk}(:))/S.config.Ts];
                ctrlEff = [ctrlEff, mean(sqrt(sum(S.data.performance.controls{(jj-1)*10+kk}(1,:).^2)) + sqrt(sum(S.data.performance.controls{(jj-1)*10+kk}(2,:).^2)) + sqrt(sum(S.data.performance.controls{(jj-1)*10+kk}(3,:).^2)))];            
            end
            if counterI==1 || counterI==2 || counterI==3 || counterI==4
                laobcB.traj.circ.dev      = [laobcB.traj.circ.dev; mean(meanDev)];
                laobcB.traj.circ.cb       = [laobcB.traj.circ.cb; mean(compBur)];
                laobcB.traj.circ.ce       = [laobcB.traj.circ.ce; mean(ctrlEff)];
            elseif counterI==5 || counterI==6 || counterI==7 || counterI==8
                laobcB.traj.square.dev    = [laobcB.traj.square.dev; mean(meanDev)];
                laobcB.traj.square.cb     = [laobcB.traj.square.cb; mean(compBur)];
                laobcB.traj.square.ce     = [laobcB.traj.square.ce; mean(ctrlEff)];
            else
                laobcB.traj.infty.dev     = [laobcB.traj.infty.dev; mean(meanDev)];
                laobcB.traj.infty.cb      = [laobcB.traj.infty.cb; mean(compBur)];
                laobcB.traj.infty.ce      = [laobcB.traj.infty.ce; mean(ctrlEff)];
            end
        end
%         if counterI==exps(countExps,1)
%             tableDispCirc = {tableDispCirc{:},  num2str(laobcB.traj.circ.dev,precision), repmat(' & ',10,1), num2str(laobcB.traj.circ.cb,precision), repmat(' & ',10,1), num2str(laobcB.traj.circ.ce,precision_ce)};
%         elseif counterI==exps(countExps,2)
%             tableDispSqua = {tableDispSqua{:}, num2str(laobcB.traj.square.dev,precision), repmat(' & ',10,1), num2str(laobcB.traj.square.cb,precision), repmat(' & ',10,1), num2str(laobcB.traj.square.ce,precision_ce), repmat(' & ',10,1)};
%         else
%             tableDispInft = {tableDispInft{:}, num2str(laobcB.traj.infty.dev,precision), repmat(' & ',10,1), num2str(laobcB.traj.infty.cb,precision), repmat(' & ',10,1), num2str(laobcB.traj.infty.ce,precision_ce), repmat(' & ',10,1)};
%         end
        %
        filename = strcat(['/home/nahuel/Dropbox/PosDoc/AC3E/Go1 MPC/Simulaciones/sims_HUFA/laobcAcel_', num2str(counterI),'.mat']);
        clear S;
        load(filename);
        for jj=1:10
            meanDev = [];
            compBur = [];
            ctrlEff = [];
            for kk=1:10
                meanDev = [meanDev, mean(S.data.performance.deviation{(jj-1)*10+kk}(:))];
                compBur = [compBur, mean(S.data.performance.compBurden{(jj-1)*10+kk}(:))/S.config.Ts];
                ctrlEff = [ctrlEff, mean(sqrt(sum(S.data.performance.controls{(jj-1)*10+kk}(1,:).^2)) + sqrt(sum(S.data.performance.controls{(jj-1)*10+kk}(2,:).^2)) + sqrt(sum(S.data.performance.controls{(jj-1)*10+kk}(3,:).^2)))];            
            end
            if counterI==1 || counterI==2 || counterI==3 || counterI==4
                laobcAccelB.traj.circ.dev      = [laobcAccelB.traj.circ.dev; mean(meanDev)];
                laobcAccelB.traj.circ.cb       = [laobcAccelB.traj.circ.cb; mean(compBur)];
                laobcAccelB.traj.circ.ce       = [laobcAccelB.traj.circ.ce; mean(ctrlEff)];
            elseif counterI==5 || counterI==6 || counterI==7 || counterI==8
                laobcAccelB.traj.square.dev    = [laobcAccelB.traj.square.dev; mean(meanDev)];
                laobcAccelB.traj.square.cb     = [laobcAccelB.traj.square.cb; mean(compBur)];
                laobcAccelB.traj.square.ce     = [laobcAccelB.traj.square.ce; mean(ctrlEff)];
            else
                laobcAccelB.traj.infty.dev     = [laobcAccelB.traj.infty.dev; mean(meanDev)];
                laobcAccelB.traj.infty.cb      = [laobcAccelB.traj.infty.cb; mean(compBur)];
                laobcAccelB.traj.infty.ce      = [laobcAccelB.traj.infty.ce; mean(ctrlEff)];
            end
        end
%         if counterI==exps(countExps,1)
%             tableDispCirc = {tableDispCirc{:},  num2str(laobcB.traj.circ.dev,precision), repmat(' & ',10,1), num2str(laobcB.traj.circ.cb,precision), repmat(' & ',10,1), num2str(laobcB.traj.circ.ce,precision_ce)};
%         elseif counterI==exps(countExps,2)
%             tableDispSqua = {tableDispSqua{:}, num2str(laobcB.traj.square.dev,precision), repmat(' & ',10,1), num2str(laobcB.traj.square.cb,precision), repmat(' & ',10,1), num2str(laobcB.traj.square.ce,precision_ce), repmat(' & ',10,1)};
%         else
%             tableDispInft = {tableDispInft{:}, num2str(laobcB.traj.infty.dev,precision), repmat(' & ',10,1), num2str(laobcB.traj.infty.cb,precision), repmat(' & ',10,1), num2str(laobcB.traj.infty.ce,precision_ce), repmat(' & ',10,1)};
%         end
        %
        filename = strcat(['/home/nahuel/Dropbox/PosDoc/AC3E/Go1 MPC/Simulaciones/sims_HUFA/mpc_', num2str(counterI),'_N20','.mat']);
        clear S;
        load(filename);
        for jj=1:10
            meanDev = [];
            compBur = [];
            ctrlEff = [];
            for kk=1:10
                meanDev = [meanDev, mean(S.data.performance.deviation{(jj-1)*10+kk}(:))];
                compBur = [compBur, mean(S.data.performance.compBurden{(jj-1)*10+kk}(:))/S.config.Ts];
                ctrlEff = [ctrlEff, mean(sqrt(sum(S.data.performance.controls{(jj-1)*10+kk}(1,:).^2)) + sqrt(sum(S.data.performance.controls{(jj-1)*10+kk}(2,:).^2)) + sqrt(sum(S.data.performance.controls{(jj-1)*10+kk}(3,:).^2)))];            
            end
            if counterI==1 || counterI==2 || counterI==3 || counterI==4
                mpc1B.traj.circ.dev      = [mpc1B.traj.circ.dev; mean(meanDev)];
                mpc1B.traj.circ.cb       = [mpc1B.traj.circ.cb; mean(compBur)];
                mpc1B.traj.circ.ce       = [mpc1B.traj.circ.ce; mean(ctrlEff)];
            elseif counterI==5 || counterI==6 || counterI==7 || counterI==8
                mpc1B.traj.square.dev    = [mpc1B.traj.square.dev; mean(meanDev)];
                mpc1B.traj.square.cb     = [mpc1B.traj.square.cb; mean(compBur)];
                mpc1B.traj.square.ce     = [mpc1B.traj.square.ce; mean(ctrlEff)];
            else
                mpc1B.traj.infty.dev     = [mpc1B.traj.infty.dev; mean(meanDev)];
                mpc1B.traj.infty.cb      = [mpc1B.traj.infty.cb; mean(compBur)];
                mpc1B.traj.infty.ce      = [mpc1B.traj.infty.ce; mean(ctrlEff)];
            end
        end
%         if counterI==exps(countExps,1)
%             tableDispCirc = {tableDispCirc{:}, repmat(' & ',10,1), num2str(mpc1B.traj.circ.dev,precision), repmat(' & ',10,1), num2str(mpc1B.traj.circ.cb,precision), repmat(' & ',10,1), num2str(mpc1B.traj.circ.ce,precision_ce)};
%         elseif counterI==exps(countExps,2)
%             tableDispSqua = {tableDispSqua{:}, num2str(mpc1B.traj.square.dev,precision), repmat(' & ',10,1), num2str(mpc1B.traj.square.cb,precision), repmat(' & ',10,1), num2str(mpc1B.traj.square.ce,precision_ce), repmat(' & ',10,1)};
%         else
%             tableDispInft = {tableDispInft{:}, num2str(mpc1B.traj.infty.dev,precision), repmat(' & ',10,1), num2str(mpc1B.traj.infty.cb,precision), repmat(' & ',10,1), num2str(mpc1B.traj.infty.ce,precision_ce), repmat(' & ',10,1)};
%         end
        %
        filename = strcat(['/home/nahuel/Dropbox/PosDoc/AC3E/Go1 MPC/Simulaciones/sims_HUFA/mpc_', num2str(counterI),'_N35','.mat']);
        clear S;
        load(filename);
        for jj=1:10
            meanDev = [];
            compBur = [];
            ctrlEff = [];
            for kk=1:10
                meanDev = [meanDev, mean(S.data.performance.deviation{(jj-1)*10+kk}(:))];
                compBur = [compBur, mean(S.data.performance.compBurden{(jj-1)*10+kk}(:))/S.config.Ts];
                ctrlEff = [ctrlEff, mean(sqrt(sum(S.data.performance.controls{(jj-1)*10+kk}(1,:).^2)) + sqrt(sum(S.data.performance.controls{(jj-1)*10+kk}(2,:).^2)) + sqrt(sum(S.data.performance.controls{(jj-1)*10+kk}(3,:).^2)))];            
            end
            if counterI==1 || counterI==2 || counterI==3 || counterI==4
                mpc2B.traj.circ.dev      = [mpc2B.traj.circ.dev; mean(meanDev)];
                mpc2B.traj.circ.cb       = [mpc2B.traj.circ.cb; mean(compBur)];
                mpc2B.traj.circ.ce       = [mpc2B.traj.circ.ce; mean(ctrlEff)];
            elseif counterI==5 || counterI==6 || counterI==7 || counterI==8
                mpc2B.traj.square.dev    = [mpc2B.traj.square.dev; mean(meanDev)];
                mpc2B.traj.square.cb     = [mpc2B.traj.square.cb; mean(compBur)];
                mpc2B.traj.square.ce     = [mpc2B.traj.square.ce; mean(ctrlEff)];
            else
                mpc2B.traj.infty.dev     = [mpc2B.traj.infty.dev; mean(meanDev)];
                mpc2B.traj.infty.cb      = [mpc2B.traj.infty.cb; mean(compBur)];
                mpc2B.traj.infty.ce      = [mpc2B.traj.infty.ce; mean(ctrlEff)];
            end
        end
%         if counterI==exps(countExps,1)
%             tableDispCirc = {tableDispCirc{:}, repmat(' & ',10,1), num2str(mpc2B.traj.circ.dev,precision), repmat(' & ',10,1), num2str(mpc2B.traj.circ.cb,precision), repmat(' & ',10,1), num2str(mpc2B.traj.circ.ce,precision_ce)};
%         elseif counterI==exps(countExps,2)
%             tableDispSqua = {tableDispSqua{:}, num2str(mpc2B.traj.square.dev,precision), repmat(' & ',10,1), num2str(mpc2B.traj.square.cb,precision), repmat(' & ',10,1), num2str(mpc2B.traj.square.ce,precision_ce)};
%         else
%             tableDispInft = {tableDispInft{:}, num2str(mpc2B.traj.infty.dev,precision), repmat(' & ',10,1), num2str(mpc2B.traj.infty.cb,precision), repmat(' & ',10,1), num2str(mpc2B.traj.infty.ce,precision_ce), repmat(' \\ ',10,1)};
%         end
        %
        filename = strcat(['/home/nahuel/Dropbox/PosDoc/AC3E/Go1 MPC/Simulaciones/sims_HUFA/mpc_', num2str(counterI),'_N1','.mat']);
        clear S;
        load(filename);
        for jj=1:10
            meanDev = [];
            compBur = [];
            ctrlEff = [];
            for kk=1:10
                meanDev = [meanDev, mean(S.data.performance.deviation{(jj-1)*10+kk}(:))];
                compBur = [compBur, mean(S.data.performance.compBurden{(jj-1)*10+kk}(:))/S.config.Ts];
                ctrlEff = [ctrlEff, mean(sqrt(sum(S.data.performance.controls{(jj-1)*10+kk}(1,:).^2)) + sqrt(sum(S.data.performance.controls{(jj-1)*10+kk}(2,:).^2)) + sqrt(sum(S.data.performance.controls{(jj-1)*10+kk}(3,:).^2)))];            
            end
            if counterI==1 || counterI==2 || counterI==3 || counterI==4
                mpc3B.traj.circ.dev      = [mpc3B.traj.circ.dev; mean(meanDev)];
                mpc3B.traj.circ.cb       = [mpc3B.traj.circ.cb; mean(compBur)];
                mpc3B.traj.circ.ce       = [mpc3B.traj.circ.ce; mean(ctrlEff)];
            elseif counterI==5 || counterI==6 || counterI==7 || counterI==8
                mpc3B.traj.square.dev    = [mpc3B.traj.square.dev; mean(meanDev)];
                mpc3B.traj.square.cb     = [mpc3B.traj.square.cb; mean(compBur)];
                mpc3B.traj.square.ce     = [mpc3B.traj.square.ce; mean(ctrlEff)];
            else
                mpc3B.traj.infty.dev     = [mpc3B.traj.infty.dev; mean(meanDev)];
                mpc3B.traj.infty.cb      = [mpc3B.traj.infty.cb; mean(compBur)];
                mpc3B.traj.infty.ce      = [mpc3B.traj.infty.ce; mean(ctrlEff)];
            end
        end
%         if counterI==exps(countExps,1)
%             tableDispCirc = {tableDispCirc{:}, repmat(' & ',10,1), num2str(mpc3B.traj.circ.dev,precision), repmat(' & ',10,1), num2str(mpc3B.traj.circ.cb,precision), repmat(' & ',10,1), num2str(mpc3B.traj.circ.ce,precision_ce)};
%         elseif counterI==exps(countExps,2)
%             tableDispSqua = {tableDispSqua{:}, num2str(mpc3B.traj.square.dev,precision), repmat(' & ',10,1), num2str(mpc3B.traj.square.cb,precision), repmat(' & ',10,1), num2str(mpc3B.traj.square.ce,precision_ce)};
%         else
%             tableDispInft = {tableDispInft{:}, num2str(mpc3B.traj.infty.dev,precision), repmat(' & ',10,1), num2str(mpc3B.traj.infty.cb,precision), repmat(' & ',10,1), num2str(mpc3B.traj.infty.ce,precision_ce), repmat(' \\ ',10,1)};
%         end
    end
    % do some plots    
    if counterI==1 || counterI==2 || counterI==3 || counterI==4
              
        vels        = linspace(0.1,1,10);
        labc_clr    = 'm';
        laobc_clr   = 'c';
        mpc1_clr    = 'b';
        mpc2_clr    = 'r';
        alpha_val   = 0.6;
        line_width  = 2;
        font_size1  = 25;
        % no noises 
        figure; hold on; grid on;
        boxplot([labcB.traj.circ.ce(1:10),laobcB.traj.circ.ce(1:10),laobcAccelB.traj.circ.ce(1:10), mpc1B.traj.circ.ce(1:10), mpc2B.traj.circ.ce(1:10), mpc3B.traj.circ.ce(1:10) ],'Labels',{'labc','laobc','laobcAcel','mpc_1','mpc_2','mpc_3'})
        ylabel({'$ctrl.\,effort$'},'interpreter','latex','fontsize',19); exportgraphics(gcf,'circ-no-noise-ce.eps','contenttype','vector')
        % disturbance process
        figure; hold on; grid on;
        boxplot([labcB.traj.circ.ce(11:20),laobcB.traj.circ.ce(11:20),laobcAccelB.traj.circ.ce(11:20), mpc1B.traj.circ.ce(11:20), mpc2B.traj.circ.ce(11:20), mpc3B.traj.circ.ce(11:20) ],'Labels',{'labc','laobc','laobcAcel','mpc_1','mpc_2','mpc_3'})
        ylabel({'$ctrl.\,effort$'},'interpreter','latex','fontsize',19); exportgraphics(gcf,'circ-proc-dist-ce.eps','contenttype','vector')
        % measurements noise
        figure; hold on; grid on;
        boxplot([labcB.traj.circ.ce(21:30),laobcB.traj.circ.ce(21:30),laobcAccelB.traj.circ.ce(21:30), mpc1B.traj.circ.ce(21:30), mpc2B.traj.circ.ce(21:30), mpc3B.traj.circ.ce(21:30) ],'Labels',{'labc','laobc','laobcAcel','mpc_1','mpc_2','mpc_3'})
        ylabel({'$ctrl.\,effort$'},'interpreter','latex','fontsize',19); exportgraphics(gcf,'circ-meas-noise-ce.eps','contenttype','vector')
        % measurements noise
        figure; hold on; grid on;
        boxplot([labcB.traj.circ.ce(31:40),laobcB.traj.circ.ce(31:40),laobcAccelB.traj.circ.ce(31:40), mpc1B.traj.circ.ce(31:40), mpc2B.traj.circ.ce(31:40), mpc3B.traj.circ.ce(31:40) ],'Labels',{'labc','laobc','laobcAcel','mpc_1','mpc_2','mpc_3'})
        ylabel({'$ctrl.\,effort$'},'interpreter','latex','fontsize',19); exportgraphics(gcf,'circ-both-noises-ce.eps','contenttype','vector')
        % no noises 
        figure; hold on; grid on;
        boxplot([labcB.traj.circ.dev(1:10),laobcB.traj.circ.dev(1:10),laobcAccelB.traj.circ.dev(1:10), mpc1B.traj.circ.dev(1:10), mpc2B.traj.circ.dev(1:10), mpc3B.traj.circ.dev(1:10) ],'Labels',{'labc','laobc','laobcAcel','mpc_1','mpc_2','mpc_3'})
        ylabel({'$dev.\,(m)$'},'interpreter','latex','fontsize',19); exportgraphics(gcf,'circ-no-noise-dev.eps','contenttype','vector')
        % disturbance process
        figure; hold on; grid on;
        boxplot([labcB.traj.circ.dev(11:20),laobcB.traj.circ.dev(11:20),laobcAccelB.traj.circ.dev(11:20), mpc1B.traj.circ.dev(11:20), mpc2B.traj.circ.dev(11:20), mpc3B.traj.circ.dev(11:20) ],'Labels',{'labc','laobc','laobcAcel','mpc_1','mpc_2','mpc_3'})
        ylabel({'$dev.\,(m)$'},'interpreter','latex','fontsize',19); exportgraphics(gcf,'circ-proc-dist-dev.eps','contenttype','vector')
        % measurements noise
        figure; hold on; grid on;
        boxplot([labcB.traj.circ.dev(21:30),laobcB.traj.circ.dev(21:30),laobcAccelB.traj.circ.dev(21:30), mpc1B.traj.circ.dev(21:30), mpc2B.traj.circ.dev(21:30), mpc3B.traj.circ.dev(21:30) ],'Labels',{'labc','laobc','laobcAcel','mpc_1','mpc_2','mpc_3'})
        ylabel({'$dev.\,(m)$'},'interpreter','latex','fontsize',19); exportgraphics(gcf,'circ-meas-noise-dev.eps','contenttype','vector')
        % measurements noise
        figure; hold on; grid on;
        boxplot([labcB.traj.circ.dev(31:40),laobcB.traj.circ.dev(31:40),laobcAccelB.traj.circ.dev(31:40), mpc1B.traj.circ.dev(31:40), mpc2B.traj.circ.dev(31:40), mpc3B.traj.circ.dev(31:40) ],'Labels',{'labc','laobc','laobcAcel','mpc_1','mpc_2','mpc_3'})
        ylabel({'$dev.\,(m)$'},'interpreter','latex','fontsize',19); exportgraphics(gcf,'circ-both-noises-dev.eps','contenttype','vector')
        % no noises 
        figure; hold on; grid on;
        boxplot([labcB.traj.circ.cb(1:10),laobcB.traj.circ.cb(1:10),laobcAccelB.traj.circ.cb(1:10), mpc1B.traj.circ.cb(1:10), mpc2B.traj.circ.cb(1:10), mpc3B.traj.circ.cb(1:10) ],'Labels',{'labc','laobc','laobcAcel','mpc_1','mpc_2','mpc_3'})
        ylabel({'$rtf$'},'interpreter','latex','fontsize',19); exportgraphics(gcf,'circ-no-noise-rtf.eps','contenttype','vector')
        % disturbance process
        figure; hold on; grid on;
        boxplot([labcB.traj.circ.cb(11:20),laobcB.traj.circ.cb(11:20),laobcAccelB.traj.circ.cb(11:20), mpc1B.traj.circ.cb(11:20), mpc2B.traj.circ.cb(11:20), mpc3B.traj.circ.cb(11:20) ],'Labels',{'labc','laobc','laobcAcel','mpc_1','mpc_2','mpc_3'})
        ylabel({'$rtf$'},'interpreter','latex','fontsize',19); exportgraphics(gcf,'circ-proc-dist-rtf.eps','contenttype','vector')
        % measurements noise
        figure; hold on; grid on;
        boxplot([labcB.traj.circ.cb(21:30),laobcB.traj.circ.cb(21:30),laobcAccelB.traj.circ.cb(21:30), mpc1B.traj.circ.cb(21:30), mpc2B.traj.circ.cb(21:30), mpc3B.traj.circ.cb(21:30) ],'Labels',{'labc','laobc','laobcAcel','mpc_1','mpc_2','mpc_3'})
        ylabel({'$rtf$'},'interpreter','latex','fontsize',19); exportgraphics(gcf,'circ-meas-noise-rtf.eps','contenttype','vector')
        % measurements noise
        figure; hold on; grid on;
        boxplot([labcB.traj.circ.cb(31:40),laobcB.traj.circ.cb(31:40),laobcAccelB.traj.circ.cb(31:40), mpc1B.traj.circ.cb(31:40), mpc2B.traj.circ.cb(31:40), mpc3B.traj.circ.cb(31:40) ],'Labels',{'labc','laobc','laobcAcel','mpc_1','mpc_2','mpc_3'})
        ylabel({'$rtf$'},'interpreter','latex','fontsize',19); exportgraphics(gcf,'circ-both-noises-rtf.eps','contenttype','vector')
    elseif counterI==5 || counterI==6 || counterI==7 || counterI==8
        vels        = linspace(0.1,1,10);
        labc_clr    = 'm';
        laobc_clr   = 'c';
        mpc1_clr    = 'b';
        mpc2_clr    = 'r';
        alpha_val   = 0.6;
        line_width  = 2;
        font_size1  = 25;
        % no noises 
        figure; hold on; grid on;
        boxplot([labcB.traj.square.ce(1:10),laobcB.traj.square.ce(1:10),laobcAccelB.traj.square.ce(1:10), mpc1B.traj.square.ce(1:10), mpc2B.traj.square.ce(1:10), mpc3B.traj.square.ce(1:10) ],'Labels',{'labc','laobc','laobcAcel','mpc_1','mpc_2','mpc_3'})
        ylabel({'$ctrl.\,effort$'},'interpreter','latex','fontsize',19); exportgraphics(gcf,'square-no-noise-ce.eps','contenttype','vector')
        % disturbance process
        figure; hold on; grid on;
        boxplot([labcB.traj.square.ce(11:20),laobcB.traj.square.ce(11:20),laobcAccelB.traj.square.ce(11:20), mpc1B.traj.square.ce(11:20), mpc2B.traj.square.ce(11:20), mpc3B.traj.square.ce(11:20) ],'Labels',{'labc','laobc','laobcAcel','mpc_1','mpc_2','mpc_3'})
        ylabel({'$ctrl.\,effort$'},'interpreter','latex','fontsize',19); exportgraphics(gcf,'square-proc-dist-ce.eps','contenttype','vector')
        % measurements noise
        figure; hold on; grid on;
        boxplot([labcB.traj.square.ce(21:30),laobcB.traj.square.ce(21:30),laobcAccelB.traj.square.ce(21:30), mpc1B.traj.square.ce(21:30), mpc2B.traj.square.ce(21:30), mpc3B.traj.square.ce(21:30) ],'Labels',{'labc','laobc','laobcAcel','mpc_1','mpc_2','mpc_3'})
        ylabel({'$ctrl.\,effort$'},'interpreter','latex','fontsize',19); exportgraphics(gcf,'square-meas-noise-ce.eps','contenttype','vector')
        % measurements noise
        figure; hold on; grid on;
        boxplot([labcB.traj.square.ce(31:40),laobcB.traj.square.ce(31:40),laobcAccelB.traj.square.ce(31:40), mpc1B.traj.square.ce(31:40), mpc2B.traj.square.ce(31:40), mpc3B.traj.square.ce(31:40) ],'Labels',{'labc','laobc','laobcAcel','mpc_1','mpc_2','mpc_3'})
        ylabel({'$ctrl.\,effort$'},'interpreter','latex','fontsize',19); exportgraphics(gcf,'square-both-noises-ce.eps','contenttype','vector')
        % no noises 
        figure; hold on; grid on;
        boxplot([labcB.traj.square.dev(1:10),laobcB.traj.square.dev(1:10),laobcAccelB.traj.square.dev(1:10), mpc1B.traj.square.dev(1:10), mpc2B.traj.square.dev(1:10), mpc3B.traj.square.dev(1:10) ],'Labels',{'labc','laobc','laobcAcel','mpc_1','mpc_2','mpc_3'})
        ylabel({'$dev.\,(m)$'},'interpreter','latex','fontsize',19); exportgraphics(gcf,'square-no-noise-dev.eps','contenttype','vector')
        % disturbance process
        figure; hold on; grid on;
        boxplot([labcB.traj.square.dev(11:20),laobcB.traj.square.dev(11:20),laobcAccelB.traj.square.dev(11:20), mpc1B.traj.square.dev(11:20), mpc2B.traj.square.dev(11:20), mpc3B.traj.square.dev(11:20) ],'Labels',{'labc','laobc','laobcAcel','mpc_1','mpc_2','mpc_3'})
        ylabel({'$dev.\,(m)$'},'interpreter','latex','fontsize',19); exportgraphics(gcf,'square-proc-dist-dev.eps','contenttype','vector')
        % measurements noise
        figure; hold on; grid on;
        boxplot([labcB.traj.square.dev(21:30),laobcB.traj.square.dev(21:30),laobcAccelB.traj.square.dev(21:30), mpc1B.traj.square.dev(21:30), mpc2B.traj.square.dev(21:30), mpc3B.traj.square.dev(21:30) ],'Labels',{'labc','laobc','laobcAcel','mpc_1','mpc_2','mpc_3'})
        ylabel({'$dev.\,(m)$'},'interpreter','latex','fontsize',19); exportgraphics(gcf,'square-meas-noise-dev.eps','contenttype','vector')
        % measurements noise
        figure; hold on; grid on;
        boxplot([labcB.traj.square.dev(31:40),laobcB.traj.square.dev(31:40),laobcAccelB.traj.square.dev(31:40), mpc1B.traj.square.dev(31:40), mpc2B.traj.square.dev(31:40), mpc3B.traj.square.dev(31:40) ],'Labels',{'labc','laobc','laobcAcel','mpc_1','mpc_2','mpc_3'})
        ylabel({'$dev.\,(m)$'},'interpreter','latex','fontsize',19); exportgraphics(gcf,'square-both-noises-dev.eps','contenttype','vector')
        % no noises 
        figure; hold on; grid on;
        boxplot([labcB.traj.square.cb(1:10),laobcB.traj.square.cb(1:10),laobcAccelB.traj.square.cb(1:10), mpc1B.traj.square.cb(1:10), mpc2B.traj.square.cb(1:10), mpc3B.traj.square.cb(1:10) ],'Labels',{'labc','laobc','laobcAcel','mpc_1','mpc_2','mpc_3'})
        ylabel({'$rtf$'},'interpreter','latex','fontsize',19); exportgraphics(gcf,'square-no-noise-rtf.eps','contenttype','vector')
        % disturbance process
        figure; hold on; grid on;
        boxplot([labcB.traj.square.cb(11:20),laobcB.traj.square.cb(11:20),laobcAccelB.traj.square.cb(11:20), mpc1B.traj.square.cb(11:20), mpc2B.traj.square.cb(11:20), mpc3B.traj.square.cb(11:20) ],'Labels',{'labc','laobc','laobcAcel','mpc_1','mpc_2','mpc_3'})
        ylabel({'$rtf$'},'interpreter','latex','fontsize',19); exportgraphics(gcf,'square-proc-dist-rtf.eps','contenttype','vector')
        % measurements noise
        figure; hold on; grid on;
        boxplot([labcB.traj.square.cb(21:30),laobcB.traj.square.cb(21:30),laobcAccelB.traj.square.cb(21:30), mpc1B.traj.square.cb(21:30), mpc2B.traj.square.cb(21:30), mpc3B.traj.square.cb(21:30) ],'Labels',{'labc','laobc','laobcAcel','mpc_1','mpc_2','mpc_3'})
        ylabel({'$rtf$'},'interpreter','latex','fontsize',19); exportgraphics(gcf,'square-meas-noise-rtf.eps','contenttype','vector')
        % measurements noise
        figure; hold on; grid on;
        boxplot([labcB.traj.square.cb(31:40),laobcB.traj.square.cb(31:40),laobcAccelB.traj.square.cb(31:40), mpc1B.traj.square.cb(31:40), mpc2B.traj.square.cb(31:40), mpc3B.traj.square.cb(31:40) ],'Labels',{'labc','laobc','laobcAcel','mpc_1','mpc_2','mpc_3'})
        ylabel({'$rtf$'},'interpreter','latex','fontsize',19); exportgraphics(gcf,'square-both-noises-rtf.eps','contenttype','vector')
    else
        vels        = linspace(0.1,1,10);
        labc_clr    = 'm';
        laobc_clr   = 'c';
        mpc1_clr    = 'b';
        mpc2_clr    = 'r';
        alpha_val   = 0.6;
        line_width  = 2;
        font_size1  = 25;
        % no noises 
        figure; hold on; grid on;
        boxplot([labcB.traj.infty.ce(1:10),laobcB.traj.infty.ce(1:10),laobcAccelB.traj.infty.ce(1:10), mpc1B.traj.infty.ce(1:10), mpc2B.traj.infty.ce(1:10), mpc3B.traj.infty.ce(1:10) ],'Labels',{'labc','laobc','laobcAcel','mpc_1','mpc_2','mpc_3'})
        ylabel({'$ctrl.\,effort$'},'interpreter','latex','fontsize',19); exportgraphics(gcf,'infty-no-noise-ce.eps','contenttype','vector');
        % disturbance process
        figure; hold on; grid on;
        boxplot([labcB.traj.infty.ce(11:20),laobcB.traj.infty.ce(11:20),laobcAccelB.traj.infty.ce(11:20), mpc1B.traj.infty.ce(11:20), mpc2B.traj.infty.ce(11:20), mpc3B.traj.infty.ce(11:20) ],'Labels',{'labc','laobc','laobcAcel','mpc_1','mpc_2','mpc_3'})
        ylabel({'$ctrl.\,effort$'},'interpreter','latex','fontsize',19); exportgraphics(gcf,'infty-proc-dist-ce.eps','contenttype','vector')
        % measurements noise
        figure; hold on; grid on;
        boxplot([labcB.traj.infty.ce(21:30),laobcB.traj.infty.ce(21:30),laobcAccelB.traj.infty.ce(21:30), mpc1B.traj.infty.ce(21:30), mpc2B.traj.infty.ce(21:30), mpc3B.traj.infty.ce(21:30) ],'Labels',{'labc','laobc','laobcAcel','mpc_1','mpc_2','mpc_3'})
        ylabel({'$ctrl.\,effort$'},'interpreter','latex','fontsize',19); exportgraphics(gcf,'infty-meas-noise-ce.eps','contenttype','vector')
        % measurements noise
        figure; hold on; grid on;
        boxplot([labcB.traj.infty.ce(31:40),laobcB.traj.infty.ce(31:40),laobcAccelB.traj.infty.ce(31:40), mpc1B.traj.infty.ce(31:40), mpc2B.traj.infty.ce(31:40), mpc3B.traj.infty.ce(31:40) ],'Labels',{'labc','laobc','laobcAcel','mpc_1','mpc_2','mpc_3'})
        ylabel({'$ctrl.\,effort$'},'interpreter','latex','fontsize',19); exportgraphics(gcf,'infty-both-noises-ce.eps','contenttype','vector')
        % no noises 
        figure; hold on; grid on;
        boxplot([labcB.traj.infty.dev(1:10),laobcB.traj.infty.dev(1:10),laobcAccelB.traj.infty.dev(1:10), mpc1B.traj.infty.dev(1:10), mpc2B.traj.infty.dev(1:10), mpc3B.traj.infty.dev(1:10) ],'Labels',{'labc','laobc','laobcAcel','mpc_1','mpc_2','mpc_3'})
        ylabel({'$dev.\,(m)$'},'interpreter','latex','fontsize',19); exportgraphics(gcf,'infty-no-noise-dev.eps','contenttype','vector')
        % disturbance process
        figure; hold on; grid on;
        boxplot([labcB.traj.infty.dev(11:20),laobcB.traj.infty.dev(11:20),laobcAccelB.traj.infty.dev(11:20), mpc1B.traj.infty.dev(11:20), mpc2B.traj.infty.dev(11:20), mpc3B.traj.infty.dev(11:20) ],'Labels',{'labc','laobc','laobcAcel','mpc_1','mpc_2','mpc_3'})
        ylabel({'$dev.\,(m)$'},'interpreter','latex','fontsize',19); exportgraphics(gcf,'infty-proc-dist-dev.eps','contenttype','vector')
        % measurements noise
        figure; hold on; grid on;
        boxplot([labcB.traj.infty.dev(21:30),laobcB.traj.infty.dev(21:30),laobcAccelB.traj.infty.dev(21:30), mpc1B.traj.infty.dev(21:30), mpc2B.traj.infty.dev(21:30), mpc3B.traj.infty.dev(21:30) ],'Labels',{'labc','laobc','laobcAcel','mpc_1','mpc_2','mpc_3'})
        ylabel({'$dev.\,(m)$'},'interpreter','latex','fontsize',19); exportgraphics(gcf,'infty-meas-noise-dev.eps','contenttype','vector')
        % measurements noise
        figure; hold on; grid on;
        boxplot([labcB.traj.infty.dev(31:40),laobcB.traj.infty.dev(31:40),laobcAccelB.traj.infty.dev(31:40), mpc1B.traj.infty.dev(31:40), mpc2B.traj.infty.dev(31:40), mpc3B.traj.infty.dev(31:40) ],'Labels',{'labc','laobc','laobcAcel','mpc_1','mpc_2','mpc_3'})
        ylabel({'$dev.\,(m)$'},'interpreter','latex','fontsize',19); exportgraphics(gcf,'infty-both-noises-dev.eps','contenttype','vector')
        % no noises 
        figure; hold on; grid on;
        boxplot([labcB.traj.infty.cb(1:10),laobcB.traj.infty.cb(1:10),laobcAccelB.traj.infty.cb(1:10), mpc1B.traj.infty.cb(1:10), mpc2B.traj.infty.cb(1:10), mpc3B.traj.infty.cb(1:10) ],'Labels',{'labc','laobc','laobcAcel','mpc_1','mpc_2','mpc_3'})
        ylabel({'$rtf$'},'interpreter','latex','fontsize',19); exportgraphics(gcf,'infty-no-noise-rtf.eps','contenttype','vector'); ylim([0 0.15])
        % disturbance process
        figure; hold on; grid on;
        boxplot([labcB.traj.infty.cb(11:20),laobcB.traj.infty.cb(11:20),laobcAccelB.traj.infty.cb(11:20), mpc1B.traj.infty.cb(11:20), mpc2B.traj.infty.cb(11:20), mpc3B.traj.infty.cb(11:20) ],'Labels',{'labc','laobc','laobcAcel','mpc_1','mpc_2','mpc_3'})
        ylabel({'$rtf$'},'interpreter','latex','fontsize',19); exportgraphics(gcf,'infty-proc-dist-rtf.eps','contenttype','vector'); ylim([0 0.15])
        % measurements noise
        figure; hold on; grid on;
        boxplot([labcB.traj.infty.cb(21:30),laobcB.traj.infty.cb(21:30),laobcAccelB.traj.infty.cb(21:30), mpc1B.traj.infty.cb(21:30), mpc2B.traj.infty.cb(21:30), mpc3B.traj.infty.cb(21:30) ],'Labels',{'labc','laobc','laobcAcel','mpc_1','mpc_2','mpc_3'})
        ylabel({'$rtf$'},'interpreter','latex','fontsize',19); exportgraphics(gcf,'infty-meas-noise-rtf.eps','contenttype','vector'); ylim([0 0.15])
        % measurements noise
        figure; hold on; grid on;
        boxplot([labcB.traj.infty.cb(31:40),laobcB.traj.infty.cb(31:40),laobcAccelB.traj.infty.cb(31:40), mpc1B.traj.infty.cb(31:40), mpc2B.traj.infty.cb(31:40), mpc3B.traj.infty.cb(31:40) ],'Labels',{'labc','laobc','laobcAcel','mpc_1','mpc_2','mpc_3'})
        ylabel({'$rtf$'},'interpreter','latex','fontsize',19); exportgraphics(gcf,'infty-both-noises-rtf.eps','contenttype','vector'); ylim([0 0.15])
    end    

%     tableDisp = [tableDispCirc{:}, tableDispSqua{:}, tableDispInft{:}]
end

