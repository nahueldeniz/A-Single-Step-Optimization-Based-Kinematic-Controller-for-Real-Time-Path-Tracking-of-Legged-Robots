
filename = 'labc_1.mat';

labc                    = struct;
labc.traj.circ          = {};
labc.traj.circ.dev      = [];
labc.traj.circ.cb       = [];
labc.traj.circ.ce       = [];
labc.traj.square        = {};
labc.traj.square.dev    = [];
labc.traj.square.cb     = [];
labc.traj.square.ce     = [];
labc.traj.infty         = {};
labc.traj.infty.dev     = [];
labc.traj.infty.cb      = [];
labc.traj.infty.ce      = [];

laobc                   = struct;
laobc.traj.circ         = {};
laobc.traj.circ.dev     = [];
laobc.traj.circ.cb      = [];
laobc.traj.circ.ce      = [];
laobc.traj.square       = {};
laobc.traj.square.dev   = [];
laobc.traj.square.cb    = [];
laobc.traj.square.ce    = [];
laobc.traj.infty        = {};
laobc.traj.infty.dev    = [];
laobc.traj.infty.cb     = [];
laobc.traj.infty.ce     = [];

mpc1                    = struct;
mpc1.traj.circ          = {};
mpc1.traj.circ.dev      = [];
mpc1.traj.circ.cb       = [];
mpc1.traj.circ.ce       = [];
mpc1.traj.square        = {};
mpc1.traj.square.dev    = [];
mpc1.traj.square.cb     = [];
mpc1.traj.square.ce     = [];
mpc1.traj.infty         = {};
mpc1.traj.infty.dev     = [];
mpc1.traj.infty.cb      = [];
mpc1.traj.infty.ce      = [];

mpc2                    = struct;
mpc2.traj.circ          = {};
mpc2.traj.circ.dev      = [];
mpc2.traj.circ.cb       = [];
mpc2.traj.circ.ce       = [];
mpc2.traj.square        = {};
mpc2.traj.square.dev    = [];
mpc2.traj.square.cb     = [];
mpc2.traj.square.ce     = [];
mpc2.traj.infty         = {};
mpc2.traj.infty.dev     = [];
mpc2.traj.infty.cb      = [];
mpc2.traj.infty.ce      = [];


precision = '%0.2g';
precision_ce = '%0.4g';
exps = [4 8 12];

for nn=1:size(exps,1)
    tableDispCirc           = {};
    tableDispSqua           = {repmat(' & ',10,1)};
    tableDispInft           = {repmat(' & ',10,1)};
    
    for ii=exps(nn,:)
        filename = strcat(['/home/nahuel/Dropbox/PosDoc/AC3E/Go1 MPC/Simulaciones/sims_HUFA/labc_', num2str(ii),'.mat']);
        load(filename);
        for jj=1:10
            meanDev = 0;
            compBur = 0;
            ctrlEff = 0;
            for kk=1:10
                meanDev = meanDev + mean(S.data.performance.deviation{(jj-1)*10+kk}(:));
                compBur = compBur + mean(S.data.performance.compBurden{(jj-1)*10+kk}(:))/S.config.Ts;
                ctrlEff = ctrlEff + mean(sqrt(sum(S.data.performance.controls{(jj-1)*10+kk}(1,:).^2)) + sqrt(sum(S.data.performance.controls{(jj-1)*10+kk}(2,:).^2)) + sqrt(sum(S.data.performance.controls{(jj-1)*10+kk}(3,:).^2)));            
            end
            if ii==exps(nn,1)
                labc.traj.circ.dev      = [labc.traj.circ.dev; mean(meanDev)];
                labc.traj.circ.cb       = [labc.traj.circ.cb; mean(compBur)];
                labc.traj.circ.ce       = [labc.traj.circ.ce; mean(ctrlEff)];
            elseif ii==exps(nn,2)
                labc.traj.square.dev    = [labc.traj.square.dev; mean(meanDev)];
                labc.traj.square.cb     = [labc.traj.square.cb; mean(compBur)];
                labc.traj.square.ce     = [labc.traj.square.ce; mean(ctrlEff)];
            else
                labc.traj.infty.dev     = [labc.traj.infty.dev; mean(meanDev)];
                labc.traj.infty.cb      = [labc.traj.infty.cb; mean(compBur)];
                labc.traj.infty.ce      = [labc.traj.infty.ce; mean(ctrlEff)];
            end
        end
        if ii==exps(nn,1)
            tableDispCirc = {tableDispCirc{:},  num2str(labc.traj.circ.dev,precision), repmat(' & ',10,1), num2str(labc.traj.circ.cb,precision), repmat(' & ',10,1), num2str(labc.traj.circ.ce,precision_ce)};
        elseif ii==exps(nn,2)
            tableDispSqua = {tableDispSqua{:}, num2str(labc.traj.square.dev,precision), repmat(' & ',10,1), num2str(labc.traj.square.cb,precision), repmat(' & ',10,1), num2str(labc.traj.square.ce,precision_ce), repmat(' & ',10,1)};
        else
            tableDispInft = {tableDispInft{:}, num2str(labc.traj.infty.dev,precision), repmat(' & ',10,1), num2str(labc.traj.infty.cb,precision), repmat(' & ',10,1), num2str(labc.traj.infty.ce,precision_ce), repmat(' & ',10,1)};
        end
        %
        filename = strcat(['/home/nahuel/Dropbox/PosDoc/AC3E/Go1 MPC/Simulaciones/sims_HUFA/laobc_', num2str(ii),'.mat']);
        load(filename);
        for jj=1:10
            meanDev = 0;
            compBur = 0;
            ctrlEff = 0;
            for kk=1:10
                meanDev = meanDev + mean(S.data.performance.deviation{(jj-1)*10+kk}(:));
                compBur = compBur + mean(S.data.performance.compBurden{(jj-1)*10+kk}(:))/S.config.Ts;
                ctrlEff = ctrlEff + mean(sqrt(sum(S.data.performance.controls{(jj-1)*10+kk}(1,:).^2)) + sqrt(sum(S.data.performance.controls{(jj-1)*10+kk}(2,:).^2)) + sqrt(sum(S.data.performance.controls{(jj-1)*10+kk}(3,:).^2)));            
            end
            if ii==exps(nn,1)
                laobc.traj.circ.dev      = [laobc.traj.circ.dev; mean(meanDev)];
                laobc.traj.circ.cb       = [laobc.traj.circ.cb; mean(compBur)];
                laobc.traj.circ.ce       = [laobc.traj.circ.ce; mean(ctrlEff)];
            elseif ii==exps(nn,2)
                laobc.traj.square.dev    = [laobc.traj.square.dev; mean(meanDev)];
                laobc.traj.square.cb     = [laobc.traj.square.cb; mean(compBur)];
                laobc.traj.square.ce     = [laobc.traj.square.ce; mean(ctrlEff)];
            else
                laobc.traj.infty.dev     = [laobc.traj.infty.dev; mean(meanDev)];
                laobc.traj.infty.cb      = [laobc.traj.infty.cb; mean(compBur)];
                laobc.traj.infty.ce      = [laobc.traj.infty.ce; mean(ctrlEff)];
            end
        end
        if ii==exps(nn,1)
            tableDispCirc = {tableDispCirc{:},  num2str(laobc.traj.circ.dev,precision), repmat(' & ',10,1), num2str(laobc.traj.circ.cb,precision), repmat(' & ',10,1), num2str(laobc.traj.circ.ce,precision_ce)};
        elseif ii==exps(nn,2)
            tableDispSqua = {tableDispSqua{:}, num2str(laobc.traj.square.dev,precision), repmat(' & ',10,1), num2str(laobc.traj.square.cb,precision), repmat(' & ',10,1), num2str(laobc.traj.square.ce,precision_ce), repmat(' & ',10,1)};
        else
            tableDispInft = {tableDispInft{:}, num2str(laobc.traj.infty.dev,precision), repmat(' & ',10,1), num2str(laobc.traj.infty.cb,precision), repmat(' & ',10,1), num2str(laobc.traj.infty.ce,precision_ce), repmat(' & ',10,1)};
        end
        %
        filename = strcat(['/home/nahuel/Dropbox/PosDoc/AC3E/Go1 MPC/Simulaciones/sims_HUFA/mpc_', num2str(ii),'_N20','.mat']);
        load(filename);
        for jj=1:10
            meanDev = 0;
            compBur = 0;
            ctrlEff = 0;
            for kk=1:10
                meanDev = meanDev + mean(S.data.performance.deviation{(jj-1)*10+kk}(:));
                compBur = compBur + mean(S.data.performance.compBurden{(jj-1)*10+kk}(:))/S.config.Ts;
                ctrlEff = ctrlEff + mean(sqrt(sum(S.data.performance.controls{(jj-1)*10+kk}(1,:).^2)) + sqrt(sum(S.data.performance.controls{(jj-1)*10+kk}(2,:).^2)) + sqrt(sum(S.data.performance.controls{(jj-1)*10+kk}(3,:).^2)));            
            end
            if ii==exps(nn,1)
                mpc1.traj.circ.dev      = [mpc1.traj.circ.dev; mean(meanDev)];
                mpc1.traj.circ.cb       = [mpc1.traj.circ.cb; mean(compBur)];
                mpc1.traj.circ.ce       = [mpc1.traj.circ.ce; mean(ctrlEff)];
            elseif ii==exps(nn,2)
                mpc1.traj.square.dev    = [mpc1.traj.square.dev; mean(meanDev)];
                mpc1.traj.square.cb     = [mpc1.traj.square.cb; mean(compBur)];
                mpc1.traj.square.ce     = [mpc1.traj.square.ce; mean(ctrlEff)];
            else
                mpc1.traj.infty.dev     = [mpc1.traj.infty.dev; mean(meanDev)];
                mpc1.traj.infty.cb      = [mpc1.traj.infty.cb; mean(compBur)];
                mpc1.traj.infty.ce      = [mpc1.traj.infty.ce; mean(ctrlEff)];
            end
        end
        if ii==exps(nn,1)
            tableDispCirc = {tableDispCirc{:}, repmat(' & ',10,1), num2str(mpc1.traj.circ.dev,precision), repmat(' & ',10,1), num2str(mpc1.traj.circ.cb,precision), repmat(' & ',10,1), num2str(mpc1.traj.circ.ce,precision_ce)};
        elseif ii==exps(nn,2)
            tableDispSqua = {tableDispSqua{:}, num2str(mpc1.traj.square.dev,precision), repmat(' & ',10,1), num2str(mpc1.traj.square.cb,precision), repmat(' & ',10,1), num2str(mpc1.traj.square.ce,precision_ce), repmat(' & ',10,1)};
        else
            tableDispInft = {tableDispInft{:}, num2str(mpc1.traj.infty.dev,precision), repmat(' & ',10,1), num2str(mpc1.traj.infty.cb,precision), repmat(' & ',10,1), num2str(mpc1.traj.infty.ce,precision_ce), repmat(' & ',10,1)};
        end
        %
        filename = strcat(['/home/nahuel/Dropbox/PosDoc/AC3E/Go1 MPC/Simulaciones/sims_HUFA/mpc_', num2str(ii),'_N35','.mat']);
        load(filename);
        for jj=1:10
            meanDev = 0;
            compBur = 0;
            ctrlEff = 0;
            for kk=1:10
                meanDev = meanDev + mean(S.data.performance.deviation{(jj-1)*10+kk}(:));
                compBur = compBur + mean(S.data.performance.compBurden{(jj-1)*10+kk}(:))/S.config.Ts;
                ctrlEff = ctrlEff + mean(sqrt(sum(S.data.performance.controls{(jj-1)*10+kk}(1,:).^2)) + sqrt(sum(S.data.performance.controls{(jj-1)*10+kk}(2,:).^2)) + sqrt(sum(S.data.performance.controls{(jj-1)*10+kk}(3,:).^2)));            
            end
            if ii==exps(nn,1)
                mpc2.traj.circ.dev      = [mpc2.traj.circ.dev; mean(meanDev)];
                mpc2.traj.circ.cb       = [mpc2.traj.circ.cb; mean(compBur)];
                mpc2.traj.circ.ce       = [mpc2.traj.circ.ce; mean(ctrlEff)];
            elseif ii==exps(nn,2)
                mpc2.traj.square.dev    = [mpc2.traj.square.dev; mean(meanDev)];
                mpc2.traj.square.cb     = [mpc2.traj.square.cb; mean(compBur)];
                mpc2.traj.square.ce     = [mpc2.traj.square.ce; mean(ctrlEff)];
            else
                mpc2.traj.infty.dev     = [mpc2.traj.infty.dev; mean(meanDev)];
                mpc2.traj.infty.cb      = [mpc2.traj.infty.cb; mean(compBur)];
                mpc2.traj.infty.ce      = [mpc2.traj.infty.ce; mean(ctrlEff)];
            end
        end
        if ii==exps(nn,1)
            tableDispCirc = {tableDispCirc{:}, repmat(' & ',10,1), num2str(mpc2.traj.circ.dev,precision), repmat(' & ',10,1), num2str(mpc2.traj.circ.cb,precision), repmat(' & ',10,1), num2str(mpc2.traj.circ.ce,precision_ce)};
        elseif ii==exps(nn,2)
            tableDispSqua = {tableDispSqua{:}, num2str(mpc2.traj.square.dev,precision), repmat(' & ',10,1), num2str(mpc2.traj.square.cb,precision), repmat(' & ',10,1), num2str(mpc2.traj.square.ce,precision_ce)};
        else
            tableDispInft = {tableDispInft{:}, num2str(mpc2.traj.infty.dev,precision), repmat(' & ',10,1), num2str(mpc2.traj.infty.cb,precision), repmat(' & ',10,1), num2str(mpc2.traj.infty.ce,precision_ce), repmat(' \\ ',10,1)};
        end
    end
    
    tableDisp = [tableDispCirc{:}, tableDispSqua{:}, tableDispInft{:}]
end

