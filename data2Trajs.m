clear all;
%
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
exps = [5];
warning('off');
maxJJ = 8;
axFontSize = 38;
graphFontSize = 30;
traj_shape = '-rectangular';

for countExps=1:size(exps,1)
    tableDispCirc           = {};
    tableDispSqua           = {repmat(' & ',10,1)};
    tableDispInft           = {repmat(' & ',10,1)};
    
    for counterI=exps(countExps,:)                
        filename = strcat(['/home/nahuel/Dropbox/PosDoc/AC3E/Go1 MPC/Simulaciones/sims_HUFA/labc_', num2str(counterI),'.mat']);
        clear S;
        load(filename);
        figure(1); hold on; grid on; daspect([1,1,1]);
        plot(S.path.coordinates(1,:),S.path.coordinates(2,:),'k','linewidth',4);
        xlim([2 12]); ylim([0 8]); box('on'); ax = gca; ax.GridAlpha = 0.4; xlabel({'$x\,(m)$'},'interpreter','latex','fontsize',axFontSize); ylabel({'$y\,(m)$'},'interpreter','latex','fontsize',axFontSize); ax.FontSize = graphFontSize;
        nameFIgure = strcat(['labc_',num2str(counterI),traj_shape,'.eps']);        
        for jj=1:maxJJ
            meanDev = [];
            compBur = [];
            ctrlEff = [];
            for kk=1:10
                meanDev = [meanDev, mean(S.data.performance.deviation{(jj-1)*10+kk}(:))];
                compBur = [compBur, mean(S.data.performance.compBurden{(jj-1)*10+kk}(:))/S.config.Ts];
                ctrlEff = [ctrlEff, mean(sqrt(sum(S.data.performance.controls{(jj-1)*10+kk}(1,:).^2)) + sqrt(sum(S.data.performance.controls{(jj-1)*10+kk}(2,:).^2)) + sqrt(sum(S.data.performance.controls{(jj-1)*10+kk}(3,:).^2)))];            
                %
                figure(1);
                patchline(S.data.performance.xsim{(jj-1)*10+kk}(1,:),S.data.performance.xsim{(jj-1)*10+kk}(2,:),'linestyle','-','edgecolor','b','linewidth',1.5,'edgealpha',(100-((jj-1)*10+kk))/100);
%                 t = linspace(0,length(S.data.performance.deviation{(jj-1)*10+kk}(:))*S.config.Ts,length(S.data.performance.deviation{(jj-1)*10+kk}(:)));
%                 patchline(t, S.data.performance.deviation{(jj-1)*10+kk}(:),'linestyle','-','edgecolor','b','linewidth',1.5,'edgealpha',0.75);
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
        end        
        exportgraphics(figure(1),nameFIgure,'contenttype','vector')
        %
        filename = strcat(['/home/nahuel/Dropbox/PosDoc/AC3E/Go1 MPC/Simulaciones/sims_HUFA/laobc_', num2str(counterI),'.mat']);                
        clear S;
        load(filename);
        figure(2); hold on; grid on; daspect([1,1,1]);
        plot(S.path.coordinates(1,:),S.path.coordinates(2,:),'k','linewidth',4);
        xlim([2 12]); ylim([0 8]); box('on'); ax = gca; ax.GridAlpha = 0.4; xlabel({'$x\,(m)$'},'interpreter','latex','fontsize',axFontSize); ylabel({'$y\,(m)$'},'interpreter','latex','fontsize',axFontSize); ax.FontSize = graphFontSize;
        nameFIgure = strcat(['laobc_',num2str(counterI),traj_shape,'.eps']);        
        for jj=1:maxJJ
            meanDev = [];
            compBur = [];
            ctrlEff = [];
            for kk=1:10
                meanDev = [meanDev, mean(S.data.performance.deviation{(jj-1)*10+kk}(:))];
                compBur = [compBur, mean(S.data.performance.compBurden{(jj-1)*10+kk}(:))/S.config.Ts];
                ctrlEff = [ctrlEff, mean(sqrt(sum(S.data.performance.controls{(jj-1)*10+kk}(1,:).^2)) + sqrt(sum(S.data.performance.controls{(jj-1)*10+kk}(2,:).^2)) + sqrt(sum(S.data.performance.controls{(jj-1)*10+kk}(3,:).^2)))];            
                %
                figure(2);
                patchline(S.data.performance.xsim{(jj-1)*10+kk}(1,:),S.data.performance.xsim{(jj-1)*10+kk}(2,:),'linestyle','-','edgecolor','b','linewidth',1.5,'edgealpha',(100-((jj-1)*10+kk))/100);
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
        exportgraphics(figure(2),nameFIgure,'contenttype','vector')
        %
        filename = strcat(['/home/nahuel/Dropbox/PosDoc/AC3E/Go1 MPC/Simulaciones/sims_HUFA/laobcAcel_', num2str(counterI),'.mat']);        
        clear S;
        load(filename);
        clear pplayer; clear pc; clear pcl; clear pcl_wogrd; clear pcl_wogrd_sampled; clear pcsToView;
        figure(3); hold on; grid on; daspect([1,1,1]);
        plot(S.path.coordinates(1,:),S.path.coordinates(2,:),'k','linewidth',4);
        xlim([2 12]); ylim([0 8]); box('on'); ax = gca; ax.GridAlpha = 0.4; xlabel({'$x\,(m)$'},'interpreter','latex','fontsize',axFontSize); ylabel({'$y\,(m)$'},'interpreter','latex','fontsize',axFontSize); ax.FontSize = graphFontSize;
        nameFIgure = strcat(['laobcAcel_',num2str(counterI),traj_shape,'.eps']);        
        for jj=1:maxJJ
            meanDev = [];
            compBur = [];
            ctrlEff = [];
            for kk=1:10
                meanDev = [meanDev, mean(S.data.performance.deviation{(jj-1)*10+kk}(:))];
                compBur = [compBur, mean(S.data.performance.compBurden{(jj-1)*10+kk}(:))/S.config.Ts];
                ctrlEff = [ctrlEff, mean(sqrt(sum(S.data.performance.controls{(jj-1)*10+kk}(1,:).^2)) + sqrt(sum(S.data.performance.controls{(jj-1)*10+kk}(2,:).^2)) + sqrt(sum(S.data.performance.controls{(jj-1)*10+kk}(3,:).^2)))];            
                %
                figure(3);
                patchline(S.data.performance.xsim{(jj-1)*10+kk}(1,:),S.data.performance.xsim{(jj-1)*10+kk}(2,:),'linestyle','-','edgecolor','b','linewidth',1.5,'edgealpha',(100-((jj-1)*10+kk))/100);
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
        exportgraphics(figure(3),nameFIgure,'contenttype','vector')
        %
        filename = strcat(['/home/nahuel/Dropbox/PosDoc/AC3E/Go1 MPC/Simulaciones/sims_HUFA/mpc_', num2str(counterI),'_N20','.mat']);        
        clear S;
        load(filename);
        clear pplayer; clear pc; clear pcl; clear pcl_wogrd; clear pcl_wogrd_sampled; clear pcsToView;
        figure(4); hold on; grid on; daspect([1,1,1]);
        plot(S.path.coordinates(1,:),S.path.coordinates(2,:),'k','linewidth',4);
        xlim([2 12]); ylim([0 8]); box('on'); ax = gca; ax.GridAlpha = 0.4; xlabel({'$x\,(m)$'},'interpreter','latex','fontsize',axFontSize); ylabel({'$y\,(m)$'},'interpreter','latex','fontsize',axFontSize); ax.FontSize = graphFontSize;
        nameFIgure = strcat(['mpc_',num2str(counterI),'N20',traj_shape, '.eps']);        
        for jj=1:maxJJ
            meanDev = [];
            compBur = [];
            ctrlEff = [];
            for kk=1:10
                meanDev = [meanDev, mean(S.data.performance.deviation{(jj-1)*10+kk}(:))];
                compBur = [compBur, mean(S.data.performance.compBurden{(jj-1)*10+kk}(:))/S.config.Ts];
                ctrlEff = [ctrlEff, mean(sqrt(sum(S.data.performance.controls{(jj-1)*10+kk}(1,:).^2)) + sqrt(sum(S.data.performance.controls{(jj-1)*10+kk}(2,:).^2)) + sqrt(sum(S.data.performance.controls{(jj-1)*10+kk}(3,:).^2)))];            
                %
                figure(4);
                patchline(S.data.performance.xsim{(jj-1)*10+kk}(1,:),S.data.performance.xsim{(jj-1)*10+kk}(2,:),'linestyle','-','edgecolor','r','linewidth',1.5,'edgealpha',(100-((jj-1)*10+kk))/100);
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
        exportgraphics(figure(4),nameFIgure,'contenttype','vector')
        %
        filename = strcat(['/home/nahuel/Dropbox/PosDoc/AC3E/Go1 MPC/Simulaciones/sims_HUFA/mpc_', num2str(counterI),'_N35','.mat']);        
        clear S;
        load(filename);
        figure(5); hold on; grid on; daspect([1,1,1]);
        plot(S.path.coordinates(1,:),S.path.coordinates(2,:),'k','linewidth',4);
        xlim([2 12]); ylim([0 8]); box('on'); ax = gca; ax.GridAlpha = 0.4; xlabel({'$x\,(m)$'},'interpreter','latex','fontsize',axFontSize); ylabel({'$y\,(m)$'},'interpreter','latex','fontsize',axFontSize); ax.FontSize = graphFontSize;
        nameFIgure = strcat(['mpc_',num2str(counterI),'N35',traj_shape '.eps']);        
        for jj=1:maxJJ
            meanDev = [];
            compBur = [];
            ctrlEff = [];
            for kk=1:10
                meanDev = [meanDev, mean(S.data.performance.deviation{(jj-1)*10+kk}(:))];
                compBur = [compBur, mean(S.data.performance.compBurden{(jj-1)*10+kk}(:))/S.config.Ts];
                ctrlEff = [ctrlEff, mean(sqrt(sum(S.data.performance.controls{(jj-1)*10+kk}(1,:).^2)) + sqrt(sum(S.data.performance.controls{(jj-1)*10+kk}(2,:).^2)) + sqrt(sum(S.data.performance.controls{(jj-1)*10+kk}(3,:).^2)))];            
                %
                figure(5);
                patchline(S.data.performance.xsim{(jj-1)*10+kk}(1,:),S.data.performance.xsim{(jj-1)*10+kk}(2,:),'linestyle','-','edgecolor','r','linewidth',1.5,'edgealpha',(100-((jj-1)*10+kk))/100);
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
        exportgraphics(figure(5),nameFIgure,'contenttype','vector')
        %
        filename = strcat(['/home/nahuel/Dropbox/PosDoc/AC3E/Go1 MPC/Simulaciones/sims_HUFA/mpc_', num2str(counterI),'_N1','.mat']);        
        clear S;
        load(filename);
        figure(6); hold on; grid on; daspect([1,1,1]);
        plot(S.path.coordinates(1,:),S.path.coordinates(2,:),'k','linewidth',4);
        xlim([2 12]); ylim([0 8]); box('on'); ax = gca; ax.GridAlpha = 0.4; xlabel({'$x\,(m)$'},'interpreter','latex','fontsize',axFontSize); ylabel({'$y\,(m)$'},'interpreter','latex','fontsize',axFontSize); ax.FontSize = graphFontSize;
        nameFIgure = strcat(['mpc_',num2str(counterI),'N1',traj_shape '.eps']);        
        for jj=1:maxJJ
            meanDev = [];
            compBur = [];
            ctrlEff = [];
            for kk=1:10
                meanDev = [meanDev, mean(S.data.performance.deviation{(jj-1)*10+kk}(:))];
                compBur = [compBur, mean(S.data.performance.compBurden{(jj-1)*10+kk}(:))/S.config.Ts];
                ctrlEff = [ctrlEff, mean(sqrt(sum(S.data.performance.controls{(jj-1)*10+kk}(1,:).^2)) + sqrt(sum(S.data.performance.controls{(jj-1)*10+kk}(2,:).^2)) + sqrt(sum(S.data.performance.controls{(jj-1)*10+kk}(3,:).^2)))];            
                %
                figure(6);
                patchline(S.data.performance.xsim{(jj-1)*10+kk}(1,:),S.data.performance.xsim{(jj-1)*10+kk}(2,:),'linestyle','-','edgecolor','r','linewidth',1.5,'edgealpha',(100-((jj-1)*10+kk))/100);
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
        exportgraphics(figure(6),nameFIgure,'contenttype','vector')
        %
    end    
end

figure(7); hold on; grid on;
vels = linspace(0.1,length(laobcB.traj.square.ce)/10, length(laobcB.traj.square.ce));
patchline(vels, labcB.traj.square.ce,'linestyle','-','edgecolor','b','linewidth',1.5,'edgealpha',0.25);
patchline(vels, laobcB.traj.square.ce,'linestyle','-','edgecolor','b','linewidth',1.5,'edgealpha',0.5);
patchline(vels, laobcAccelB.traj.square.ce,'linestyle','-','edgecolor','b','linewidth',1.5,'edgealpha',0.75);

patchline(vels, mpc1B.traj.square.ce,'linestyle','-','edgecolor','r','linewidth',1.5,'edgealpha',0.25);
patchline(vels, mpc2B.traj.square.ce,'linestyle','-','edgecolor','r','linewidth',1.5,'edgealpha',0.5);
patchline(vels, mpc3B.traj.square.ce,'linestyle','-','edgecolor','r','linewidth',1.5,'edgealpha',0.75);

box('on'); ax = gca; ax.GridAlpha = 0.4; xlabel({'$v\,(m/s)$'},'interpreter','latex','fontsize',axFontSize); ylabel({'$\sqrt{\sum u_i^2}$'},'interpreter','latex','fontsize',axFontSize); ax.FontSize = graphFontSize;
axis('tight')
exportgraphics(figure(7),'ctrl_effor_rectangular.eps','contenttype','vector')


figure(8); hold on; grid on;
vels = linspace(0.1,length(laobcB.traj.square.ce)/10, length(laobcB.traj.square.ce));
patchline(vels, labcB.traj.square.dev,'linestyle','-','edgecolor','b','linewidth',1.5,'edgealpha',0.25);
patchline(vels, laobcB.traj.square.dev,'linestyle','-','edgecolor','b','linewidth',1.5,'edgealpha',0.5);
patchline(vels, laobcAccelB.traj.square.dev,'linestyle','-','edgecolor','b','linewidth',1.5,'edgealpha',0.75);

patchline(vels, mpc1B.traj.square.dev,'linestyle','-','edgecolor','r','linewidth',1.5,'edgealpha',0.25);
patchline(vels, mpc2B.traj.square.dev,'linestyle','-','edgecolor','r','linewidth',1.5,'edgealpha',0.5);
patchline(vels, mpc3B.traj.square.dev,'linestyle','-','edgecolor','r','linewidth',1.5,'edgealpha',0.75);

box('on'); ax = gca; ax.GridAlpha = 0.4; xlabel({'$v\,(m/s)$'},'interpreter','latex','fontsize',axFontSize); ylabel({'$\vert q_{r,k}-q_{k}\vert$'},'interpreter','latex','fontsize',axFontSize); ax.FontSize = graphFontSize;
axis('tight')
exportgraphics(figure(8),'dev_effor_rectangular.eps','contenttype','vector')


figure(9); hold on; grid on;
vels = linspace(0.1,length(laobcB.traj.square.ce)/10, length(laobcB.traj.square.ce));
patchline(vels, labcB.traj.square.cb,'linestyle','-','edgecolor','b','linewidth',1.5,'edgealpha',0.25);
patchline(vels, laobcB.traj.square.cb,'linestyle','-','edgecolor','b','linewidth',1.5,'edgealpha',0.5);
patchline(vels, laobcAccelB.traj.square.cb,'linestyle','-','edgecolor','b','linewidth',1.5,'edgealpha',0.75);

patchline(vels, mpc1B.traj.square.cb,'linestyle','-','edgecolor','r','linewidth',1.5,'edgealpha',0.25);
patchline(vels, mpc2B.traj.square.cb,'linestyle','-','edgecolor','r','linewidth',1.5,'edgealpha',0.5);
patchline(vels, mpc3B.traj.square.cb,'linestyle','-','edgecolor','r','linewidth',1.5,'edgealpha',0.75);

box('on'); ax = gca; ax.GridAlpha = 0.4; xlabel({'$v\,(m/s)$'},'interpreter','latex','fontsize',axFontSize); ylabel({'$\rtf$'},'interpreter','latex','fontsize',axFontSize); ax.FontSize = graphFontSize;
axis('tight')
exportgraphics(figure(9),'rtf_rectangular.eps','contenttype','vector')


