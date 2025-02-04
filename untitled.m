k = 1:20;

x = [1 0 0 0 0 1 0 0 0 0 1 0 0 0 0 1 0 0 0 1];
r = [0 0 0 1 0 0 0 1 0 0 0 1 0 0 0 1 0 0 0 1];

indx0 = find(x==0);
x(indx0) = NaN;
indr0 = find(r==0);
r(indr0) = NaN;

figure; hold on; grid on;
ylim([0 1.5])

stem(k,x,'b','MarkerFaceColor','b','MarkerSize',18,'LineWidth',3)
count_k     = 0;
indxs       = find(x==1);
indx_last   = indxs(end-1:end);
for i=1:length(x)
    if x(i)==1
        if count_k == 0
            str_aux = strcat('$x_{k}$');
        else
            str_aux = strcat(['$x_{k+',num2str(count_k),'}$']);
        end      
        if i==indx_last(end-1) || i==indx_last(end)
            text(i-0.25,x(i)*1.18,{str_aux},'Interpreter','latex','FontSize',45)
        else
            text(i-0.75,x(i)*1.1,{str_aux},'Interpreter','latex','FontSize',45)
        end
        count_k = count_k + 1;
    end
end
%
stem(k,r,'r-.','MarkerFaceColor','r','MarkerSize',15,'LineWidth',3)
count_k = 0;
for i=1:length(x)
    if r(i)==1
        if count_k == 0
            str_aux = strcat('$r_{k}$');
        else
            str_aux = strcat(['$r_{k+',num2str(count_k),'}$']);
        end        
        text(i-0.25,r(i)*1.1,{str_aux},'Interpreter','latex','FontSize',45)
        count_k = count_k + 1;
    end
end

% plot the error's arrows
x1  = [1; 4];
x2  = [6; 8];
x3  = [11; 12];
X   = [x1, x2, x3];
y   = [1/4; 1/4];
for i=1:size(X,2)
    [Xplt, Yplt] = coord2norm(gca,X(:,i)',y');
    err_arrow = annotation('doublearrow',Xplt,Yplt);
end
text(2.15,0.32,{'$e_{k}$'},'Interpreter','latex','FontSize',45)
text(6.65,0.32,{'$e_{k+1}$'},'Interpreter','latex','FontSize',45)
text(10.95,0.32,{'$e_{k+2}$'},'Interpreter','latex','FontSize',45)

% plot the reference speed
x1  = [4; 8];
x2  = [8; 12];
x3  = [12; 16];
x4  = [16; 20];
X   = [x1, x2, x3, x4];
y   = [1/2; 1/2];
for i=1:size(X,2)
    [Xplt, Yplt] = coord2norm(gca,X(:,i)',y');
    err_arrow = annotation('doublearrow',Xplt,Yplt);
end
text(6.1,0.58,{'$v_{r,k+1}$'},'Interpreter','latex','FontSize',45)
text(9.7,0.58,{'$v_{r,k+2}$'},'Interpreter','latex','FontSize',45)
text(13.75,0.58,{'$v_{r,k+3}$'},'Interpreter','latex','FontSize',45)
text(17.75,0.58,{'$v_{r,k+4}$'},'Interpreter','latex','FontSize',45)

% plot the x speed
x1  = [1; 6];
x2  = [6; 11];
x3  = [11; 16];
x4  = [16; 20];
X   = [x1, x2, x3, x4];
y   = [3/4; 3/4];
for i=1:size(X,2)
    [Xplt, Yplt] = coord2norm(gca,X(:,i)',y');
    err_arrow = annotation('doublearrow',Xplt,Yplt);
end
text(3.3,0.83,{'$v_{x,k+1}$'},'Interpreter','latex','FontSize',45)
text(8.3,0.83,{'$v_{x,k+2}$'},'Interpreter','latex','FontSize',45)
text(13.2,0.83,{'$v_{x,k+3}$'},'Interpreter','latex','FontSize',45)
text(17.75,0.83,{'$v_{x,k+4}$'},'Interpreter','latex','FontSize',45)


xlabel('$k$','Interpreter','latex','FontSize',45)
set(gca,'GridAlpha',0.4)
set(gca,'YTickLabel',[]);
set(gca,'XTickLabel',[]);
ax = gca;
ax.XAxis.FontSize = 30;

% exportgraphics(gcf,'plt_stability.eps','contenttype','vector')