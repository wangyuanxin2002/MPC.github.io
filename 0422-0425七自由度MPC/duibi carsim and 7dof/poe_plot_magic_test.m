% 定义 x 的范围
alpha = linspace(-12, 12, 1000);




    B = 10.62;%10;
    C = 1.4;%1.9;
    D = 1;%0.5;
    E = 0.549;%0.97;

%     B1 = 0.1813;%10;
%     C1 = 1.415;%1.9;
%     D1 = 0.5;%0.5;
%     E1 = -0.1848;%0.97;

%     B = 10;
%     C = 1.9;
%     D = 0.5;
%     E = 0.97;
% % 
%     B1 = 0.11;
%     C1 = 1.2;
%     D1 = 0.02;
%     E1 = -0.1848;
% 
% 
    B11 = 0.1813;
    C11 = 1.4;
    D11 = 1;
    E11 = -0.18;
% % 

% 
%     Ftxfr = (Fzfr.*D.*sin(C.*atan(B.*kappa(1) - E.*(B.*kappa(1) - atan(B.*kappa(1))))));%
% 
%     
    y1 = (-1000.*D.*sin(C.*atan(B.*alpha- E.*(B.*alpha - atan(B.*alpha)))));%
    
y2 = (-1000.*D11.*sin(C11.*atan(B11.*alpha- E11.*(B11.*alpha - atan(B11.*alpha)))));%
% 计算 y = atan(x) 和 y = tan(x)
% Ftyfr = atan(x);
% y2 = tan(x);

% 创建图形窗口
figure;

% 绘制 y = atan(x)
subplot(2, 1, 1);
plot(alpha, y1);
title('y = atan(x)');
xlabel('x');
ylabel('y');

% 绘制 y = tan(x)
subplot(2, 1, 2);
plot(alpha, y2);
title('y = tan(x)');
xlabel('x');
ylabel('y');

% % 调整子图之间的间距
% spacing = 0.05;
% subplotSpacing = 0.1;
% set(gcf, 'Position', get(0, 'Screensize'));
% set(gcf, 'PaperPositionMode', 'auto');
% set(gcf, 'Units', 'normalized');
% set(gcf, 'OuterPosition', [0 0.05 1 0.95]);
% set(gcf, 'Resize', 'off');
% set(gcf, 'DockControls', 'off');
% set(gcf, 'WindowStyle', 'normal');
% set(gcf, 'WindowStyle', 'docked');
% set(gcf, 'WindowStyle', 'modal');
% set(gcf, 'WindowState', 'normal');
% set(gcf, 'WindowState', 'maximized');
% set(gcf, 'WindowState', 'fullscreen');
% hSub = findobj(gcf, 'type', 'axes');
% for k = 1:numel(hSub)
%     set(hSub(k), 'Position', get(hSub(k), 'OuterPosition') + spacing*k);
% end
% set(gcf, 'PaperPositionMode', 'auto');
% set(gcf, 'InvertHardcopy', 'off');
% set(gcf, 'Visible', 'on');