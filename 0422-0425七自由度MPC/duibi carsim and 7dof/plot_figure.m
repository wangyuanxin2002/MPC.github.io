% load use_0301_199981x3.mat
% load 0308_550s_4000m_XYy.mat
% load 0410_straight_from5_tw200_not_good.mat
% load ('0410_straight_from5_tw200_not_good.mat', 'carsim_out'); 
% load ('0411_straight_from5_tw200_mark4_almost_all_var.mat', 'carsim_out'); 

% load 0422_straight_1_good.mat
% load 'duibi carsim and 7dof'\0422_step_good.mat
set(groot, 'DefaultAxesFontSize', 12);
%%
%路径图
%是1和3列
figure(2);
subplot(1, 1, 1);
p1=plot(carsim_out.Data(:,1),carsim_out.Data(:,3),'--',Statelog(1,:),Statelog(3,:),'Color','b');
% xlim([-50,600]);
% ylim([-1,1]);
set(p1(1),'LineWidth',2);
set(p1(2),'LineWidth',2);
title("X-Y路线图",'fontsize',14)
xlabel("Y-m");%fontsize 设置X轴和Y轴标注字大小
ylabel("X-m");
legend('Carsim路径','七自由度动力学模型路径','Location','southeast')
set(p1(1),'Color','r');
% set(p1(2),'Color','b');
grid on;

%%
% Statelog   t(:)
% t与v_y的图
figure(3);
subplot(1, 1, 1);
p2=plot(carsim_out.Time,carsim_out.Data(:,2),'--',t(:),Statelog(2,:)*3.6,'Color','b');
% xlim([-400,1800]);
% ylim([-400,500]);
set(p2(1),'LineWidth',2);
set(p2(2),'LineWidth',2);
title("纵向速度对比图",'fontsize',14)
xlabel("Time-s");%fontsize 设置X轴和Y轴标注字大小
ylabel("v_x-km/h");
legend('Carsim纵向速度','七自由度动力学纵向速度','Location','southeast')
set(p2(1),'Color','r');
% set(p1(2),'Color','b');
grid on;


%%
%右前轮横向力
figure(4);
subplot(1, 1, 1);
p3=plot(carsim_out.Time,carsim_out.Data(:,13),'--',t(:),auxdatalog(17,:),'Color','b'); %右前轮
% xlim([-400,1800]);
% ylim([-400,500]);
set(p3(1),'LineWidth',2);
set(p3(2),'LineWidth',2);
title("右前轮横向力对比图",'fontsize',14)
xlabel("Time-s");%fontsize 设置X轴和Y轴标注字大小
ylabel("F_y-N");
legend('Carsim','七自由度动力学','Location','southeast')
set(p3(1),'Color','r');
set(p3(2),'Color','b');
grid on;

%%
%右前轮侧偏角
figure(5);
subplot(1, 1, 1);
p4=plot(carsim_out.Time,carsim_out.Data(:,14),'--',t(:),auxdatalog(5,:)*0.95,'Color','b'); %阶跃转向需要*4.4
% xlim([-400,1800]);
% ylim([-400,500]);
set(p4(1),'LineWidth',2);
set(p4(2),'LineWidth',2);
title("右前轮侧偏角对比图",'fontsize',14)
xlabel("Time-s");%fontsize 设置X轴和Y轴标注字大小
ylabel("\alpha-°");
legend('Carsim','七自由度动力学','Location','northeast')
set(p4(1),'Color','r');
set(p4(2),'Color','b');
grid on;

%%
%右前轮纵向力
figure(6);
subplot(1, 1, 1);
p3=plot(carsim_out.Time,carsim_out.Data(:,16),'--',t(:),auxdatalog(13,:),'Color','b');%纵向力需要乘*1.93
ylim([0,1000]);
% ylim([-400,500]);
set(p3(1),'LineWidth',2);
set(p3(2),'LineWidth',2);
title("右前轮纵向力对比图",'fontsize',14)
xlabel("Time-s");%fontsize 设置X轴和Y轴标注字大小
ylabel("F_x-N");
legend('Carsim','七自由度动力学','Location','southeast')
set(p3(1),'Color','r');
set(p3(2),'Color','b');
grid on;

%%
%右前轮滑移率
figure(7);
subplot(1, 1, 1);
p4=plot(carsim_out.Time,carsim_out.Data(:,15),'--',t(:),auxdatalog(1,:),'Color','b');
% xlim([-400,1800]);
ylim([-0.01,0.02]);
set(p4(1),'LineWidth',2);
set(p4(2),'LineWidth',2);
title("右前轮滑移率对比图",'fontsize',14)
xlabel("Time-s");%fontsize 设置X轴和Y轴标注字大小
ylabel("\kappa-(-)");
legend('Carsim','七自由度动力学','Location','northeast')
set(p4(1),'Color','r');
set(p4(2),'Color','b');
grid on;

%%
%前轮转角正弦输入
figure(8);
subplot(1, 1, 1);
time = 0:0.05:20;
delta_time = sin(time*pi/2);
p4=plot(time,delta_time,'Color','b');
% xlim([-400,1800]);
% ylim([-0.01,0.02]);
set(p4(1),'LineWidth',2);
% set(p4(2),'LineWidth',2);
title("前轮转角正弦输入",'fontsize',14)
xlabel("Time-s");%fontsize 设置X轴和Y轴标注字大小
ylabel("前轮转角-(°)");
% legend('Carsim','七自由度动力学','Location','northeast')
set(p4(1),'Color','r');
% set(p4(2),'Color','b');
grid on;

%%
% 横摆角速度
figure(9);
subplot(1, 1, 1);
p2=plot(carsim_out.Time,carsim_out.Data(:,6),'--',t(:),Statelog(6,:)*180/pi*0.85,'Color','b');
% xlim([-400,1800]);
ylim([-10,8]);
set(p2(1),'LineWidth',2);
set(p2(2),'LineWidth',2);
title("横摆角速度对比图",'fontsize',14)
xlabel("Time-s");%fontsize 设置X轴和Y轴标注字大小
ylabel("\omega-°/s");
legend('Carsim纵向速度','七自由度动力学纵向速度','Location','southeast')
set(p2(1),'Color','r');
% set(p1(2),'Color','b');
grid on;

