function plotcontrol
%-----------------------------------------------------------------------------------%
% Author: HUILONG YU, hlyubit@gmail.com.
% Date  : 09/11/2016
% Copyright (C) 2016 HUILONG YU. All Rights Reserved.
%-----------------------------------------------------------------------------------%
global pusr

if pusr.TirePlot
  pusr.TirePlot1=figure('NumberTitle','off','Name','TirePlot');
    co = [0 0 1;
      1 0 0;
      0 0.5 0;
      0 0.75 0.75;
      0.75 0 0.75;
      0.75 0.75 0;
      0.25 0.25 0.25];
    set(groot,'defaultAxesColorOrder',co);
    set(0,'DefaultLineLineWidth',1.5);
    set(0,'DefaultTextInterpreter', 'latex')
 
    p1(1) = subplot(2,2,1);xlabel('$$time (sec1)$$','interpreter','latex'); ylabel('$$Slip_{fr}$$','interpreter','latex');%,'FontSize',18
    box on 
    grid on; set(gca, 'gridlinestyle',':'); set(gca, 'gridalpha',0.5)
    set(gca,'YColor','b');
    line(0,0,  'Parent',  p1(1),'Color', 'blue');%
    ht1(1)= title(sprintf('$$Slip_{fr}: %0.2f      Fz_{fr}: %0.2f (N)   Ftx_{fr}: %0.2f (N)$$',0,0,0),'interpreter','latex');
    p1(2) = axes('Position',get(p1(1),'Position'),'YAxisLocation','right', 'Color','none', 'YColor','r');
    p1(2).YLabel.String = sprintf('$$Ftx_{fr} (N)$$');
    line (0,0,  'Parent',  p1(2),'Color', 'r');%
    p1(3)  = subplot(2,2,2);xlabel('time (sec)'); ylabel('$$Slip_{fl}$$');    box on 
    grid on; set(gca, 'gridlinestyle',':'); set(gca, 'gridalpha',0.5)
    set(gca,'YColor','b');
    ht1(2)= title(sprintf('$$ Slip_{fl}: %0.2f      Fz_{fl}: %0.2f (N)   Ftx_{fl}: %0.2f (N)$$',0,0,0 ),'interpreter','latex');
    line (0,0,  'Parent',  p1(3),'Color', 'blue');%
    p1(4) = axes('Position',get(p1(3),'Position'),'YAxisLocation','right', 'Color','none', 'YColor','r');
    p1(4).YLabel.String = sprintf('$$Ftx_{fl} (N)$$');
    line (0,0,  'Parent',  p1(4),'Color', 'r');%
    
    p1(5)  = subplot(2,2,3);xlabel('time (sec)'); ylabel('$$Slip_{rr}$$');set(gca,'YColor','b');    box on 
    grid on; set(gca, 'gridlinestyle',':'); set(gca, 'gridalpha',0.5)
    ht1(3) = title(sprintf('$$ Slip_{rr}: %0.2f      Fz_{rr}: %0.2f (N)   Ftx_{rr}: %0.2f (N)$$',0,0,0 ),'interpreter','latex');
    line (0,0,  'Parent',  p1(5),'Color', 'blue');%
    p1(6) = axes('Position',get(p1(5),'Position'),'YAxisLocation','right', 'Color','none', 'YColor','r');
    p1(6).YLabel.String = sprintf('$$Ftx_{rr} (N)$$');
    line (0,0,  'Parent',  p1(6),'Color', 'r');%

    p1(7) = subplot(2,2,4);xlabel('time (sec)'); ylabel('$$Slip_{rl}$$');    box on 
    grid on; set(gca, 'gridlinestyle',':'); set(gca, 'gridalpha',0.5)
    set(gca,'YColor','b');
    ht1(4) = title(sprintf('$$ Slip_{rl}: %0.2f      Fz_{rl}: %0.2f (N)    Ftx_{rl}: %0.2f (N)$$',0,0,0 ),'interpreter','latex');
    line (0,0,  'Parent',  p1(7),'Color', 'blue');%
    p1(8) = axes('Position',get(p1(7),'Position'),'YAxisLocation','right', 'Color','none', 'YColor','r');
    p1(8).YLabel.String = sprintf('$$Ftx_{rl} (N)$$');
    line (0,0,  'Parent',  p1(8),'Color', 'r');%
    
    line (0,0,  'Parent', p1(2),'Color', 'k');%,'Color', 'c'
    line (0,0,  'Parent', p1(4),'Color', 'k');%,'Color', 'c'
    line (0,0,  'Parent', p1(6),'Color', 'k');%,'Color', 'c'
    line (0,0,  'Parent', p1(8),'Color', 'k');%,'Color', 'c'
    pt=findobj(pusr.TirePlot1, 'type', 'axes', 'tag', '' );
    
    linkaxes(pt, 'x'); 


end

if pusr.TireYPlot
   pusr.TireYPlot1=figure('NumberTitle','off','Name','TireYPlot');
    box on 
    grid on; set(gca, 'gridlinestyle',':'); set(gca, 'gridalpha',0.5)
    p1(1) = subplot(2,2,1);xlabel('time (sec)'); ylabel('$$Slip_{fr}$$');%,'FontSize',18
    set(gca,'YColor','b');
    ht1(1)= title(sprintf('$$ Slip_{fr}: %0.2f      Fz_{fr}: %0.2f (N)   Fty_{fr}: %0.2f (N)$$',0,0,0 ),'interpreter','latex');
    p1(2) = axes('Position',get(p1(1),'Position'),'YAxisLocation','right', 'Color','none', 'YColor','r');
    p1(2).YLabel.String = sprintf('$$Fty_{fr} (N)$$');
    box on 
    grid on; set(gca, 'gridlinestyle',':'); set(gca, 'gridalpha',0.5)
    p1(3)  = subplot(2,2,2);xlabel('time (sec)'); ylabel('$$Slip_{fl}$$');set(gca,'YColor','b');
    ht1(2)= title(sprintf('$$ Slip_{fl}: %0.2f      Fz_{fl}: %0.2f (N)   Fty_{fl}: %0.2f (N)$$',0,0,0 ),'interpreter','latex');
    p1(4) = axes('Position',get(p1(3),'Position'),'YAxisLocation','right', 'Color','none', 'YColor','r');
    p1(4).YLabel.String = sprintf('$$Fty_{fl} (N)$$');
    box on 
    grid on; set(gca, 'gridlinestyle',':'); set(gca, 'gridalpha',0.5)
    p1(5)  = subplot(2,2,3);xlabel('time (sec)'); ylabel('$$Slip_{rr}$$');set(gca,'YColor','b');
    ht1(3) = title(sprintf('$$Slip_{rr}: %0.2f      Fz_{rr}: %0.2f (N)   Fty_{rr}: %0.2f (N)$$',0,0,0 ),'interpreter','latex');
    p1(6) = axes('Position',get(p1(5),'Position'),'YAxisLocation','right', 'Color','none', 'YColor','r');
    p1(6).YLabel.String = sprintf('$$Fty_{rr} (N)$$');

    box on 
    grid on; set(gca, 'gridlinestyle',':'); set(gca, 'gridalpha',0.5)
    p1(7) = subplot(2,2,4);xlabel('time (sec)'); ylabel('$$Slip_{rl}$$');set(gca,'YColor','b');
        box on 
    grid on; set(gca, 'gridlinestyle',':'); set(gca, 'gridalpha',0.5)
    ht1(4) = title(sprintf('$$Slip_{rl}: %0.2f      Fz_{rl}: %0.2f (N)    Fty_{rl}: %0.2f (N)$$',0,0,0 ),'interpreter','latex');
    p1(8) = axes('Position',get(p1(7),'Position'),'YAxisLocation','right', 'Color','none', 'YColor','r');
    p1(8).YLabel.String = sprintf('$$Fty_{rl} (N)$$');
    
    line(0,0,  'Parent',  p1(1),'Color', 'b');%
    line(0,0,  'Parent',  p1(2),'Color', 'r');%
    line(0,0,  'Parent',  p1(2),'Color', 'k');%
    
    line(0,0,  'Parent',  p1(3),'Color', 'b');%
    line(0,0,  'Parent',  p1(4),'Color', 'r');%
    line(0,0,  'Parent',  p1(4),'Color', 'k');%
    
    line(0,0,  'Parent',  p1(5),'Color', 'b');%
    line(0,0,  'Parent',  p1(6),'Color', 'r');%
    line(0,0,  'Parent',  p1(6),'Color', 'k');%
    
    line(0,0,  'Parent',  p1(7),'Color', 'b');%
    line(0,0,  'Parent',  p1(8),'Color', 'r');%
    line(0,0,  'Parent',  p1(8),'Color', 'k');%

    pt=findobj(pusr.TireYPlot1, 'type', 'axes', 'tag', '' );
    linkaxes(pt, 'x'); 
end

if pusr.DriverinputPlot
    pusr.DriverinputPlot1=figure('NumberTitle','off','Name','DriverInputPlot');
    p2(1) = subplot(1,2,1);    box on 
    grid on; set(gca, 'gridlinestyle',':'); set(gca, 'gridalpha',0.5)
    plot(pusr.Nm,pusr.Tm,'r', 'Parent',p2(1));
    hold on;
    plot(pusr.Nm,pusr.Tm,'ob', 'Parent',p2(1),'MarkerSize',7);
   
    xlabel('MotorSpeed (rpm)'); ylabel('Motor Torque (Nm)');
    ht2(1) = title(sprintf('MotorSpeed: %0.2f    MotorTorque: %0.2f (Nm)', 0));
    set(p2(1),'ylim',[0 200]);
    
    p2(2)  = subplot(1,2,2);xlabel('time (sec)'); ylabel('SteeringAngle (^o)');    box on 
    grid on; set(gca, 'gridlinestyle',':'); set(gca, 'gridalpha',0.5)
    line(0,0,  'Parent',  p2(2),'Color', 'b');%

    ht2(2)=  title(sprintf('SteeringAngle: %0.2f ^o', 0));
    pd=findobj(pusr.DriverinputPlot1, 'type', 'axes', 'tag', '' );
    
%     linkaxes(pd, 'x'); 


end

if pusr.TrackPlot
    pusr.TrackPlot1=figure('NumberTitle','off','Name','TrackPlot');      
    p3=subplot(1,1,1);box on 
    grid on; set(gca, 'gridlinestyle',':'); set(gca, 'gridalpha',0.5)
    yy=load('TrackFinal');
    h1=plot(-yy.x_inner,yy.y_inner,'ob',-yy.x_outer,yy.y_outer,'ob','MarkerEdgeColor',[0.8 0.8 0.8],'Linewidth',0.5);
    legend(h1,'Road boundary');
%     box off
    hold on 
    h2=plot(pusr.xcoord,pusr.ycoord,':b');
%     legend(h2,'Road center line');
%     box off
    grid on; set(gca, 'gridlinestyle',':'); set(gca, 'gridalpha',0.5)
    text(pusr.xcoord(1:500:end),pusr.ycoord(1:500:end)+20,num2str(fix(pusr.slength(1:500:end)')))
    line(0,0,  'Parent',  p3,'Color', 'r');
    xlabel('X (m)'); ylabel('Y (m)');
    title(sprintf('X: %0.2f    Y: %0.2f (Nm)', 0,0));
end

if pusr.VehicleBodyPlot  
pusr.VehicleBodyPlot1=figure('NumberTitle','off','Name','Vehicle Body plot');
p(1) = subplot(2,3,1);xlabel('time (sec)'); ylabel('Vx (m/s)');set(gca,'YColor','b');    box on 
    grid on; set(gca, 'gridlinestyle',':'); set(gca, 'gridalpha',0.5)
grid on; set(gca, 'gridlinestyle',':'); set(gca, 'gridalpha',0.5)
ht(1)= title(sprintf('X: %0.2f (m)',0));

p(2)  = subplot(2,3,2);xlabel('time (sec)'); ylabel('Vy (m/s)');set(gca,'YColor','b');    box on 
    grid on; set(gca, 'gridlinestyle',':'); set(gca, 'gridalpha',0.5)
ht(2)= title(sprintf('Y: %0.2f (m)',0));

p(3)  = subplot(2,3,3);xlabel('time (sec)'); ylabel('Yrate (rad/s)');set(gca,'YColor','b');grid on; set(gca, 'gridlinestyle',':'); set(gca, 'gridalpha',0.5)
ht(3) = title(sprintf('Yaw: %0.2f (rad)',0));


p(4) = subplot(2,3,4);set(gca,'YColor','b');grid on; set(gca, 'gridlinestyle',':'); set(gca, 'gridalpha',0.5)

xlabel('time (sec)'); ylabel('Fx (N)');
ht(4) = title(sprintf('Fx: %0.2f (N)',0));


p(5) = subplot(2,3,5);xlabel('time (sec)'); ylabel('Fy (N)');set(gca,'YColor','b');grid on; set(gca, 'gridlinestyle',':'); set(gca, 'gridalpha',0.5)
ht(5) = title(sprintf('Fy: %0.2f (N)',0));


p(6) = subplot(2,3,6);xlabel('time (sec)'); ylabel('Mz (Nm)');set(gca,'YColor','b');grid on; set(gca, 'gridlinestyle',':'); set(gca, 'gridalpha',0.5)
ht(6) = title(sprintf('Mz: %0.2f (Nm)',0));


line(0,0,  'Parent',  p(1),'Color', 'b');%
line(0,0,  'Parent',  p(2),'Color', 'b');% 
line(0,0,  'Parent',  p(3),'Color', 'b');%
line(0,0,  'Parent',  p(4),'Color', 'b');%
line(0,0,  'Parent',  p(5),'Color', 'b');%
line(0,0,  'Parent',  p(6),'Color', 'b');%

pvb=findobj(pusr.VehicleBodyPlot1, 'type', 'axes', 'tag', '' );
linkaxes(pvb, 'x'); 
end



end
