function plot7dof(t,Xin,Xout)
global pusr auxdata
  



%-----------------------------------------------------------------------------------%
%                                     Plot data                                     %
%-----------------------------------------------------------------------------------%

%% Plot 
 
if pusr.TirePlot
%     tic  
    p1=findobj(pusr.TirePlot1, 'type', 'axes', 'tag', '' );   
    p1(8).Title.String=sprintf('$$Slip_{fr}: %0.3f      Fz_{fr}: %0.2f (N)   Ftx_{fr}: %0.2f (N)$$',auxdata(:,1),auxdata(:,9), auxdata(:,13));
    p1(6).Title.String=sprintf('$$Slip_{fl}: %0.3f      Fz_{fl}: %0.2f (N)   Ftx_{fl}: %0.2f (N)$$',auxdata(:,2),auxdata(:,10),auxdata(:,14));
    p1(4).Title.String=sprintf('$$Slip_{rr}: %0.3f      Fz_{rr}: %0.2f (N)   Ftx_{rr}: %0.2f (N)$$',auxdata(:,3),auxdata(:,11),auxdata(:,15));
    p1(2).Title.String=sprintf('$$Slip_{rl}: %0.3f      Fz_{rl}: %0.2f (N)   Ftx_{rl}: %0.2f (N)$$',auxdata(:,4),auxdata(:,12),auxdata(:,16));
    set(p1,'xlim',[0 t+eps]);
    
    p11=findobj(pusr.TirePlot1, 'type', 'line', 'tag', '' );


    set(p11(12),'XData',pusr.ttlog(1:pusr.tlog),'YData',pusr.kappalog(1:pusr.tlog,1));
    set(p11(11),'XData',pusr.ttlog(1:pusr.tlog),'YData',pusr.Ftlog(1:pusr.tlog,1));
    set(p11(10),'XData',pusr.ttlog(1:pusr.tlog),'YData',pusr.Fzlog(1:pusr.tlog,1)); 
    
    set(p11(9),'XData',pusr.ttlog(1:pusr.tlog),'YData',pusr.kappalog(1:pusr.tlog,2));
    set(p11(8),'XData',pusr.ttlog(1:pusr.tlog),'YData',pusr.Ftlog(1:pusr.tlog,2));
    set(p11(7),'XData',pusr.ttlog(1:pusr.tlog),'YData',pusr.Fzlog(1:pusr.tlog,2));
    
    set(p11(6),'XData',pusr.ttlog(1:pusr.tlog),'YData',pusr.kappalog(1:pusr.tlog,3));
    set(p11(5),'XData',pusr.ttlog(1:pusr.tlog),'YData',pusr.Ftlog(1:pusr.tlog,3));    
    set(p11(4),'XData',pusr.ttlog(1:pusr.tlog),'YData',pusr.Fzlog(1:pusr.tlog,3));
    
    set(p11(3),'XData',pusr.ttlog(1:pusr.tlog),'YData',pusr.kappalog(1:pusr.tlog,4));
    set(p11(2),'XData',pusr.ttlog(1:pusr.tlog),'YData',pusr.Ftlog(1:pusr.tlog,4));
    set(p11(1),'XData',pusr.ttlog(1:pusr.tlog),'YData',pusr.Fzlog(1:pusr.tlog,4));
%     tpl=toc
   
end
if pusr.TireYPlot
    
    p1=findobj(pusr.TireYPlot1, 'type', 'axes', 'tag', '' );
    p1(8).Title.String=sprintf('$$Slip_{fr}: %0.3f      Fz_{fr}: %0.2f (N)   Fty_{fr}: %0.2f (N)$$',auxdata(:,5),auxdata(:,9),  auxdata(:,17));
    p1(6).Title.String=sprintf('$$Slip_{fl}: %0.3f      Fz_{fl}: %0.2f (N)   Fty_{fl}: %0.2f (N)$$',auxdata(:,6),auxdata(:,10), auxdata(:,18));
    p1(4).Title.String=sprintf('$$Slip_{rr}: %0.3f      Fz_{rr}: %0.2f (N)   Fty_{rr}: %0.2f (N)$$',auxdata(:,7),auxdata(:,11), auxdata(:,19));
    p1(2).Title.String=sprintf('$$Slip_{rl}: %0.3f      Fz_{rl}: %0.2f (N)   Fty_{rl}: %0.2f (N)$$',auxdata(:,8),auxdata(:,12), auxdata(:,20));
    set(p1,'xlim',[0 t+eps]);

    p11=findobj(pusr.TireYPlot1, 'type', 'line', 'tag', '' );   
%     axis([0 t -inf inf]);   
    set(p11(12),'XData',pusr.ttlog(1:pusr.tlog),'YData',pusr.alphalog(1:pusr.tlog,1));
    set(p11(11),'XData',pusr.ttlog(1:pusr.tlog),'YData',pusr.Ftlog(1:pusr.tlog,5));
    set(p11(10),'XData',pusr.ttlog(1:pusr.tlog),'YData',pusr.Fzlog(1:pusr.tlog,1)); 
    
    set(p11(9),'XData',pusr.ttlog(1:pusr.tlog),'YData',pusr.alphalog(1:pusr.tlog,2));
    set(p11(8),'XData',pusr.ttlog(1:pusr.tlog),'YData',pusr.Ftlog(1:pusr.tlog,6));
    set(p11(7),'XData',pusr.ttlog(1:pusr.tlog),'YData',pusr.Fzlog(1:pusr.tlog,2));
    
    set(p11(6),'XData',pusr.ttlog(1:pusr.tlog),'YData',pusr.alphalog(1:pusr.tlog,3));
    set(p11(5),'XData',pusr.ttlog(1:pusr.tlog),'YData',pusr.Ftlog(1:pusr.tlog,7));    
    set(p11(4),'XData',pusr.ttlog(1:pusr.tlog),'YData',pusr.Fzlog(1:pusr.tlog,3));
    
    set(p11(3),'XData',pusr.ttlog(1:pusr.tlog),'YData',pusr.alphalog(1:pusr.tlog,4));
    set(p11(2),'XData',pusr.ttlog(1:pusr.tlog),'YData',pusr.Ftlog(1:pusr.tlog,8));
    set(p11(1),'XData',pusr.ttlog(1:pusr.tlog),'YData',pusr.Fzlog(1:pusr.tlog,4));
    
%    ty=toc
end
if pusr.DriverinputPlot
    p2=findobj(pusr.DriverinputPlot1, 'type', 'axes', 'tag', '' );
    
    p2(2).Title.String=sprintf('MotorSpeed: %0.2f (rpm)    MotorTorque: %0.2f (Nm)',auxdata(:,21),auxdata(:,25)/7.97);
    p2(1).Title.String=sprintf('SteerAngle: %0.3f' ,auxdata(:,29)*180/pi);
%     set(p2,'xlim',[0 t]);
    
    p22=findobj(pusr.DriverinputPlot1, 'type', 'line', 'tag', '' ); 

    set(p22(2),'XData',pusr.Ndlog(1:pusr.tlog,1),'YData',pusr.Tdlog(1:pusr.tlog,1)/7.97);
    set(p22(1),'XData',pusr.ttlog(1:pusr.tlog),'YData',pusr.Steerlog(1:pusr.tlog,1)*180/pi);   
end

if pusr.TrackPlot
    p6=findobj(pusr.TrackPlot1, 'type', 'axes', 'tag', '' );  
    p6.Title.String=sprintf('X: %0.2f (m)    Y: %0.2f (M)',Xin(:,11),Xin(:,12));
    

    p66=findobj(pusr.TrackPlot1, 'type', 'line', 'tag', '' ); 
    
    set(p66(1),'XData',pusr.Xlog(1:pusr.tlog,11),'YData',pusr.Xlog(1:pusr.tlog,12));
   
end
if pusr.VehicleBodyPlot
    p=findobj(pusr.VehicleBodyPlot1, 'type', 'axes', 'tag', '' );
    p(6).Title.String= sprintf('X: %0.2f (m)',  Xin(:,11));
    p(5).Title.String= sprintf('Y: %0.2f (m)',  Xin(:,12));
    p(4).Title.String= sprintf('Yaw:%0.3f (rad)',Xin(:,13));
    p(3).Title.String= sprintf('Fx: %0.2f (N)',  auxdata(:,33));
    p(2).Title.String= sprintf('Fy: %0.2f (N)',  auxdata(:,34));
    p(1).Title.String= sprintf('Mz: %0.2f (Nm)', auxdata(:,35));
    set(p,'xlim',[0 t+eps]);
    
    p11=findobj(pusr.VehicleBodyPlot1, 'type', 'line', 'tag', '' );  
        
    set(p11(6),'XData',pusr.ttlog(1:pusr.tlog),'YData',pusr.Xlog(1:pusr.tlog,1));
    set(p11(5),'XData',pusr.ttlog(1:pusr.tlog),'YData',pusr.Xlog(1:pusr.tlog,2));     
    set(p11(4),'XData',pusr.ttlog(1:pusr.tlog),'YData',pusr.Xlog(1:pusr.tlog,3));
    
    set(p11(3),'XData',pusr.ttlog(1:pusr.tlog),'YData',pusr.Flog(1:pusr.tlog,1));    
    set(p11(2),'XData',pusr.ttlog(1:pusr.tlog),'YData',pusr.Flog(1:pusr.tlog,2));
    set(p11(1),'XData',pusr.ttlog(1:pusr.tlog),'YData',pusr.Flog(1:pusr.tlog,3));
    
end


drawnow
end