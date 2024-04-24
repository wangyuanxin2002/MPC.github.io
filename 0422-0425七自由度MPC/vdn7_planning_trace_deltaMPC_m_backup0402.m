function [sys,x0,str,ts] = planning_trace_forge0311(t,x,u,flag)


switch flag
 case 0
  [sys,x0,str,ts] = mdlInitializeSizes; % Initialization
  
 case 2
  sys = mdlUpdates(t,x,u); % Update discrete states
  
 case 3
  sys = mdlOutputs(t,x,u); % Calculate outputs
 
%  case 4
%   sys = mdlGetTimeOfNextVarHit(t,x,u); % Get next sample time 

 case {1,4,9} % Unused flags
  sys = [];
  
 otherwise
  error(['unhandled flag = ',num2str(flag)]); % Error handling
end
% End of dsfunc.

%==============================================================
% Initialization
%==============================================================

function [sys,x0,str,ts] = mdlInitializeSizes%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%运行的第一个函数，初始化函数

% Call simsizes for a sizes structure, fill it in, and convert it  
% to a sizes array.%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%调用simsizes的大小结构，填充它，并将其转换为大小数组？

sizes = simsizes;
sizes.NumContStates  = 0;
sizes.NumDiscStates  = 10;
sizes.NumOutputs     = 5;
sizes.NumInputs      = 12;
sizes.DirFeedthrough = 1; 
sizes.NumSampleTimes = 1;
sys = simsizes(sizes); 
x0 =[0.001;0.0001;0.0001;0.00001;0.00001;0.00001;0.0001;0.00001;0.00001;0.00001];    %Inputs初值
global U;
U=[0 0 0 0 0];%控制量初始化              ,这里面加了一个期望轨迹的输出，如果去掉，U为一维的   U=[0];@2
% global x;
% x = zeros(md.ne + md.pye + md.me + md.Hu*md.me,1);   
% Initialize the discrete states.
str = [];             % Set str to an empty matrix.
ts  = [0.01 0];       % sample time: [period, offset]
%End of mdlInitializeSizes
		      
%==============================================================
% Update the discrete states
%==============================================================
function sys = mdlUpdates(t,x,u)%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%运行的第二个函数，case=2(因为没有1
  
sys = x;
%End of mdlUpdate.

%==============================================================
% Calculate outputs
%==============================================================
function sys = mdlOutputs(t,x,u) %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%主函数
    global a b; 
    global U;
    %global kesi;
    tic
    Nx = 10;%状态量的个数
    Nu = 5;%控制量的个数  
    Np = 10;%预测步长
    Nc = 10;%控制步长
    Ny = 2;%观测量（输出量）个数
%     Row = 1000;%松弛因子权重
    fprintf('Update start, t=%6.3f\n',t)
   
    %输入接口转换,x_dot后面加一个非常小的数，是防止出现分母为零的情况 %%%%%%%%%状态量从数组里读取到变量里，同时变换单位
    % y_dot=u(1)/3.6-0.000000001*0.4786;%CarSim输出的是km/h，转换为m/s
%     y_dot4
%     x_dot2
%     phi5
%     phi_dot6
%     Y3
%     X1
    X = u(1); 
    x_dot = u(2)/3.6+0.0001;    %CarSim输出的是km/h，转换为m/s
    Y = u(3);                   %单位为m
    y_dot = u(4)/3.6;           %m/s
    phi = u(5)*3.141592654/180; %CarSim输出的为角度，角度转换为弧度
    phi_dot = u(6)*pi/180; 
    domegaFr = u(7) *pi/30; % L1右前  rpm转换rad/s
    domegaFl = u(8) *pi/30; % 1L 左前 Front left
    domegaRr = u(9) *pi/30; % 2r 后右 rear right
    domegaRl = u(10) *pi/30;% 
    ddXbc=u(11)*9.8;
    ddYbc=u(12)*9.8;
    Tw1 = U(1);
    Tw2 = U(2);
    Tw3 = U(3);
    Tw4 = U(4);
    delta_f=U(5);
    U1 = [ Tw1; Tw2; Tw3; Tw4; delta_f]; 
    Xcur = [X; x_dot; Y; y_dot; phi; phi_dot;domegaFr;domegaFl;domegaRr;domegaRl];
%% 车辆参数输入

    T=0.01;%仿真步长
    T_all = 20;
%% 参考轨迹生成
    shape=2.4;%参数名称，用于参考轨迹生成
    dx1=25;dx2=21.95;
    dy1=4.05;dy2=5.7;
    Xs1=27.19;Xs2=56.46;
    X_predict=zeros(Np,1);%用于保存预测时域内的纵向位置信息
    Y_ref=zeros(Np,1);%用于保存预测时域内的期望轨迹
        
%     RefSt = [0 6 0 0 0 0 0 0 0 0 0 0]';%参考状态 R efSt R efSt = [0 10 0 0 0 0]';
%     RefSt(2) = 6;
%     RefSt(3) = 0;%.1*(t^2);%0;

    %权重矩阵设置 
  %%      %权重矩阵设置 
    Q_cell=cell(Np,Np);
    for i=1:1:Np
        for j=1:1:Np
            if i==j
                Q_cell{i,j}=[1 0;0 1];
            else 
                Q_cell{i,j}=zeros(Ny,Ny);               
            end
        end 
    end 
    RrT= 1e15;

    Rr =  [RrT RrT RrT RrT 1e15 ];
    R_r = diag([Rr(1),Rr(2),Rr(3),Rr(4),Rr(5)]);
    R = kron(eye(Nc),R_r);

%% model
    ac = genA_7dof_0320(x_dot,y_dot,phi,domegaFr,domegaFl,domegaRr,domegaRl,X,Y,phi,Tw1,Tw2,Tw3,Tw4,delta_f,ddXbc,ddYbc);
    a = ac*T+eye(size(ac));

    bc = genB_7dof_0320(x_dot,y_dot,phi,domegaFr,domegaFl,domegaRr,domegaRl,X,Y,phi,Tw1,Tw2,Tw3,Tw4,delta_f,ddXbc,ddYbc);
    b =T*bc;

    state_k1=zeros(Nx,1);%预测的下一时刻状态量，用于计算偏差
    %% 求状态

    dXb = x_dot;
    dYb = y_dot;
    dYaw = phi_dot;
%     Xb = X;
%     Yb = Y;
    Yaw = phi;
    steer = delta_f;

    global pusr
    %------------%
    %     常量    %
    %------------%
    pusr.HalfCdArouDrag=0.0313828;   %200.85/(80^2)=0.0313828
    pusr.HalfCdArouFrontDown=0.0193125; %123.6/(80^2)=0.0193125 From VI-grade simulation results
    pusr.HalfCdArouRearDown=0.0331875; %212.4/(80^2)=0.0331875
    pusr.lr  = 1.040;
    pusr.lf  = 1.560;
    pusr.l   = 2.6;
    pusr.wf  = 1.480;
    pusr.wr  = 1.485;
    pusr.hg  = 0.540;
    pusr.hd  = 0.17;
    pusr.x   = [1.560 1.560 -1.040 -1.040];
    pusr.y   = [-1.480/2 1.480/2 -1.485/2 1.485/2];
    pusr.mt  = 1110;
    
    pusr.Jzz = 711.3; % 
    pusr.Jwy = [0.5 0.5 0.5 0.5];
    pusr.Rt  =[0.298 0.298 0.298 0.298];
    
    omega     = [domegaFr domegaFl domegaRr domegaRl];
    Fw                 = pusr.HalfCdArouDrag*(dXb.^2)*(3.6.^2);
    FrontDownForceDrag = 0.5*pusr.HalfCdArouFrontDown*(dXb.^2)*(3.6.^2);
    RearDownForceDrag  = 0.5*pusr.HalfCdArouRearDown*(dXb.^2)*(3.6.^2);
    
    %-----------------------------------------------------------------------------------%
    %                                    Normal load                                    %
    %-----------------------------------------------------------------------------------%
    % ddYbc = ddYbc*0;
    % ddXbc = ddXbc*0;
    
    Ftemp1  = pusr.lr*pusr.mt*9.8*0.5/pusr.l; % 0.5*前轴重 左右相同
    Ftemp2  = pusr.hg*pusr.mt*ddXbc*0.5/pusr.l; % ax加速度导致的前后轴载荷转移
    Ftemp3  = pusr.lr*pusr.hg*pusr.mt*ddYbc/(pusr.l*pusr.wf); %
    Ftemp4  = pusr.hg*pusr.hg*pusr.mt*ddYbc.*ddXbc/(pusr.l*pusr.wf*9.8); %
    Ftemp5  = 0.5*FrontDownForceDrag; %
    Ftemp6  = 0.5*pusr.hd*Fw/pusr.l; % 
    Ftemp7  = (FrontDownForceDrag*pusr.hg*pusr.l-pusr.hg*pusr.hd*Fw).*ddYbc... 
              /(pusr.l*pusr.wf*9.8); %
          
    Ftemp8  = pusr.lf*pusr.mt*9.8*0.5/pusr.l; % 0.5*后轴重 左右相同
    Ftemp9  = pusr.lf*pusr.hg*pusr.mt*ddYbc/(pusr.l*pusr.wr); % ay*重/轴距等 左右不同
    Ftemp10 = pusr.hg*pusr.hg*pusr.mt*ddYbc.*ddXbc/(pusr.l*pusr.wr*9.8); %
    Ftemp11 = 0.5*RearDownForceDrag; % 阻力
    Ftemp12 = (RearDownForceDrag*pusr.hg*pusr.l+pusr.hg*pusr.hd*Fw).*ddYbc... 
              /(pusr.l*pusr.wr*9.8); % 阻力导致的左右偏差
    Fzfl    = Ftemp1-Ftemp2-Ftemp3+Ftemp4+Ftemp5-Ftemp6-Ftemp7; %
    Fzfr    = Ftemp1-Ftemp2+Ftemp3-Ftemp4+Ftemp5-Ftemp6+Ftemp7; %
    Fzrl    = Ftemp8+Ftemp2-Ftemp9-Ftemp10+Ftemp11+Ftemp6-Ftemp12; %
    Fzrr    = Ftemp8+Ftemp2+Ftemp9+Ftemp10+Ftemp11+Ftemp6+Ftemp12; %
%     Fz      = [Fzfr Fzfl Fzrr Fzrl]; %
    
    %-----------------------------------------------------------------------------------%
    %                                     Tire slip                                     %
    %-----------------------------------------------------------------------------------%
    Vbx   = repmat(dXb,size(pusr.y))-repmat(pusr.y,size(dXb)).*dYaw;
    Vby   = repmat(dYb,size(pusr.x))+repmat(pusr.x,size(dYb)).*dYaw;
    
    Vwx   = Vbx.*cos([steer,steer,0,0])+Vby.*sin([steer,steer,0,0]); 
    Vwy   =-Vbx.*sin([steer,steer,0,0])+Vby.*cos([steer,steer,0,0]); 
    
    kappa = (omega.*repmat(pusr.Rt,size(dXb))-Vwx)./Vwx;
    alpha = -[steer,steer,0,0]+atan(Vby./Vbx);
    % 
    % kappa(kappa>1)=1;
    % kappa(kappa<-1)=-1;
    % 
    % alpha(alpha>1.57)=1.57;alpha
    % alpha(alpha<-1.57)=-1.57;
    %-----------------------------------------------------------------------------------%
    %                                     Tire model                                    %
    %-----------------------------------------------------------------------------------%
    % [Ftx, Fty] = MfTire(Fz,kappa,alpha(4),pusr.gamma,Vwx);
    
    B0 = 10;
    C0 = 1.9;
    D0 = 1;
    E0 = 0.97;
    Ftxfr = (Fzfr.*D0.*sin(C0.*atan(B0.*kappa(1) - E0.*(B0.*kappa(1) - atan(B0.*kappa(1))))));%
    Ftxfl = (Fzfl.*D0.*sin(C0.*atan(B0.*kappa(2) - E0.*(B0.*kappa(2) - atan(B0.*kappa(2))))));%
    Ftxrr = (Fzrr.*D0.*sin(C0.*atan(B0.*kappa(3) - E0.*(B0.*kappa(3) - atan(B0.*kappa(3))))));%
    Ftxrl = (Fzrl.*D0.*sin(C0.*atan(B0.*kappa(4) - E0.*(B0.*kappa(4) - atan(B0.*kappa(4))))));%
    
    Ftyfr = (-Fzfr.*D0.*sin(C0.*atan(B0.*alpha(1) - E0.*(B0.*alpha(1) - atan(B0.*alpha(1))))));%
    Ftyfl = (-Fzfl.*D0.*sin(C0.*atan(B0.*alpha(2) - E0.*(B0.*alpha(2) - atan(B0.*alpha(2))))));%
    Ftyrr = (-Fzrr.*D0.*sin(C0.*atan(B0.*alpha(3) - E0.*(B0.*alpha(3) - atan(B0.*alpha(3))))));%
    Ftyrl = (-Fzrl.*D0.*sin(C0.*atan(B0.*alpha(4) - E0.*(B0.*alpha(4) - atan(B0.*alpha(4))))));%
    
    Fx  = (Ftxfr+Ftxfl).*cos(steer)-(Ftyfr+Ftyfl).*sin(steer) + Ftxrr+Ftxrl -Fw;
    
    Fy  = (Ftxfr+Ftxfl).*sin(steer)+(Ftyfr+Ftyfl).*cos(steer) + Ftyrr+Ftyrl;
    Tyaw = Ftxfr*sin(steer)* pusr.x(1) + Ftxfl*sin(steer)* pusr.x(2) +Ftyfr*cos(steer)* pusr.x(1) + ...
        Ftyfl*cos(steer)* pusr.x(2) + Ftyrr* pusr.x(3)+Ftyrl* pusr.x(4) ...
         - ( (Ftxfr*pusr.y(1)+Ftxfl*pusr.y(2)).*cos(steer)-(Ftyfr*pusr.y(1)+Ftyfl*pusr.y(2)).*sin(steer) + Ftxrr*pusr.y(3)+Ftxrl*pusr.y(4) );
   
    Ftx = [Ftxfr, Ftxfl, Ftxrr, Ftxrl];
    Fty = [Ftyfr, Ftyfl, Ftyrr, Ftyrl];
    Fx         = sum(Ftx.*cos([steer,steer,0,0])-Fty.*sin([steer,steer,0,0]),2)-Fw;
    Fy         = sum(Ftx.*sin([steer,steer,0,0])+Fty.*cos([steer,steer,0,0]),2);
    Tyaw = sum((Ftx.*sin([steer,steer,0,0])+Fty.*cos([steer,steer,0,0])).*repmat(pusr.x,size(dYb)))...
         - sum((Ftx.*cos([steer,steer,0,0])-Fty.*sin([steer,steer,0,0])).*repmat(pusr.y,size(dYb))); 

    %-----------------------------------------------------------------------------------%
    %                        Derivetives of the 7 dof vehicle model                     %
    %-----------------------------------------------------------------------------------%
    % ddXbc  = Fx/pusr.mt; % ddXbc作用是保存上一轮车辆的加速度，这个只能从外面传进来。。然后ddXbc在外面要更新一次。
    % ddYbc  = Fy/pusr.mt;
    state_k1(2,1)       = Fx/pusr.mt ; % +dYaw.*dYb; % eq2 ddXb
    state_k1(4,1)       = Fy/pusr.mt ; % -dYaw.*dXb; % eq4 ddYb
    state_k1(6,1)     = Tyaw/pusr.Jzz; % eq6 ddYaw 
    state_k1(7,1)   = (Tw1-Ftxfr.*pusr.Rt(1))./pusr.Jwy(1); % eq7 ddomegaFr
    state_k1(8,1)   = (Tw2-Ftxfl.*pusr.Rt(2))./pusr.Jwy(2); % eq8 ddomegaFl
    state_k1(9,1)   = (Tw3-Ftxrr.*pusr.Rt(3))./pusr.Jwy(3); % eq9 ddomegaRr
    state_k1(10,1)   = (Tw4-Ftxrl.*pusr.Rt(4))./pusr.Jwy(4); % eq10 ddomegaRl
    
    state_k1(1,1)     = dXb.*cos(Yaw)-dYb.*sin(Yaw); %  % eq1 dgXb用于输出。dXb用于计算 dgXb
    state_k1(3,1)     = dXb.*sin(Yaw)+dYb.*cos(Yaw); % % eq3  dgYb
    state_k1(5,1)     = dYaw; % eq5  dgYaw
    

%% 求解完了 求矩阵

    state_NL = state_k1;%state_NL = state_k1./T; 推导中的意思是，statek1要乘T，这里之前没乘，所以现在NL不用除
    state_L = ac*Xcur(1:10,1);
%     Err = (state_NL-state_L)*T-b*U1;
%     KK = Err; 
    d_k = (state_NL-state_L)*T-b*U1;
    d_piao_k=zeros(Nx+Nu,1);%d_k的增广形式，参考falcone(B,4c)%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%？？？？？？？？？？？
    d_piao_k(1:10,1)=d_k;
    d_piao_k(11:15,1)=0;
%% ABC right    
    A_cell=cell(2,2);
    B_cell=cell(2,1);
    A_cell{1,1}=a;
    A_cell{1,2}=b;
    A_cell{2,1}=zeros(Nu,Nx);
    A_cell{2,2}=eye(Nu);
    B_cell{1,1}=b;
    B_cell{2,1}=eye(Nu);
    A=cell2mat(A_cell);
    B=cell2mat(B_cell);
    C = [0 1 0 0 0 0 0 0 0 0 0 0 0 0 0
             0 0 1 0 0 0 0 0 0 0 0 0 0 0 0];
%     C = [0 1 0 0 0 0
%         0 0 1 0 0 0];

    %A=zeros(Nu+Nx,Nu+Nx);
% ABC right and done
%% psi phi theta G（可忽略）
    PSI_cell=cell(Np,1);
    THETA_cell=cell(Np,Nc);
    GAMMA_cell=cell(Np,Np);
    PHI_cell=cell(Np,1);

    for p=1:1:Np
        PHI_cell{p,1}=d_piao_k; % (7-32)理论上来说，这个是要实时更新Np次的，但是为了简便，这里又一次近似
        for q=1:1:Np
            if q<=p
                GAMMA_cell{p,q}=C*A^(p-q); % right(7-31) 
            else 
                GAMMA_cell{p,q}=zeros(Ny,Nx+Nu); % right(7-31) 
            end 
        end
    end
    for j=1:1:Np
    PSI_cell{j,1}=C*A^j; % right(7-28)
        for k=1:1:Nc  %Nc
            if k<=j
                THETA_cell{j,k}=C*A^(j-k)*B; % right(7-29)
            else 
                THETA_cell{j,k}=zeros(Ny,Nu); % right(7-29)
            end
        end
    end
    PSI=cell2mat(PSI_cell);%size(PSI)=[Ny*Np Nx+Nu]
    THETA=cell2mat(THETA_cell);%size(THETA)=[Ny*Np Nu*Nc]
    GAMMA=cell2mat(GAMMA_cell);%大写的GAMMA   %G: 二次规划问题中的代价函数的 Hessian 矩阵。??
    PHI=cell2mat(PHI_cell);
    Q=cell2mat(Q_cell);
%     H_cell=cell(1,1);
%     H_cell{1,1}=THETA'*Q*THETA+R;
% %     H_cell{1,2}=zeros(Nu*Nc,1);
% %     H_cell{2,1}=zeros(1,Nu*Nc);
% %     H_cell{2,2}=Row;
%     H=cell2mat(H_cell);
    H=THETA'*Q*THETA+R;
    %%
    error_1=zeros(Ny*Np,1);
    Yita_ref_cell=cell(Np,1);
    for p=1:1:Np
        if t+p*T>T_all
            X_DOT=x_dot*cos(phi)-y_dot*sin(phi);%惯性坐标系下纵向速度

            X_predict(Np,1)=X+X_DOT*Np*T;
            %X_predict(Np,1)=X+X_dot*Np*t;
            z1=shape/dx1*(X_predict(Np,1)-Xs1)-shape/2;
            z2=shape/dx2*(X_predict(Np,1)-Xs2)-shape/2;
            Y_ref(p,1)=dy1/2*(1+tanh(z1))-dy2/2*(1+tanh(z2));
%             phi_ref(p,1)=atan(dy1*(1/cosh(z1))^2*(1.2/dx1)-dy2*(1/cosh(z2))^2*(1.2/dx2));
            Yita_ref_cell{p,1}=[Y_ref(p,1);10];
            
        else
            X_DOT=x_dot*cos(phi)-y_dot*sin(phi);%惯性坐标系下纵向速度
            X_predict(p,1)=X+X_DOT*p*T;%首先计算出未来X的位置，X(t)=X+X_dot*t
            z1=shape/dx1*(X_predict(p,1)-Xs1)-shape/2;
            z2=shape/dx2*(X_predict(p,1)-Xs2)-shape/2;
            Y_ref(p,1)=dy1/2*(1+tanh(z1))-dy2/2*(1+tanh(z2));
%             phi_ref(p,1)=atan(dy1*(1/cosh(z1))^2*(1.2/dx1)-dy2*(1/cosh(z2))^2*(1.2/dx2));
            Yita_ref_cell{p,1}=[Y_ref(p,1);10];

        end
    end
    kesi = zeros(Nu+Nx,1);
    kesi(1:10,1)=Xcur;
    kesi(11:15,1)=U1;
    Yita_ref=cell2mat(Yita_ref_cell);
    error_1=Yita_ref-PSI*kesi-GAMMA*PHI; %求偏差
%     f_cell=cell(1,1);
%      f_cell{1,1}=2*error_1'*Q*THETA;
%     f_cell{1,2}=0;
%     f=(cell2mat(f_cell))';
%     f=-cell2mat(f_cell);
    f=-2*error_1'*Q*THETA;
    %%

 %% 以下为约束生成区域
umin = [-4000;-4000;-4000;-4000;-0.42];   % 控制量上下限 umin 和 umax
umax = [4000;4000;4000;4000;0.42];     % control variables maximum
umin = [-400;-400;-400;-400;-0.01];   % 控制量上下限 umin 和 umax
umax = [400;400;400;400;0.01];     % control variables maximum
lb = kron(ones(Nc,1),umin);% control variables' lower boundary
ub = kron(ones(Nc,1),umax);% control variables' upper boundary
A_Cons1 = kron(tril(ones(Nc),0),eye(Nu));
Umax = kron(ones(Nc,1),umax);
Umin = kron(ones(Nc,1),umin);
U11 = kron(ones(Nc,1),U1);
A_Cons = [A_Cons1;-A_Cons1];
Ypre_max = [5;10];
Ypre_min = [0;0];
Lb = lb;                        %Lb, Ub: 控制量的上下界。
Ub = ub;
A1 = A_Cons;                    %控制量的约束矩阵。
ubA = [Umax-U11;-Umin+U11];         %lbA, ubA: 控制量约束矩阵的上下界。
lbA = -inf*ones(size(ubA));     
Count = 0; % 或者任何你认为合适的默认值 迭代次数？
    %% 开始求解过程
%        options = optimset('Algorithm','active-set');
%        options = optimoptions('quadprog', 'Display', 'off', 'MaxIter', 500);
       options = optimset('Algorithm','interior-point-convex'); 
       x_start=zeros(Nc*2,1);%加入一个起始点   %%%%%%%%%%%%%%%%%%%%%%%%%%%%x_start=zeros(Nc+1,1);
%       [X,fval,exitflag]=quadprog(G,w,[],[],[],[],Lb,Ub,x_start,options);%quadprog(H,f,A_cons,b_cons,[],[],lb,ub,x_start,options);
      [X,fval,exitflag]=quadprog(H,f,[],[],[],[],[],[],x_start,options);
%       fprintf('exitflag=%d\n',exitflag);
%       fprintf('H=%4.2f\n',G(1,1));
%       fprintf('f=%4.2f\n',w(1,1));
    %% 计算输出
%     Fx_piao=X(1);
%     delta_piao=X(2);%得到控制增量
    U(1) = U1(1,1) + X(1);
    U(2) = U1(2,1) + X(3);
    U(3) = U1(3,1) + X(3);
    U(4) = U1(4,1) + X(4);
    U(5) = U1(5,1) + X(5);
%     U(1)=U1(1,1) + Fx_piao;         %当前时刻的控制量为上一刻时刻控制+控制增量j
%     U(2)=U1(2,1) + delta_piao;          %10;%
%Tw1-steer
    
   %U(2)=Yita_ref(2);%输出dphi_ref
    sys= U;
    toc
% End of mdlOutputs.


