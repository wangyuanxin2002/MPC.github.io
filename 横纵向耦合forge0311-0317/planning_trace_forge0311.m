function [sys,x0,str,ts] = planning_trace_forge0311(t,x,u,flag)
% 该程序功能：用LTV MPC 和车辆简化动力学模型（小角度假设）设计控制器，作为Simulink的控制器
% 程序版本 V1.0，MATLAB版本：R2011a,采用S函数的标准形式，
% 程序编写日期 2013.12.11
% 最近一次改写 2013.12.16
% 状态量=[y_dot,x_dot,phi,phi_dot,Y,X]，控制量为前轮偏角delta_f               和Fx


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
sizes.NumDiscStates  = 6;
sizes.NumOutputs     = 2;%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%2   1;!1
sizes.NumInputs      = 6;
sizes.DirFeedthrough = 1; % Matrix D is non-empty.？？？？？？？？？？？？？？？？？？？？？？？
sizes.NumSampleTimes = 1;
sys = simsizes(sizes); 
x0 =[0.001;0.0001;0.0001;0.00001;0.00001;0.00001];    
global U;
U=[0 0];%控制量初始化              ,这里面加了一个期望轨迹的输出，如果去掉，U为一维的   U=[0];@2
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
    Nx = 6;%状态量的个数
    Nu = 2;%控制量的个数                                         %%%%%%%%%%%%%%%%%%%%%%%%%%%%2 原1 @3
%     Ny = 3;%输出量的个数
    Np = 40;%预测步长
    Nc = 20;%控制步长
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
    x_dot = u(2)/3.6+0.0001;  %CarSim输出的是km/h，转换为m/s
    Y = u(3);   %单位为m
    y_dot = u(4)/3.6;
    phi = u(5)*3.141592654/180;  %CarSim输出的为角度，角度转换为弧度
    phi_dot = u(6)*pi/180; 

    Fx = U(1);
    delta_f=U(2);
    fprintf('Update start, u(1)=%4.2f\n, u(2)=%4.2f\n',U(1),U(2))
    U1 = [Fx; delta_f]; 
     Xcur = [X; x_dot; Y; y_dot; phi; phi_dot];
%% 车辆参数输入
%syms sf sr;%分别为前后车轮的滑移率,需要提供     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%为什么需要提供
    Sf=0.2; Sr=0.2;
%syms lf lr;%前后车轮距离车辆质心的距离，车辆固有参数
    lf=1.232;lr=1.468;
%syms C_cf C_cr C_lf C_lr;%分别为前后车轮的纵横向侧偏刚度，车辆固有参数
    Ccf = -66900;Ccr = -62700;Clf = -66900;Clr = -62700;
%syms m g I;%m为车辆质量，g为重力加速度，I为车辆绕Z轴的转动惯量，车辆固有参数
    m=1723;g=9.8;I=4175;
    T=0.01;%仿真步长

    RefSt = [0 10 2 0 0 0]';%参考状态 R efSt R efSt = [0 10 0 0 0 0]';
    RefSt(2) = 10;
    RefSt(3) = 0.1*t;

    %权重矩阵设置 
%     q = [1e3 1e5]; 
%     Rr = [1e-6 1e5];
    q = [10 10]; Rr = [1e-6 1e3];
    %最基本也最重要的矩阵，是控制器的基础，采用动力学模型，该矩阵与车辆参数密切相关，通过对动力学方程求解雅克比矩阵得到
    ac = genA_1_by_MPC_threedof_forge0312(X, x_dot, Y, y_dot, phi, phi_dot, Ccf, Ccr, I, lf, lr, delta_f, m ,Clf, Clr, Sf,Sr, Fx);
    a = ac*T+eye(size(ac));
    
    bc = genB_1_by_MPC_threedof_forge0312(X, x_dot, Y, y_dot, phi, phi_dot, Ccf, Ccr, I, lf, lr, delta_f, m ,Clf, Clr, Sf,Sr,Fx);
    b =T*bc;

    d_k=zeros(Nx,1);%计算偏差
    state_k1=zeros(Nx,1);%预测的下一时刻状态量，用于计算偏差
    state_k1(4,1)=T*((Ccr+Ccf*cos(delta_f))*y_dot/m/x_dot+(Ccf*lf*cos(delta_f)-Ccr*lr)*phi_dot/m/x_dot-Ccf*cos(delta_f)*delta_f/m+sin(delta_f)*Fx/m-phi_dot*x_dot);
    state_k1(2,1)=T*(-(Ccf*sin(delta_f))*y_dot/m/x_dot-(Ccf*lf*sin(delta_f))*phi_dot/m/x_dot+Ccf*sin(delta_f)*delta_f/m+Fx*cos(delta_f)/m+phi_dot*y_dot);
    state_k1(5,1)=T*phi_dot;
    state_k1(6,1)=T*((Ccf*lf*cos(delta_f)-Ccr*lr)*y_dot/I/x_dot+(Ccf*lf^2*cos(delta_f)+Ccr*lr^2)*phi_dot/I/x_dot-Ccf*lf*cos(delta_f)*delta_f/I+lf*sin(delta_f)*Fx/I);
    state_k1(3,1)=T*(x_dot*sin(phi)+y_dot*cos(phi));
    state_k1(1,1)=T*(x_dot*cos(phi)-y_dot*sin(phi));

    state_NL = state_k1./T;
    state_L = ac*Xcur;
    Err = (state_NL-state_L)*T;
    KK = Err; 
    C = [0 1 0 0 0 0
        0 0 1 0 0 0];
P = kron(ones(Np,1),zeros(size(C*a)));
H = kron(zeros(Np,Nc),zeros(size(C*b)));
E = kron(ones(Np,1),zeros(size(C*a)));
for i = 1:Np
    P((i-1)*size(C,1)+1:i*size(C,1),:) = C*a^(i-1);
    if i==1
        E((i-1)*size(C,1)+1:i*size(C,1),:) = 0;
    else
        for n = 2:i
            E((i-1)*size(C,1)+1:i*size(C,1),:) = E((i-1)*size(C,1)+1:i*size(C,1),:)+C*a^(i-n);
        end
    end
    for j = 1:Nc
        if j<=i
            if i==1
                H((i-1)*size(C*b,1)+1:i*size(C*b,1),(j-1)*size(C*b,2)+1:j*size(C*b,2)) = zeros(size(C*b));
            else
                if j==i
                    H((i-1)*size(C*b,1)+1:i*size(C*b,1),(j-1)*size(C*b,2)+1:j*size(C*b,2)) = zeros(size(C*b));
                else
                    H((i-1)*size(C,1)+1:i*size(C,1),(j-1)*size(C*b,2)+1:j*size(C*b,2)) = C*a^(i-j-1)*b;
                end
            end
        else
            H((i-1)*size(C*b,1)+1:i*size(C*b,1),(j-1)*size(C*b,2)+1:j*size(C*b,2)) = zeros(size(C*b));
        end
    end
end
Q = kron(eye(Np),diag([q(1),q(2)]));%定义了代价函数的权重矩阵。这部分代码定义了状态权重矩阵Q和控制权重矩阵R
R_r = diag([Rr(1),Rr(2)]);
R = kron(eye(Nc),R_r);
K = E*KK-kron(ones(Np,1),C*RefSt);%%%%%%RefSt 参考状态在这里，与当前矩阵求一下误差矩阵，就行了
G1 = (H'*Q*H+R);
G = G1;                             %G: 二次规划问题中的代价函数的 Hessian 矩阵。
g1 = ( P*Xcur+K)'*Q*H;
w = g1(:);                          %w: 二次规划问题中的线性项。

 %% 以下为约束生成区域
 umin = [-4000;-0.42];   % 控制量上下限 umin 和 umax
umax = [4000;0.42];     % control variables maximum
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
      [X,fval,exitflag]=quadprog(G,w,[],[],[],[],Lb,Ub,x_start,options);%quadprog(H,f,A_cons,b_cons,[],[],lb,ub,x_start,options);
      fprintf('exitflag=%d\n',exitflag);
      fprintf('H=%4.2f\n',G(1,1));
      fprintf('f=%4.2f\n',w(1,1));
    %% 计算输出
%     Fx_piao=X(1);
%     delta_piao=X(2);%得到控制增量
    U(1) = X(1);
    U(2) = X(2);
%     U(1)=U1(1,1) + Fx_piao;         %当前时刻的控制量为上一刻时刻控制+控制增量j
%     U(2)=U1(2,1) + delta_piao;          %10;%

    
   %U(2)=Yita_ref(2);%输出dphi_ref
    sys= U;
    toc
% End of mdlOutputs.


