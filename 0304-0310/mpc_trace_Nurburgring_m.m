function [sys,x0,str,ts] = chapter5_2_2(t,x,u,flag)
% 该程序功能：用LTV MPC 和车辆简化动力学模型（小角度假设）设计控制器，作为Simulink的控制器
% 程序版本 V1.0，MATLAB版本：R2011a,采用S函数的标准形式，
% 程序编写日期 2013.12.11
% 最近一次改写 2013.12.16
% 状态量=[y_dot,x_dot,phi,phi_dot,Y,X]，控制量为前轮偏角delta_f


switch flag,
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
sizes.NumOutputs     = 1;%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%2
sizes.NumInputs      = 8;
sizes.DirFeedthrough = 1; % Matrix D is non-empty.？？？？？？？？？？？？？？？？？？？？？？？
sizes.NumSampleTimes = 1;
sys = simsizes(sizes); 
x0 =[0.001;0.0001;0.0001;0.00001;0.00001;0.00001];    
%x0 =[10;10;10;10;10;10];    
global U;
U=[0];%控制量初始化              ,这里面加了一个期望轨迹的输出，如果去掉，U为一维的


% 
% % 生成参考轨迹
% for i = 1:Np
%     % 从路点数据中获取横纵坐标
%     X_ref = Xcoord(i);
%     Y_ref = Ycoord(i);
%     
%     % 将横纵坐标保存到参考轨迹数组中
%     X_predict(i) = X_ref;
%     Y_predict(i) = Y_ref;
% end

% global x;
% x = zeros(md.ne + md.pye + md.me + md.Hu*md.me,1);   
% Initialize the discrete states.
str = [];             % Set str to an empty matrix.
ts  = [0.02 0];       % sample time: [period, offset]
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
    %global u_piao;
    global U;
    %global kesi;
    tic
    Nx=6;%状态量的个数
    Nu=1;%控制量的个数                                         %%%%%%%%%%%%%%%%%%%%%%%%%%%%2
    Ny=2;%输出量的个数
    Np = 10;%预测步长
    Nc = 5;%控制步长
    Row=1000;%松弛因子权重
    fprintf('Update start, t=%6.3f\n',t)
    %导入离散路点
    filePath = 'use_0301_199981x3.mat'; % 路点文件名（或者加上路径  999901x3   10000x3  199981x3 59995x3
    data = load(filePath); % 加载路点文件
    xcoord = data.xcoord;   
    ycoord= data.ycoord; 
%     scoord= data.scoord;

    %输入接口转换,x_dot后面加一个非常小的数，是防止出现分母为零的情况 %%%%%%%%%状态量从数组里读取到变量里，同时变换单位
   % y_dot=u(1)/3.6-0.000000001*0.4786;%CarSim输出的是km/h，转换为m/s
    y_dot=u(1)/3.6;
    x_dot=u(2)/3.6+0.0001;%CarSim输出的是km/h，转换为m/s
    phi=u(3)*3.141592654/180; %CarSim输出的为角度，角度转换为弧度
    phi_dot=u(4)*3.141592654/180;
    Y=u(5);%单位为m
    X=u(6);%单位为米
    Y_dot=u(7);
    X_dot=u(8);
%     journey=u(9);
%     fprintf('journey = %f\n', journey);
%% 车辆参数输入
%syms sf sr;%分别为前后车轮的滑移率,需要提供     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%为什么需要提供
    Sf=0.2; Sr=0.2;
%syms lf lr;%前后车轮距离车辆质心的距离，车辆固有参数
    lf=1.232;lr=1.468;
%syms C_cf C_cr C_lf C_lr;%分别为前后车轮的纵横向侧偏刚度，车辆固有参数
    Ccf=66900;Ccr=62700;Clf=66900;Clr=62700;
%syms m g I;%m为车辆质量，g为重力加速度，I为车辆绕Z轴的转动惯量，车辆固有参数
    m=1723;g=9.8;I=4175;
   

%% 参考轨迹生成
    shape=2.4;%参数名称，用于参考轨迹生成                            %含义在仿真结果部分介绍
    dx1=25;dx2=21.95;%没有任何实际意义，只是参数名称
    dy1=4.05;dy2=5.7;%没有任何实际意义，只是参数名称
    Xs1=27.19;Xs2=56.46;%参数名称
    X_predict=zeros(Np,1);%用于保存预测时域内的纵向位置信息，这是计算期望轨迹的基础
    Y_predict=zeros(Np,1);
%     journey_predict = zeros(Np,1); %新加入，用于保存时域内的路程信息，这是计算期望轨迹的基础
    phi_ref=zeros(Np,1);%用于保存预测时域内的期望轨迹
    Y_ref=zeros(Np,1);%用于保存预测时域内的期望轨迹
    


    
    %  以下计算kesi,即状态量与控制量合在一起   
    kesi=zeros(Nx+Nu,1);
    kesi(1)=y_dot;%u(1)==X(1)
    kesi(2)=x_dot;%u(2)==X(2)
    kesi(3)=phi; %u(3)==X(3)
    kesi(4)=phi_dot;
    kesi(5)=Y;
    kesi(6)=X;
    kesi(7)=U(1);
    delta_f=U(1);
    fprintf('Update start, u(1)=%4.2f\n',U(1))

    T=0.02;%仿真步长
    T_all=50;%总的仿真时间，主要功能是防止计算期望轨迹越界
     
    %权重矩阵设置 
    Q_cell=cell(Np,Np);
    for i=1:1:Np
        for j=1:1:Np
            if i==j
                %Q_cell{i,j}=[200 0;0 100;];
                Q_cell{i,j}=[10000 0;0 5000;];
            else 
                Q_cell{i,j}=zeros(Ny,Ny);               
            end
        end 
    end 
    %R=5*10^4*eye(Nu*Nc);
    R=50000*eye(Nu*Nc);
%     R =[     5000 ,          0    ,       0      ,     0     ,      0;
%            0   ,   5000    ,       0      ,     0      ,     0;
%            0    ,       0  ,    5000      ,     0   ,        0;
%            0    ,       0      ,     0   ,   5000      ,     0;
%            0    ,       0      ,     0      ,     0  ,   5000];


    %最基本也最重要的矩阵，是控制器的基础，采用动力学模型，该矩阵与车辆参数密切相关，通过对动力学方程求解雅克比矩阵得到
    ac = genA_1_by_MPC_bool(X, x_dot, Y, y_dot, phi, phi_dot, Ccf, Ccr, I, lf, lr, delta_f, m );
    a = ac*T+eye(size(ac));
%     a=[                 1 - (259200*T)/(m*x_dot),                                                         -T*(phi_dot + (2*((460218*phi_dot)/5 - 62700*y_dot))/(1723*x_dot^2) - (133800*((154*phi_dot)/125 + y_dot))/(1723*x_dot^2)),                                    0,                     -T*(x_dot - 96228/(8615*x_dot)), 0, 0
%         T*(phi_dot - (133800*delta_f)/(1723*x_dot)),                                                                                                                  (133800*T*delta_f*((154*phi_dot)/125 + y_dot))/(1723*x_dot^2) + 1,                                    0,           T*(y_dot - (824208*delta_f)/(8615*x_dot)), 0, 0
%                                                   0,                                                                                                                                                                                  0,                                    1,                                                   T, 0, 0
%             (33063689036759*T)/(7172595384320*x_dot), T*(((2321344006605451863*phi_dot)/8589934592000 - (6325188028897689*y_dot)/34359738368)/(4175*x_dot^2) + (5663914248162509*((154*phi_dot)/125 + y_dot))/(143451907686400*x_dot^2)),                                   0, 1 - (813165919007900927*T)/(7172595384320000*x_dot), 0, 0
%                                           T*cos(phi),                                                                                                                                                                         T*sin(phi),  T*(x_dot*cos(phi) - y_dot*sin(phi)),                                                   0, 1, 0
%                                          -T*sin(phi),                                                                                                                                                                         T*cos(phi), -T*(y_dot*cos(phi) + x_dot*sin(phi)),                                                   0, 0, 1];
   bc = genB_1_by_MPC_bool(X, x_dot, Y, y_dot, phi, phi_dot, Ccf, Ccr, I, lf, lr, delta_f, m ,Clf, Clr, Sf, Sr);
   b =T*bc;
%     b=[                                                               133800*T/1723
%        T*((267600*delta_f)/1723 - (133800*((154*phi_dot)/125 + y_dot))/(1723*x_dot))
%                                                                                  0
%                                                 5663914248162509*T/143451907686400
%                                                                                  0
%                                                                                  0];  
%     b=[                                                               133800*T/1723
%        T*((267600*delta_f)/1723 - (133800*((154*phi_dot)/125 + y_dot))/(1723*x_dot))
%                                                                                  0
%                                                 5663914248162509*T/143451907686400
%                                                                                  0
%                                                                                  0];  
    d_k=zeros(Nx,1);%计算偏差
    state_k1=zeros(Nx,1);%预测的下一时刻状态量，用于计算偏差
    %以下即为根据离散非线性模型预测下一时刻状态量
    %注意，为避免前后轴距的表达式（a,b）与控制器的a,b矩阵冲突，将前后轴距的表达式改为lf和lr
    state_k1(1,1)=y_dot+T*(-x_dot*phi_dot+2*(Ccf*(delta_f-(y_dot+lf*phi_dot)/x_dot)+Ccr*(lr*phi_dot-y_dot)/x_dot)/m);
    state_k1(2,1)=x_dot+T*(y_dot*phi_dot+2*(Clf*Sf+Clr*Sr+Ccf*delta_f*(delta_f-(y_dot+phi_dot*lf)/x_dot))/m);
    state_k1(3,1)=phi+T*phi_dot;
    state_k1(4,1)=phi_dot+T*((2*lf*Ccf*(delta_f-(y_dot+lf*phi_dot)/x_dot)-2*lr*Ccr*(lr*phi_dot-y_dot)/x_dot)/I);
    state_k1(5,1)=Y+T*(x_dot*sin(phi)+y_dot*cos(phi));
    state_k1(6,1)=X+T*(x_dot*cos(phi)-y_dot*sin(phi));
    d_k=state_k1-a*kesi(1:6,1)-b*kesi(7,1);%根据falcone公式（2.11b）求得d(k,t)
    d_piao_k=zeros(Nx+Nu,1);%d_k的增广形式，参考falcone(B,4c)%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%？？？？？？？？？？？
    d_piao_k(1:6,1)=d_k;
    d_piao_k(7,1)=0;
    
    A_cell=cell(2,2);
    B_cell=cell(2,1);
    A_cell{1,1}=a;
    A_cell{1,2}=b;
    A_cell{2,1}=zeros(Nu,Nx);
    A_cell{2,2}=eye(Nu);
    B_cell{1,1}=b;
    B_cell{2,1}=eye(Nu);
    %A=zeros(Nu+Nx,Nu+Nx);
    A=cell2mat(A_cell);
    B=cell2mat(B_cell);
   % C=[0 0 1 0 0 0 0;0 0 0 1 0 0 0;0 0 0 0 1 0 0;];%这是和输出量紧密关联的
    C=[0 0 1 0 0 0 0;
        0 0 0 0 1 0 0;];
    PSI_cell=cell(Np,1);
    THETA_cell=cell(Np,Nc);
    GAMMA_cell=cell(Np,Np);
    PHI_cell=cell(Np,1);
    for p=1:1:Np
        PHI_cell{p,1}=d_piao_k;%理论上来说，这个是要实时更新的，但是为了简便，这里又一次近似
        for q=1:1:Np
            if q<=p
                GAMMA_cell{p,q}=C*A^(p-q);
            else 
                GAMMA_cell{p,q}=zeros(Ny,Nx+Nu);
            end 
        end
    end
    for j=1:1:Np
     PSI_cell{j,1}=C*A^j;
        for k=1:1:Nc
            if k<=j
                THETA_cell{j,k}=C*A^(j-k)*B;
            else 
                THETA_cell{j,k}=zeros(Ny,Nu);
            end
        end
    end
    PSI=cell2mat(PSI_cell);%size(PSI)=[Ny*Np Nx+Nu]
    THETA=cell2mat(THETA_cell);%size(THETA)=[Ny*Np Nu*Nc]
    GAMMA=cell2mat(GAMMA_cell);%大写的GAMMA   %G: 二次规划问题中的代价函数的 Hessian 矩阵。??
    PHI=cell2mat(PHI_cell);
    Q=cell2mat(Q_cell);
    H_cell=cell(2,2);
    H_cell{1,1}=THETA'*Q*THETA+R;
    H_cell{1,2}=zeros(Nu*Nc,1);
    H_cell{2,1}=zeros(1,Nu*Nc);
    H_cell{2,2}=Row;
    H=cell2mat(H_cell);
    error_1=zeros(Ny*Np,1);
    Yita_ref_cell=cell(Np,1);
    qianhou = 200; %MPC跟踪路点fai的前后点数 %如果是10000,3差不多。如果是999901。可以给一个50了吧
%     final_index = 0.1
    for p=1:1:Np
            X1_DOT=x_dot*cos(phi)-y_dot*sin(phi); %这个求解的是全局坐标系 x 方向的速度
            Y1_DOT = x_dot*sin(phi) + y_dot*cos(phi);

            X_predict(p,1) = X + X1_DOT*p*T; %这里之前不小心用了t。错了呀
            Y_predict(p,1) = Y + Y1_DOT*p*T; %
            distances = (xcoord - X_predict(p,1)).^2 + (ycoord - Y_predict(p,1)).^2; %用长度集减路径集
            %以下用于寻找精准的路点
            [min_distance, min_positive_index] = min(distances);

%             fu_distances = distances(distances <= 0);
%             min_positive_index = length(fu_distances) + 1
            if min_positive_index <2
                min_positive_index = min_positive_index +1
            end
%           fprintf('====%d\n',X);
%             fu_value1 = xcoord(min_positive_index - 1 ); % 获取最小距离对应的前一个xcoord值
%             zheng_value2 = xcoord(min_positive_index); % 获取最小距离对应的当前xcoord值
%             final_index = min_positive_index - 1.0 + (X_predict(p,1) - fu_value1)/(zheng_value2 - fu_value1); 
%             
            %使用路点找准确的y
%             fu_y1 = ycoord(min_positive_index-1); % 获取最小距离对应的前一个ycoord值
%             zheng_y2 = ycoord(min_positive_index); % 获取最小距离对应的当前ycoord值
%             Y_ref(p,1) = fu_y1 + (zheng_y2 - fu_y1)*(final_index - min_positive_index -1); 
            % 计算航向角参考值 phi_ref
            Y_ref(p,1) = ycoord(min_positive_index);
            if min_positive_index >= qianhou+1 && min_positive_index < length(xcoord) - qianhou
                phi_ref(p, 1) = atan2((ycoord(min_positive_index + qianhou) - ycoord(min_positive_index - qianhou)), ...
                     (xcoord(min_positive_index + qianhou) - xcoord(min_positive_index - qianhou)));
                 if abs(phi_ref(p, 1)) > pi/2
                    % 检查 phi_ref(p, 1) 的正负
                    if phi_ref(p, 1) > 0 && phi < 0
                        % phi_ref(p, 1) 大于 0 且 phi 小于 0，phi 加 2pi
                        phi_ref(p, 1) = phi_ref(p, 1) - (2 * pi);
                    elseif phi_ref(p, 1) < 0 && phi > 0
                        % phi_ref(p, 1) 小于 0 且 phi 大于 0，phi 减 2pi
                        phi_ref(p, 1) = phi_ref(p, 1) + (2 * pi);
                    end
                end
            else
                phi_ref(p, 1) = 0; % 或者根据您的需求设置默认值
            end

            

            Yita_ref_cell{p,1}=[phi_ref(p,1);Y_ref(p,1)];
    end
    if mod(t, 1) == 0
        % 生成动态的文件名
        filename = sprintf('data_%d.mat', t);
    
        % 保存数据到本地文件
%         save(filename, 'journey_predict', 'Y_ref', 'phi_ref', 'X', 'T', 'PHI', 'final_index');
    end

    Yita_ref=cell2mat(Yita_ref_cell);
    error_1=Yita_ref-PSI*kesi-GAMMA*PHI; %求偏差
    f_cell=cell(1,2);
    f_cell{1,1}=2*error_1'*Q*THETA;
    f_cell{1,2}=0;
%     f=(cell2mat(f_cell))';
    f=-cell2mat(f_cell);
    
 %% 以下为约束生成区域
 %控制量约束
    A_t=zeros(Nc,Nc);%见falcone论文 P181
    for p=1:1:Nc
        for q=1:1:Nc
            if q<=p 
                A_t(p,q)=1;
            else 
                A_t(p,q)=0;
            end
        end 
    end 
    A_I=kron(A_t,eye(Nu));%求克罗内克积
    Ut=kron(ones(Nc,1),U(1));
    umin=-0.3488;%维数与控制变量的个数相同
    umax=0.3488;
    delta_umin=-0.0148;
    delta_umax=0.0148;
    Umin=kron(ones(Nc,1),umin);
    Umax=kron(ones(Nc,1),umax);
    
    %输出量约束
    ycmax=[0.21;5];
    ycmin=[-0.3;-3];
    Ycmax=kron(ones(Np,1),ycmax);
    Ycmin=kron(ones(Np,1),ycmin);
    
    %结合控制量约束和输出量约束
    A_cons_cell={A_I zeros(Nu*Nc,1);-A_I zeros(Nu*Nc,1);THETA zeros(Ny*Np,1);-THETA zeros(Ny*Np,1)};
    b_cons_cell={Umax-Ut;-Umin+Ut;Ycmax-PSI*kesi-GAMMA*PHI;-Ycmin+PSI*kesi+GAMMA*PHI};
    A_cons=cell2mat(A_cons_cell);%（求解方程）状态量不等式约束增益矩阵，转换为绝对值的取值范围
    b_cons=cell2mat(b_cons_cell);%（求解方程）状态量不等式约束的取值
    
    %状态量约束
    M=10; 
    delta_Umin=kron(ones(Nc,1),delta_umin);
    delta_Umax=kron(ones(Nc,1),delta_umax);
    lb=[delta_Umin;0];%（求解方程）状态量下界，包含控制时域内控制增量和松弛因子
    ub=[delta_Umax;M];%（求解方程）状态量上界，包含控制时域内控制增量和松弛因子
    
    %% 开始求解过程
       options = optimset('Algorithm','active-set');
%        options = optimset('Algorithm','interior-point-convex');
       x_start=zeros(Nc+1,1);%加入一个起始点
       [X,fval,exitflag]=quadprog(H,f,[],[],[],[],[],[],x_start,options);
      %[X,fval,exitflag]=quadprog(H,f,A_cons,b_cons,[],[],lb,ub,x_start,options);
      fprintf('exitflag=%d\n',exitflag);
      fprintf('H=%4.2f\n',H(1,1));
      fprintf('f=%4.2f\n',f(1,1));
    %% 计算输出
    u_piao=X(1);%得到控制增量
    U(1)=kesi(7,1)+u_piao;%当前时刻的控制量为上一刻时刻控制+控制增量
   %U(2)=Yita_ref(2);%输出dphi_ref
    sys= U;
    toc
% End of mdlOutputs.


