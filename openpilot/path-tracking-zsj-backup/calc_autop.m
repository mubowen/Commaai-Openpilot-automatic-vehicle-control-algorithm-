function steer_cmd =  calc_autop(trajref, delta_x,...
    veh_pose, ref_pose, autop_params, index, veh_params)
% 输出:
% steering_cmd      : 前轮偏角反馈控制量, rad
% acc               : 加速度补偿 m/s^2
% steer_feedforward ：前轮偏角，rad

% 输入:

% trajref           : 期望路径[X, Y, Theta, Radius]
% delta_x           : 期望位姿与车辆当前位姿的偏差[dx, dy, dtheta]
% veh_pose          : 车辆当前位姿[x, y, theta]
% mpc_params        : MPC的参数
% index             : 滚动优化在trajref的初始index
% veh_params        : 车辆参数

%% update parameters
NX = 4; % number of different state variable
NU = 1; % number of control variable
N = 20; % number of intervals in the horizon

% set up the weight matrix
Q=60*eye(N+1,N+1);
R=100*eye(N+1,N+1);

%control variable
steer_cmd=zeros(NU,1);

% lb=[0.8; -0.44;0.8;-0.44];
% ub=[1.2;0.44;1.2;0.44];
lb=[-0.28];
ub=[0.28];
A=[];
b=[];
Aeq=[];
Beq=[];

options=optimset('Algorithm','active-set');
[A,fval,exitflag]=fmincon(@(x)Autopilot_cost_function(...
    x,trajref(index:index+N+1,1:3), ...
    veh_pose, ref_pose, autop_params, index, veh_params,N,R,Q)...
    ,[0],A,b,Aeq,Beq,lb,ub,[],options);

steer_cmd=A(1);

end
  
  
  
  
  
  
 
 
 