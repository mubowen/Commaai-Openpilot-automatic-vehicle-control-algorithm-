function   autop_params =  load_autopilot_params(veh_params)
% 设置MPC的参数
 
% 输入:
% veh_params : vehicle的参数
% set up vehicle parameters 
 cf = 155494.663;        %前轮侧偏刚度(cornering stiffness)  
 cr = 155494.663;        %后轮侧偏刚度
 
 mass_fl = 520;          %前悬长度
 mass_fr = 520;
 mass_rl = 520;
 mass_rr = 520;
 
 chi=0;                  %steer ratio rear
 steer_ratio=1;
 eps = 0.01;
 cutoff_freq = 10;
 max_iteration = 150;
 % 当前线速度
 v=veh_params.velocity;
 
 mass_front =  mass_fl + mass_fr;
 mass_rear =  mass_rl + mass_rr;
 mass = mass_front + mass_rear;
 
 lf = veh_params.wheel_base * (1.0 - mass_front / mass); %前悬长度
 lr = veh_params.wheel_base * (1.0 - mass_rear / mass);  %后悬长度
 iz = lf * lf * mass_front + lr * lr * mass_rear; %车辆绕z轴转动的转动惯量
 
 wheel_max_degree=veh_params.max_steer_angle;
 
 %The slip factor is a measure of how the curvature changes with speed
 %it's positive for Oversteering vehicle, negative (usual case) otherwise.
 sf=mass*(cf*lf-cr*lr)/(cf*cr*veh_params.wheel_base^2);
 autop_params.sf=sf;

 % 当前线速度
 v=veh_params.velocity; 
 
 if v>0.1
     % dynamic system
    
     % set up matrix A and B
     A=zeros(2,2);
     B=zeros(2,1);

     % set up A without speed
     A(1,1)=-(cf+cr)/(mass);
     A(1,2)=-(cf*lf-cr*lr)/(mass);
     A(2,1)=-(cf*lf-cr*lr)/(iz);
     A(2,2)=-(cf*lf^2+cr*lr^2)/(iz);

     % set up B without speed
     B(1,1)=(cf+chi*cr)/(mass*steer_ratio);
     B(2,1)=(cf*lf-chi*cr*lr)/(iz*steer_ratio);

     % update velocity for A matrix
     A(1,1)=A(1,1)/v;
     A(1,2)=A(1,2)/v-v;
     A(2,1)=A(2,1)/v;
     A(2,2)=A(2,2)/v;

     % solve for the state x=[v,r]' 
     % with v lateral speed [m/s], and r rotational speed [rad/s]
     % Calculate the steady state solution when x_dot = 0,
     % Ax + Bu = 0 => x = A^{-1} B u
     state=-(A\B); %* steer_angle;
 
 else 
     % Calculate the steady state solution at low speeds
 % At low speeds the tire slip is undefined, so a kinematic
 % model is used.
     K=zeros(2,2);
     K(1,1)=lr/steer_ratio/veh_params.wheel_base * v;
     K(2,1)=1/steer_ratio/veh_params.wheel_base * v;
     
     
     state=K; %* steer_angle;
 end
 
end