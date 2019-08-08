function cost = Autopilot_cost_function(x, trajref, ...
    veh_pose, ref_pose, autop_params, index, veh_params,N,R,Q)
% delta is control variable: steer angle

%% Initial state
v_des=veh_params.v_des; % desired speed m/s
xx=veh_pose(1,1); % current x position
yy=veh_pose(1,2); % current y position
psi=veh_pose(1,3); % current vehicle heading angle
%delta   % steering angle

cost=0;        % cost function

x_predict=zeros(N,1);
y_predict=zeros(N,1);
psi_predict=zeros(N,1);

x_error=zeros(N+1,1);
y_error=zeros(N+1,1);
psi_error=zeros(N+1,1);

delta=zeros(N,1);    % control variable delta is steering angle

% set up Non uniform time grid
% First 5 timesteps are 0.05, after that it's 0.15
T=zeros(N,1);
for i=1:5
    T(i,1)=0.05;
end
for i=5:N
    T(i,1)=0.05;
    %T(i,1)=0.15;    %autopilot variable
end

%curvature factor [1/m]
 curv_factor=1/(1-autop_params.sf*v_des^2)/veh_params.wheel_base;


%% Update the state 

for i =1: 1: N
    
    if i == 1
        delta(i,1)= x(1);
        x_predict(i,1)=xx+T(i,1)*v_des*cos(psi);
        y_predict(i,1)=yy+T(i,1)*v_des*sin(psi);
         psi_predict(i,1)=psi+T(i,1)*v_des*delta(i,1)...
            *curv_factor;
        
%         psi_predict(i,1)=psi+T(i,1)*v_des*tan(delta(i,1))...
%             /veh_params.wheel_base;
    else
        delta(i,1)=x(1);
        x_predict(i,1)=x_predict(i-1)+T(i,1)*v_des*cos(psi_predict(i-1));
        y_predict(i,1)=y_predict(i-1)+T(i,1)*v_des*sin(psi_predict(i-1));

         psi_predict(i,1)= psi_predict(i-1)+T(i,1)*v_des*delta(i,1)...
            *curv_factor;
         
%         psi_predict(i,1)= psi_predict(i-1)+T(i,1)*v_des*tan(delta(i,1))...
%             /veh_params.wheel_base;

    end

    %calculate the tracking error based on predit horizon
    x_real=zeros(N+1,1);
    y_real=zeros(N+1,1);
    x_real(1,1) = xx;
    x_real(2:N+1,1) = x_predict;
    y_real(1,1) = yy;
    y_real(2:N+1,1) = y_predict;

    x_error(i,1)=x_real(i,1)-trajref(i,1);
    y_error(i,1)=y_real(i,1)-trajref(i,2);
    psi_error(i,1)=psi_predict(i,1)-trajref(i,3);
    
end
    i=i+1;
    x_error(i,1)=x_real(i,1)-trajref(i,1);
    y_error(i,1)=y_real(i,1)-trajref(i,2);
    
%calculate the cost function
cost=cost+y_error'*R*y_error+ x_error'*Q*x_error;


end