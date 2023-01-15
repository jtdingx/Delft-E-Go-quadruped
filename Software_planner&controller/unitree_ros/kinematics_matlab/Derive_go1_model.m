clear; clc;

syms q_hip q_thigh q_calf 'real'   % states

syms leg_offset_x leg_offset_y thigh_y thigh_length calf_length

P_hip = sym([leg_offset_x;leg_offset_y; 0]);%%% hip->body
P_thigh = sym([0;thigh_y; 0]);              %%% thigh->hip
P_calf = sym([0;0;thigh_length]);           %%% calf->thigh
P_feet = sym([0;0;calf_length]);            %%% feet->calf

%%% Rx, Ry, Rz;
R_hip = [1,0,0;0,cos(q_hip), -sin(q_hip); 0, sin(q_hip), cos(q_hip)]; 

R_thigh = [cos(q_thigh),0,sin(q_thigh);0,1,0;-sin(q_thigh),0,cos(q_thigh)]; 

R_calf = [cos(q_calf),0,sin(q_calf);0,1,0;-sin(q_calf),0,cos(q_calf)]; 


%% global Kinematics:
syms body_px body_py body_pz body_r body_p body_y 'real'
R_bodyx = [1,0,0;0,cos(body_r), -sin(body_r); 0, sin(body_r), cos(body_r)]; 

R_bodyy = [cos(body_p),0,sin(body_p);0,1,0;-sin(body_p),0,cos(body_p)]; 

R_bodyz = [cos(body_y),-sin(body_y),0;sin(body_y),cos(body_y),0;0,0,1]; 

R_body = R_bodyz * R_bodyy* R_bodyx;
P_body = sym([body_px;body_py; body_pz]);

T_body_origin = [R_body,P_body; 0,0,0,1];
% %% locally kinematics
% % T_body_origin = [1,0,0,0;0,1,0,0;0,0,1,0;0,0,0,1];

x = T_body_origin * [P_hip;1];
pos_hip = x(1:3,:);

T_thigh_body = [R_hip,P_hip;0,0,0,1];

x = T_body_origin *T_thigh_body * [P_thigh;1];
pos_thigh = x(1:3,:);

T_calf_hip = [R_thigh,P_thigh;0,0,0,1];
x = T_body_origin *T_thigh_body * T_calf_hip*[P_calf;1];
pos_calf = x(1:3,:);

T_feet_calf = [R_calf,P_calf;0,0,0,1];
x = T_body_origin *T_thigh_body * T_calf_hip*T_feet_calf*[P_feet;1];
pos_feet = x(1:3,:);

Jaco = jacobian(pos_feet,[q_hip q_thigh q_calf]);
%%%% Generate a function for computing the forward kinematics:

matlabFunction(pos_hip,pos_thigh,pos_calf,pos_feet,Jaco,...
    'file','autoGen_go1Kinematics.m',...
    'vars',{q_hip q_thigh q_calf leg_offset_x leg_offset_y thigh_y thigh_length calf_length},...
    'outputs',{'pos_hip','pos_thigh','pos_calf','pos_feet','Jaco'});

% 
% %%% Generate a function to computing inverse kinematics
% syms feetx feety feetz 'real'
% xxx = pos_feet(1);
% 
% equal1 = pos_feet(1) - feetx;
% equal2 = pos_feet(2) - feety;
% equal3 = pos_feet(3) - feetz;
% sol = solve(equal1,equal2,equal3, q_hip, q_thigh, q_calf);
% q_hip_desired = sol.q_hip;
% q_thigh_desired = sol.q_thigh;
% q_calf_desired = sol.q_calf;
% matlabFunction(q_hip_desired,q_thigh_desired,q_calf_desired,...
%     'file','autoGen_go1_inverse_Kinematics.m',...
%     'vars',{feetx feety feetz leg_offset_x leg_offset_y thigh_y thigh_length calf_length},...
%     'outputs',{'q_hip_desired','q_thigh_desired','q_calf_desired'});



