function [FR_foot, FL_foot, RR_foot,RL_foot] = forward_kin_go1()


%% FR leg
leg_offset_x = 0.1881; 
leg_offset_y = -0.04675;
thigh_y = -0.08; 
thigh_length = -0.213;
calf_length = -0.213;

q_hip = 0.0;
q_thigh = 0.6;
q_calf = -1;
[FR_pos_thigh,FR_pos_calf,FR_pos_feet, Jaco_FR] = autoGen_go1Kinematics(q_hip,q_thigh,q_calf,leg_offset_x,leg_offset_y,thigh_y,thigh_length,calf_length);

%% FL leg
leg_offset_x = 0.1881; 
leg_offset_y = 0.04675;
thigh_y = 0.08; 
thigh_length = -0.213;
calf_length = -0.213;

q_hip = 0.1;
q_thigh = 0.6;
q_calf = -1;
[FL_pos_thigh,FL_pos_calf,FL_pos_feet, Jaco_FL] = autoGen_go1Kinematics(q_hip,q_thigh,q_calf,leg_offset_x,leg_offset_y,thigh_y,thigh_length,calf_length);


%% RR leg
leg_offset_x = -0.1881; 
leg_offset_y = -0.04675;
thigh_y = -0.08; 
thigh_length = -0.213;
calf_length = -0.213;

q_hip = 0;
q_thigh = 0.6;
q_calf = -1;
[RR_pos_thigh,RR_pos_calf,RR_pos_feet, Jaco_RL] = autoGen_go1Kinematics(q_hip,q_thigh,q_calf,leg_offset_x,leg_offset_y,thigh_y,thigh_length,calf_length);

%% RL leg
leg_offset_x = -0.1881; 
leg_offset_y = 0.04675;
thigh_y = 0.08; 
thigh_length = -0.213;
calf_length = -0.213;

q_hip = 0.1;
q_thigh = 0.6;
q_calf = -1;
[RL_pos_thigh,RL_pos_calf,RL_pos_feet, Jaco_RL] = autoGen_go1Kinematics(q_hip,q_thigh,q_calf,leg_offset_x,leg_offset_y,thigh_y,thigh_length,calf_length);


FR_foot = [FR_pos_thigh,FR_pos_calf,FR_pos_feet];
FL_foot = [FL_pos_thigh,FL_pos_calf,FL_pos_feet];
RR_foot = [RR_pos_thigh,RR_pos_calf,RR_pos_feet];
RL_foot = [RL_pos_thigh,RL_pos_calf,RL_pos_feet];
% 
% 
view(30,10)
axis equal
hold on;
leg_thigh = [FR_pos_thigh,FL_pos_thigh,RL_pos_thigh,RR_pos_thigh,FR_pos_thigh];
plot3(leg_thigh(1,:),leg_thigh(2,:),leg_thigh(3,:),'Linewidth',2);

plot3(FR_foot(1,:),FR_foot(2,:),FR_foot(3,:),'Linewidth',2);
plot3(FL_foot(1,:),FL_foot(2,:),FL_foot(3,:),'Linewidth',2);
plot3(RR_foot(1,:),RR_foot(2,:),RR_foot(3,:),'Linewidth',2);
plot3(RL_foot(1,:),RL_foot(2,:),RL_foot(3,:),'Linewidth',2);


end