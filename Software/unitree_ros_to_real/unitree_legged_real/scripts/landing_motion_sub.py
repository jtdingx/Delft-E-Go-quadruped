#! /usr/bin/env python

import rospy
from std_msgs.msg import String ###data type
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
# from TO_class_landing_motion import TO_opt
import casadi as ca




# import casadi as ca
import numpy as np
from casadi import *
import time
from pathlib import Path


class TO_opt:
    def __init__(self, des_jump_length, number_of_steps, VERBOSE):
        self.des_jump_length = des_jump_length
        self.number_of_steps = number_of_steps
        self.VERBOSE = VERBOSE

        ### 2D joint calculation structure parameters======
        self.thigh_length = -0.213
        self.calf_length = -0.213
        self.front_l_body = 0.3762 / 2
        self.rear_l_body = -0.3762 / 2
        self.alpha_pitch = 0

        self.front_x_f, self.front_z_f = self.front_l_body, 0
        self.rear_x_f, self.rear_z_f = self.rear_l_body, 0


        ## opt problem for Casadi
        self.opti = ca.Opti()
        self.opti.solver('ipopt')

        #### consider to stabilize the landing recovery motion in 1s?%%%%%% reach the 0.1s should be OK to solve the problem
        self.Ns = 5
        self.Ts = 0.5
        

        self.G = vertcat(0,-9.81)
        self.M = 13

        self.max_velX = 3
        self.max_velZ = 4

        self.offset_jump_vertically = 0.0

        self.l0 = 0.32 
        self.max_height_stance = 0.39
        self.l_max = 0.42


        self.N = self.Ns

        self.X = self.opti.variable(8,self.N+1) # state trajectory plus 4 angle joint
        self.posX = self.X[0,:]
        self.posZ = self.X[1,:]
        self.velX = self.X[2,:]
        self.velZ = self.X[3,:]
        self.front_thigh = self.X[4,:]
        self.front_calf = self.X[5,:]
        self.rear_thigh = self.X[6,:]
        self.rear_calf = self.X[7,:]

        self.U = self.opti.variable()   # control state

        self.U_in = self.opti.variable(2,self.N)
        self.x_acc = self.U_in[0,:]
        self.z_acc = self.U_in[1,:]         

        print("!!!!name an object!!!")

    def forward_kin_close_form(self,bodyx,bodyy,q_thigh,q_calf,leg_offset_x):
        x_f_end = bodyx + leg_offset_x + self.calf_length*(np.cos(q_calf)*np.sin(q_thigh) + np.cos(q_thigh)*np.sin(q_calf)) + self.thigh_length*np.sin(q_thigh)
        y_f_end = bodyy + self.calf_length*(np.cos(q_calf)*np.cos(q_thigh) - np.sin(q_calf)*np.sin(q_thigh)) + self.thigh_length*np.cos(q_thigh)

        return x_f_end,y_f_end

    # inverse kinematics for solving the initial guess #####
    def inverse_kin_close_forma(self,x,y,leg_x_f,leg_y_f,l_body_offset,alpha): ##### angle solution caused illness
        x_e, y_e = leg_x_f - (x + l_body_offset * np.cos(alpha)), leg_y_f - (y + l_body_offset * np.sin(alpha))

        q2 = np.arccos((abs(self.thigh_length)**2 + abs(self.calf_length)**2 - (x_e**2 + y_e**2)) / (2 * abs(self.thigh_length) * abs(self.calf_length)))
        q3 = np.arcsin(np.sin(q2)*(abs(self.calf_length)/np.sqrt((x_e)**2 + y_e**2))) - np.arctan(-x_e/y_e) 
        #q1 = -q2 - q3 #- alpha

        return q2-np.pi, q3

    # Jacobian matri, symbolic result from matlab, used for casadi calculation
    def Jacobian_kine(self,q_thigh, q_calf):
        J_matrix = self.opti.variable(2,2)
        #### transpose ####
        J_matrix[0,0] = self.calf_length*(np.cos(q_calf)*np.cos(q_thigh) - np.sin(q_calf)*np.sin(q_thigh)) + self.thigh_length*np.cos(q_thigh)
        J_matrix[1,0] = self.calf_length*(np.cos(q_calf)*np.cos(q_thigh) - np.sin(q_calf)*np.sin(q_thigh))
        J_matrix[0,1] = - self.calf_length*(np.cos(q_calf)*np.sin(q_thigh) + np.cos(q_thigh)*np.sin(q_calf)) - self.thigh_length*np.sin(q_thigh)
        J_matrix[1,1] = -self.calf_length*(np.cos(q_calf)*np.sin(q_thigh) + np.cos(q_thigh)*np.sin(q_calf))

        return J_matrix

    # Jacobian matri, symbolic result from matlab
    def Jacobian_kinex(self,q_thigh, q_calf):
        J_matrixx = np.zeros([2,2])
        #### transpose ####
        J_matrixx[0,0] = self.calf_length*(np.cos(q_calf)*np.cos(q_thigh) - np.sin(q_calf)*np.sin(q_thigh)) + self.thigh_length*np.cos(q_thigh)
        J_matrixx[1,0] = self.calf_length*(np.cos(q_calf)*np.cos(q_thigh) - np.sin(q_calf)*np.sin(q_thigh))
        J_matrixx[0,1] = - self.calf_length*(np.cos(q_calf)*np.sin(q_thigh) + np.cos(q_thigh)*np.sin(q_calf)) - self.thigh_length*np.sin(q_thigh)
        J_matrixx[1,1] = -self.calf_length*(np.cos(q_calf)*np.sin(q_thigh) + np.cos(q_thigh)*np.sin(q_calf))

        return J_matrixx
    


    ##### definition cost items
    def J_q(self, q0,q1,q2,q3,des):
        return 0.001*sum2((q0-des[4])**2)+sum2((q1-des[5])**2)+sum2((q2-des[6])**2)+sum2((q3-des[7])**2)

    def J_dt(self,dts,des):
        return 0.01*(dts - des[0])**2
    
    def J_lift(self,velZ,velX):
        return 100*velZ**2 +100*velX**2
    
    def J_flight(self,posZ,posX):
        return 1000*sum2((posZ-self.l0)**2)+1000*sum2((posX)**2)


    def J_stance(self,accX,accZ):
        J_jerk = 0
        for k in range(self.Ns-1):
            J_jerk += ((self.U_in[:,k+1]-self.U_in[:,k]))**2
        J_jerk = sum2(J_jerk) 

        return 0.3*sum2((accZ)**2) + 1*sum2((accX)**2) + .001*(J_jerk[0]) + .001*(J_jerk[1])



    def generate_trajectory(self,detx,detz,bodyvx,bodyvz,q_thigh_ini,q_calf_ini):
        t1 = time.time()
        ##### cost items ########
        # J = lambda posX,posZ,velX,velZ,accX,accZ,q1,q2,q3,q4,U,des: self.J_lift(velZ[self.Ns-1],velX[self.Ns-1])+ self.J_flight(posZ[:self.Ns],posX[:self.Ns])+ J_stance(posX[:self.Ns],posZ[:self.Ns],velX[:self.Ns],accX[:self.Ns],accZ[:self.Ns],des) + self.J_q(q1,q2,q3,q4,des)+ self.J_dt(U,des) 

        J = lambda posX,posZ,velX,velZ,accX,accZ,q1,q2,q3,q4,U,des: self.J_stance(accX[:self.Ns],accZ[:self.Ns]) + self.J_q(q1,q2,q3,q4,des)+ self.J_dt(U,des) 

        L = lambda x,z: sqrt(fmax((x)**2 + (z)**2, 0.00001)) ###spring length: assuming the root locates at the original point ####
        F_s = lambda x,z: k_spring * (self.l0-L(x,z))*vertcat(x,z) / L(x,z)   ##### spring force vector 

        
        ##### state transfer function 
        x_fs = lambda X,U,p_ddot: X + vertcat(X[2:4]*U + 0.5*p_ddot*U**2, p_ddot*U) # x_2 = x_1 + v*dt + 0.5*a*dt^2, v_2 = v_1 + a *dt stance phase

        ###### initial state ************* measured from the landing state ************#
        bodyx = 0
        l0_ini = detz
        
        self.front_x_f_ini = self.front_x_f + detx
        self.rear_x_f_ini = self.rear_x_f + detx

        # q_calf_ini, q_thigh_ini = self.inverse_kin_close_forma(bodyx,l0_ini,self.front_x_f_ini,self.front_z_f,self.front_l_body,self.alpha_pitch)
        # # print(" ############# leg joint initial guess ########## ")
        # print(q_calf_ini)
        # print(q_thigh_ini)
        #####  
        slip_bodyx = -detx
        slip_bodyz = detz
        bodyvx = 1.43
        bodyvz = -2.5

        dts_guess = self.Ts/self.Ns
        dtf_guess = 0
        q1_guess = q_thigh_ini
        q2_guess = q_calf_ini
        q3_guess = q_thigh_ini
        q4_guess = q_calf_ini

        k_spring = 5000 #8200 # spring stiffness
        



        for k in range(self.Ns-1):
            # self.opti.subject_to(0.1 <= posX[k]**2 + self.posZ[k]**2)
            p_ddot = F_s(self.posX[k],self.posZ[k])/self.M + self.G + vertcat(self.U_in[0,k],self.U_in[1,k])
            x_next = x_fs(self.X[:4,k],self.U,p_ddot)

            # Ensure the robot does not go below 0.2m in z direction during stance#####could be
            self.opti.subject_to(self.opti.bounded(0.15,self.posZ[k],self.max_height_stance))
            # self.opti.subject_to(self.opti.bounded(-0.1881,self.posX[k],0.1881))
            self.opti.subject_to(self.opti.bounded(0.15**2,self.posZ[k]**2 + self.posX[k]**2,self.l_max**2)) #####maximal leg length,
            
            # self.X[:,k+1]==x_next
            self.opti.subject_to(self.X[:4,k+1]==x_next) # close the gaps

            # self.opti.subject_to(self.opti.bounded(0,self.velX[k],self.max_velX))
            # self.opti.subject_to(self.opti.bounded(-self.max_velZ,self.velZ[k],self.max_velZ))
            # Make sure the downwards acceleration doesn't exceed self.G
            self.opti.subject_to(p_ddot[1] >= self.G[1])

            if k==0:  ###make sure we have the negative az and forward ax
                self.opti.subject_to(self.opti.bounded(-100,self.X[3,k+1],0))
                self.opti.subject_to(self.opti.bounded(0,self.X[2,k+1],10)) 


            ###### kinematics constraints with joint constraints #######
            front_leg_x, front_leg_z = self.forward_kin_close_form(self.posX[k],self.posZ[k],self.front_thigh[k],self.front_calf[k],self.front_l_body)
            self.opti.subject_to(self.opti.bounded(self.front_x_f-0.01,front_leg_x,self.front_x_f+0.01))
            self.opti.subject_to(self.opti.bounded(self.front_z_f-0.01,front_leg_z,self.front_z_f+0.01))

            rear_leg_x, rear_leg_z = self.forward_kin_close_form(self.posX[k],self.posZ[k],self.rear_thigh[k],self.rear_calf[k],self.rear_l_body)
            self.opti.subject_to(self.opti.bounded(self.rear_x_f-0.01,rear_leg_x,self.rear_x_f+0.01))
            self.opti.subject_to(self.opti.bounded(self.rear_z_f-0.01,rear_leg_z,self.rear_z_f+0.01)) 
            
            # joint limitation 
            # self.opti.subject_to(self.opti.bounded(-1.047,front_hip[k],1.047))
            # self.opti.subject_to(self.opti.bounded(-1.047,front_hip[k],1.047))        
            self.opti.subject_to(self.opti.bounded(-0.663,self.front_thigh[k],2.966))
            self.opti.subject_to(self.opti.bounded(-2.721,self.front_calf[k],-0.837))
            self.opti.subject_to(self.opti.bounded(-0.663,self.rear_thigh[k],2.966))
            self.opti.subject_to(self.opti.bounded(-2.721,self.rear_calf[k],-0.837))

            #### CoP constraints #### not usedd as constrained


            # ## dynamic: torque limits
            # ## equivalent GRF calculation ####
            # total_grf = F_s(self.posX[k],self.posZ[k]) + vertcat(self.U_in[0,k],self.U_in[1,k]) *self.M
            # # Front_leg_grf = (self.posX[k] - self.rear_l_body)/(self.front_l_body - self.rear_l_body) * total_grf/2
            # # Rear_leg_grf = (self.front_l_body - self.posX[k])/(self.front_l_body - self.rear_l_body) * total_grf/2
            # Front_leg_grf = (self.posX[k] - rear_leg_x)/(front_leg_x - rear_leg_x) * total_grf/2
            # Rear_leg_grf = (front_leg_x - self.posX[k])/(front_leg_x - rear_leg_x) * total_grf/2

            # front_jaco = Jacobian_kine(self.front_thigh[k],self.front_calf[k])
            # front_torque = -(front_jaco@Front_leg_grf)

            # rear_jaco = Jacobian_kine(self.rear_thigh[k],self.rear_calf[k])
            # rear_torque = -(rear_jaco@Rear_leg_grf)

            # # # # self.opti.subject_to(self.opti.bounded(-1.047,front_hip_torque[k],1.047))
            # # # # self.opti.subject_to(self.opti.bounded(-1.047,front_hip_torque[k],1.047))        
            # self.opti.subject_to(self.opti.bounded(-28*1.2,front_torque[0],28*1.2))
            # self.opti.subject_to(self.opti.bounded(-32*1.2,front_torque[1],32*1.2))
            # self.opti.subject_to(self.opti.bounded(-28*1.2,rear_torque[0],28*1.2))
            # self.opti.subject_to(self.opti.bounded(-32*1.2,rear_torque[1],32*1.2))         


            


        # Set initial conditions for the problem:
        self.opti.subject_to(self.posX[0] == slip_bodyx)
        self.opti.subject_to(self.posZ[0] == slip_bodyz)
        self.opti.subject_to(self.velX[0] == bodyvx)
        self.opti.subject_to(self.velZ[0] == bodyvz)
        
        self.opti.subject_to(self.front_thigh[0] == q_thigh_ini)
        self.opti.subject_to(self.front_calf[0] == q_calf_ini)
        self.opti.subject_to(self.rear_thigh[0] == q_thigh_ini)
        self.opti.subject_to(self.rear_calf[0] == q_calf_ini)
        # Set initial guesses for self.U:
        #self.opti.set_initial(self.U,dts_guess)
        # self.opti.set_initial(self.U_in[1],1.0)

        des = vertcat(dts_guess,dtf_guess,self.des_jump_length,self.max_height_stance,q1_guess,q2_guess,q3_guess,q4_guess)

        self.opti.minimize(J(self.posX,self.posZ,self.velX,self.velZ,self.U_in[0,:],self.U_in[1,:],
        self.front_thigh,self.front_calf,self.rear_thigh,self.rear_calf,self.U, des))

        # Set constraints for final states:
        self.opti.subject_to(self.posZ[self.Ns-1] == self.l0)
        self.opti.subject_to(self.posX[self.Ns-1] == 0)
        # self.opti.subject_to(self.opti.bounded(0.99*self.l0,self.posZ[self.Ns-1],self.l0*1.01))
        #self.opti.subject_to(self.opti.bounded(rear_leg_x,self.posX[self.Ns-1],front_leg_x))
        self.opti.subject_to(self.velX[self.Ns-1] == 0)
        self.opti.subject_to(self.velZ[self.Ns-1] == 0)



        ##### set the solver, should be very familiar with the IPOPT solver #######
        p_opts = dict(print_time=False, verbose=False)
        s_opts = dict(print_level=0, max_iter=100)
        self.opti.solver('ipopt', p_opts, s_opts)
        
        sol = self.opti.solve()
        t2 = time.time()
        print("-----")
        print("optimal dt is: ", sol.value(self.U))
        idx = -1
        print("Final state:", sol.value(self.posX)[idx],sol.value(self.posZ)[idx])
        print("Solving time: ", (t2-t1)*1000, " ms")


        import matplotlib.pyplot as plt
        dts = sol.value(self.U)

        x_pos = sol.value(self.posX)[:-1]
        z_pos = sol.value(self.posZ)[:-1]

        u_in1 = sol.value(self.x_acc)[:-1]
        u_in2 = sol.value(self.z_acc)[:-1]


        # Add an offset to the vertical position if desired
        z_pos += self.offset_jump_vertically

        t_ops_s = np.array([dts*k for k in range(self.Ns)])

        t_ops = t_ops_s.reshape(self.Ns,1)





        print("Peak height is: ", np.max(z_pos))

        ### FK validation
        q1 = sol.value(self.front_thigh)[:-1]
        q2 = sol.value(self.front_calf)[:-1]
        q4 = sol.value(self.rear_thigh)[:-1]
        q5 = sol.value(self.rear_calf)[:-1]    
        front_x_end =np.zeros([1,self.Ns])
        front_z_end =np.zeros([1,self.Ns])
        rear_x_end =np.zeros([1,self.Ns])
        rear_z_end =np.zeros([1,self.Ns])    
        for i in range(self.Ns):
            xp, zp=  self.forward_kin_close_form(x_pos[i],z_pos[i],q1[i],q2[i],self.front_l_body)
            front_x_end[0,i] = xp
            front_z_end[0,i] = zp
            xp, zp=  self.forward_kin_close_form(x_pos[i],z_pos[i],q4[i],q5[i],self.rear_l_body)
            rear_x_end[0,i] = xp
            rear_z_end[0,i] = zp        
            if(i % 10 ==0):
                print("rear leg x:", xp)

        ### joint torque validation
        front_leg_force =np.zeros([2,self.Ns])
        rear_leg_force =np.zeros([2,self.Ns])    
        front_leg_torque =np.zeros([2,self.Ns])
        rear_leg_torque =np.zeros([2,self.Ns])     
        
        for k in range(self.Ns-1):
            total_grf = F_s(x_pos[k],z_pos[k]) + vertcat(u_in1[k],u_in2[k]) *self.M
            Front_leg_grf = (x_pos[k] - self.rear_l_body)/(self.front_l_body - self.rear_l_body) * total_grf/2
            front_leg_force[0,k] = Front_leg_grf[0]
            front_leg_force[1,k] = Front_leg_grf[1]
            Rear_leg_grf = (self.front_l_body - x_pos[k])/(self.front_l_body - self.rear_l_body) * total_grf/2
            rear_leg_force[0,k] = Rear_leg_grf[0]
            rear_leg_force[1,k] = Rear_leg_grf[1]

            front_jaco = self.Jacobian_kinex(q1[i],q2[i])
            front_torque = -front_jaco * Front_leg_grf
            front_leg_torque[0,k] = front_torque[0]
            front_leg_torque[1,k] = front_torque[1]
            rear_jaco = self.Jacobian_kinex(q4[i],q5[i])
            rear_torque = -rear_jaco * Rear_leg_grf
            rear_leg_torque[0,k] = rear_torque[0]
            rear_leg_torque[1,k] = rear_torque[1]
        
        if self.VERBOSE:
            fig,ax = plt.subplots(4)



            ax[0].plot(t_ops[:self.Ns],z_pos[:self.Ns],'r',label="CoM z-Stance")
            ax[0].plot(t_ops[:self.Ns],front_z_end[0,:self.Ns],'r--',label="Front_leg_z")
            ax[0].plot(t_ops[:self.Ns],rear_z_end[0,:self.Ns],'g-.',label="Rear_leg_z")
            ax[0].set_xlabel("Time (s)")
            ax[0].set_ylabel("Height (m)")
            ax[0].grid()
            ax[0].legend()

            ax[1].plot(t_ops[:self.Ns],x_pos[:self.Ns],'r',label="CoM x-Stance")
            ax[1].plot(t_ops[:self.Ns],front_x_end[0,:self.Ns],'r--',label="Front_leg_x")
            ax[1].plot(t_ops[:self.Ns],rear_x_end[0,:self.Ns],'g-.',label="Rear_leg_x")
            ax[1].set_xlabel("Time (s)")
            ax[1].set_ylabel("Length (m)")
            ax[1].grid()
            ax[1].legend()

            ax[2].plot(t_ops[:self.Ns],sol.value(self.velZ)[:self.Ns],'r',label="Stance phase")
            ax[2].set_xlabel("Time (s)")
            ax[2].set_ylabel("Upward velocity (m/s)")
            ax[2].grid()
            ax[2].legend()

            ax[3].plot(t_ops[:self.Ns],sol.value(self.velX)[:self.Ns],'r',label="Stance phase")
            ax[3].set_xlabel("Time (s)")
            ax[3].set_ylabel("Forward velocity (m/s)")
            ax[3].grid()
            ax[3].legend()

            fig,ax1 = plt.subplots(2)
            ax1[0].plot(t_ops[:self.Ns],sol.value(self.U_in)[0,:self.Ns],'r',label="U[1] Stance phase")
            ax1[0].plot(t_ops[:self.Ns],(front_leg_force[0,:self.Ns]+rear_leg_force[0,:self.Ns])/self.M,'r--',label=" Cx stance phase")
            ax1[0].set_xlabel("Time (s)")
            ax1[0].set_ylabel("Acceleration X (m/s2)")
            ax1[0].grid()
            ax1[0].legend()

            ax1[1].plot(t_ops[:self.Ns],sol.value(self.U_in)[1,:self.Ns],'r',label="U[1] Stance phase")
            ax1[1].plot(t_ops[:self.Ns],(front_leg_force[1,:self.Ns]+rear_leg_force[1,:self.Ns])/self.M,'r--',label=" Cz stance phase")
            ax1[1].set_xlabel("Time (s)")
            ax1[1].set_ylabel("Acceleration Z (m/s2)")
            ax1[1].grid()
            ax1[1].legend()

            fig,ax2 = plt.subplots(2)
            ax2[0].plot(t_ops[:self.Ns],sol.value(self.front_thigh)[:self.Ns],'r',label="front leg - Stance phase")
            ax2[0].plot(t_ops[:self.Ns],sol.value(self.rear_thigh)[:self.Ns],'g--',label="rear leg - Stance phase")
            ax2[0].set_xlabel("Time (s)")
            ax2[0].set_ylabel("Thigh angle (rad)")
            ax2[0].grid()
            ax2[0].legend()  

    
            ax2[1].plot(t_ops[:self.Ns],sol.value(self.front_calf)[:self.Ns],'r',label="front leg - Stance phase")
            ax2[1].plot(t_ops[:self.Ns],sol.value(self.rear_calf)[:self.Ns],'g--',label="rear leg - Stance phase")
            ax2[1].set_xlabel("Time (s)")
            ax2[1].set_ylabel("Calf angle (rad)")
            ax2[1].grid()
            ax2[1].legend() 

            fig,ax3 = plt.subplots(2)
            ax3[0].plot(t_ops[:self.Ns],front_leg_force[0,:self.Ns],'r--',label="front x force")
            ax3[0].plot(t_ops[:self.Ns],front_leg_force[1,:self.Ns],'g',label="front z force")
            ax3[0].plot(t_ops[:self.Ns],rear_leg_force[0,:self.Ns],'r',label="rear x force")
            ax3[0].plot(t_ops[:self.Ns],rear_leg_force[1,:self.Ns],'g--',label="rear z force")
                    
            ax3[0].set_xlabel("Time (s)")
            ax3[0].set_ylabel("Grf force (self.N)")
            ax3[0].grid()
            ax3[0].legend()  


            ax3[1].plot(t_ops[:self.Ns],front_leg_torque[0,:self.Ns],'r--',label="front thigh torque")
            ax3[1].plot(t_ops[:self.Ns],front_leg_torque[1,:self.Ns],'g',label="front calf torque")
            ax3[1].plot(t_ops[:self.Ns],rear_leg_torque[0,:self.Ns],'r',label="rear thigh torque")
            ax3[1].plot(t_ops[:self.Ns],rear_leg_torque[1,:self.Ns],'g--',label="rear calf torque")
                    
            ax3[1].set_xlabel("Time (s)")
            ax3[1].set_ylabel("Joint torque (Nm)")
            ax3[1].grid()
            ax3[1].legend()  

    
            plt.show()




# TO_landing = TO_opt(des_jump_length=0.5, number_of_steps = 800, VERBOSE = 0)

# TO_landing.generate_trajectory(detx = 0.174,detz = 0.268,bodyvx = 1.43,bodyvz = -2.5,q_thigh_ini=0.146,q_calf_ini=-1.445)

landing_state_recevie = []
for i in range(25):
    landing_state_recevie.append(i)

####callback ###  parameters: list[9]: 0: flag, 
def receive_data_callback(JointState):
    global landing_state_recevie
    landing_state_recevie = JointState.position





if __name__ == "__main__":
    
    rospy.init_node("Landing_motion_TOx")    ###node name


    sub = rospy.Subscriber("/rt2nrt/state",JointState,receive_data_callback, queue_size=10)
    pub2 = rospy.Publisher("landing_motion_2rtctrl",JointState, queue_size=10) 

    TO_landing = TO_opt(des_jump_length=0.5, number_of_steps = 800, VERBOSE = 1)
    

    rate = rospy.Rate(20) ####publish data in 1Hz
    count = 0

    while not rospy.is_shutdown():
        count += 1
        if (count==1):
            TO_landing.generate_trajectory(detx = 0.174,detz = 0.268,bodyvx = 1.43,bodyvz = -2.5,q_thigh_ini=0.146,q_calf_ini=-1.445)

        print(landing_state_recevie[0])
        rate.sleep()
        # rospy.spin()