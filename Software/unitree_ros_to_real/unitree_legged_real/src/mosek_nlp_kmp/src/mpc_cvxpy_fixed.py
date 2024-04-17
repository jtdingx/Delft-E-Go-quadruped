import cvxpy as cp
import numpy as np
from cvxpygen import cpg
import pinocchio as pin


n = 12#9
m = 6

N = 10
dt= 0.01

n_grf = 12
m_grf = 6
l_grf = 4

x_init = cp.Parameter(n,name="x_init")


A = cp.Parameter((m_grf*N,n_grf),name="A")#,sparsity = sparsity_list)


A_dyn = np.array([      [1,0,0,0,0,0,dt,0,0,0,0,0], # x
                        [0,1,0,0,0,0,0,dt,0,0,0,0], # y
                        [0,0,1,0,0,0,0,0,dt,0,0,0], # z
                        [0,0,0,1,0,0,0,0,0,dt,0,0], # roll
                        [0,0,0,0,1,0,0,0,0,0,dt,0], # pitch
                        [0,0,0,0,0,1,0,0,0,0,0,dt], # yaw
                        [0,0,0,0,0,0,1,0,0,0,0,0], # x_dot
                        [0,0,0,0,0,0,0,1,0,0,0,0], # y_dot
                        [0,0,0,0,0,0,0,0,1,0,0,0], # z_dot
                        [0,0,0,0,0,0,0,0,0,1,0,0], # roll_dot
                        [0,0,0,0,0,0,0,0,0,0,1,0], # pitch_dot
                        [0,0,0,0,0,0,0,0,0,0,0,1]]) # yaw_dot

B_dyn = np.array([ [0.5*dt**2,0,0,0,0,0], 
                   [0,0.5*dt**2,0,0,0,0], 
                   [0,0,0.5*dt**2,0,0,0], 
                   [0,0,0,0.5*dt**2,0,0],
                   [0,0,0,0,0.5*dt**2,0],
                   [0,0,0,0,0,0.5*dt**2],
                   [dt,0,0,0,0,0],
                   [0,dt,0,0,0,0],
                   [0,0,dt,0,0,0],
                   [0,0,0,dt,0,0],
                   [0,0,0,0,dt,0],
                   [0,0,0,0,0,dt]])

# B * [accX, accY, accZ, accR, accP, accYa]

# B_dyn = np.array([ [0.5*dt**2,0,0,0,0,0], 
#                     [0,0.5*dt**2,0,0,0,0], 
#                     [0,0,0.5*dt**2,0,0,0], 
#                     [0,0,0,dt,0,0],
#                     [0,0,0,0,dt,0],
#                     [0,0,0,0,0,dt],
#                     [0.5*dt**2,0,0,0,0,0],
#                     [0,0.5*dt**2,0,0,0,0],
#                     [0,0,0.5*dt**2,0,0,0]])


Fz_max = 160
no_contact = cp.Parameter((l_grf,N),name="no_contact")
# frict_coeff = cp.Parameter(1,nonneg=True,name="frict_coeff")

np.random.seed(0)
A.value = np.random.randn(m_grf*N,n_grf)
no_contact.value = np.random.randn(l_grf,N)
# frict_coeff.value = np.array([.4])
frict_coeff = 0.7


mass = 13.0
grav = np.array([0,0,-9.8])

x_init.value = np.zeros(n)

X = cp.Variable((n,N+1),name="X") # x,y,z,roll,pitch,yaw,x_dot,y_dot,z_dot

U = cp.Variable((m,N),name="U") # x_ddot,y_ddot,z_ddot,roll_dot,pitch_dot,yaw_dot

F = cp.Variable((n_grf,N),name="F")


X_ref = cp.Parameter((n,N),name="X_ref")

X_ref.value = np.zeros((n,N))


constr = []


constr += [X[:,0] == x_init]
constr += [X[:,1:] == A_dyn@X[:,:N] + B_dyn@U]
for i in range(N):
    # constr += [X[:,i+1] == A_dyn@X[:,i] + B_dyn@A[:,i*12:(i+1)*12]@F[:,i]]
    constr += [U[:3,i] == A[i*3:(i+1)*3,:]@F[:,i] + grav]
    constr += [U[3:,i] == (A[(i+1)*3:(i+2)*3,:]) @ F[:,i]]

    for foot in range(4):
    # index [foot*3+2] means Fz of foot X.
        constr += [no_contact[foot,i]*F[foot * 3 + 2,i] == 0]
        

        constr += [0 <= F[foot * 3 + 2,i], F[foot * 3 + 2,i] <= Fz_max]
        constr += [-frict_coeff * F[foot * 3 + 2,i] <= F[foot * 3,i], F[foot * 3,i] <= frict_coeff * F[foot * 3 + 2,i]]
        constr += [-frict_coeff * F[foot * 3 + 2,i] <= F[foot * 3 + 1,i], F[foot * 3 + 1,i] <= frict_coeff * F[foot * 3 + 2,i]]
        # constr += [-frict_coeff[0] * F[foot * 3 + 2,i] <= -F[foot * 3,i], -F[foot * 3,i] <= frict_coeff[0] * F[foot * 3 + 2,i]]
        # constr += [-frict_coeff[0] * F[foot * 3 + 2,i] <= -F[foot * 3 + 1,i], -F[foot * 3 + 1,i] <= frict_coeff[0] * F[foot * 3 + 2,i]]

    # if i < N-1:
    #     constr += [cost_jerk[:,i] == (U[:,i+1] - U[:,i]) / dt]
# constr += [err == (X[:,:N]-X_ref)]

##### N= 6~10;
R = cp.Parameter((m,m),name="R")
R.value = 100*np.array([1.0,1.0,1.0,1.0,1.0,1.0])*np.eye(m) 

R_grf = cp.Parameter((n,n),name="R_grf")
R_grf.value = 0.001*np.array([5.0,5.0,2,5.0,5.0,2,5.0,5.0,2,5.0,5.0,2])*np.eye(n) 

Q = 1*np.array([20,20,10,20,20,10,10,10,100,10,10,100])*np.eye(n) 

###
cost = cp.sum_squares(Q@(X[:,:N]-X_ref))+ cp.sum_squares(R@U) #+ cp.sum_squares(R_grf@F)

prob = cp.Problem(cp.Minimize(cost), constr)

cpg.generate_code(prob,code_dir='/home/jiatao/Documents/unitree_sdk_hardware_test/go1_remote_control/src/unitree_ros_to_real/unitree_legged_real/src/mosek_nlp_kmp/src/hardware_jump',solver='OSQP')#,solver='ECOS')