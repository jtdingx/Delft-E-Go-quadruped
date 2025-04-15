#include "pinocchio_model.h"


using namespace std;
using namespace Eigen;
using namespace pinocchio;


template<typename T>
bool is_in_vector(const std::vector<T> & vector, const T & elt)
{
  return vector.end() != std::find(vector.begin(),vector.end(),elt);
}



WBDClass::WBDClass()
{
    Ig.setZero();
    
    // You should change here to set up your own URDF file or just pass it as an argument of this example.
    const std::string urdf_filename = "/home/jiatao/unitree_ros_simu_cvx/src/unitree_ros/robots/go1_description/urdf/go1.urdf";
    
    // Load the urdf model
    pinocchio::urdf::buildModel(urdf_filename,pinocchio::JointModelFreeFlyer(),model);

    //// define the gravity by hand!!!!!!!!!!!!
    model.gravity.linear( Eigen::Vector3d(0,0,-9.81));
    
    std::cout << "floating-base robot joint number: " << model.nq << std::endl;
    std::cout << "floating-base robot joint velocity number: " << model.nv << std::endl;

    // Print the list of joints in the original model
    std::cout << "List of joints in the original model:" << std::endl;
    for(JointIndex joint_id = 1; joint_id < model.joints.size(); ++joint_id)
      std::cout << "\t- " << model.names[joint_id] << std::endl;



    // Create a list of joint to lock
    // std::vector<std::string> list_of_joints_to_lock_by_name;
    // list_of_joints_to_lock_by_name.push_back("elbow_joint");
    // list_of_joints_to_lock_by_name.push_back("wrist_3_joint"); // It can be in the wrong order
    // list_of_joints_to_lock_by_name.push_back("wrist_2_joint");
    // list_of_joints_to_lock_by_name.push_back("blabla"); // Joint not in the model
    
    // Print the list of joints to remove + retrieve the joint id
    // std::vector<JointIndex> list_of_joints_to_lock_by_id;
    // for(std::vector<std::string>::const_iterator it = list_of_joints_to_lock_by_name.begin();
    //     it != list_of_joints_to_lock_by_name.end(); ++it)
    // {
    //   const std::string & joint_name = *it;
    //   if(model.existJointName(joint_name)) // do not consider joint that are not in the model
    //     list_of_joints_to_lock_by_id.push_back(model.getJointId(joint_name));
    //   else
    //     std::cout << "joint: " << joint_name << " does not belong to the model" << std::endl;
    // }
    
  //   // Sample any random configuration
  //   Eigen::VectorXd q_rand = randomConfiguration(model);
  // //  std::cout << "q_rand: " << q_rand.transpose() << std::endl;
  //   // But should be also a neutral configuration
  //   Eigen::VectorXd q_neutral= neutral(model);
  //   PINOCCHIO_UNUSED_VARIABLE(q_neutral);
  // //  std::cout << "q_neutral: " << q_neutral.transpose() << std::endl;
    // Data data(model);
    Coriolis_M.setZero(model.nv,model.nv);
    gravity_M.setZero(model.nv);
    torque_drift.setZero(model.nv);
    tau_nle.setZero(model.nv);



    std::cout << "!!!! A floating-base model is succuessfully build using pinocchio!!!" << std::endl;      

}


WBDClass::~WBDClass()
{
}


// void WBDClass::build_pino_model()
// {

// }


void WBDClass::computeInertiaMatrix(Eigen::Matrix<double,3,1> base_vel,Eigen::Matrix<double,3,1> base_ang_vel,Eigen::Matrix<double,12,1> q0, Eigen::Matrix<double,12,1> dq0,Eigen::Matrix<double,3,1> base_pos_Ig,Eigen::Quaterniond base_quat_Ig){
    pinocchio::Data data(model), data_Coriolis(model), data_drift(model), data_nle(model);

    // Eigen::Matrix<double,3,1> base_vel;
    // base_vel.setZero();

    // Eigen::Matrix<double,3,1> base_ang_vel;
    // base_ang_vel.setZero();

    // Eigen::Matrix<double,12,1> dq0;
    // dq0.setZero();

    Eigen::Matrix<double,4,1> base_quat_arr;
    base_quat_arr << base_quat_Ig.x(),base_quat_Ig.y(),base_quat_Ig.z(),base_quat_Ig.w();

    // Eigen::Matrix<double,19,1> q_pin; // q_pin is made up of base_pos (3,1), base_quat (4,1) and q (12,1)
    Eigen::VectorXd q_pin = Eigen::VectorXd::Zero(model.nq);
    q_pin << base_pos_Ig, base_quat_arr, q0;

  
    // Eigen::Matrix<double,18,1> v_pin; 
    Eigen::VectorXd v_pin = Eigen::VectorXd::Zero(model.nv);

    // v_pin is made up of base_vel (3,1), base_ang_vel (3,1) and joint velocity dq (12,1)
    v_pin << base_vel, base_ang_vel, dq0;

    // Compute centroidal inertia matrix using pinnochio

    pinocchio::ccrba(model, data, q_pin, v_pin);
    Ig = data.Ig.inertia().matrix();
    
    ///// compute CoriolisMatrix
    pinocchio::computeCoriolisMatrix(model,data_Coriolis, q_pin,v_pin); //// q_pin and v_pin should be the reference pos and vel
    Coriolis_M = data_Coriolis.C;
    VectorXd cori = Coriolis_M * v_pin;
    torque_coriolis = cori.block<12,1>(6,0);
    

    ////// bias torque: 
    Eigen::VectorXd aq0 = Eigen::VectorXd::Zero(model.nv);
    torque_drift = rnea(model, data_drift, q_pin, v_pin, aq0);
    //  %= data_drift.tau;

    //tau_nle = nonLinearEffects(model,data_nle,q_pin,v_pin);
    //std::cout<<tau_nle.isApprox(torque_drift, 1e-12)<<std::endl;

    torque_bias = torque_drift.block<12,1>(6,0);
    //std::cout<<model.gravity<<std::endl; 





    
}

void WBDClass::compute_gravity(Eigen::Matrix<double,3,1> base_vel,Eigen::Matrix<double,3,1> base_ang_vel,Eigen::Matrix<double,12,1> q0, Eigen::Matrix<double,12,1> dq0,Eigen::Matrix<double,3,1> base_pos_Ig,Eigen::Quaterniond base_quat_Ig){
    // pinocchio::Data data(model);

    // // Eigen::Matrix<double,3,1> base_vel;
    // // base_vel.setZero();

    // // Eigen::Matrix<double,3,1> base_ang_vel;
    // // base_ang_vel.setZero();

    // // Eigen::Matrix<double,12,1> dq0;
    // // dq0.setZero();

    Eigen::Matrix<double,4,1> base_quat_arr;
    base_quat_arr << base_quat_Ig.x(),base_quat_Ig.y(),base_quat_Ig.z(),base_quat_Ig.w();

    // Eigen::Matrix<double,19,1> q_pin; // q_pin is made up of base_pos (3,1), base_quat (4,1) and q (12,1)
    Eigen::VectorXd q_pin = Eigen::VectorXd::Zero(model.nq);
    q_pin << base_pos_Ig, base_quat_arr, q0;

  
    // // Eigen::Matrix<double,18,1> v_pin; 
    // Eigen::VectorXd v_pin = Eigen::VectorXd::Zero(model.nv);

    // // v_pin is made up of base_vel (3,1), base_ang_vel (3,1) and joint velocity dq (12,1)
    // v_pin << base_vel, base_ang_vel, dq0;

    // // Compute centroidal inertia matrix using pinnochio

    // pinocchio::ccrba(model, data, q_pin, v_pin);
    // Ig = data.Ig.inertia().matrix();
    
    // ///// compute CoriolisMatrix
    // pinocchio::computeCoriolisMatrix(model,data, q_pin,v_pin); //// q_pin and v_pin should be the reference pos and vel
    // Coriolis_M = data.C;
    

  
    //// compute generalized gravity ///
    //******************solved by  initilizing the gravity when building the model ************/////
    pinocchio::Data data_ref(model);
    gravity_M = pinocchio::computeGeneralizedGravity(model,data_ref, q_pin);
    //std::cout<<data.g<<std::endl;
    // MatrixXd g_partial_dq(model.nv,model.nv); 
    // g_partial_dq.setZero();
    // pinocchio::computeGeneralizedGravityDerivatives(model,data_ref,q_pin,g_partial_dq);
    // gravity_M = data_ref.g;
   
    // /////// the following code is reliable for computing generalized gravity /////////
    // pinocchio::Data data_rnea(model);
    // // Compare with Jcom
    // crba(model,data_rnea,q_pin);
    // Data::Matrix3x Jcom = getJacobianComFromCrba(model,data_rnea);
    
    // VectorXd g_ref(-data_rnea.mass[0]*Jcom.transpose()*Model::gravity981);   
    // gravity_M = g_ref;
    // //std::cout<<g_ref<<std::endl; 
    // //std::cout<<g_ref.isApprox(data_ref.g)<<std::endl;
    torque_gravity = gravity_M.block<12,1>(6,0);
    
}

