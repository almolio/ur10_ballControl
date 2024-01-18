#ifndef UR_ROBOT_LLI_SIMPLEEFFORTCONTROL_H
#define UR_ROBOT_LLI_SIMPLEEFFORTCONTROL_H

#include <tum_ics_ur_robot_lli/RobotControllers/ControlEffort.h>
#include <ur_model/ur_model.h>
#include <nav_msgs/Odometry.h>

namespace tum_ics_ur_robot_lli
{
  namespace RobotControllers
  {

    class BlueControl : public ControlEffort
    {
    public:
      enum State {INITIALIZED, JOINT, CARTESIAN};

    private:
      State state_;

      bool is_first_iter_;
      ros::Time time_prev_;
      double elapsed_;

      ur::URModel model_; 

      Vector6d tau_;

      //////////////////////////////////////////////////////////////////////////
      // joint states
      //////////////////////////////////////////////////////////////////////////
      Vector6d q_start_;
      JointState q_init_;
      JointState q_home_;
      JointState q_park_;

      //////////////////////////////////////////////////////////////////////////
      // ros connecitons
      //////////////////////////////////////////////////////////////////////////
      std::string ns_;
      ros::NodeHandle nh_;
      ros::Publisher control_data_pub_;
      ros::Subscriber redball_state_sub_;

      //////////////////////////////////////////////////////////////////////////
      // traj generators & tracking
      //////////////////////////////////////////////////////////////////////////
      double spline_duration_, spline_speed_, respline_thres_, speed_degration_;
      int spline_type_, counter_;
      double roll_d_, pitch_d_, yaw_d_; // desired orientation during tracking
      double zone_dim_; // safety zone dimension
      Vector3d home_; // home position
      Vector6d q_goal_;
      cc::CartesianPosition goal_c_;
      cc::CartesianPosition X_start_, X_goal_;
      cc::CartesianState x_state_blue_ball_;
      bool holding_;

      //////////////////////////////////////////////////////////////////////////
      // gains
      //////////////////////////////////////////////////////////////////////////
      Matrix6d Kp_, Kp_cart_, Kd_, Ki_;

      //////////////////////////////////////////////////////////////////////////
      // joint space contorller
      //////////////////////////////////////////////////////////////////////////
      JointState q_state_des_;  // desired
      JointState q_state_ref_;  // reference

      //////////////////////////////////////////////////////////////////////////
      // cartesian space controller
      //////////////////////////////////////////////////////////////////////////
      cc::CartesianState x_state_des_;  // desired
      cc::CartesianState x_state_ref_;  // reference
      cc::CartesianState x_state_cur_;  // current
      cc::CartesianVelocity Xp_goal_;
      cc::CartesianState x_state_des_prev_;  // desired
      Vector3d vel_prev_, acc_prev_;  // desired velocity / acc from previous spline
      Vector3d xcurr_;  // desired velocity / acc from previous spline

      //////////////////////////////////////////////////////////////////////////
      // adaptive controller
      //////////////////////////////////////////////////////////////////////////
      VectorXd theta_; 

      // update time
      ros::Time last_update_time_;
      bool should_respline_;

      // adaptive control learning rate
      double gamma_;

      // spring constant for impedance
      Vector6d springK_;

    public:
      BlueControl(const QString &name = "BlueController");
      ~BlueControl();

      //////////////////////////////////////////////////////////////////////////
      // flow
      //////////////////////////////////////////////////////////////////////////

      bool init();
      bool start();
      Vector6d update(const RobotTime &time, const JointState &state);
      bool stop();

      void setQInit(const JointState &q_init) {q_init_ = q_init;}
      void setQHome(const JointState &q_home) {q_init_ = q_home;}
      void setQPark(const JointState &q_park)  {q_park_ = q_park;}

      //////////////////////////////////////////////////////////////////////////
      // traj generators
      //////////////////////////////////////////////////////////////////////////

      void cartesianSpaceTrajGen(int traj_type, const JointState &state);

      //////////////////////////////////////////////////////////////////////////
      // controller
      //////////////////////////////////////////////////////////////////////////

      Vector6d jointSpaceControl(
        const JointState &cur, const JointState &des, double dt);

      Vector6d cartesianSpaceControl(
        const JointState &cur, const cc::CartesianState &des, double dt);

      Vector6d cartesianAvoiding(
        const JointState &cur, const cc::CartesianState &des, double dt);       

      Vector6d computeYrTh(
        const Vector6d &S_q, const JointState &cur, const JointState &ref, double dt); 


      //////////////////////////////////////////////////////////////////////////
      // callbacks
      //////////////////////////////////////////////////////////////////////////

      void desiredEEFPoseCallback(const nav_msgs::Odometry& msg);
      void desiredEEFPoseCallback_blue(const nav_msgs::Odometry& msg);

      //////////////////////////////////////////////////////////////////////////
      // utility
      //////////////////////////////////////////////////////////////////////////

      Matrix6d computeDampenedJacobianInverse(const cc::Jacobian& J, double lambda = 0.001);
      Vector6d computeImpedanceTau(const JointState &state, const Vector3d& X_red_ , const int j);

    };

  } // namespace RobotControllers
} // namespace tuics_ur_robot_lli

#endif // UR_ROBOT_LLI_SIMPLEEFFORTCONTROL_H
