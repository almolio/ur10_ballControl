#include <blue_controller/blue_controller.h>
#include <tum_ics_ur_robot_msgs/ControlData.h>
#include <control_core/types.h>
#include <ur_model/model_base.h>
#include <math.h>

namespace tum_ics_ur_robot_lli
{
  namespace RobotControllers
  {

    BlueControl::BlueControl(const QString &name) :
      ControlEffort(name, SPLINE_TYPE,JOINT_SPACE, 1.0),
      state_(INITIALIZED),
      is_first_iter_(true),
      ns_("~blue_ctrl"),
      Kp_(Matrix6d::Zero()),
      Kp_cart_(Matrix6d::Zero()),
      Ki_(Matrix6d::Zero()),
      Kd_(Matrix6d::Zero()),
      q_goal_(Vector6d::Zero()),
      model_("ur10_model")
    {
    }

    BlueControl::~BlueControl()
    {
    }

    bool BlueControl::init()
    {
      ROS_WARN_STREAM("BlueControl::init");

      //////////////////////////////////////////////////////////////////////////
      // load parameters
      //////////////////////////////////////////////////////////////////////////
      
      std::vector<double> vec;
      if (!ros::param::has(ns_))
      {
        ROS_ERROR_STREAM("BlueControl init(): Control gains not defined in:" << ns_);
        m_error = true;
        return false;
      }

      // Learning rate for adaptive control
      if (!ros::param::has(ns_ + "/learning_rate")) {
        ROS_ERROR_STREAM("ERROR: Learning rate no set!");
        m_error = true;
        return false;
      }
      ros::param::get(ns_ + "/learning_rate", gamma_);

      // D GAINS
      ros::param::get(ns_ + "/gains_d", vec);
      if (vec.size() < STD_DOF)
      {
        ROS_ERROR_STREAM("gains_d: wrong number of dimensions:" << vec.size());
        m_error = true;
        return false;
      } 
      Kd_ = Eigen::Map<Eigen::VectorXd>(vec.data(), STD_DOF).asDiagonal();

      // P GAINS
      ros::param::get(ns_ + "/gains_p", vec);      // Learning rate for adaptive control
      if (!ros::param::has(ns_ + "/learning_rate")) {
        ROS_ERROR_STREAM("ERROR: Learning rate no set!");
        m_error = true;
        return false;
      }
      ros::param::get(ns_ + "/learning_rate", gamma_);
      if (vec.size() < STD_DOF)
      {
        ROS_ERROR_STREAM("gains_p: wrong number of dimensions:" << vec.size());
        m_error = true;
        return false;
      }
      Kp_ = Eigen::Map<Eigen::VectorXd>(vec.data(), STD_DOF).asDiagonal();

      // P GAINS
      ros::param::get(ns_ + "/gains_p_cart", vec);
      if (vec.size() < STD_DOF)
      {
        ROS_ERROR_STREAM("gains_p: wrong number of dimensions:" << vec.size());
        m_error = true;
        return false;
      }
      Kp_cart_ = Eigen::Map<Eigen::VectorXd>(vec.data(), STD_DOF).asDiagonal();
      Kp_cart_ *= 0.01;

      // I GAINS
      ros::param::get(ns_ + "/gains_i", vec);
      if (vec.size() < STD_DOF)
      {
        ROS_ERROR_STREAM("gains_i: wrong number of dimensions:" << vec.size());
        m_error = true;
        return false;
      }
      Ki_ = Eigen::Map<Eigen::VectorXd>(vec.data(), STD_DOF).asDiagonal();

      // GOAL
      ros::param::get(ns_ + "/goal", vec);
      if (vec.size() < STD_DOF)
      {
        ROS_ERROR_STREAM("gains_p: wrong number of dimensions:" << vec.size());
        m_error = true;
        return false;
      }
      q_goal_ = DEG2RAD(Eigen::Map<Eigen::VectorXd>(vec.data(), STD_DOF));

      ros::param::get(ns_ + "/goal_c", vec);
      if (vec.size() < 7)
      {
        ROS_ERROR_STREAM("goal_c: wrong number of dimensions:" << vec.size());
        m_error = true;
        return false;
      }
      goal_c_ = Eigen::Map<Eigen::VectorXd>(vec.data(), 7);


      //////////////////////////////////////////////////////////////////////////
      // trajectory generators & tracking
      //////////////////////////////////////////////////////////////////////////

      // Fetch spline duration
      if (!ros::param::has(ns_ + "/spline_duration")) {
        ROS_ERROR_STREAM("ERROR: Spline duration not set!");
        m_error = true;
        return false;
      }
      ros::param::get(ns_ + "/spline_duration", spline_duration_);

      // switch between fixed spline and spline to tracked ball
      if (!ros::param::has(ns_ + "/spline_type")) {
        ROS_ERROR_STREAM("ERROR: spline_type not set!");
        m_error = true;
        return false;
      }
      ros::param::get(ns_ + "/spline_type", spline_type_);

      // duration of splines
      if (!ros::param::has(ns_ + "/spline_speed")) {
        ROS_ERROR_STREAM("ERROR: spline_speed not set!");
        m_error = true;
        return false;
      }
      ros::param::get(ns_ + "/spline_speed", spline_speed_);

      // duration of splines
      if (!ros::param::has(ns_ + "/respline_thres")) {
        ROS_ERROR_STREAM("ERROR: respline_thres not set!");
        m_error = true;
        return false;
      }
      ros::param::get(ns_ + "/respline_thres", respline_thres_);

      // smootheness during respline
      if (!ros::param::has(ns_ + "/speed_degration")) {
        ROS_ERROR_STREAM("ERROR: speed_degration not set!");
        m_error = true;
        return false;
      }
      ros::param::get(ns_ + "/speed_degration", speed_degration_);

      // dimension of safety zone
      if (!ros::param::has(ns_ + "/zone_dim")) {
        ROS_ERROR_STREAM("ERROR: zone_dim not set!");
        m_error = true;
        return false;
      }
      ros::param::get(ns_ + "/zone_dim", zone_dim_);

      ros::param::get(ns_ + "/home", vec);
      if (vec.size() < 3)
      {
        ROS_ERROR_STREAM("home: wrong number of dimensions:" << vec.size());
        m_error = true;
        return false;
      }
      home_ = Eigen::Map<Eigen::VectorXd>(vec.data(), 3);
      counter_ = 0;

      ros::param::get(ns_ + "/springK", vec);
      if (vec.size() < STD_DOF)
      {
        ROS_ERROR_STREAM("Wrong Spring Constant size:" << vec.size());
        m_error = true;
        return false;
      }
      springK_ = Eigen::Map<Eigen::VectorXd>(vec.data(), STD_DOF);
      ROS_WARN_STREAM("SPRING CONSTANTS  "  << springK_);

      //////////////////////////////////////////////////////////////////////////
      // desired orientation
      //////////////////////////////////////////////////////////////////////////

      // desired orientation
      if (!ros::param::has(ns_ + "/roll_d")) {
        ROS_ERROR_STREAM("ERROR: roll_d not set!");
        m_error = true;
        return false;
      }
      ros::param::get(ns_ + "/roll_d", roll_d_);

      // desired orientation
      if (!ros::param::has(ns_ + "/pitch_d")) {
        ROS_ERROR_STREAM("ERROR: pitch_d not set!");
        m_error = true;
        return false;
      }
      ros::param::get(ns_ + "/yaw_d", pitch_d_);

      // desired orientation
      if (!ros::param::has(ns_ + "/yaw_d")) {
        ROS_ERROR_STREAM("ERROR: yaw_d not set!");
        m_error = true;
        return false;
      }
      ros::param::get(ns_ + "/yaw_d", yaw_d_);

      //////////////////////////////////////////////////////////////////////////
      // initalize model
      //////////////////////////////////////////////////////////////////////////

      if (!model_.initRequest(nh_)) 
      {
        ROS_ERROR_STREAM("ERORR: initializing model failed!");
        m_error = true;
        return false;  
      }
      theta_ = model_.parameterInitalGuess();

      //////////////////////////////////////////////////////////////////////////
      // reset status
      //////////////////////////////////////////////////////////////////////////
      time_prev_ = ros::Time::now();
      
      q_state_des_.q.setZero();
      q_state_des_.qp.setZero();
      q_state_des_.qpp.setZero();
      x_state_des_.setZero();
      holding_ = false;
      state_ = JOINT;
      counter_ = 0;

      //////////////////////////////////////////////////////////////////////////
      // accept ros connections
      //////////////////////////////////////////////////////////////////////////
      control_data_pub_ = nh_.advertise<tum_ics_ur_robot_msgs::ControlData>("controller_data", 1);
      redball_state_sub_ = nh_.subscribe("/redball/state/filtered", 1, &BlueControl::desiredEEFPoseCallback, this);

      //////////////////////////////////////////////////////////////////////////
      // initialize time
      //////////////////////////////////////////////////////////////////////////
      last_update_time_ = ros::Time::now();

      //////////////////////////////////////////////////////////////////////////
      // print status
      //////////////////////////////////////////////////////////////////////////
      
      ROS_WARN_STREAM("------------------------------------------------------");
      ROS_WARN_STREAM("Kd: \n" << Kd_);
      ROS_WARN_STREAM("Kp: \n" << Kp_);
      ROS_WARN_STREAM("Kp_cart: \n" << Kp_cart_);
      ROS_WARN_STREAM("q_goal [DEG]: \n" << q_goal_.transpose());
      ROS_WARN_STREAM("goal_c_ [p,Q]: \n" << goal_c_.transpose());
      ROS_WARN_STREAM("------------------------------------------------------");

      return true;
    }

    ////////////////////////////////////////////////////////////////////////////
    // flow
    ////////////////////////////////////////////////////////////////////////////

    bool BlueControl::start()
    {
      return true;
    }

    Vector6d BlueControl::update(const RobotTime &time, const JointState &state)
    {
      

      tau_.setZero();

      if (is_first_iter_)
      {
        is_first_iter_ = false;
        elapsed_ = 0;
        q_start_ = state.q;
        ROS_WARN_STREAM("START [DEG]: \n" << q_start_.transpose());
      }

      // update time
      ros::Time time_cur = ros::Time::now();
      double dt = (time_cur - time_prev_).toSec();
      time_prev_ = time_cur;
      elapsed_ += dt;

      ////////////////////////////////////////////////////////////////////////
      // update statemachine
      ////////////////////////////////////////////////////////////////////////

      // next state
      State next_state = state_;
      if(state_ == JOINT && elapsed_ > spline_duration_)
      {
        next_state = CARTESIAN;
      }

      // transitions
      if(state_ == JOINT && next_state == CARTESIAN)
      {
        ROS_WARN_STREAM("SWITCHING: JOINT -> CARTESIAN");
        elapsed_ = 0.0;
        X_start_ = model_.T_ef_0(state.q);
        X_goal_.linear() = home_;
        X_goal_.angular() = Eigen::Quaterniond(1,0,0,0);
        // ROS_WARN_STREAM("X_goal lin = " << X_goal_.linear().toString());
        // ROS_WARN_STREAM("X_goal ang = " << X_goal_.angular().toString());
      }
      state_ = next_state;

      // execute state
      if (state_ == JOINT)
      {
        // joint traj generator
        VVector6d res;
        res = getJointPVT5(q_start_, q_goal_, elapsed_, spline_duration_);
        q_state_des_.q = res[0];
        q_state_des_.qp = res[1];
        q_state_des_.qpp = res[2];
        // ROS_WARN_STREAM("q_state_des_ position = " << res[0]);
        // ROS_WARN_STREAM("q_state_des_ velocity = " << res[1]);
        // ROS_WARN_STREAM("q_state_des_ accel = " << res[2]);

        // controller
        tau_ = jointSpaceControl(state, q_state_des_, dt);
      }
      else if(state_ == CARTESIAN)
      {
        auto delta_t = (ros::Time::now() - last_update_time_).toSec();

        if (should_respline_) {
          // ROS_WARN_STREAM("Respline!");
          X_start_ = model_.T_ef_0(state.q);
          elapsed_ = 0;
          should_respline_ = false;
        }
        else if (delta_t < spline_duration_ && holding_ == false)
        {
          // ROS_INFO_STREAM_THROTTLE(1.0, "*********************RESPLINING*********************");
            // keep resplining
          // cartesian traj generator
          VVector6d res;
          Vector6d x_start, x_goal;

          x_start.head(3) = X_start_.linear();
          x_goal.head(3) = X_goal_.linear();
          // ROS_WARN_STREAM("x_goal ** = " << x_goal);

          // compute duration of spline
          Vector3d delta_x, vec;
          vec.setZero();
          delta_x = X_start_.linear() - X_goal_.linear();
          double d = (delta_x).norm();
          spline_duration_ = d / spline_speed_;

          res = getJointPVT5(x_start, x_goal, elapsed_, spline_duration_);
          Quaterniond Q = X_start_.angular().slerp(std::min(elapsed_/spline_duration_,1.0), X_goal_.angular());

          // compute velocity to push ball out of the safety zone   
          double coef = 1;    
          if (d < 0.12)
          {
            xcurr_ = model_.T_ef_0(state.q).position();
            
            delta_x = X_goal_.linear() - xcurr_;
            d = (delta_x).norm();
            vec = res[1].head(3);
            // ROS_WARN_STREAM("desired speed : " << vec.transpose());
            coef = 0.4;
            vec = coef * vec.norm() * delta_x / d;
            // ROS_WARN_STREAM("adding speed : " << vec.transpose());
          }

          // decay of vel from previous spline
          double decay = pow(speed_degration_, counter_);
          x_state_des_.pos() << res[0].head(3), Q.coeffs();
          x_state_des_.vel().linear() = coef * (1-decay) * res[1].head(3) + decay * x_state_des_prev_.vel().linear() + vec;
          x_state_des_.acc().linear() = (1-decay) * res[2].head(3) + decay * x_state_des_prev_.acc().linear();

        } // hold position
        else {
          // ROS_INFO_STREAM_THROTTLE(1.0, "*********************HOLDING POSITION*********************");
          X_goal_.linear() = home_;
          X_goal_.angular() = Eigen::Quaterniond(1,0,0,0);
          elapsed_ = 0;
          counter_ = 0;
          holding_ = true;
          last_update_time_ = ros::Time::now();
          // ROS_INFO_STREAM_THROTTLE(1.0, "X_goal lin = " << X_goal_.linear().toString());
          // ROS_INFO_STREAM_THROTTLE(1.0, "X_goal ang = " << X_goal_.angular().toString());
          double decay = pow(speed_degration_, counter_);
          x_state_des_.pos() << X_goal_.linear();
          x_state_des_.vel().linear().setZero();
          x_state_des_.acc().linear().setZero();
        }

        // controller
        tau_ = cartesianSpaceControl(state, x_state_des_, dt);
      }

      return tau_;
    }

    bool BlueControl::stop()
    {
      return true;
    }
    ////////////////////////////////////////////////////////////////////////////
    // trajectory generator
    ////////////////////////////////////////////////////////////////////////////

    void BlueControl::cartesianSpaceTrajGen(int traj_type, const JointState &state)
    {
      VVector6d res;
      Vector6d x_start, x_goal;
      Quaterniond Q;
      // generate spline per the fixed goal set in the .yaml config
      if(traj_type == 1)
      {
        // cartesian traj generator
        x_start.head(3) = X_start_.linear(); 
        x_goal.head(3) = X_goal_.linear();

        // // compute time to get there
        // double d = (X_start_.linear() - x_state_blue_ball_.pos().linear()).norm();
        // spline_duration_ = d/spline_speed_;

        res = getJointPVT5(x_start, x_goal, elapsed_, spline_duration_);
        Q = X_start_.angular().slerp(std::min(elapsed_/spline_duration_,1.0), X_goal_.angular());

      }
      // pick goal from odometry message
      if(traj_type == 2)
      {
        // set data from odometry message to goal
        X_start_ = model_.T_ef_0(state.q);
        x_start.head(3) = X_start_.linear(); 
        x_goal.head(3) = x_state_blue_ball_.pos().linear();

        // compute time to get there
        double d = (X_start_.linear() - x_state_blue_ball_.pos().linear()).norm();
        spline_duration_ = d/spline_speed_;

        res = getJointPVT5(x_start, x_goal, elapsed_, spline_duration_);

        Q = X_start_.angular().slerp(std::min(elapsed_/spline_duration_,1.0), X_goal_.angular());
        ROS_INFO_STREAM_THROTTLE(1.0, "tracking ball at position : " << x_goal.transpose());
      }
      x_state_des_.pos() << res[0].head(3), Q.coeffs();
      x_state_des_.vel().linear() = res[1].head(3);
      x_state_des_.acc().linear() = res[2].head(3);
    }

    ////////////////////////////////////////////////////////////////////////////
    // callbacks
    ////////////////////////////////////////////////////////////////////////////

    void BlueControl::desiredEEFPoseCallback(const nav_msgs::Odometry& msg) 
    {
      auto delta_t = (ros::Time::now() - last_update_time_).toSec();
      // ROS_WARN_STREAM("delta t = " << delta_t << "\n");
      if (delta_t < respline_thres_ * spline_duration_ && holding_ == false) {
        should_respline_ = false;
        counter_++;
        // ROS_WARN_STREAM("Skipping callback, last update was less than 0.5 seconds ago");
        return;
      }

      if(state_ == CARTESIAN)
      {
        auto T = model_.T_B_0();
        Eigen::Vector4d transformed_ball_position(msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z, 1);
        transformed_ball_position = T * transformed_ball_position;
        
        // if ((xcurr_ - transformed_ball_position.head(3)).norm() < zone_dim_) {
        if (transformed_ball_position.head(3).norm() < zone_dim_) {
          holding_ = false;
          spline_duration_ = 10;
          // ROS_WARN_STREAM("Got new goal pose : " << msg.pose.pose.position << "\n");
          X_goal_.linear() = transformed_ball_position.head(3);
          // ROS_WARN_STREAM("goal : " << X_goal_.linear());

          // Set desired orientation 
          Eigen::Quaternion<float> q;
          q = Eigen::AngleAxisf(roll_d_, Eigen::Vector3f::UnitX()) *
            Eigen::AngleAxisf(pitch_d_, Eigen::Vector3f::UnitY()) *
            Eigen::AngleAxisf(yaw_d_, Eigen::Vector3f::UnitZ());

          // ROS_WARN_STREAM("q = " << q.coeffs()(0) << "\n");

          X_goal_.angular() = Eigen::Quaterniond(1,0,0,0);
        }
        else if (holding_ == false)
        {
          // ROS_WARN_STREAM("*********************PARKING HOME*********************");
          // ROS_WARN_STREAM("distance : " << transformed_ball_position.head(3).norm());
          X_goal_.linear() = home_;
          // ROS_WARN_STREAM("home : " << X_goal_.linear());
          X_goal_.angular() = Eigen::Quaterniond(1,0,0,0);
        }
        should_respline_ = true;
        elapsed_ = 0;
        counter_ = 0;
        last_update_time_ = ros::Time::now();
        x_state_des_prev_.vel().linear() = x_state_des_.vel().linear();
        x_state_des_prev_.acc().linear() = x_state_des_.acc().linear();
      }

    }

    void BlueControl::desiredEEFPoseCallback_blue(const nav_msgs::Odometry& msg) 
    {
        // desired pose
        Vector4d ball_pre;
        ball_pre <<  msg.pose.pose.position.x,
                     msg.pose.pose.position.y,
                     msg.pose.pose.position.z,
                     1;

        Vector4d result = model_.T_B_0() * ball_pre;
        x_state_blue_ball_.pos().linear() = result.head(3);
    }

    ////////////////////////////////////////////////////////////////////////////
    // control
    ////////////////////////////////////////////////////////////////////////////

    Vector6d BlueControl::cartesianSpaceControl(
      const JointState &cur, const cc::CartesianState &des, double dt)
    {
      auto T_ef_0 = model_.T_ef_0(cur.q);
      auto J_ef_0 = model_.J_ef_0(cur.q);
      auto Jp_ef_0 = model_.Jp_ef_0(cur.q, cur.qp);

      // current cartesian state
      x_state_cur_.pos().linear() = T_ef_0.translation();
      x_state_cur_.pos().angular() = T_ef_0.rotation();
      x_state_cur_.vel() = J_ef_0*cur.qp;

      // control errors
      cc::CartesianVector X_delta;
      X_delta.linear() = x_state_cur_.pos().linear() -  x_state_des_.pos().linear();
      Eigen::AngleAxisd aa(x_state_cur_.pos().angular()*des.pos().angular().inverse());
      X_delta.angular() = aa.angle()*aa.axis();

      ROS_INFO_STREAM_THROTTLE(1.0, "setting X_delta to: " << X_delta.transpose());

      cc::CartesianVector Xp_delta;
      Xp_delta = x_state_cur_.vel() - x_state_des_.vel();

      // references
      x_state_ref_.vel() = des.vel() - Kp_cart_*X_delta;
      x_state_ref_.acc() = des.acc() - Kp_cart_*Xp_delta;

      // conversion back to jointspace
      auto J_ef_0_pinv = computeDampenedJacobianInverse(J_ef_0);      
      q_state_ref_.qp = J_ef_0_pinv*x_state_ref_.vel();
      q_state_ref_.qpp = J_ef_0_pinv*(x_state_ref_.acc() - Jp_ef_0*cur.qp);

      // control action
      Vector6d S_q = cur.qp - q_state_ref_.qp;
      return -Kd_ * S_q + computeYrTh(S_q, cur, q_state_ref_, dt);
    } 
    
    Vector6d BlueControl::jointSpaceControl(const JointState &cur, const JointState &des, double dt)
    { 
      // references
      q_state_ref_.qp = des.qp - Kp_*(cur.q - des.q);
      q_state_ref_.qpp = des.qpp - Kp_*(cur.qp - des.qp);

      // control action
      Vector6d S_q = cur.qp - q_state_ref_.qp;
      return -Kd_ * S_q + computeYrTh(S_q, cur, q_state_ref_, dt);
    }

    Vector6d BlueControl::computeYrTh(const Vector6d &S_q, const JointState &cur, const JointState &ref, double dt)
    {
      const auto& Yr = model_.regressor(cur.q, cur.qp, ref.q, ref.qp);
      theta_ -= gamma_ * Yr.transpose() * S_q * dt;
      return Yr * theta_;
    }

    Vector6d BlueControl::cartesianAvoiding(
      const JointState &cur, const cc::CartesianState &des, double dt)
      {
      
      auto T_ef_0 = model_.T_ef_0(cur.q);
      auto J_ef_0 = model_.J_ef_0(cur.q).linear();   
      auto Jp_ef_0 = model_.Jp_ef_0(cur.q, cur.qp).linear();   

      // current cartesian state
      x_state_cur_.pos().linear() = T_ef_0.translation();  
      x_state_cur_.pos().angular() = T_ef_0.rotation();
      x_state_cur_.vel().linear() = J_ef_0*cur.qp;    

      // control errors
      cc::CartesianVector X_delta;
      X_delta.linear() = x_state_cur_.pos().linear() -  x_state_des_.pos().linear();
      Eigen::AngleAxisd aa(x_state_cur_.pos().angular()*des.pos().angular().inverse());

      cc::CartesianVector Xp_delta;
      Xp_delta.linear() = x_state_cur_.vel().linear() - x_state_des_.vel().linear(); 

      // references
      x_state_ref_.vel().linear() = des.vel().linear() - Kp_cart_.block(0,0,2,2)*X_delta;  
      x_state_ref_.acc().linear() = des.acc().linear() - Kp_cart_.block(0,0,2,2)*Xp_delta;

      // Compute the inverse, for the nulspace 
      auto J_ef_0_pinv = computeDampenedJacobianInverse((J_ef_0.transpose()*J_ef_0)) * J_ef_0.transpose();
      q_state_ref_.qp = J_ef_0_pinv*x_state_ref_.vel().linear(); 
      q_state_ref_.qpp = J_ef_0_pinv*(x_state_ref_.acc().linear() - Jp_ef_0*cur.qp);

      // control action
      Vector6d S_q = cur.qp - q_state_ref_.qp;
      Vector3d ballpos; 
      ballpos << x_state_blue_ball_.pos().linear();

      // Get the nullspace stuff 
      auto tauTotalAvoid = computeImpedanceTau(cur, ballpos, 2);
      auto Null_sp = Matrix6d::Identity() - J_ef_0.transpose()  *  J_ef_0_pinv.transpose();
      Vector6d task2 = Null_sp * tauTotalAvoid;

      // Combining the control torque
      Vector6d tau_trackin = -Kd_.block(0,0,2,2) * S_q + computeYrTh(S_q, cur, q_state_ref_, dt); 
      ROS_INFO_STREAM( "Tau Avoidance  " << task2);

      return -Kd_ * S_q + computeYrTh(S_q, cur, q_state_ref_, dt) + task2; 
      }


    ////////////////////////////////////////////////////////////////////////////
    // utilties
    ////////////////////////////////////////////////////////////////////////////

    Matrix6d BlueControl::computeDampenedJacobianInverse(const cc::Jacobian& J, double lambda) 
    {
      return J.transpose() * (J * J.transpose() + lambda * cc::Jacobian::Identity()).inverse();
    }

    Vector6d BlueControl::computeImpedanceTau(const JointState& state, const Vector3d& X_red_, const int j)
    {
      // See if the ball is by the robot before activate the avoidance  
      cc::HomogeneousTransformation T_i_0;
      Vector3d F_red_i;
      Vector6d tau_red_i;
      double d;
      double d_min = 10e-5;
      T_i_0 = model_.T_j_0(state.q, j);
      d = (X_red_ - T_i_0.position()).norm();  
      ROS_INFO_STREAM( " Distance Ball from joint j " << j << "  " << d);
      if(d < d_min)
        d = d_min;

      // if the ball is within the detection zone 
      Vector3d current_spingK; 
      // current_spingK << springK_[j] ,springK_[j] ,springK_[j] ; 
      double SPRING = springK_[0]; // * 10e10;

      current_spingK << SPRING, SPRING,SPRING;
      if(d < 0.5)
      {
        F_red_i.x() = current_spingK.x() /  (X_red_ - T_i_0.position()).x();
        F_red_i.y() = current_spingK.y() /  (X_red_ - T_i_0.position()).y();
        F_red_i.z() = current_spingK.z() /  (X_red_ - T_i_0.position()).z();

        tau_red_i = model_.J_j_0(state.q,j).block(0, 0, 2, 5).transpose() * F_red_i;
      }
      else    // else we can set everything to zero 
          tau_red_i.setZero(); 
        
      ROS_INFO_STREAM("Tau Avoidance joint i " << tau_red_i.transpose());

      return tau_red_i;
    }

  } // namespace RobotControllers
} // namespace tum_ics_ur_robot_lli
