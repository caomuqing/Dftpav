/**
 * @file ssc_server.cc
 * @author HKUST Aerial Robotics Group
 * @brief implementation for ssc planner server
 * @version 0.1
 * @date 2019-02
 * @copyright Copyright (c) 2019
 */

/**
 * @file traj_server_ros.cpp revised version
 * @brief for minco planner server
 * @version 0.2
 * @date 2021-04
 * @copyright Copyright (c) 2021
 */

#include "plan_utils/traj_server_ros.h"

namespace plan_utils
{

  TrajPlannerServer::TrajPlannerServer(ros::NodeHandle nh, int ego_id)
      : nh_(nh), work_rate_(20.0), ego_id_(ego_id) 
  {
    p_input_smm_buff_ = new moodycamel::ReaderWriterQueue<SemanticMapManager>(
        config_.kInputBufferSize);
    // p_smm_vis_   = new semantic_map_manager::Visualizer(nh, ego_id);
    p_traj_vis_  = new TrajVisualizer(nh, ego_id);

    nh.getParam("enable_urban", enable_urban_);
    p_planner_   = new plan_manage::TrajPlanner(nh, ego_id, enable_urban_);
    
   

  }
  //use this!
  TrajPlannerServer::TrajPlannerServer(ros::NodeHandle nh, double work_rate, int ego_id)
      : nh_(nh), work_rate_(work_rate), ego_id_(ego_id) 
  {
    p_input_smm_buff_ = new moodycamel::ReaderWriterQueue<SemanticMapManager>(
        config_.kInputBufferSize);
    // p_smm_vis_   = new semantic_map_manager::Visualizer(nh, ego_id);
    p_traj_vis_  = new TrajVisualizer(nh, ego_id);

    nh.getParam("enable_urban", enable_urban_);
    nh.param("isparking", isparking,true);
    p_planner_   = new plan_manage::TrajPlanner(nh, ego_id, enable_urban_);
    last_people_angle_time_ = ros::Time::now();
    last_activate_rotation_time_ = ros::Time::now();

    parking_sub_ = nh_.subscribe("/shortterm_goal", 1, &TrajPlannerServer::ParkingCallback, this);
    scan_sub_= nh_.subscribe("/scan", 1,  &TrajPlannerServer::ScanCallback, this);
    people_angle_sub_= nh_.subscribe("/angle_list", 1,  &TrajPlannerServer::peopleAngleCallback, this);
    odom_sub_= nh_.subscribe("/odom", 1,  &TrajPlannerServer::odom_cb, this);
    weights_sub = nh_.subscribe("/set_weights", 1000, &TrajPlannerServer::weightsCallback, this);
    reset_sub = nh_.subscribe("/reset_planner", 1000, &TrajPlannerServer::resetCallback, this);

    // if(!isparking){
    //   trajplan = std::bind(&plan_manage::TrajPlanner::RunOnce,p_planner_);
    // }
    // else{
    //   trajplan = std::bind(&plan_manage::TrajPlanner::RunOnceParking,p_planner_);
    // }
    

  }

  void TrajPlannerServer::PushSemanticMap(const SemanticMapManager& smm) 
  {
    if (p_input_smm_buff_) p_input_smm_buff_->try_enqueue(smm);
  }


  void TrajPlannerServer::Init(const std::string& config_path) 
  {

    p_planner_->Init(config_path, cfg_);

    wei_obs_ = cfg_.opt_cfg().wei_sta_obs();
    wei_surround_ = cfg_.opt_cfg().wei_dyn_obs();
    wei_feas_ = cfg_.opt_cfg().wei_feas();
    wei_sqrvar_ = cfg_.opt_cfg().wei_sqrvar();
    wei_time_ = cfg_.opt_cfg().wei_time();

    wei_obs_NN = wei_obs_;
    wei_surround_NN = wei_surround_;
    wei_feas_NN = wei_feas_; 
    wei_sqrvar_NN = wei_sqrvar_;
    wei_time_NN = wei_time_;
    wei_jerk_NN = wei_jerk_;

    nh_.param("use_sim_state", use_sim_state_, true);
    nh_.param("gain_heading_follow", gain_heading_follow_, 0.4);
    nh_.param("gain_heading_y_correction", gain_heading_y_correction_, 0.6);
    std::string traj_topic = std::string("/vis/agent_") +
                           std::to_string(ego_id_) +
                           std::string("/minco/exec_traj");

    // to publish command to sim
    ctrl_signal_pub_ = nh_.advertise<vehicle_msgs::ControlSignal>("ctrl", 20);
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 20);
    executing_traj_vis_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(traj_topic, 1);
    debug_pub =  nh_.advertise<std_msgs::Bool>("/DEBUG", 1);
    obstmap_pub = nh_.advertise<std_msgs::Int32MultiArray>("/obst_map", 1000);
    weights_pub = nh_.advertise<traj_planner::Weights>("/using_weights", 1000);

    end_pt_.setZero();


  }

  //call the main thread
  void TrajPlannerServer::Start() 
  {
    // printf("\033[34mTrajPlannerServer]TrajPlannerServer::Start\n\033[0m");


    if (is_replan_on_) {
      return;
    }
    is_replan_on_ = true;  // only call once
    //@yuwei : set map interface to planner
    p_planner_->set_map_interface(&map_adapter_);
    printf("[TrajPlannerServer]Planner server started.\n");

    std::thread(&TrajPlannerServer::MainThread, this).detach();
    std::thread(&TrajPlannerServer::PublishThread, this).detach();
  }
  void TrajPlannerServer::PublishThread(){
    using namespace std::chrono;
    system_clock::time_point current_start_time{system_clock::now()};
    system_clock::time_point next_start_time{current_start_time};
    const milliseconds interval{static_cast<int>(20)}; // 20ms, 50hz
    while (true) {
      current_start_time = system_clock::now();
      next_start_time = current_start_time + interval;
      PublishData();
     
      std::this_thread::sleep_until(next_start_time);
    }
  }
  void TrajPlannerServer::MainThread() {
    using namespace std::chrono;
    system_clock::time_point current_start_time{system_clock::now()};
    system_clock::time_point next_start_time{current_start_time};
    const milliseconds interval{static_cast<int>(1000.0 / work_rate_)}; // 50ms, 20hz
    while (true) {
      current_start_time = system_clock::now();
      next_start_time = current_start_time + interval;
      PlanCycleCallback();
      std::this_thread::sleep_until(next_start_time);
    }
  }

  void TrajPlannerServer::addStopTraj() {
    double current_time = ros::Time::now().toSec();
    m.lock();
    if (executing_traj_ == nullptr || executing_traj_->size() - 1 < exe_traj_index_ ||
        (final_traj_index_ == exe_traj_index_ && executing_traj_->at(exe_traj_index_).duration <= num_decelerate_+0.01) )
    {
      m.unlock();
      return;
    }
    common::State state;
    common::State real_state;
    if(map_adapter_.GetEgoState(&real_state)!=kSuccess) return;

    executing_traj_->at(exe_traj_index_).traj.GetState(current_time-executing_traj_->at(exe_traj_index_).start_time, &state);
    double decelerate = state.velocity>0? 2.0 : -2.0;
    double duration = state.velocity/decelerate;
    double theta = real_state.angle; //use the real robot heading
    Eigen::Vector2d vel(state.velocity * cos(theta), state.velocity * sin(theta)); //use the velocity of thed desired
    Eigen::Vector2d acc(-decelerate * cos(theta)/2.0, -decelerate * sin(theta)/2.0);
    Eigen::Matrix<double, 2, 6> posmatrix;
    posmatrix.setZero();
    posmatrix.col(5) = real_state.vec_position; //use the position of the real robot
    posmatrix.col(4) = vel;
    posmatrix.col(3) = acc;

    // plan_utils::Piece stoptraj(1.0, posmatrix, 1.0);
    
    std::vector<double> durs;
    std::vector<CoefficientMat> cMats1;
    cMats1.push_back(posmatrix);
    durs.push_back(duration);
    plan_utils::Trajectory _traj(durs, cMats1, 1);
    plan_utils::TrajContainer trajcon;
    trajcon.addSingulTraj(_traj, current_time);

    next_traj_ = std::unique_ptr<plan_utils::SingulTrajData>(new plan_utils::SingulTrajData(trajcon.singul_traj));
    num_decelerate_ = duration;
    executing_traj_ = std::move(next_traj_);
    next_traj_.release();
    final_traj_index_ = executing_traj_->size() - 1;
    exe_traj_index_ = 0; 
    m.unlock();
  }

  void TrajPlannerServer::PlanCycleCallback() {
    if (!is_replan_on_) {
      return;
    }
    while (p_input_smm_buff_->try_dequeue(last_smm_)) {
      is_map_updated_ = true;
    }
    /*find the latest smm
    if no smm, the process will return */
    if (!is_map_updated_) {
      printf("No map is updated");
      return;
    }
    // Update map
    auto map_ptr =
        std::make_shared<semantic_map_manager::SemanticMapManager>(last_smm_);
    map_adapter_.set_map(map_ptr);  //=maptp, the mapinterface used in p_planner
    // PublishData();
    //vis smm, vis lasttraj, pub control law
    double current_time = ros::Time::now().toSec();
    if (executing_traj_!=nullptr && current_time > executing_traj_->at(final_traj_index_).end_time) {

      printf("[TrajPlannerServer]Mission complete.\n");
      m.lock();
      executing_traj_.release();
      m.unlock();
      p_planner_->release_initial_state();
      // return;
      // printf("[TrajPlannerServer]Mission complete1.\n");
    }
     std_msgs::Bool debug;
      debug.data = 1;
      debug_pub.publish(debug);
    if (enable_urban_){
      // the requency is high enough so that we will not consider event triggered replanning
      static int fsm_num = 0;
      // printf("[TrajPlannerServer]Mission complete2.\n");
      fsm_num++;
      int new_goal = 0;
      if (fsm_num > 1000000000||(CheckReplan(new_goal)) || executing_traj_==nullptr) {

        if (new_goal == 0) addStopTraj();
          if (executing_traj_ != nullptr) 
          {
            p_traj_vis_->displayPolyTraj(executing_traj_);
            Display();    
          }          
     
        ErrorType replanResult; 
        if (next_traj_ == nullptr) {
          replanResult = Replan();
          // printf("[TrajPlannerServer]Mission complete4.\n");
          // return;
        }
        if (next_traj_ !=nullptr && replanResult == kSuccess) {
          
        // ros::Duration(100.0).sleep();
          
          if (CheckReplanTraj(next_traj_, 0, next_traj_->size() - 1)) //check if the new plan works, if not, delete it again
          {
            // if (executing_traj_ == nullptr || executing_traj_->size() - 1 < exe_traj_index_) 
            // {
            //   return;
            // }          
            // p_traj_vis_->displayPolyTraj(executing_traj_);
            // Display();            
            next_traj_.release(); //do not use the generated trajectory
            // std::cout<<"check replan for new plan returns true!"<<std::endl;
            printf("\033[32m[PlanCycleCallback]check replan for new plan returns true! %lf ms.\n\033[0m");
          }
          else
          {
            m.lock();
            executing_traj_ = std::move(next_traj_);
            next_traj_.release();
            fsm_num = 0;
            final_traj_index_ = executing_traj_->size() - 1;
            exe_traj_index_ = 0; 
            m.unlock();            
            p_traj_vis_->displayPolyTraj(executing_traj_);
            Display();
          }
          return;
        }
        else // no feasible replan found
        {
          // m.lock();
          // if (executing_traj_!=nullptr) executing_traj_.release();
          // m.unlock();

          // addStopTraj();
          // printf("\033[32m[PlanCycleCallback]no feasible replan found! adding stop traj!! %lf ms.\n\033[0m");
          if (next_traj_ != nullptr) next_traj_.release();
          if (executing_traj_ == nullptr || executing_traj_->size() - 1 < exe_traj_index_) 
          {
            return;
          }          
          p_traj_vis_->displayPolyTraj(executing_traj_);
          Display();
        }
      }
      else{     
        Display();    

      }

    }
    else{
       
    }
  }


  void TrajPlannerServer::PublishData() {
    using common::VisualizationUtil;
    m.lock();
    ros::Time current_time = ros::Time::now();
    if(!is_map_updated_) 
    {
      m.unlock();
      return;}
    // smm visualization
    if(is_map_updated_)
    {
      // p_smm_vis_->VisualizeDataWithStamp(current_time, last_smm_);
      // p_smm_vis_->SendTfWithStamp(current_time, last_smm_);
    }

    common::State _desired_state;
    if(map_adapter_.GetEgoState(&_desired_state)==kSuccess)
    {
        //for reinforcement learning
        int grid_number = 30;
        double grid_res = 0.2;
        vec_Vec2f vec_obs = p_planner_->display_vec_obs();
        double origin_x = _desired_state.vec_position[0] + grid_res * (double)grid_number/2.0;
        double origin_y = _desired_state.vec_position[1] + grid_res * (double)grid_number/2.0;
        Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic> obst_map(grid_number, grid_number);
        obst_map.setZero();

        for (size_t i = 0; i < vec_obs.size(); ++i)
        {
          double x_diff = vec_obs[i](0) - _desired_state.vec_position[0];
          double y_diff = vec_obs[i](1) - _desired_state.vec_position[1];
          if (fabs(x_diff)> grid_res * (double)grid_number/2.0 || fabs(y_diff)> grid_res * (double)grid_number/2.0)
          continue;
          int coord_x = std::floor((origin_x-vec_obs[i](0))/grid_res);
          int coord_y = std::floor((origin_y-vec_obs[i](1))/grid_res);
          if (coord_x<0 || coord_x>grid_number || coord_y<0 || coord_y>grid_number)
          {
            std::cout << "NOT GOOD!!!!!!!!!!!!!!!!!!!!!!!!!!"<<std::endl;
            std::cout << "coord_x::"<<coord_x<<"coord_y::"<<coord_y<<std::endl;
          }

          obst_map(coord_x, coord_y) = 1;

        }

        for (auto state: traj_upcoming_)
        {
          double x_diff = state(0) - _desired_state.vec_position[0];
          double y_diff = state(1) - _desired_state.vec_position[1];
          if (fabs(x_diff)> grid_res * (double)grid_number/2.0 || fabs(y_diff)> grid_res * (double)grid_number/2.0)
          continue;
          int coord_x = std::floor((origin_x-state(0))/grid_res);
          int coord_y = std::floor((origin_y-state(1))/grid_res);

          obst_map(coord_x, coord_y) = 2;          
        }
        
        std_msgs::Int32MultiArray array_msg = eigenToMultiArray(obst_map);
        obstmap_pub.publish(array_msg);
        publish_weights();
        // std::cout<<obst_map<<std::endl;
    }
    
    if (executing_traj_ == nullptr ||exe_traj_index_ > final_traj_index_ ||
     executing_traj_->at(exe_traj_index_).duration < 1e-5) {
       common::State state = ego_state;
       state.velocity = 0.0;
       state.steer = 0.0;
       state.acceleration = 0.0;
       state.curvature = 0.0;
       double t = current_time.toSec();
       state.time_stamp = t;
       if(ctrl_state_hist_.size()==0){
           ctrl_state_hist_.push_back(ego_state);
       }
       else{
          FilterSingularityState(ctrl_state_hist_, &state);
          ctrl_state_hist_.push_back(state);
          if (ctrl_state_hist_.size() > 100) ctrl_state_hist_.erase(ctrl_state_hist_.begin());
       }
       
     
      
       vehicle_msgs::ControlSignal ctrl_msg;
       vehicle_msgs::Encoder::GetRosControlSignalFromControlSignal(
              common::VehicleControlSignal(state), ros::Time(t),
              std::string("map"), &ctrl_msg);
       ctrl_signal_pub_.publish(ctrl_msg);

        common::State desired_state;
        if(map_adapter_.GetEgoState(&desired_state)==kSuccess)
        {

          geometry_msgs::Twist cmd_msg;
          cmd_msg.linear.x = 0.0;
          cmd_msg.linear.y = 0.0;
          cmd_msg.linear.z = 0.0;
          cmd_msg.angular.x = 0.0;
          cmd_msg.angular.y = 0.0;
          cmd_msg.angular.z = 0.0;
          // cmd_msg.angular.z = state.velocity*tan(state.steer)/0.7;

          cmd_vel_pub_.publish(cmd_msg);              
        } 
       m.unlock();
       return;
     }

    // trajectory feedback 
    {
      if (use_sim_state_) {
        common::State state;
        
        double t = current_time.toSec();

        state.time_stamp = t;

        if (executing_traj_->at(exe_traj_index_).end_time <= t){
          exe_traj_index_ += 1;
        }
        if (exe_traj_index_ > final_traj_index_){ 
          m.unlock();return;}
        // std::cout << " exe_traj_index_ " << exe_traj_index_ <<  std::endl;
        // std::cout << " final_traj_index_ " << final_traj_index_ <<  std::endl;
        executing_traj_->at(exe_traj_index_).traj.GetState(t-executing_traj_->at(exe_traj_index_).start_time, &state);
        // std::cout<<"state: "<<state.vec_position.transpose()<<"\n"; 
        FilterSingularityState(ctrl_state_hist_, &state);
        ctrl_state_hist_.push_back(state);
        if (ctrl_state_hist_.size() > 100) ctrl_state_hist_.erase(ctrl_state_hist_.begin());
        
        vehicle_msgs::ControlSignal ctrl_msg;
        vehicle_msgs::Encoder::GetRosControlSignalFromControlSignal(
                common::VehicleControlSignal(state), ros::Time(t),
                std::string("map"), &ctrl_msg);

        //hzchzc

        /*
        vec_position: 
    x: 9.96869580051
    y: 58.806571087
    z: 0.0
  angle: 0.777951436039
  curvature: 0.000542476085517
  velocity: 9.70671770974
  acceleration: 0.443944701338
  steer: 0.00154605561188
        */

        // ctrl_msg.state.acceleration = 0;
        // ctrl_msg.state.curvature = 0;
        // ctrl_msg.state.acceleration = 0;
        // ctrl_msg.state.steer = 0;


        // std::cout<<"ctrl_msg: "<<ctrl_msg.state.vec_position.x<<" "<<ctrl_msg.state.vec_position.y<<std::endl;
      common::State desired_state;
      if(map_adapter_.GetEgoState(&desired_state)==kSuccess)
      {
        ctrl_signal_pub_.publish(ctrl_msg);

        double vel_cmd = 0.0, yaw_cmd = 0.0;
        double max_yaw_rate_ = 40.0f/180.0f*3.14159;

        if (false && (ros::Time::now()-last_activate_rotation_time_).toSec()<4.0)
        {
          double yaw_rate_tmp =wrapToPi(end_pt_(2) - desired_state.angle) * gain_heading_follow_;

          if (yaw_rate_tmp>max_yaw_rate_) yaw_rate_tmp = max_yaw_rate_;
          else if (yaw_rate_tmp<-max_yaw_rate_) yaw_rate_tmp = -max_yaw_rate_;    
          
          yaw_cmd = yaw_rate_tmp;
        }
        else
        {
          Eigen::Matrix<double, 2, 2> rot;
          double theta = -desired_state.angle;
          rot << cos(theta), -sin(theta),
                  sin(theta), cos(theta);
          Vecf<2> pos_error = rot * (state.vec_position - desired_state.vec_position);           
          // std::cout<<"angle: "<<desired_state.angle/3.14*180<< " pos error x: "<<pos_error(0)<<" pos error y: "<<pos_error(1)<<std::endl;    

          // std::cout<<"state.angle: "<<state.angle<<"desired_state.angle "<<desired_state.angle<<std::endl;    
          // std::cout<<"yaw_rate_tmp "<<yaw_rate_tmp<<std::endl;    
          // std::cout<<"ratio is "<<state.velocity*tan(state.steer)/0.7/yaw_rate_tmp<<std::endl;    
          double maxx = 1.65, minx = -1.30;
          vel_cmd = std::min(maxx, std::max(minx, state.velocity) + pos_error(0)*0.4); //hard limit
          if (scan_min_<0.75 || scan_min2_ <0.65) vel_cmd = std::min(0.0, vel_cmd);
          if ((ros::Time::now()-last_people_angle_time_).toSec()<0.5) //people check using image detection
          {
            for (auto people : angle_list_)
            {
              double angle_range = 15; //degree
              double angle_span = 35;
              //people is near the center and spans a large angle
              if (((people(0)> -angle_range && people(0)< angle_range)||
                  (people(1)> -angle_range && people(1)< angle_range)) &&
                    abs(people(0)-people(1))>angle_span)
                {
                  vel_cmd = std::min(0.0, vel_cmd);
                  break;
                }
            }
          }

          //yaw correction
          double yaw_rate_tmp_follow =wrapToPi(state.angle - desired_state.angle) * gain_heading_follow_;
          double yaw_rate_tmp = (vel_cmd>0.0? 1.0 : -1.0) * gain_heading_y_correction_ * pos_error(1) + yaw_rate_tmp_follow;
          if (yaw_rate_tmp>max_yaw_rate_) yaw_rate_tmp = max_yaw_rate_;
          else if (yaw_rate_tmp<-max_yaw_rate_) yaw_rate_tmp = -max_yaw_rate_;    

          yaw_cmd = state.velocity*(tan(state.steer)/0.65)+yaw_rate_tmp; //yaw depends on steering curvature

          double min_turning_radius = 0.48;
          if (fabs(vel_cmd)<0.01) {yaw_cmd = 0.0;}
          else if (fabs(vel_cmd/yaw_cmd)<min_turning_radius) {yaw_cmd = (vel_cmd/yaw_cmd>0? 1.0:-1.0) * vel_cmd/min_turning_radius; }//if longitudinal vel is small, disable turning
          // else if (fabs(wrapToPi(state.angle - desired_state.angle))>0.15)
          if (yaw_rate_tmp>max_yaw_rate_) yaw_rate_tmp = max_yaw_rate_;
          else if (yaw_rate_tmp<-max_yaw_rate_) yaw_rate_tmp = -max_yaw_rate_;    
        }

        geometry_msgs::Twist cmd_msg;
        cmd_msg.linear.x = vel_cmd;
        cmd_msg.linear.y = 0.0;
        cmd_msg.linear.z = 0.0;
        cmd_msg.angular.x = 0.0;
        cmd_msg.angular.y = 0.0;
        // cmd_msg.angular.z = yaw_rate_tmp;
        cmd_msg.angular.z = yaw_cmd;

        cmd_vel_pub_.publish(cmd_msg);           

      }
   
      }
    }

    
    // trajectory visualization
    /*{
      {
        auto color = common::cmap["magenta"];
        if (require_intervention_signal_) color = common::cmap["yellow"];
        visualization_msgs::MarkerArray traj_mk_arr;
        if (require_intervention_signal_) {
          visualization_msgs::Marker traj_status;
          common::State state_begin;
          state_begin.time_stamp = executing_traj_->at(exe_traj_index_).start_time;  // define the timestamp before call get state
          executing_traj_->at(exe_traj_index_).traj.GetState(0.0, &state_begin);
          Vec3f pos = Vec3f(state_begin.vec_position[0],
                            state_begin.vec_position[1], 5.0);
          common::VisualizationUtil::GetRosMarkerTextUsingPositionAndString(
              pos, std::string("Intervention Needed!"), common::cmap["red"],
              Vec3f(5.0, 5.0, 5.0), 0, &traj_status);
          traj_mk_arr.markers.push_back(traj_status);
        }
        int num_traj_mks = static_cast<int>(traj_mk_arr.markers.size());
        common::VisualizationUtil::FillHeaderIdInMarkerArray(
            current_time, std::string("map"), last_trajmk_cnt_,
            &traj_mk_arr);
        last_trajmk_cnt_ = num_traj_mks;
        executing_traj_vis_pub_.publish(traj_mk_arr);
      }
    }*/
    m.unlock();
  }

  // double TrajPlannerServer::wrapToPi(double angle) {
  //   double pi = 3.1415927;
  //   while (angle > pi) {
  //       angle -= 2 * pi;
  //   }
  //   while (angle < -pi) {
  //       angle += 2 * pi;
  //   }
  //   return angle;
  //   }    

  std_msgs::Int32MultiArray TrajPlannerServer::eigenToMultiArray(const Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic>& matrix) 
  {
      std_msgs::Int32MultiArray array_msg;

      // Set dimensions (rows and columns)
      array_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
      array_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
      array_msg.layout.dim[0].label = "rows";
      array_msg.layout.dim[1].label = "cols";
      array_msg.layout.dim[0].size = matrix.rows();
      array_msg.layout.dim[1].size = matrix.cols();
      array_msg.layout.dim[0].stride = matrix.rows() * matrix.cols();
      array_msg.layout.dim[1].stride = matrix.cols();

      // Flatten the matrix into a 1D array
      array_msg.data.resize(matrix.size());
      for (int i = 0; i < matrix.rows(); ++i) {
          for (int j = 0; j < matrix.cols(); ++j) {
              array_msg.data[i * matrix.cols() + j] = matrix(i, j);
          }
      }

      return array_msg;
  }

    double TrajPlannerServer::wrapToPi(double angleinradian)
    {
        angleinradian = fmod(angleinradian + M_PI, 2*M_PI);
        if (angleinradian < 0)
            angleinradian += 2*M_PI;
        return angleinradian - M_PI;
    }

  // minco display
  void TrajPlannerServer::Display(){  
    // std::cout  <<  "\033[34m[TrajPlannerServer]TrajPlannerServer::Display. \033[0m" << std::endl; 
 
    p_traj_vis_->displayPolyH(p_planner_->display_hPolys());
    p_traj_vis_->displayObs(p_planner_->display_vec_obs());
    p_traj_vis_->displayKinoPath(p_planner_->display_kino_path());
    p_traj_vis_->displayInnerList(p_planner_->display_InnterPs(), 0);
    p_traj_vis_->displayFitSurTrajs(p_planner_->display_surround_trajs());



    
  }

  ErrorType TrajPlannerServer::FilterSingularityState(
      const vec_E<common::State>& hist, common::State* filter_state) {
    if (hist.empty()) {
      return kWrongStatus;
    }
    double duration = filter_state->time_stamp - hist.back().time_stamp;
    double max_steer = M_PI / 4.0;
    double singular_velocity = kBigEPS;
    double max_orientation_rate = tan(max_steer) / 2.85 * singular_velocity;
    double max_orientation_change = max_orientation_rate * duration;

    if (fabs(filter_state->velocity) < singular_velocity && 
        fabs(normalize_angle(filter_state->angle - hist.back().angle)) >
            max_orientation_change) {
      // printf(
      //     "[TrajPlannerServer]Detect singularity velocity %lf angle (%lf, %lf).\n",
      //     filter_state->velocity, hist.back().angle, filter_state->angle);
      filter_state->angle = hist.back().angle;
      // printf("[TrajPlannerServer]Filter angle to %lf.\n", hist.back().angle);
    }
    return kSuccess;
  }

  //check the new plan after generation
  bool TrajPlannerServer::CheckReplanTraj(std::unique_ptr<SingulTrajData>& executing_traj, int exe_traj_index, int final_traj_index){
      //return 1: replan 0: not
      // std::cout<<"checking replan!! "<<std::endl;
      if(executing_traj==nullptr) return true;
      bool is_near = false;
      bool is_collision = false;
      bool is_close_turnPoint = false;
      double cur_time = ros::Time::now().toSec();
      Eigen::Vector2d localTarget;
      localTarget = executing_traj->back().traj.getPos(executing_traj->back().duration);
      double totaltrajTime = 0.0;
      for(int i = 0; i<executing_traj->size(); i++){
          totaltrajTime += executing_traj->at(i).duration;
      }
      //is close to turnPoint?
      if(exe_traj_index == final_traj_index_) is_close_turnPoint = false;
      else{
        if((executing_traj->at(exe_traj_index).end_time - cur_time)<2.5)
          is_close_turnPoint = true;
      }
      //is near?
      if((executing_traj->back().end_time - cur_time)<2*totaltrajTime / 3.0) is_near = true;
      else is_near = false;
      if(is_near && !is_close_turnPoint&&(localTarget-end_pt_.head(2)).norm()>0.1){
        return true;
      }
      //collision-check    
      p_planner_->updateSurrTraj();
      plan_utils::SurroundTrajData dymicObs;
      p_planner_->getSurrTraj(dymicObs);  
      double tt = 0.0;
      double t0 = ros::Time::now().toSec() -executing_traj->at(exe_traj_index).start_time;
      double ti = ros::Time::now().toSec();
      double tadd = 0.0;
      int i = exe_traj_index;
      bool got_initpos = false;
      Eigen::Vector2d initpos;
      while(i < executing_traj->size() && tadd <3.0){

          common::State fullstate;
          executing_traj->at(i).traj.GetState(t0, &fullstate);

          Eigen::Vector2d pos;
          Eigen::Vector3d state;
          double yaw;
          if (!got_initpos) 
          {
            initpos = fullstate.vec_position;
            got_initpos = true;
          }
          pos = fullstate.vec_position;
          yaw = fullstate.angle;
          state << pos[0],pos[1],yaw;
          map_adapter_.CheckIfCollisionUsingPosAndYaw(vp_,state,&is_collision);
          if(is_collision)  return true;   

          // std::cout<<"dymicObs.size is "<<dymicObs.size()<<std::endl;

          // dynamic obstacles recheck
          for (int sur_id = 0; sur_id < dymicObs.size(); sur_id++)
          {
            if (ti > dymicObs[sur_id].start_time && 
                ti - dymicObs[sur_id].start_time < dymicObs[sur_id].duration)
            {
              // std::cout<<"ttt is "<<ttt<<std::endl;

              // std::cout<<"dymicObs[sur_id].duration is "<<dymicObs[sur_id].duration<<std::endl;
              Eigen::Vector2d surround_p = dymicObs[sur_id].traj.getPos(ti - dymicObs[sur_id].start_time);
              // std::cout<<"surround_p is "<<surround_p<<std::endl;
              // std::cout<<"pos is "<<pos<<std::endl;

              if ((surround_p - pos).norm()<0.50)
              { 
                Eigen::Vector2d init_p = dymicObs[sur_id].traj.getPos(0.0);
                Eigen::Vector2d init_dp = init_p - initpos;
                double yawtoagent = atan(init_dp(1)/ init_dp(0));
                if (yawtoagent <-1.57 || yawtoagent > 1.57) continue; //agent come from the back, dun care
                map_adapter_.CheckCollisionUsingGlobalPosition(init_p, &is_collision);
                if (!is_collision) return true; //filter away noises from static obstacles
              } 
            }    
          }
      
          t0 += 0.05;
          ti += 0.05;
          tadd += 0.05;
          if (t0 > executing_traj->at(i).duration)
          {
            t0 -= executing_traj->at(i).duration;
            i++;
          }
      }
      // tracking error recheck
      common::State desired_state;
      if(map_adapter_.GetEgoState(&desired_state)==kSuccess)
      {
        Eigen::Vector2d pos = desired_state.vec_position;
        double yaw = desired_state.angle;
        common::State state;
        double t = ros::Time::now().toSec();
        executing_traj->at(exe_traj_index).traj.GetState(t-executing_traj->at(exe_traj_index).start_time, &state);
        if ((ros::Time::now().toSec()-last_replan)>1.0 &&
          ((pos - state.vec_position).norm()>0.30 )) //|| fabs(yaw - state.angle)>0.15
          return true;

      }
      return false;
      // map_adapter_.CheckIfCollisionUsingPosAndYaw   
  }


  bool TrajPlannerServer::CheckReplan(int& new_goal){
      //return 1: replan 0: not
      // std::cout<<"checking replan!! "<<std::endl;
      if(executing_traj_==nullptr) return true;
      if(weight_changed)
      {
        weight_changed = false;
        return true;
      }
      bool is_near = false;
      bool is_collision = false;
      bool is_close_turnPoint = false;
      double cur_time = ros::Time::now().toSec();
      Eigen::Vector2d localTarget;
      localTarget = executing_traj_->back().traj.getPos(executing_traj_->back().duration);
      double totaltrajTime = 0.0;
      for(int i = 0; i<executing_traj_->size(); i++){
          totaltrajTime += executing_traj_->at(i).duration;
      }
      //is close to turnPoint?
      if(exe_traj_index_ == final_traj_index_) is_close_turnPoint = false;
      else{
        if((executing_traj_->at(exe_traj_index_).end_time - cur_time)<2.5)
          is_close_turnPoint = true;
      }
      //is near?
      if((executing_traj_->back().end_time - cur_time)<2*totaltrajTime / 3.0) is_near = true;
      else is_near = false;
      if(is_near && !is_close_turnPoint&&(localTarget-end_pt_.head(2)).norm()>0.1){
        new_goal = 1;
        return true;
      }
      //collision-check    
      p_planner_->updateSurrTraj();
      p_planner_->UpdateObsGrids();
      plan_utils::SurroundTrajData dymicObs;
      p_planner_->getSurrTraj(dymicObs);  
      double tt = 0.0;
      double t0 = ros::Time::now().toSec() -executing_traj_->at(exe_traj_index_).start_time;
      double ti = ros::Time::now().toSec();
      double tadd = 0.0;
      int i = exe_traj_index_;
      bool got_initpos = false;
      Eigen::Vector2d initpos;
      traj_upcoming_.clear();
      while(i < executing_traj_->size() && tadd <5.0){

          common::State fullstate;
          executing_traj_->at(i).traj.GetState(t0, &fullstate);

          Eigen::Vector2d pos;
          Eigen::Vector3d state;
          double yaw;
          if (!got_initpos) 
          {
            initpos = fullstate.vec_position;
            got_initpos = true;
          }
          pos = fullstate.vec_position;
          yaw = fullstate.angle;
          state << pos[0],pos[1],yaw;
          traj_upcoming_.push_back(state);
          map_adapter_.CheckIfCollisionUsingPosAndYaw(vp_,state,&is_collision);
          if(is_collision)  return true;   

          // std::cout<<"dymicObs.size is "<<dymicObs.size()<<std::endl;

          // dynamic obstacles recheck
          for (int sur_id = 0; sur_id < dymicObs.size(); sur_id++)
          {
            if (ti > dymicObs[sur_id].start_time && 
                ti - dymicObs[sur_id].start_time < dymicObs[sur_id].duration)
            {
              // std::cout<<"ttt is "<<ttt<<std::endl;

              // std::cout<<"dymicObs[sur_id].duration is "<<dymicObs[sur_id].duration<<std::endl;
              Eigen::Vector2d surround_p = dymicObs[sur_id].traj.getPos(ti - dymicObs[sur_id].start_time);
              // std::cout<<"surround_p is "<<surround_p<<std::endl;
              // std::cout<<"pos is "<<pos<<std::endl;

              if ((surround_p - pos).norm()<0.50)
              { 
                Eigen::Vector2d init_p = dymicObs[sur_id].traj.getPos(0.0);
                Eigen::Vector2d init_dp = init_p - initpos;
                double yawtoagent = atan(init_dp(1)/ init_dp(0));
                if (yawtoagent <-1.57 || yawtoagent > 1.57) continue; //agent come from the back, dun care
                map_adapter_.CheckCollisionUsingGlobalPosition(init_p, &is_collision);
                if (!is_collision) return true; //filter away noises from static obstacles
              } 
            }    
          }
      
          t0 += 0.05;
          ti += 0.05;
          tadd += 0.05;
          if (t0 > executing_traj_->at(i).duration)
          {
            t0 -= executing_traj_->at(i).duration;
            i++;
          }
      }
      // tracking error recheck
      common::State desired_state;
      if(map_adapter_.GetEgoState(&desired_state)==kSuccess)
      {
        Eigen::Vector2d pos = desired_state.vec_position;
        double yaw = desired_state.angle;
        common::State state;
        double t = ros::Time::now().toSec();
        executing_traj_->at(exe_traj_index_).traj.GetState(t-executing_traj_->at(exe_traj_index_).start_time, &state);
        tracking_error_ = (pos - state.vec_position).norm();
        if ((ros::Time::now().toSec()-last_replan)>1.0 &&
          (tracking_error_ >0.30 )) //|| fabs(yaw - state.angle)>0.15
          return true;

      }
      return false;
      // map_adapter_.CheckIfCollisionUsingPosAndYaw   
  }



  ErrorType TrajPlannerServer::Replan() {
    if (!is_replan_on_) return kWrongStatus;
    time_profile_tool_.tic();
    // define the timestamp before call get state
    common::State desired_state;
    if(map_adapter_.GetEgoState(&desired_state)!=kSuccess){
      return kWrongStatus;
    }
    desired_state.time_stamp = ros::Time::now().toSec()+Budget;
    if(executing_traj_ ==nullptr){
      p_planner_->set_initial_state(desired_state);
      
      update_weights();
      planning_success_ = p_planner_->RunOnceParking(wei_obs_, wei_surround_, wei_feas_, wei_sqrvar_, wei_time_, wei_jerk_);
      if (planning_success_ != kSuccess) {
        if (planning_success_ == kReached) planning_success_ = kSuccess;
        Display();
        return kWrongStatus;
      }
        //wait 
      last_replan = ros::Time::now().toSec();
      double curt = ros::Time::now().toSec();
      if(0){
      // if(curt>desired_state.time_stamp){
        ROS_WARN("exceed time budget");
        return kWrongStatus;
      }
      else{
        while(true){
        curt = ros::Time::now().toSec();
        if(curt>=desired_state.time_stamp){
          break;
        } 
       }
      }
      desired_state_hist_.clear();
      desired_state_hist_.push_back(last_smm_.ego_vehicle().state());
      ctrl_state_hist_.clear();
      ctrl_state_hist_.push_back(last_smm_.ego_vehicle().state());
      //record the hist state and control 
    }
    else if(executing_traj_->at(exe_traj_index_).duration >= 0.0){

      //locate the piece idx
      int pidx = exe_traj_index_;
      while(true){
        if(desired_state.time_stamp<=executing_traj_->at(pidx).start_time+executing_traj_->at(pidx).duration){
          break;
        }
        else{
          pidx++;
          if(pidx>=executing_traj_->size()){
            pidx--;
            break;
          }
        }
      }

      common::State _state = desired_state;
      double t = desired_state.time_stamp - executing_traj_->at(pidx).start_time; 
      executing_traj_->at(pidx).traj.GetState(t, &desired_state);
      FilterSingularityState(desired_state_hist_, &desired_state);
      desired_state_hist_.push_back(desired_state);
      if (desired_state_hist_.size() > 100)
        desired_state_hist_.erase(desired_state_hist_.begin());

      Eigen::Vector2d pos = desired_state.vec_position;
      desired_state.angle = _state.angle;
      if (((desired_state.vec_position - _state.vec_position).norm()>0.30 )) //desired too far from current
      {
        // Eigen::Matrix2d init_R;
        // init_R << cos(-_state.angle),  -sin(-_state.angle),
        //           sin(-_state.angle),   cos(-_state.angle);        
        // _state.vec_position =  _state.vec_position + init_R*Eigen::Vector2d(_state.velocity, 0.0)*0.2;
        p_planner_->set_initial_state(desired_state);
        // printf("\033[32m---------------------------------------------------------------\n\033[0m");
        // printf("\033[32m---------------------------------------------------------------\n\033[0m");
        // printf("\033[32m---------------------------------------------------------------\n\033[0m");
        // printf("\033[32m---------------------------------------------------------------\n\033[0m");
        // printf("\033[32m---------------------------------------------------------------\n\033[0m");
        // printf("\033[32m---------------------------------------------------------------\n\033[0m");
        // printf("\033[32m---------------------------------------------------------------\n\033[0m");
        // printf("\033[32m---------------------------------------------------------------\n\033[0m");
        // printf("\033[32m---------------------------------------------------------------\n\033[0m");
        // printf("\033[32m---------------------------------------------------------------\n\033[0m");
        // printf("\033[32m---------------------------------------------------------------\n\033[0m");
      }
      else
      {
        p_planner_->set_initial_state(desired_state); 
      }
      update_weights();
      planning_success_ = p_planner_->RunOnceParking(wei_obs_, wei_surround_, wei_feas_, wei_sqrvar_, wei_time_, wei_jerk_);

      if (planning_success_ != kSuccess) {
        if (planning_success_ == kReached) planning_success_ = kSuccess;
        Display();      
        return kWrongStatus;
      }
      printf("\033[32m[TrajPlannerServer]Traj planner succeed in %lf ms.\n\033[0m",
      time_profile_tool_.toc());

      //wait 
      last_replan = ros::Time::now().toSec();
      double curt = ros::Time::now().toSec();
      if(curt>desired_state.time_stamp){
        ROS_WARN("exceed time budget");
        return kWrongStatus;
      }
      else{
        while(true){
        curt = ros::Time::now().toSec();
        if(curt>=desired_state.time_stamp){
          break;
        } 
       }
      }

    }
    else{
      return kWrongStatus;
    }


    next_traj_ = std::move(p_planner_->trajectory());
    if(next_traj_->at(0).duration <= 1e-5){
      next_traj_.release();
      return kWrongStatus;
    }
    return kSuccess;
  }


  void TrajPlannerServer::ParkingCallback(const geometry_msgs::PoseStamped &msg)
  {
    std::cout << "Triggered parking mode!" << std::endl;
    end_pt_ <<  msg.pose.position.x, msg.pose.position.y,
                tf::getYaw(msg.pose.orientation), 1.0e-2;
    std::cout<<"end_pt: "<<end_pt_.transpose()<<std::endl;
  // end_pt_ << -20.2606 ,-7.24047 ,-3.13672 ,    0.01;
   

    have_parking_target_ = true;
    p_planner_->setParkingEnd(end_pt_);
   
    common::State state;
    if(map_adapter_.GetEgoState(&state)==kSuccess)
    {
      Eigen::Matrix<double, 2, 2> rot;
      double theta = -state.angle;
      rot << cos(theta), -sin(theta),
              sin(theta), cos(theta);
      Vecf<2> pos_error = rot * (end_pt_.head(2) - state.vec_position);  
      double angle_to_target = atan2(pos_error(1), pos_error(0));        
      double angle_difference = wrapToPi(tf::getYaw(msg.pose.orientation) - state.angle);
      if (fabs(angle_to_target)>0.6*M_PI && fabs(angle_difference)>0.6*M_PI)
        last_activate_rotation_time_ = ros::Time::now();

    }
  }

void TrajPlannerServer::resetCallback(const std_msgs::Int32 &msg)
{
  p_planner_->resetPlanning();
  m.lock();
  executing_traj_.release();
  m.unlock();
}


void TrajPlannerServer::peopleAngleCallback(const std_msgs::Float32MultiArray::ConstPtr& angle_msg)
{
  angle_list_.clear();
  for (int i=0; i<angle_msg->data.size()/2; i++)
  {
    Eigen::Vector2d peopleangle(-angle_msg->data[2*i], -angle_msg->data[2*i+1]);
    if (angle_msg->data[2*i] < angle_msg->data[2*i+1])
      peopleangle << -angle_msg->data[2*i+1], -angle_msg->data[2*i];

    angle_list_.push_back(peopleangle);
  }
  last_people_angle_time_ = ros::Time::now();
}

void TrajPlannerServer::odom_cb(const nav_msgs::Odometry::ConstPtr& msg)
{
  Eigen::Vector2d velll(msg->twist.twist.linear.x, msg->twist.twist.linear.y);
  vell_ = velll.norm();
}

void TrajPlannerServer::ScanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg)
{
    int i;
    double scan_min=100, scan_min2 = 100;
    double lsr_len=scan_msg->ranges.size();
    double lsr_angle_inc=scan_msg->angle_increment;
    double lsr_angle_min=scan_msg->angle_min;
    bool collision = false;
    for (i=0;i<(lsr_len-1);i++){

      double angle=((double)(i)*lsr_angle_inc)+lsr_angle_min;

      if (fabs(angle)<0.30)
      {
       if (scan_msg->ranges[i]<scan_min)
          scan_min=scan_msg->ranges[i];        
      }

      if (fabs(angle)<0.95)
      {
       if (scan_msg->ranges[i]<scan_min2)
          scan_min2=scan_msg->ranges[i];        
      }

      double x = scan_msg->ranges[i] * cos(angle);
      double y = scan_msg->ranges[i] * sin(angle);

      double half_width = 0.6 / 2.0;
      double half_length = 0.85 / 2.0;

      // Check if the point (x, y) lies within the UGV's bounding box
      if (x >= -half_length && x <= half_length && y >= -half_width && y <= half_width) {
          collision = true;
      }
    }
    in_collision_ = collision;

    scan_min_ = scan_min;
    scan_min2_ = scan_min2;
}

  void TrajPlannerServer::update_weights()
  {
    wei_obs_ = wei_obs_NN;
    wei_surround_ = wei_surround_NN;
    wei_feas_ = wei_feas_NN; 
    wei_sqrvar_ = wei_sqrvar_NN;
    wei_time_ = wei_time_NN;
    wei_jerk_ = wei_jerk_NN;
  }

  void TrajPlannerServer::publish_weights()
  {
    traj_planner::Weights weights_msg;
    weights_msg.header.stamp = ros::Time::now();
    weights_msg.wei_obs = wei_obs_;
    weights_msg.wei_surround = wei_surround_;
    weights_msg.wei_feas = wei_feas_;
    weights_msg.wei_sqrvar = wei_sqrvar_;
    weights_msg.wei_time = wei_time_;
    weights_msg.wei_jerk = wei_jerk_;
    weights_msg.planning_success = planning_success_;
    weights_msg.tracking_error = tracking_error_;
    weights_msg.collision = in_collision_;
    weights_msg.reached = p_planner_->getReachedTarget();

    weights_pub.publish(weights_msg);

  }

void TrajPlannerServer::weightsCallback(const traj_planner::Weights::ConstPtr& msg) {
    wei_obs_NN = msg->wei_obs;
    wei_surround_NN = msg->wei_surround;
    wei_feas_NN = msg->wei_feas;
    wei_sqrvar_NN = msg->wei_sqrvar;
    wei_time_NN = msg->wei_time;
    wei_jerk_NN = msg->wei_jerk;

    weight_changed = true;
    // For debugging, print the updated values
    ROS_INFO("Setting weights: wei_obs_ = %f, wei_surround_ = %f, wei_feas_ = %f, wei_sqrvar_ = %f, wei_time_ = %f",
             wei_obs_NN, wei_surround_NN, wei_feas_NN, wei_sqrvar_NN, wei_time_NN);
}

}  // namespace planning