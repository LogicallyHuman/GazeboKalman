#ifndef _CAR_PLUGIN_HH_
#define _CAR_PLUGIN_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

#include <thread>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Float32MultiArray.h"


namespace gazebo{

    class CarPlugin : public ModelPlugin{

        private: physics::ModelPtr model;
        
        private: physics::JointPtr jointFL;//Front Left  joint pointer
        private: physics::JointPtr jointFR;//Front Right joint pointer
        private: physics::JointPtr jointBL;//Back  Left  joint pointer
        private: physics::JointPtr jointBR;//Back  Right joint pointer
        
        private: common::PID pidFL;
        private: common::PID pidFR;
        private: common::PID pidBL;
        private: common::PID pidBR;        
        
        private: float vFL=0.0, vFR=0.0, vBL=0.0, vBR=0.0;


        private: std::unique_ptr<ros::NodeHandle> rosNode;//A node use for ROS transport
        private: ros::Subscriber rosSub;//brief A ROS subscriber
        private: ros::CallbackQueue rosQueue;//brief A ROS callbackqueue that helps process messages
        private: std::thread rosQueueThread;//brief A thread the keeps running the rosQueue
        
        private: event::ConnectionPtr updateConnection;


        public: CarPlugin() {}

        public: ~CarPlugin() {
            std::cout << "Shutting down Car Plugin" << std::endl;
            rosNode->shutdown();   //NECESARIO
            rosQueueThread.join(); //NECESARIO
        }


        /// \brief The load function is called by Gazebo when the plugin is
        /// inserted into simulation
        /// \param[in] _model A pointer to the model that this plugin is
        /// attached to.
        /// \param[in] _sdf A pointer to the plugin's SDF element.

        public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf){

            std::cout << "Loading car plugin..." << std::endl;
            
            model = _model; // Store the model pointer for convenience.
            

            jointFL = _model->GetJoint("front_left_wheel_joint");  //Get joints
            jointFR = _model->GetJoint("front_right_wheel_joint");
            jointBL = _model->GetJoint("back_left_wheel_joint");
            jointBR = _model->GetJoint("back_right_wheel_joint");
            

            if(jointFL == nullptr || jointFR == nullptr ||jointBL == nullptr || jointBR == nullptr){
                std::cerr << "Couldn't find all joints by name\n";
                return;
            }

           
            // Initialize ros, if it has not already been initialized.
            if (!ros::isInitialized())
            {
                int argc = 0;
                char **argv = NULL;
                ros::init(argc, argv, "gazebo_client", ros::init_options::NoSigintHandler);
            }
            
            // Create our ROS node. This acts in a similar manner to
            // the Gazebo node
            this->rosNode.reset(new ros::NodeHandle("gazebo_client"));
            
            // Create a named topic, and subscribe to it.
            ros::SubscribeOptions so =
            ros::SubscribeOptions::create<std_msgs::Float32MultiArray>(
                "/" + this->model->GetName() + "/motor_control",
                1,
                boost::bind(&CarPlugin::OnRosMsg, this, _1),
                ros::VoidPtr(), &this->rosQueue);
            
            
            this->rosSub = this->rosNode->subscribe(so);
            
            this->rosQueueThread = std::thread(std::bind(&CarPlugin::QueueThread, this));// Spin up the queue helper thread.
            
            auto updateFunc = std::bind(&CarPlugin::OnUpdate, this);
            this->updateConnection = event::Events::ConnectWorldUpdateBegin(updateFunc); //Conecta el callback OnUpdate 
            
            std::cout << "Finished loading car plugin" << std::endl;
            
        }

        ///////////MESSAGE HANDLING///////////////////

        private: void QueueThread(){
            static const double timeout = 0.01;
            while (this->rosNode->ok())
            {
                this->rosQueue.callAvailable(ros::WallDuration(timeout));
            }
        }

        public: void OnRosMsg(const std_msgs::Float32MultiArrayConstPtr &_msg){
            std::cout << "ROS Msg Recieved: Speed=" << _msg->data[0] << "Turn=" << _msg->data[1] << std::endl;
            this->SetVelocities(_msg->data[0], _msg->data[1]);
        }
        
        public: void SetVelocities(const double &speed, const double &turn){

            vFL = speed + turn;
            vFR = speed - turn;
            vBL = speed + turn;
            vBR = speed - turn;

        }

        ///////////UPDATE VELOCITIES EVERY TIME STEP///////////////////


        public: void OnUpdate(){
            jointFL->SetVelocity(0, vFL);
            jointFR->SetVelocity(0, vFR);
            jointBL->SetVelocity(0, vBL);
            jointBR->SetVelocity(0, vBR);
        }


    };
    
    GZ_REGISTER_MODEL_PLUGIN(CarPlugin) // Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.

}
#endif
