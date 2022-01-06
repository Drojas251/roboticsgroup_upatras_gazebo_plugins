/**
Copyright (c) 2014, Konstantinos Chatzilygeroudis
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer
    in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived
    from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING,
BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
**/

//To DO:
/*
Add PID using control toolbox. The constructor of control_toolbox changed from the version used in ros, so update that
Changed the "cout" to ROS logging
*/

#include <roboticsgroup_upatras_gazebo_plugins/mimic_joint_plugin.h>

// GAZEBO_MAJOR_VERSION >= 8
namespace math = ignition::math;


namespace gazebo {

    MimicJointPlugin::MimicJointPlugin()
    {
        joint_.reset();
        mimic_joint_.reset();
    }

    MimicJointPlugin::~MimicJointPlugin()
    {
        update_connection_.reset();
    }

    void MimicJointPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
        model_ = _parent;
        world_ = model_->GetWorld();

        if (!model_) {
            std::cout << " Parent model is NULL! MimicJointPlugin could not be loaded" << std::endl;
            return;
        }
        
        // Check for robot namespace
        if (_sdf->HasElement("robotNamespace")) {
            robot_namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
        }

        auto model_nh = std::make_shared<rclcpp::Node>(robot_namespace_);


        // Check for joint element
        if (!_sdf->HasElement("joint")) {
            std::cout << " NO JOINTS PRESENT. MIMICJOINT PLUGIN NOT LOADED" << std::endl;
            return;
        }

        this-> joint_name_ = _sdf->GetElement("joint")->Get<std::string>();
        std::cout<<"Joint NAME:= "<< this-> joint_name_ <<std::endl;

        // Check for mimicJoint element
        if (!_sdf->HasElement("mimicJoint")) {
            std::cout << " NO MIMIC JOINTS PRESENT. MIMICJOINT PLUGIN NOT LOADED" << std::endl;
            return;
        }

        mimic_joint_name_ = _sdf->GetElement("mimicJoint")->Get<std::string>();

        // Check if PID controller wanted
        has_pid_ = _sdf->HasElement("hasPID");
        /*
        if (has_pid_) {
            std::string name = _sdf->GetElement("hasPID")->Get<std::string>();
            if (name.empty()) {
                name = "gazebo_ros_control/pid_gains/" + mimic_joint_name_;
            }
            const ros::NodeHandle nh(model_nh, name);
            pid_.init(nh);
        }*/

        // Check for multiplier element
        multiplier_ = 1.0;
        if (_sdf->HasElement("multiplier"))
            multiplier_ = _sdf->GetElement("multiplier")->Get<double>();

        // Check for offset element
        offset_ = 0.0;
        if (_sdf->HasElement("offset"))
            offset_ = _sdf->GetElement("offset")->Get<double>();

        // Check for sensitiveness element
        sensitiveness_ = 0.0;
        if (_sdf->HasElement("sensitiveness"))
            sensitiveness_ = _sdf->GetElement("sensitiveness")->Get<double>();

        // Get pointers to joints
        joint_ = model_->GetJoint(joint_name_);
        if (!joint_) {
            //RCLCPP_ERROR_STREAM("No joint named \"" << joint_name_ << "\". MimicJointPlugin could not be loaded.");
            std::cout << " NO JOINTS WITH THAT NAME. MIMICJOINT PLUGIN NOT LOADED" << std::endl;
            return;
        }
        mimic_joint_ = model_->GetJoint(mimic_joint_name_);
        if (!mimic_joint_) {
            //RCLCPP_ERROR_STREAM("No (mimic) joint named \"" << mimic_joint_name_ << "\". MimicJointPlugin could not be loaded.");
            std::cout << " NO MIMIC JOINTS WITH THAT NAME. MIMICJOINT PLUGIN NOT LOADED" << std::endl;
            return;
        }


        // Check for max effort
        max_effort_ = mimic_joint_->GetEffortLimit(0);
        std::cout<<"Max effort:= "<< max_effort_ <<std::endl;

        if (_sdf->HasElement("maxEffort")) {
            max_effort_ = _sdf->GetElement("maxEffort")->Get<double>();
        }

        // Set max effort
        if (!has_pid_) {
            mimic_joint_->SetParam("fmax", 0, max_effort_);
            std::cout<<" SET MAX Effort Param" <<std::endl;

        } 
        
        // Listen to the update event. This event is broadcast every
        // simulation iteration.
        update_connection_ = event::Events::ConnectWorldUpdateBegin(
            boost::bind(&MimicJointPlugin::UpdateChild, this));

    }

    void MimicJointPlugin::UpdateChild()
    {
        static rclcpp::Duration period(world_->Physics()->GetMaxStepSize());

        // Set mimic joint's angle based on joint's angle
        double angle = joint_->Position(0) * multiplier_ + offset_;
        double a = mimic_joint_->Position(0);


        if (fabs(angle - a) >= sensitiveness_) {
            /*if (has_pid_) {
                if (a != a)
                    a = angle;
                double error = angle - a;
                double effort = math::clamp(pid_.computeCommand(error, period), -max_effort_, max_effort_);
                mimic_joint_->SetForce(0, effort);
            }*/
            
            //else {
                mimic_joint_->SetPosition(0, angle, true);

            //} 
        }
    }

    GZ_REGISTER_MODEL_PLUGIN(MimicJointPlugin)

}  // namespace gazebo
