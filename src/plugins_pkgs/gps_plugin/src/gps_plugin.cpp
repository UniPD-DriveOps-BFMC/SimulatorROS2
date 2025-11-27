
#include "gps_plugin.hpp"
#include <gz/sim/components/Pose.hh>
#include <cstdlib>
#include <ctime>
#include <gz/sim/components/Name.hh>

#define DEBUG true

namespace gps
{   
    void GPS::Configure(const gz::sim::Entity &_entity,
                       const std::shared_ptr<const sdf::Element> &_sdf,
                       gz::sim::EntityComponentManager &_ecm,
                       gz::sim::EventManager &/*_eventMgr*/)
    {
        this->entity = _entity;
        this->model = gz::sim::Model(_entity);
        
        // Seed random number generator for noise simulation
        std::srand(std::time(nullptr));
        
        // Create publisher for GPS data using Gazebo Transport
        this->gpsPub = this->node.Advertise<gz::msgs::Pose>(this->topicName);
        
        if(DEBUG)
        {
            std::cerr << "\n\n";
            std::cerr << "====================================================================" << std::endl;
            std::cerr << "[gps_plugin] attached to: " << this->model.Name(_ecm) << std::endl;
            std::cerr << "[gps_plugin] publish to: " << this->topicName << std::endl;
            std::cerr << "[gps_plugin] Publishing: position and orientation (with ±0.1 noise)" << std::endl;
            std::cerr << "====================================================================" << std::endl;
            std::cerr << "[gps_plugin] Configured successfully with Gazebo Transport" << std::endl;
        }
    }

    void GPS::PostUpdate(const gz::sim::UpdateInfo &_info,
                        const gz::sim::EntityComponentManager &_ecm)
    {
        if (_info.paused)
            return;
            
        auto elapsed = _info.simTime - last_update_time;
        if (elapsed < update_period)
            return;
            
        last_update_time = _info.simTime;
        
        auto poseComp = _ecm.Component<gz::sim::components::Pose>(this->entity);
        if (!poseComp)
            return;
            
        auto pose = poseComp->Data();
        
        // Add noise to position (simulate GPS sensor error)
        float noise_x = (std::rand() / (float)RAND_MAX) * 0.2f - 0.1f;
        float noise_y = (std::rand() / (float)RAND_MAX) * 0.2f - 0.1f;
        
        // Create GPS message with pose and noise
        gz::msgs::Pose gpsMsg;
        gpsMsg.mutable_position()->set_x(pose.Pos().X() + noise_x);
        gpsMsg.mutable_position()->set_y(std::abs(pose.Pos().Y()) + noise_y);
        gpsMsg.mutable_position()->set_z(pose.Pos().Z());
        
        gpsMsg.mutable_orientation()->set_x(pose.Rot().X());
        gpsMsg.mutable_orientation()->set_y(pose.Rot().Y());
        gpsMsg.mutable_orientation()->set_z(pose.Rot().Z());
        gpsMsg.mutable_orientation()->set_w(pose.Rot().W());
        
        this->gpsPub.Publish(gpsMsg);
    }
}

GZ_ADD_PLUGIN(gps::GPS,
              gz::sim::System,
              gps::GPS::ISystemConfigure,
              gps::GPS::ISystemPostUpdate)
