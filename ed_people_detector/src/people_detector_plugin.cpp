#include "ed_people_detector/people_detector_plugin.h"

#include <ed/entity.h>
#include <tue/config/reader.h>

#include <ros/node_handle.h>
#include <ros/advertise_service_options.h>

#include <people_detection_3d_msgs/DetectPeople3D.h>


// ----------------------------------------------------------------------------------------------------

void PeopleDectectorPlugin::configure(tue::Configuration config)
{
    std::cout << "Configure" << std::endl;
    ros::NodeHandle nh("~/people_detector");
    ros::NodeHandle nh2("~");

    ros::AdvertiseServiceOptions opt_srv_ed_people_detector =
            ros::AdvertiseServiceOptions::create<ed_people_detector_msgs::EdDetectPeople>(
                "detect_people", boost::bind(&PeopleDectectorPlugin::srvEdDetectPeople, this, _1, _2),
                ros::VoidPtr(), &cb_queue_);

    srv_ed_people_detector_ = nh.advertiseService(opt_srv_ed_people_detector);

    std::string people_detector_3d_srv_name;
    config.value("people_detector_3d_service", people_detector_3d_srv_name);
    srv_people_detector_3d_client_ = nh2.serviceClient<people_detection_3d_msgs::DetectPeople3D>(people_detector_3d_srv_name);
}

// ----------------------------------------------------------------------------------------------------

void PeopleDectectorPlugin::initialize()
{
    std::cout << "initialize" << std::endl;
}

// ----------------------------------------------------------------------------------------------------

void PeopleDectectorPlugin::process(const ed::WorldModel& world, ed::UpdateRequest& req)
{
    std::cout << "process" << std::endl;
    // Check for services
    world_ = &world;
    cb_queue_.callAvailable();
}

// ----------------------------------------------------------------------------------------------------

bool PeopleDectectorPlugin::srvEdDetectPeople(const ed_people_detector_msgs::EdDetectPeople::Request& req, ed_people_detector_msgs::EdDetectPeople::Response& res)
{
    std::cout << "srvEdDetectPeople" << std::endl;
    people_detection_3d_msgs::DetectPeople3DRequest req_3d;
    req_3d.image_rgb = req.image_rgb;
    req_3d.image_depth = req.image_depth;
    req_3d.camera_info_depth = req.camera_info_depth;

    people_detection_3d_msgs::DetectPeople3DResponse res_3d;
    srv_people_detector_3d_client_.call(req_3d, res_3d);

}

// ----------------------------------------------------------------------------------------------------


ED_REGISTER_PLUGIN(PeopleDectectorPlugin)
