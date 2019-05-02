#include "ed_people_detector/people_detector_plugin.h"

#include <ed/entity.h>
#include <ed/error_context.h>

#include <tue/config/reader.h>

#include <ros/node_handle.h>

#include <iomanip>

// ----------------------------------------------------------------------------------------------------

void PeopleDectectorPlugin::configure(tue::Configuration config)
{
    std::cout << "Configure" << std::endl;
    ros::NodeHandle nh("~/people_detector");
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


ED_REGISTER_PLUGIN(PeopleDectectorPlugin)
