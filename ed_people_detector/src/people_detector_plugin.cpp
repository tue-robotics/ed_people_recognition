#include "ed_people_detector/people_detector_plugin.h"

#include <ed/entity.h>
#include <ed/update_request.h>
#include <ed/world_model.h>
#include <tue/config/reader.h>

#include <ros/node_handle.h>
#include <ros/advertise_service_options.h>

#include <people_detection_3d_msgs/DetectPeople3D.h>

#include <geolib/ros/msg_conversions.h>
#include <geolib/ros/tf_conversions.h>
#include <geolib/math_types.h>


void VectorOfStringToStringOfVector(const std::vector<std::string>& vector, std::string& string)
{
    std::stringstream ss;
    ss << "[";
    bool stream_first = true;
    for (auto it = vector.cbegin(); it != vector.cend(); ++it)
    {
        if (!stream_first)
        {
            ss << ", ";
            stream_first = false;
        }
        ss << *it;
    }
    ss << "]";
    string = ss.str();
}

// ----------------------------------------------------------------------------------------------------

PeopleDectectorPlugin::PeopleDectectorPlugin() : tf_listener_(nullptr)
{
}

// ----------------------------------------------------------------------------------------------------

PeopleDectectorPlugin::~PeopleDectectorPlugin()
{
    delete tf_listener_;
}

// ----------------------------------------------------------------------------------------------------

void PeopleDectectorPlugin::configure(tue::Configuration config)
{
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

    if(!srv_people_detector_3d_client_.exists())
    {
        ROS_WARN("[ED People Detector]: %s does not (yet) exist", people_detector_3d_srv_name.c_str());
    }
}

// ----------------------------------------------------------------------------------------------------

void PeopleDectectorPlugin::initialize()
{
    if (!tf_listener_)
        tf_listener_ = new tf::TransformListener;
}

// ----------------------------------------------------------------------------------------------------

void PeopleDectectorPlugin::process(const ed::WorldModel& world, ed::UpdateRequest& req)
{
    // Check for services
    world_ = &world;
    update_req_ = &req;

    cb_queue_.callAvailable();
}

// ----------------------------------------------------------------------------------------------------

bool PeopleDectectorPlugin::srvEdDetectPeople(const ed_people_detector_msgs::EdDetectPeople::Request& req, ed_people_detector_msgs::EdDetectPeople::Response& res)
{
    ROS_DEBUG_STREAM("[ED People Detector]: srvEdDetectPeople");
    people_detection_3d_msgs::DetectPeople3DRequest req_3d;
    req_3d.image_rgb = req.image_rgb;
    req_3d.image_depth = req.image_depth;
    req_3d.camera_info_depth = req.camera_info_depth;

    people_detection_3d_msgs::DetectPeople3DResponse res_3d;
    if(!srv_people_detector_3d_client_.call(req_3d, res_3d))
    {
        if(!srv_people_detector_3d_client_.exists())
        {
            ROS_ERROR("[ED People Detector]: srv_people_detector_3d_client_ service does not exist");
        }
        ROS_ERROR("[ED People Detector]: srv_people_detector_3d_client_-call failed");
        res.success = false;
        return false;
    }

    ROS_DEBUG("[ED People Detector]: detected %i people", res_3d.people.size());

    for (auto it = world_->begin(); it != world_->end(); ++it)
    {
        const ed::EntityConstPtr& e = *it;
        if (e)
        {
            if (e->hasType("unknown_person"))
                update_req_->removeEntity(e->id());
        }
    }

    for (auto it = res_3d.people.cbegin(); it != res_3d.people.cend(); ++it)
    {
        geo::Pose3D person_pose;
        // Try to transform before doing anything. So there are no people with missing attributes.
        try
        {
            tf::StampedTransform t_person_pose;
            tf_listener_->lookupTransform("map", it->header.frame_id, it->header.stamp, t_person_pose);
            geo::convert(t_person_pose, person_pose);
        }
        catch(const tf::TransformException& ex)
        {
            ROS_ERROR_STREAM("[ED People Detector]: Could not get transform to map" << ex.what());
            continue;
        }

        std::string id_string;
        if (!it->name.empty())
        {
            id_string = it->name;
            update_req_->setType(id_string, id_string);
        }
        else
        {
            id_string = ed::Entity::generateID().str();
            update_req_->setType(id_string, "unknown_person");
        }

        update_req_->setType(id_string, "person");

//        string name
//        uint8 age
//        uint8 gender
//        float64 gender_confidence
//        string posture
//        string emotion
//        string[] shirt_colors
//        image_recognition_msgs/Recognition[] body_parts_pose
//          image_recognition_msgs/CategoricalDistribution categorical_distribution
//            image_recognition_msgs/CategoryProbability[] probabilities
//              string label
//              float32 probability
//            float32 unknown_probability
//          sensor_msgs/RegionOfInterest roi
//            uint32 x_offset
//            uint32 y_offset
//            uint32 height
//            uint32 width
//            bool do_rectify
//          uint32 group_id
//        geometry_msgs/Point position
//          float64 x
//          float64 y
//          float64 z
//        geometry_msgs/Point velocity
//          float64 x
//          float64 y
//          float64 z
//        float64 reliability
//        string[] tagnames
//        string[] tags
//        geometry_msgs/Pose pointing_pose
//          geometry_msgs/Point position
//            float64 x
//            float64 y
//            float64 z
//          geometry_msgs/Quaternion orientation
//            float64 x
//            float64 y
//            float64 z
//            float64 w

        tue::Configuration data_config;
        data_config.setValue("name", it->name);
        data_config.setValue("age", it->age);
        data_config.setValue("gender", it->gender);
        data_config.setValue("gender_confidence", it->gender_confidence);
        data_config.setValue("posture", it->posture);
        data_config.setValue("emotion", it->emotion);

        std::string shirt_colors;
        VectorOfStringToStringOfVector(it->shirt_colors, shirt_colors);
        data_config.setValue("shirt_colors", shirt_colors);

        data_config.writeGroup("position");
        data_config.setValue("x", it->position.x);
        data_config.setValue("y", it->position.y);
        data_config.setValue("z", it->position.z);
        data_config.endGroup();

        data_config.writeGroup("velocity");
        data_config.setValue("x", it->velocity.x);
        data_config.setValue("y", it->velocity.y);
        data_config.setValue("z", it->velocity.z);
        data_config.endGroup();

        data_config.setValue("reliability", it->reliability);

        std::string tagnames;
        VectorOfStringToStringOfVector(it->tagnames, tagnames);
        data_config.setValue("tagnames", tagnames);

        std::string tags;
        VectorOfStringToStringOfVector(it->tags, tags);
        data_config.setValue("tags", tags);

        data_config.writeGroup("pointing_pose");
        data_config.writeGroup("position");
        data_config.setValue("x", it->pointing_pose.position.x);
        data_config.setValue("y", it->pointing_pose.position.y);
        data_config.setValue("z", it->pointing_pose.position.z);
        data_config.endGroup();

        data_config.writeGroup("orientation");
        data_config.setValue("x", it->pointing_pose.orientation.x);
        data_config.setValue("y", it->pointing_pose.orientation.y);
        data_config.setValue("z", it->pointing_pose.orientation.z);
        data_config.setValue("w", it->pointing_pose.orientation.w);
        data_config.endGroup();
        data_config.endGroup();

        update_req_->addData(id_string, data_config.data());

        res.detected_person_ids.push_back(id_string);
    }

    res.success = true;
    ROS_DEBUG("[ED People Detector]: inserted %i entities", res.detected_person_ids.size());
    return true;
}

// ----------------------------------------------------------------------------------------------------


ED_REGISTER_PLUGIN(PeopleDectectorPlugin)
