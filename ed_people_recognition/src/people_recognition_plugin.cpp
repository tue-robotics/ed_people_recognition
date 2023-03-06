#include "ed_people_recognition/people_recognition_plugin.h"

#include <ed/entity.h>
#include <ed/update_request.h>
#include <ed/world_model.h>
#include <tue/config/reader.h>

#include <ros/node_handle.h>
#include <ros/advertise_service_options.h>

#include <people_recognition_msgs/RecognizePeople3D.h>

#include <geolib/ros/msg_conversions.h>
#include <geolib/math_types.h>


void VectorOfStringToStringOfVector(const std::vector<std::string>& vector, std::string& string)
{
    std::stringstream ss;
    ss << "[";
    bool stream_first = true;
    for (auto it = vector.cbegin(); it != vector.cend(); ++it)
    {
        if (!stream_first)
            ss << ", ";
        ss << *it;
        stream_first = false;
    }
    ss << "]";
    string = ss.str();
}

// ----------------------------------------------------------------------------------------------------

PeopleRecognitionPlugin::PeopleRecognitionPlugin()
{
}

// ----------------------------------------------------------------------------------------------------

PeopleRecognitionPlugin::~PeopleRecognitionPlugin()
{
}

// ----------------------------------------------------------------------------------------------------

void PeopleRecognitionPlugin::configure(tue::Configuration config)
{
    ros::NodeHandle nh("~/people_recognition");
    ros::NodeHandle nh2("~");

    ros::AdvertiseServiceOptions opt_srv_ed_people_recognition =
            ros::AdvertiseServiceOptions::create<ed_people_recognition_msgs::EdRecognizePeople>(
                "detect_people", boost::bind(&PeopleRecognitionPlugin::srvEdRecognizePeople, this, _1, _2),
                ros::VoidPtr(), &cb_queue_);

    srv_ed_people_recognition_ = nh.advertiseService(opt_srv_ed_people_recognition);

    std::string people_recognition_3d_srv_name;
    config.value("people_recognition_3d_service", people_recognition_3d_srv_name);
    srv_people_recognition_3d_client_ = nh2.serviceClient<people_recognition_msgs::RecognizePeople3D>(people_recognition_3d_srv_name);

    if(!srv_people_recognition_3d_client_.exists())
    {
        ROS_WARN("[ED People Recognition]: %s does not (yet) exist", people_recognition_3d_srv_name.c_str());
    }
}

// ----------------------------------------------------------------------------------------------------

void PeopleRecognitionPlugin::initialize()
{
}

// ----------------------------------------------------------------------------------------------------

void PeopleRecognitionPlugin::process(const ed::WorldModel& world, ed::UpdateRequest& req)
{
    // Check for services
    world_ = &world;
    update_req_ = &req;

    cb_queue_.callAvailable();
}

// ----------------------------------------------------------------------------------------------------

bool PeopleRecognitionPlugin::srvEdRecognizePeople(const ed_people_recognition_msgs::EdRecognizePeople::Request& req, ed_people_recognition_msgs::EdRecognizePeople::Response& res)
{
    ROS_DEBUG_STREAM("[ED People Recognition]: srvEdRecognizePeople");
    people_recognition_msgs::RecognizePeople3DRequest req_3d;
    req_3d.image_rgb = req.image_rgb;
    req_3d.image_depth = req.image_depth;
    req_3d.camera_info_depth = req.camera_info_depth;

    people_recognition_msgs::RecognizePeople3DResponse res_3d;
    if(!srv_people_recognition_3d_client_.call(req_3d, res_3d))
    {
        if(!srv_people_recognition_3d_client_.exists())
            ROS_ERROR_STREAM("[ED People Recognition]: people_recognition_3d service, " << srv_people_recognition_3d_client_.getService() << ", doesn't exist");

        ROS_ERROR_STREAM("[ED People Recognition]: srv_people_recognition_3d_client_ call failed");
        res.success = false;
        return false;
    }

    ROS_DEBUG_STREAM("[ED People Recognition]: received recognition of " << res_3d.people.size() << " people");

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
        geo::Pose3D person_pose_tf = geo::Pose3D::identity();
        // Try to transform before doing anything. So there are no people with missing attributes.
        try
        {
            geometry_msgs::TransformStamped transform = tf_buffer_->lookupTransform("map", it->header.frame_id, it->header.stamp);

            geo::convert(transform.transform, person_pose_tf);
        }
        catch(const tf2::TransformException& ex)
        {
            ROS_ERROR_STREAM("[ED People Recognition]: Could not get transform to map" << ex.what());
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

        // Convert position to
        geo::Pose3D person_pose = geo::Pose3D::identity();
        geo::convert(it->position, person_pose.t);
        person_pose.t = person_pose_tf * person_pose.t;

        update_req_->setPose(id_string, person_pose);

//        std_msgs/Header header
//          uint32 seq
//          time stamp
//          string frame_id
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

        data_config.writeGroup("header");
            data_config.setValue("seq", (int) it->header.seq);
            data_config.writeGroup("stamp");
                data_config.setValue("sec", (int) it->header.stamp.sec);
                data_config.setValue("nsec", (int) it->header.stamp.nsec);
            data_config.endGroup();
            data_config.setValue("frame_id", it->header.frame_id);
        data_config.endGroup();

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
    ROS_DEBUG_STREAM("[ED People Recognition]: inserted "<< res.detected_person_ids.size() << " entities");
    return true;
}

// ----------------------------------------------------------------------------------------------------


ED_REGISTER_PLUGIN(PeopleRecognitionPlugin)
