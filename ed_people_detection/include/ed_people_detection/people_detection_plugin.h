#ifndef ed_people_detection_plugin_h_
#define ed_people_detection_plugin_h_

#include <ed/plugin.h>
#include <ed/types.h>

#include "ed_people_detection_msgs/EdDetectPeople.h"

#include <ros/callback_queue.h>
#include <ros/service.h>
#include <ros/service_client.h>

#include <tf/transform_listener.h>

class PeopleDectectionPlugin : public ed::Plugin
{

public:

    /**
     * @brief constructor
     */
    PeopleDectectionPlugin();

    /**
     * @brief destructor
     */
    virtual ~PeopleDectectionPlugin();

    /**
     * @brief configure
     * @param config
     * parameters:
     *      people_detection_3d_service: /hero/people_detection/detect_people_3d
     */
    void configure(tue::Configuration config);

    /**
     * @brief initialize
     */
    void initialize();

    /**
     * @brief process
     * @param world
     * @param req
     */
    void process(const ed::WorldModel& world, ed::UpdateRequest& req);

    bool srvEdDetectPeople(const ed_people_detection_msgs::EdDetectPeople::Request& req, ed_people_detection_msgs::EdDetectPeople::Response& res);

    // --------------------

private:

    ros::ServiceServer srv_ed_people_detection_;
    ros::ServiceClient srv_people_detection_3d_client_;

    ros::CallbackQueue cb_queue_;

    const ed::WorldModel* world_;
    ed::UpdateRequest* update_req_;

    tf::TransformListener* tf_listener_;

};

#endif
