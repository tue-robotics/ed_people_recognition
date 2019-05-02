#ifndef ed_people_detector_plugin_h_
#define ed_people_detector_plugin_h_

#include <ed/plugin.h>
#include <ed/world_model.h>

#include <ros/callback_queue.h>
#include <ros/service.h>
#include <ros/service_client.h>

/**
 * @brief The KinectNavigationPlugin class
 * ED plugin for de depth_sensor_integrator. Which uses the depth image to detect objects.
 * Based on the computed normals, points are computed that are probably measured due to an unmodeled object. These are published as a PointCloud.
 */
class PeopleDectectorPlugin : public ed::Plugin
{

public:

    /**
     * @brief constructor
     */
    PeopleDectectorPlugin() {}

    /**
     * @brief destructor
     */
    virtual ~PeopleDectectorPlugin() {}

    /**
     * @brief configure
     * @param config
     * parametergroup: depth_sensor_integration
     * parameters:
     *      frame_id: /map
     *      topic: /amigo/top_kinect/rgbd
     *      num_samples: 640
     *      slope_threshold: 1
     *      slope_window_size: 30
     *      min_distance: 0.4
     *      max_distance: 2.0
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

    // --------------------

private:

    ros::ServiceServer srv_ed_people_detector_;
    ros::ServiceClient srv_people_detector_3d_;

    ros::CallbackQueue cb_queue_;
    const ed::WorldModel* world_;

};

#endif
