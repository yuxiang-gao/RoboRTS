/*
 * The purpose of this node is to subscribe to the /lidar_objects topic and publish poses in
 * a MarkerArray
 */

#include "ros/ros.h"
#include "visualization_msgs/MarkerArray.h"
#include "visualization_msgs/Marker.h"
#include "novel_msgs/NovelObjectArray.h"
#include "geometry_msgs/Vector3.h"
#include "std_msgs/ColorRGBA.h"


class DisplayLidarObjects
{
private:
    ros::NodeHandle &nh_;

    // Messages
    
    
    // Publishers
    ros::Publisher LOmarkers_pub;

    // Subscribers
    ros::Subscriber LOsub;

    // Other
    bool literallydumb;

public:
    DisplayLidarObjects(ros::NodeHandle& noh, bool filler) :
        nh_(noh),
        literallydumb(filler)

    {
        //nh_ = noh;

        // Publishers
        LOmarkers_pub = nh_.advertise<visualization_msgs::MarkerArray>("/lidar_objects_markers", 50);

        // Subscribers
        LOsub = nh_.subscribe("/lidar_objects", 50, &DisplayLidarObjects::loCb, this);


    }
    

    void loCb(const novel_msgs::NovelObjectArray::ConstPtr& msg)
    {
        visualization_msgs::MarkerArray to_publish;
        //to_publish.header = msg->header;

        for (int i = 0; i < msg->detected_objects.size(); ++i) 
        {
            visualization_msgs::Marker to_push;
            geometry_msgs::Vector3 scale;
            scale.x = 0.1;
            scale.y = 0.1;
            scale.z = 0.1;
            std_msgs::ColorRGBA color;
            color.r = 1.0;
            color.g = 0.0;
            color.b = 0.0;
            color.a = 1.0;
            to_push.scale = scale;
            to_push.color = color;
            to_push.header = msg->header;
            to_push.header.frame_id = "/base_laser_link";
            to_push.pose = msg->detected_objects[i].pose;
            to_publish.markers.push_back(to_push);
        }

        LOmarkers_pub.publish(to_publish);

    }


};


int main (int argc, char** argv)
{
        ros::init(argc, argv, "display_lidar_objects");

        ros::NodeHandle nh;
        //tf::TransformListener listener;

        //Filter f = NOFilter(nh, listener);
        DisplayLidarObjects d = DisplayLidarObjects(nh, false);

        ros::spin();
}


