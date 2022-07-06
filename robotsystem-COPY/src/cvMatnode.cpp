#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>



class cvMat
{
    private:
        ros::NodeHandle n;
        ros::Publisher pubCv;
        ros::Subscriber subMap;
        nav_msgs::OccupancyGrid Mapmsg;

    public:

        cvMat(){
           
            pubCv = n.advertise<nav_msgs::OccupancyGrid>("cvMap", 1);
            // Depth topic subscriber
            subMap = n.subscribe("/map", 10, &cvMat::occupancyGridToCvMat,this);
        }

    //cv::Mat occupancyGridToCvMat(const nav_msgs::OccupancyGrid *map)
    void occupancyGridToCvMat(const nav_msgs::OccupancyGridConstPtr& map)
    {
        uint8_t *data = (uint8_t*) map->data.data(), testpoint = data[0];
        bool mapHasPoints = false;

        cv::Mat im(map->info.height, map->info.width, CV_8UC1);

        // transform the map in the same way the map_saver component does
        for (size_t i=0; i<map->data.size(); i++)
        {
            if (data[i] == 0)        im.data[i] = 254;
            else if (data[i] == 100) im.data[i] = 0;
            else im.data[i] = 205;

            // just check if there is actually something in the map
            if (i!=0) mapHasPoints = mapHasPoints || (data[i] != testpoint);
            testpoint = data[i];
        }

        // sanity check
        if (!mapHasPoints) { ROS_WARN("map is empty, ignoring update."); }

        //return im;

        pubCv.publish(im);
    }
};


int main(int argc, char** argv) {
    // Node initialization
    ros::init(argc, argv, "cvMatnode");
    cvMat clscvMat;

    

    // Node execution
    ros::spin();

    return 0;
}
    

