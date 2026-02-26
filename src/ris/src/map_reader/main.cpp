#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>

void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    const int width = msg->info.width;
    const int height = msg->info.height;
    const float resolution = msg->info.resolution;

    const double origin_x = msg->info.origin.position.x;
    const double origin_y = msg->info.origin.position.y;

    int free_cells = 0;
    int occupied_cells = 0;
    int unknown_cells = 0;
    int semi_cells = 0; // 1..49 (optional bucket)

    for (int i = 0; i < (int)msg->data.size(); i++)
    {
        const int8_t v = msg->data[i];

        if (v == -1) unknown_cells++;
        else if (v == 0) free_cells++;
        else if (v >= 50) occupied_cells++;
        else semi_cells++;
    }

    ROS_INFO_THROTTLE(2.0,
        "MAP frame=%s time=%.3f size=%dx%d res=%.3fm origin=(%.2f,%.2f) free=%d occ=%d semi=%d unk=%d",
        msg->header.frame_id.c_str(),
        msg->header.stamp.toSec(),
        width, height, resolution,
        origin_x, origin_y,
        free_cells, occupied_cells, semi_cells, unknown_cells
    );
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "map_reader");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/map", 1, mapCallback);

    ros::spin();

    return 0;
}
