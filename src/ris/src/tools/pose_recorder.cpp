#include <fstream>
#include <iomanip>
#include <ros/ros.h>
#include <tf/transform_listener.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pose_recorder");
    ros::NodeHandle nh("~");

    std::string target_frame = "map";
    std::string source_frame = "base_footprint";
    std::string csv_path = "/tmp/pose_recorder.csv";
    double rate_hz = 2.0;

    nh.param("target_frame", target_frame, std::string("map"));
    nh.param("source_frame", source_frame, std::string("base_footprint"));
    nh.param("csv_path", csv_path, std::string("/tmp/pose_recorder.csv"));
    nh.param("rate_hz", rate_hz, 2.0);

    std::ofstream out(csv_path.c_str(), std::ios::out | std::ios::trunc);
    if (!out.is_open())
    {
        ROS_ERROR_STREAM("Failed to open CSV file: " << csv_path);
        return 1;
    }

    out << "stamp,x,y,yaw\n";

    tf::TransformListener listener;
    ros::Rate rate(rate_hz);

    while (ros::ok())
    {
        tf::StampedTransform transform;
        try
        {
            listener.lookupTransform(target_frame, source_frame, ros::Time(0), transform);

            const double x = transform.getOrigin().x();
            const double y = transform.getOrigin().y();
            const double yaw = tf::getYaw(transform.getRotation());
            const double stamp = ros::Time::now().toSec();

            out << std::fixed << std::setprecision(6)
                << stamp << "," << x << "," << y << "," << yaw << "\n";
            out.flush();

            ROS_INFO_STREAM("pose_recorder | " << target_frame << " -> " << source_frame
                            << " | x=" << x << " y=" << y << " yaw=" << yaw);
        }
        catch (const tf::TransformException& ex)
        {
            ROS_WARN_STREAM_THROTTLE(2.0, "pose_recorder TF error: " << ex.what());
        }

        rate.sleep();
    }

    return 0;
}