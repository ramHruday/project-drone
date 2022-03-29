#include <waypoint_functions.hpp>
#include <vector>
#include <ctime>
#include <fstream>
#include <iostream>
#include <string>
#include <map>
#include <unistd.h>
#include <darknet_ros_msgs/BoundingBoxes.h>

float avg_image_p_time = 0;
float counter = 1;
std::map<std::string, int[1]> FrameMap;
bool hover;

void detection_cb(const darknet_ros_msgs::BoundingBoxes::ConstPtr &msg)
{
    for (int i = 0; i < msg->bounding_boxes.size(); i++)
    {
        if (msg->bounding_boxes[i].Class == "person")
        {

            ROS_INFO("Person found. calculating average time");
        }
        // FrameMap.insert(msg->header.frame_id,{msg->header.stamp.toSec(),0});
        ros::Duration t = msg->header.stamp - msg->image_header.stamp;
        ROS_INFO("PAvg time %d %d", msg->header.stamp.toNSec(), msg->image_header.stamp.toNSec());
        double sum = counter * avg_image_p_time + t.toNSec();
        counter++;
        avg_image_p_time = sum / counter;
    };
}

void generate_report(TimedWaypointList waypointList, float final_battery, int commandCount, std::string report_name, float total_d)
{
    int distance_travelled = 0;
    int total_time = 0;
    std::ofstream myfile;
    myfile.open(report_name);
    myfile << "<!DOCTYPE html><html><head></head><style>table {  font-family: arial, sans-serif;  border-collapse: collapse;  width: 100%;}td, th {  border: 1px solid #dddddd;  text-align: left;  padding: 8px;}tr:nth-child(even) {  background-color: #dddddd;}</style><body> <table><thead> <tr> <th>Waypoint Number</th>   <th>Speed</th>   <th>Distance</th>   <th>Trip Time </th>    <th>battery % before trip</th>  </tr></thead>"; // starting html

    for (size_t i = 0; i < waypointList.size(); i++)
    {
        timed_waypoint wayPoint = waypointList[i];
        int travel_time = (wayPoint.end_time - wayPoint.start_time);
        float battery_used = waypointList[i].battery_capacity;
        float d = travel_time * wayPoint.speed;
        distance_travelled += d;
        total_time += travel_time;
        myfile << "<tbody><tr><td>" << i
               << "</td><td>" << waypointList[i].speed
               << "</td><td>" << d
               << "</td><td>" << wayPoint.end_time << "and" << wayPoint.start_time
               << "</td><td>" << battery_used
               << "</ td></ tr></tbody>";
    }

    myfile << "<tfoot><tr style='font-weight: bold'><td></td>"
           << "</th><th>" << (float)distance_travelled / (float)total_time
           << "</th><th>" << distance_travelled << "act->" << total_d
           << "</th><th>" << total_time
           << "secs </th><th>" << final_battery
           << "% </ th></ tr> </tfoot>";
    myfile << "</table>";
    myfile << "<span style='margin-top:2px'> Number Commands published during this trip " << commandCount << "</span>";
    myfile << "<span style='margin-top:2px'> Average image processing time " << avg_image_p_time << "</span>";

    myfile << "</body></html>";
    myfile.close();
}

int run_drone(TimedWaypointList waypointList_g, bool withCommands, std::string report_name)
{

    TimedWaypointList waypointList_c = waypointList_g;
    // specify control loop rate. We recommend a low frequency to not over load the FCU with messages. Too many messages will cause the drone to be sluggish

    ros::Rate rate(2);
    int counter = 0;
    resetCommandCounter();
    while (ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
        if (check_waypoint_reached(.3) == 1)
        {
            if (counter < waypointList_c.size())
            {
                // increment speed
                if (counter > 0)
                {
                    waypointList_g[counter - 1].end_time = time(NULL);
                }

                set_speed((waypointList_c[counter].speed));

                set_destination(waypointList_c[counter].x, waypointList_c[counter].y, waypointList_c[counter].z, waypointList_c[counter].psi);
                waypointList_g[counter].start_time = time(NULL);
                waypointList_g[counter].battery_capacity = get_battery_state().percentage;
                printToConsole(counter);
                counter++;
            }
            else
            {
                // usleep(30);
                printToConsole(counter);
                waypointList_g.back().end_time = time(NULL) + 100;
                land();
                int comandCounter = getCommandCounter();
                int totalD = getTotalDistance();
                generate_report(waypointList_g, get_battery_state().percentage, comandCounter, report_name, totalD);
                break;
            }
        }
    }

    return 0;
}

int main(int argc, char **argv)
{
    // initialize ros
    ros::init(argc, argv, "gnc_node");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("/darknet_ros/bounding_boxes", 1, detection_cb);

    // initialize control publisher/subscribers
    init_publisher_subscriber(n);

    // wait for FCU connection
    wait4connect();

    // wait for used to switch to mode GUIDED
    wait4start();

    // create local reference frame
    initialize_local_frame();

    // request takeoff
    takeoff(3);

    TimedWaypointList waypointList_g;

    // generate a single waypoint with 100 distance and run ros
    waypointList_g = generateStraightLine(40, 2);
    run_drone(waypointList_g, true, "reportWith.html");
}
