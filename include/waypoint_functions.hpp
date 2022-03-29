
#include <vector>
#include <ctime>
#include <string>
#include <map>
#include <iostream>
#include <unistd.h>
#include <cmath>
#include <math.h>
#include <gnc_functions.hpp>

struct timed_waypoint : public gnc_api_waypoint
{
	time_t end_time;
	time_t start_time;
	float battery_capacity;
};

// a list of the waypoints, which consist of cordinates
typedef std::vector<timed_waypoint> TimedWaypointList;

// Map of tasks and the amount of battery percentage used
typedef std::map<std::string, int> BatteryReportMap;

TimedWaypointList generateCircle()
{
	TimedWaypointList waypointList;

	int increment = 9;
	int radius = 10;
	for (size_t i = 0; i <= increment; i++)
	{
		timed_waypoint nextWayPoint;
		nextWayPoint.x = radius * cos(i * 2 * M_PI / increment);
		nextWayPoint.y = radius * sin(i * 2 * M_PI / increment);
		nextWayPoint.z = 5;
		nextWayPoint.speed = 50 + i * 10;
		nextWayPoint.psi = 10;
		nextWayPoint.end_time = 0;
		nextWayPoint.start_time = 0;
		waypointList.push_back(nextWayPoint);
	}
	return waypointList;
}

TimedWaypointList generateStraightLine(int distance, int no_of_waypoints)
{
	TimedWaypointList waypointList;

	for (size_t i = 1; i <= no_of_waypoints; i++)
	{
		timed_waypoint nextWayPoint;
		nextWayPoint.x = 0;
		nextWayPoint.y = 0 + (distance / no_of_waypoints) * i;
		nextWayPoint.z = 5;
		nextWayPoint.speed = 80;
		nextWayPoint.psi = 10;
		nextWayPoint.end_time = 0;
		nextWayPoint.start_time = 0;
		waypointList.push_back(nextWayPoint);
	}
	return waypointList;
}
