#pragma once
struct point_cloud
{
	int week;
	double time_of_week, latitude, longitude, ell_height;
	double xyz[3];
	double ned[3];
};

struct point_within_radius
{
	int index;
	float square_distance;
};

bool cmp(const point_within_radius& a, const point_within_radius& b)
{
	return a.index < b.index;
}