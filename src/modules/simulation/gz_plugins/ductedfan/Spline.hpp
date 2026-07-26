/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

#pragma once

#include <string>
#include <vector>

namespace px4
{

struct SplineSegment
{
	double x_left{0.0};
	double x_right{0.0};
	double a{0.0};
	double b{0.0};
	double c{0.0};
	double d{0.0};
};

bool LoadSplineFromCsv(const std::string &file_path, std::vector<SplineSegment> &spline_data);
double PpvalSpline(const std::vector<SplineSegment> &spline_data, double x, bool clamp_input = false);
std::string ResolveModelFile(const std::string &uri);

} // namespace px4
