/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

#include "Spline.hpp"

#include <gz/common/Console.hh>

#include <algorithm>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <sstream>

namespace px4
{

namespace fs = std::filesystem;

namespace
{

void AppendEnvPaths(const char *name, std::vector<fs::path> &paths)
{
	const char *value = std::getenv(name);

	if (value == nullptr) {
		return;
	}

	std::stringstream stream(value);
	std::string item;

	while (std::getline(stream, item, ':')) {
		if (!item.empty()) {
			paths.emplace_back(item);
		}
	}
}

std::vector<fs::path> ResourceRoots()
{
	std::vector<fs::path> roots;
	AppendEnvPaths("GZ_SIM_RESOURCE_PATH", roots);
	AppendEnvPaths("IGN_GAZEBO_RESOURCE_PATH", roots);
	AppendEnvPaths("GZ_FILE_PATH", roots);
	AppendEnvPaths("IGN_FILE_PATH", roots);
	AppendEnvPaths("SDF_PATH", roots);

	fs::path cursor = fs::current_path();

	for (int i = 0; i < 8; ++i) {
		roots.push_back(cursor / "Tools/simulation/gz/models");
		roots.push_back(cursor / "models");

		if (!cursor.has_parent_path() || cursor == cursor.parent_path()) {
			break;
		}

		cursor = cursor.parent_path();
	}

	return roots;
}

} // namespace

std::string ResolveModelFile(const std::string &uri)
{
	if (uri.rfind("model://", 0) != 0) {
		return uri;
	}

	const std::string relative = uri.substr(std::string("model://").size());

	for (const auto &root : ResourceRoots()) {
		const fs::path candidate = root / relative;

		if (fs::exists(candidate)) {
			return candidate.string();
		}
	}

	return uri;
}

bool LoadSplineFromCsv(const std::string &file_path, std::vector<SplineSegment> &spline_data)
{
	const std::string resolved_path = ResolveModelFile(file_path);
	std::ifstream file(resolved_path.c_str());

	if (!file.is_open()) {
		gzerr << "[DuctedFanSpline] Unable to open spline file: " << file_path
		      << " resolved as: " << resolved_path << "\n";
		return false;
	}

	spline_data.clear();

	std::string line;
	int line_index = 0;

	while (std::getline(file, line)) {
		line_index++;

		if (line.empty()) {
			continue;
		}

		std::replace(line.begin(), line.end(), ',', ' ');
		std::stringstream stream(line);
		SplineSegment segment;

		if (!(stream >> segment.x_left >> segment.x_right
		      >> segment.a >> segment.b >> segment.c >> segment.d)) {
			gzerr << "[DuctedFanSpline] Unable to parse line " << line_index
			      << " in " << resolved_path << ": " << line << "\n";
			return false;
		}

		spline_data.push_back(segment);
	}

	if (spline_data.empty()) {
		gzerr << "[DuctedFanSpline] Empty spline file: " << resolved_path << "\n";
		return false;
	}

	std::sort(spline_data.begin(), spline_data.end(), [](const SplineSegment &lhs, const SplineSegment &rhs) {
		return lhs.x_left < rhs.x_left;
	});

	return true;
}

double PpvalSpline(const std::vector<SplineSegment> &spline_data, double x, bool clamp_input)
{
	if (spline_data.empty()) {
		return 0.0;
	}

	double x_eval = x;

	if (clamp_input) {
		x_eval = std::clamp(x_eval, spline_data.front().x_left, spline_data.back().x_right);
	}

	const SplineSegment *selected_segment = nullptr;

	for (const auto &segment : spline_data) {
		if (x_eval >= segment.x_left && x_eval <= segment.x_right) {
			selected_segment = &segment;
			break;
		}
	}

	if (selected_segment == nullptr) {
		selected_segment = x_eval < spline_data.front().x_left ? &spline_data.front() : &spline_data.back();
	}

	const double dx = x_eval - selected_segment->x_left;
	return selected_segment->a * dx * dx * dx
	       + selected_segment->b * dx * dx
	       + selected_segment->c * dx
	       + selected_segment->d;
}

} // namespace px4
