/*
 *  start_webots.cpp
 *
 *  Date: October 2015
 *  Authors: Nathan Burke
 *  
 */
#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <cstdio>
#include <vector>
#include <string>

using std::string;
using std::vector;

int main(int argc, char **argv) {
	ros::init(argc, argv, "webots");
	int exitCode;

	string webots_executable;
	if (!ros::param::get("~webots", webots_executable)) {
		webots_executable = "webots";
	}

	string worldPath;
	ros::param::get("~world", worldPath);

	string mode;
	ros::param::get("~mode", mode);

	vector<string> partials;
	ros::param::get("~partials", partials);

	ROS_INFO("Starting Webots simulator...");

	string mergedPath;
	if (!partials.empty()) {
		ROS_INFO("Merging %d partials...", (int )partials.size());

		// Merged file needs to be in the same dir as the world for webots protos to load
		mergedPath = worldPath + ".merged";
		std::ofstream mergedFile(mergedPath.c_str(),
				std::fstream::out | std::fstream::trunc);
		{
			std::ifstream currentFile(worldPath.c_str());
			mergedFile << currentFile.rdbuf();
			currentFile.close();
		}

		for (vector<string>::iterator it = partials.begin();
				it != partials.end(); ++it) {
			if (it->find('/') != string::npos) {
				ROS_INFO("Adding file: %s",
						it->substr(it->rfind('/') + 1, it->length() - 1).c_str());
			} else if (it->find('\\') != string::npos) {
				ROS_INFO("Adding file: %s",
						it->substr(it->rfind('\\') + 1, it->length() - 1).c_str());
			} else {
				ROS_INFO("Adding file: %s", it->c_str());
			}
			std::ifstream currentFile(it->c_str());
			mergedFile << currentFile.rdbuf();
			currentFile.close();
		}

		mergedFile.close();
	}

	std::stringstream ss;
	ss << webots_executable;

	// Set run mode
	if (!mode.empty()) {
		ss << " --mode=" << mode;
	}

	// Add any additional arguments
	string args;
	if (ros::param::get("~args", args)) {
		ss << " " << args;
	}

	// Add world file
	if (!mergedPath.empty()) {
		ROS_INFO("Loading merged world: %s", mergedPath.c_str());
		ss << " " << mergedPath;
	} else if (!worldPath.empty()) {
		ROS_INFO("Loading world: %s", worldPath.c_str());
		ss << " " << worldPath;
	} else {
		ROS_INFO("Loading default world");
	}

	// Launch Webots
	ROS_INFO("CMD: %s", ss.str().c_str());
	exitCode = system(ss.str().c_str());

	// Clean up temp files
	if (mergedPath.length() > 0) {
		ROS_INFO("Removing merged world file");
		std::remove(mergedPath.c_str());
	}

	return exitCode;
}

