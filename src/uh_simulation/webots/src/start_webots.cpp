/*
 *  start_webots.cpp
 *
 *  Starts the Webots Simulator using a system() call, so Webots can be 
 *  started from a roslaunch file.
 *
 *  Date: September 2012
 *  Authors: David Butterworth
 *  
 */

/*
 * Copyright (c) 2012, David Butterworth, KAIST
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <cstdio>
#include <vector>
#include <string>

using std::string;
using std::vector;

/*
 Define Webots executable name.
 The full path will come from the PATH environment variable.
 Prefix with 'optirun' for Linux NVidia Bumblebee graphics driver

 Webots command line arguments:
 SYNOPSIS: webots [options] [worldfile]
 OPTIONS:
 --minimize                  minimize Webots window on startup
 --mode=<mode>               choose startup mode (overrides
 application preferences)
 argument <mode> must be one of:
 pause, realtime, run or fast
 (Webots PRO is required to use:
 --mode==run or --mode=fast)
 --help                      display this help message and exit
 --sysinfo                   display information of the system and
 exit
 --version                   display version information and exit
 --uuid                      display the UUID of the computer and exit
 --stdout                    redirect the controller stdout to the
 terminal
 --stderr                    redirect the controller stderr to the
 terminal
 --disable-modules-download  skip the check for module updates
 --force-modules-download    automatically download module updates
 (if any) at startupsystem
 --start-streaming-server    starts the Webots streaming server
 (Webots PRO is required)
 [="key[=value];..."]         parameters may be given as an option:
 port=1234 : starts the streaming
 server on port 1234

 See: http://www.cyberbotics.com/dvd/common/doc/webots/guide/section2.2.html
 */

#define WEBOTS_EXECUTABLE "webots"

//----------------------------------------------------------------------------//

int main(int argc, char **argv) {
	ros::init(argc, argv, "webots");
	int exitCode;

	string webots_executable;
	if (!ros::param::get("~webots", webots_executable)) {
		webots_executable = WEBOTS_EXECUTABLE;
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

