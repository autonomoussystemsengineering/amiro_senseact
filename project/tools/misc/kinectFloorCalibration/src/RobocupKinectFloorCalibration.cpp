/*
 * RobocupKinectFloorCalibration.cpp
 *
 * Author: caschebe
 */

#include "Viewer.h"

#include <string>
#include <iostream>
#include <stdio.h>
#include <vector>
#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>

const std::string description = "RobocupKinectFloorCalibration";
const std::string version = "0.1.0";

int main(int argc, char **argv) {

	int runs = 0;
	bool verbose = false;
	double ground = 0.0;
	double pitch = 0.0;
	double threshold = 0.02;
	std::string file_out = "";
	std::string file_in = "";
	std::string file_extension = ".template";

	boost::program_options::options_description desc("Allowed options");
	desc.add_options()("help,h", "show this help message")("verbose,v",
			"enable more info output")("runs,r",
			boost::program_options::value<int>(), "number of runs")("ground,g",
			boost::program_options::value<double>(),
			"lower ground by this value")("pitch,p",
			boost::program_options::value<double>(),
			"increase pitch by this value")("threshold,t",
			boost::program_options::value<double>(),
			"threshold for ground plane detection")("file-out,f",
			boost::program_options::value<std::string>(), "output file")(
			"file-in,i", boost::program_options::value<std::string>(),
			"input template")("file-extension,e",
			boost::program_options::value<std::string>(), "template extension");

	boost::program_options::variables_map vm;
	store(parse_command_line(argc, argv, desc), vm);
	notify(vm);

	if (vm.count("help")) {
		std::cout << description << std::endl << "Version: " + version
				<< std::endl << desc;
		return 0;
	}
	if (vm.count("verbose")) {
		verbose = true;
	}
	if (vm.count("runs")) {
		runs = vm["runs"].as<int>();
	}
	if (vm.count("ground")) {
		ground = vm["ground"].as<double>();
	}
	if (vm.count("pitch")) {
		pitch = vm["pitch"].as<double>();
	}
	if (vm.count("threshold")) {
		threshold = vm["threshold"].as<double>();
	}
	if (vm.count("file-out")) {
		file_out = vm["file-out"].as<std::string>();
	}
	if (vm.count("file-in")) {
		file_in = vm["file-in"].as<std::string>();
	}
	if (vm.count("file-extension")) {
		file_extension = vm["file-extension"].as<std::string>();
	}

	Viewer v(
			new Analyzer(runs, ground, pitch, threshold, file_out, file_in,
					file_extension, verbose));
	v.run();

	return 0;
}
