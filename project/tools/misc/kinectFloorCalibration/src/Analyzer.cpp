/*
 * Analyzer.cpp
 *
 * Author: caschebe
 */

#include "Analyzer.h"

#include <iostream>
#include <fstream>
#include <boost/algorithm/string.hpp>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>

pcl::PointCloud<PointType>::Ptr Analyzer::analyze(
		const pcl::PointCloud<PointType>::ConstPtr cloud) {

	pcl::PointCloud<PointType>::Ptr returnCloud(new pcl::PointCloud<PointType>);

	if (!cloud->size()) {
		std::cerr << "warning: no input cloud" << std::endl;
		return returnCloud;
	}

	// find table plane
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	pcl::SACSegmentation<PointType> seg;
	seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setMaxIterations(100);
	seg.setDistanceThreshold(this->threshold);
	seg.setInputCloud(cloud);
	seg.segment(*inliers, *coefficients);

	if (!inliers->indices.size()) {
		std::cerr << "warning: no plane inliers" << std::endl;
		return returnCloud;
	}

	// calculate calibration parameters
	this->calculate(coefficients->values[0], coefficients->values[1],
			coefficients->values[2], coefficients->values[3]);

	// generate pointcloud for visualization
	pcl::PointCloud<PointType>::Ptr cloud_(new pcl::PointCloud<PointType>);
	pcl::PointCloud<PointType>::Ptr cloud_plane(new pcl::PointCloud<PointType>);
	pcl::ExtractIndices<PointType> extract;
	extract.setInputCloud(cloud);
	extract.setKeepOrganized(false);
	extract.setIndices(inliers);
	extract.setNegative(false);
	extract.filter(*cloud_plane);
	extract.setNegative(true);
	extract.filter(*cloud_);

	returnCloud->width = cloud_->size() + cloud_plane->size();
	returnCloud->height = 1;
	returnCloud->is_dense = false;
	returnCloud->points.resize(returnCloud->width * returnCloud->height);
	size_t j = 0;
	for (size_t i = 0; i < cloud_->size(); i++) {
		returnCloud->points[j].x = cloud_->points[i].x;
		returnCloud->points[j].y = cloud_->points[i].y;
		returnCloud->points[j].z = cloud_->points[i].z;
		returnCloud->points[j].rgb = cloud_->points[i].rgb;
		//returnCloud->points[j].r = 128;
		//returnCloud->points[j].g = 128;
		//returnCloud->points[j].b = 128;
		j++;
	}
	for (size_t i = 0; i < cloud_plane->size(); i++) {
		returnCloud->points[j].x = cloud_plane->points[i].x;
		returnCloud->points[j].y = cloud_plane->points[i].y;
		returnCloud->points[j].z = cloud_plane->points[i].z;
		returnCloud->points[j].r = 255;
		returnCloud->points[j].g = 0;
		returnCloud->points[j].b = 0;
		j++;
	}

	return returnCloud;
}

void Analyzer::calculate(const double x, const double y, const double z,
		const double d) {

	if (this->finished) {
		return;
	}

	const int sign = (d > 0) ? 1 : ((d < 0) ? -1 : 0);
	if (sign == 0) {
		std::cerr << "warning: sign == 0" << std::endl;
		return;
	}

	// transform Kinect -> ROS
	const double x_t = sign * z;
	const double y_t = sign * -x;
	const double z_t = sign * -y;
	const double d_t = sign * d;

	// x_t -> forward
	// y_t -> left
	// z_t -> up

	const double pitch = asin(-x_t);
	const double roll = atan(y_t / z_t);
	const double D = d_t;

	this->runs++;
	this->avgPitch += pitch;
	this->avgRoll += roll;
	this->avgD += D;

	const double pitch_avg = this->avgPitch / this->runs + this->Pitch_safe;
	const double roll_avg = this->avgRoll / this->runs;
	const double d_avg = this->avgD / this->runs - this->D_safe;

	if (verbose || this->maxRuns == 0) {
		std::cout << "runs = " << this->runs << " / " << this->maxRuns
				<< std::endl << "normal (x y z) = (" << x_t << " " << y_t << " "
				<< z_t << ")" << std::endl << "pitch = " << pitch_avg << " ("
				<< pitch << " + " << this->Pitch_safe << ")" << std::endl
				<< "roll = " << roll_avg << " (" << roll << ")" << std::endl
				<< "D = " << d_avg << " (" << D << " - " << this->D_safe << ")"
				<< std::endl;
	}

	if (this->maxRuns == 0 || this->runs < this->maxRuns) {
		return;
	}

	this->finished = true;

	const double factor = 1000;
	const double pitch_r = round(factor * pitch_avg) / factor;
	const double roll_r = round(factor * roll_avg) / factor;
	const double D_r = round(factor * d_avg) / factor;

	std::cout << std::endl << "--- final parameters:" << std::endl;
	std::cout << "pitch = " << pitch_r << std::endl;
	std::cout << "roll = " << roll_r << std::endl;
	std::cout << "Z = " << D_r << std::endl << std::endl;

	if (this->fileOut.empty()) {
		std::cerr << "warning: no output file specified" << std::endl;
		return;
	}

	if (this->fileIn.empty() && !this->fileExtension.empty()) {
		this->fileIn = this->fileOut + this->fileExtension;
	} else {
		std::cerr << "error: no template filename or extension specified"
				<< std::endl;
		return;
	}

	std::ifstream in;
	in.open(this->fileIn.c_str());
	if (!in.is_open()) {
		std::cerr << "error: could not open template file: " << this->fileIn
				<< std::endl;
		return;
	}

	std::ofstream out;
	out.open(this->fileOut.c_str());
	if (!out.is_open()) {
		std::cerr << "error: could not open output file: " << this->fileOut
				<< std::endl;
		return;
	}

	std::stringstream file;
	while (in.good()) {
		std::string line;
		getline(in, line);
		file << line << std::endl;
	}
	in.close();

	std::string data = file.str();

	std::stringstream stream;

	stream << pitch_r;
	boost::replace_all(data, "${PITCH}", stream.str());
	stream.str("");

	stream << roll_r;
	boost::replace_all(data, "${ROLL}", stream.str());
	stream.str("");

	stream << D_r;
	boost::replace_all(data, "${Z}", stream.str());
	stream.str("");

	out << "<!-- WARNING!" << std::endl
			<< "This file was auto-generated by the calibration tool!"
			<< std::endl
			<< "Persistent changes should be made to the associated template file!"
			<< std::endl << "-->" << std::endl << std::endl;
	out << data;
	out.close();

	std::cout << "parameters written to file " << this->fileOut << std::endl
			<< "template file was " << this->fileIn << std::endl;
}
