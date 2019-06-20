/*
 * Analyzer.h
 *
 * Author: caschebe
 */

#ifndef ANALYZER_H_
#define ANALYZER_H_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

typedef pcl::PointXYZRGBA PointType;

class Analyzer {
public:
	bool finished;

	Analyzer(int maxRuns = 0, double D_safe = 0, double Pitch_safe = 0,
			double threshold = 0, std::string fileOut = "", std::string fileIn =
					"", std::string fileExtension = "", bool verbose = false) :
			finished(false), verbose(verbose), runs(0), maxRuns(maxRuns), fileOut(
					fileOut), fileIn(fileIn), fileExtension(fileExtension), avgPitch(
					0), avgRoll(0), avgD(0), Pitch_safe(Pitch_safe), D_safe(
					D_safe), threshold(threshold) {
	}

	pcl::PointCloud<PointType>::Ptr analyze(
			const pcl::PointCloud<PointType>::ConstPtr cloud);

protected:
	bool verbose;
	int runs;
	int maxRuns;
	std::string fileOut;
	std::string fileIn;
	std::string fileExtension;
	double avgPitch;
	double avgRoll;
	double avgD;
	double D_safe;
	double Pitch_safe;
	double threshold;

	void calculate(const double x, const double y, const double z,
			const double d);
};

#endif /* ANALYZER_H_ */
