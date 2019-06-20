// std includes
#include <iostream>
using namespace std;

#define INFO_MSG_
#define DEBUG_MSG_

#include "../../includes/MSG.h"

#include <math.h>

#include <boost/program_options.hpp>
#include <boost/shared_ptr.hpp>

// OpenCV
#include <opencv/cxcore.h>
#include <opencv/cv.h>
#include <opencv/ml.h>

// RSB
#include <rsb/Informer.h>
#include <rsb/Factory.h>
#include <rsb/Event.h>
#include <rsb/Handler.h>
#include <rsb/converter/Repository.h>
#include <rsc/threading/SynchronizedQueue.h>
#include <rsb/util/QueuePushHandler.h>

// RST
#include <rsb/converter/Repository.h>
#include <rsb/converter/ProtocolBufferConverter.h>

// RST Proto types
#include <types/LocatedLaserScan.pb.h>

#include <include/leg_detector/calc_leg_features.h>
#include <include/leg_detector/laser_processor.h>

using namespace boost;
using namespace std;
using namespace rsb;
using namespace rsb::converter;
using namespace rsb::patterns;


int main(int argc, const char **argv) {
	DEBUG_MSG("Start waypoint")
	namespace po = boost::program_options;
	std::string lidarInScope = "/AMiRo_Hokuyo/lidar";
	std::string stateOutScope = "/waypoint/state";
	std::string commandInscope = "/waypoint/command";
	float range = 1.0;
	float diffThreshold = 0.3f;
	float probBound = 0.999f;
	// LEG DETECTOR CONFIGURATION
	float legDist = 0.5f;
	float splitConnected = 0.06f;
	int removeLessThan = 5;

	po::options_description options("Allowed options");
	options.add_options()("help,h", "Display a help message.")
	    ("lidarinscope", po::value<std::string>(&lidarInScope), "Scope for receiving lidar data")
	    ("startNow,s", "Initializes the waypoint immediately without waiting for inscope.")
			("stateoutscope", po::value<std::string>(&stateOutScope), "Scope for sending states")
			("commandinscope", po::value<std::string>(&commandInscope), "Scope for receiving commands")
			("range,r", po::value<float>(&range), "Range of detection in m")
			("diffThreshold", po::value<float>(&diffThreshold), "Difference threshold in m")
			("legPairDistance", po::value<float>(&legDist), "Range between detected legs to be classified as a pair")
			("probBound", po::value<float>(&probBound), "Minimum leg pair probability")
			("splitConnected", po::value<float>(&splitConnected), "Minimum distance between datapoints to be a cluster")
			("removeLessThan", po::value<int>(&removeLessThan), "Minimum number in a cluster");

	// allow to give the value as a positional argument
	po::positional_options_description p;
	p.add("value", 1);

	po::variables_map vm;
	po::store(po::command_line_parser(argc, argv).options(options).positional(p).run(), vm);

	// first, process the help option
	if (vm.count("help")) {
		std::cout << options << "\n";
		exit(1);
	}

	// afterwards, let program options handle argument errors
	po::notify(vm);

	// Get the RSB factory
    rsb::Factory& factory = rsb::getFactory();

	// Register
	boost::shared_ptr<rsb::converter::ProtocolBufferConverter<rst::vision::LocatedLaserScan> > scanConverter(
			new rsb::converter::ProtocolBufferConverter<rst::vision::LocatedLaserScan>());
	rsb::converter::converterRepository<std::string>()->registerConverter(scanConverter);

	// ---------------- Informer ---------------------

	// Prepare RSB listener for incomming lidar scans
	rsb::ListenerPtr lidarListener = factory.createListener(lidarInScope);
	boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<rst::vision::LocatedLaserScan>>>lidarQueue(new rsc::threading::SynchronizedQueue<boost::shared_ptr<rst::vision::LocatedLaserScan>>(1));
	lidarListener->addHandler(
			rsb::HandlerPtr(new rsb::util::QueuePushHandler<rst::vision::LocatedLaserScan>(lidarQueue)));

	rsb::ListenerPtr commandListner = factory.createListener(commandInscope);
	boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<std::string>>>commandQueue(new rsc::threading::SynchronizedQueue<boost::shared_ptr<std::string>>(1));
	commandListner->addHandler(rsb::HandlerPtr(new rsb::util::QueuePushHandler<std::string>(commandQueue)));

	rsb::Informer<std::string>::Ptr stateInformer = factory.createInformer<std::string>(stateOutScope);

	rst::vision::LocatedLaserScan scan, initscan, initscan_raw;

	bool enabled = false;
  bool triggered_new = false;

	// ----------------------------------------------------------------------------
	// Init the decision tree
	std::string tree("bigDataSet_Tree.yaml");
	CvRTrees forest;
	forest.load(tree.c_str());
  int feat_count_ = forest.get_active_var_mask()->cols;
  printf("Loaded forest with %d features: %s\n", feat_count_, tree.c_str());

  // ----------------------------------------------------------------------------
  CvMat *tmp_mat = cvCreateMat(1, feat_count_, CV_32FC1);
	while (true) {
	  bool triggered = false;
		// check if the checkpoint is enabled/disabled by the stateMachine
		if (!enabled) {
			bool initNow = vm.count("startNow");
			if (!initNow) {
				std::string command(*commandQueue->pop());
				initNow = command.compare("init") == 0;
			}

			if (initNow) {
				DEBUG_MSG("init")
				enabled = true;
				// initialize scan
				initscan_raw = *lidarQueue->pop();
				initscan = initscan_raw;
				for (int i = 0; i < initscan_raw.scan_values_size(); ++i) {
					if (initscan_raw.scan_values(i) < initscan_raw.scan_values_min()) {
						initscan_raw.set_scan_values(i, 10000);
					}
				}
				// filter initial scan
				for (int i = 1; i < initscan.scan_values_size() - 1; ++i) {
					initscan.set_scan_values(i,
							min(initscan_raw.scan_values(i - 1),
									min(initscan_raw.scan_values(i), initscan_raw.scan_values(i + 1))));
					if (initscan.scan_values(i) == 10000) {
						initscan.set_scan_values(i, 0);
					}
				}
			}
		} else {
			if (!commandQueue->empty()) {
				std::string command(*commandQueue->pop());
				if (command.compare("stop") == 0) {
					enabled = false;
				}
			}
		}

		if (enabled) {
      // Fetch a new scan and store it to scan
      scan = *lidarQueue->pop();

      // ----------------LEG DETECTION-----------------------------

      // CUT SCAN
      const float maxRange = range;
      for (int idx = 0; idx < scan.scan_values_size(); ++idx) {
         if (scan.scan_values(idx) > maxRange) {
           scan.mutable_scan_values()->Set(idx, 0);
         }
      }

      // THE LEG DETECTOR
      laser_processor::ScanMask mask_;
      laser_processor::ScanProcessor processor(&scan, mask_);
      processor.splitConnected(splitConnected /*from contructor*/ );  // connected_thresh_
      processor.removeLessThan(removeLessThan);

  //	  processor.getClusters().list()
       mask_.mask_.center();

      // Detection step: build up the set of "candidate" clusters
      // For each candidate, find the closest tracker (within threshold) and add to the match list
      // If no tracker is found, start a new one
      std::vector<float> prob_vec, x_mean_vec, y_mean_vec;
      for (list<laser_processor::SampleSet*>::iterator i = processor.getClusters().begin();
           i != processor.getClusters().end();
           i++)
      {
        vector<float> f = calcLegFeatures(*i, &scan);

  //	    std::cout << "detections prob: ";
        for (int k = 0; k < feat_count_; k++) {
          tmp_mat->data.fl[k] = (float)(f[k]);
  //	      std::cout << f[k] << " ";
        }
        float probability = forest.predict_prob(tmp_mat);
  //	    std::cout << probability;

        // Number of points
        int num_points = (*i)->size();
        float x_mean = 0.0;
        float y_mean = 0.0;
        vector<float> x_median_set;
        vector<float> y_median_set;
        for (laser_processor::SampleSet::iterator it = (*i)->begin(); it != (*i)->end(); it++) {
          x_mean += ((*it)->x) / num_points;
          y_mean += ((*it)->y) / num_points;
          x_median_set.push_back((*it)->x);
          y_median_set.push_back((*it)->y);
        }
  //	    std::cout << " x: " << x_mean << ", y: " << y_mean << std::endl << std::flush;

        // Store the stuff
        prob_vec.push_back(probability);
        x_mean_vec.push_back(x_mean);
        y_mean_vec.push_back(y_mean);
      }

  //	  std::vector<float> dist(prob_vec.size());
      int id = 0;
      triggered_new = false;
      for (unsigned int idx = 0; idx < x_mean_vec.size(); ++idx) {
        for (unsigned int idy = idx+1; idy < x_mean_vec.size(); ++idy) {
          const float dist = sqrt(pow(float(x_mean_vec[idx] - x_mean_vec[idy]),2) + pow(float(y_mean_vec[idx] - y_mean_vec[idy]),2));
          if ( prob_vec[idy] > probBound && prob_vec[idx] > probBound && dist < legDist) { // Only get legs which are legDist away and have high probability (>0.999)
            triggered_new = true;
            DEBUG_MSG( "Dist: " << float(dist) << ", P: " << float(prob_vec[idx]) << " " << float(prob_vec[idy]) );
            break;
          }
          ++id;
        }
      }

  //    std::cout << std::endl;

      // ---------------------------------------------

      // OLD check if the waypoint is triggered
  //    for (int i = 1; i < scan.scan_values_size() - 1; ++i) {
  //      if ((scan.scan_values(i) > scan.scan_values_min() && initscan.scan_values(i) > initscan.scan_values_min()
  //          && scan.scan_values(i) < range && initscan.scan_values(i) - scan.scan_values(i) > diffThreshold)
  //          || (initscan.scan_values(i) < initscan.scan_values_min()
  //              && scan.scan_values(i) > scan.scan_values_min() && scan.scan_values(i) < range)) {
  //        //DEBUG_MSG(abs(scan.scan_values(i) - initscan.scan_values(i)));
  //        triggered_new = true;
  //        break;
  //      }
  //    }

    }

      // send update state
      if (triggered_new) {
        stateInformer->publish(Informer<string>::DataPtr(new string("entered")));
        DEBUG_MSG("entered");
      } else {
        stateInformer->publish(Informer<string>::DataPtr(new string("left")));
        DEBUG_MSG("left");
      }
		usleep(200000);
	}
  cvReleaseMat(&tmp_mat);
  tmp_mat = 0;

	return 0;
}

