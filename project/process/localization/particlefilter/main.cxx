
/*
 * Includes
 */
#include <MSG.h>                            // error/debug/... messages
#include <boost/program_options.hpp>        // handles program options

// RSB related
#include <boost/shared_ptr.hpp>
// RSB
#include <rsb/Factory.h>
#include <rsb/Informer.h>
#include <rsb/Handler.h>
#include <rsc/threading/SynchronizedQueue.h>
#include <rsb/util/QueuePushHandler.h>

// RST converter
#include <rsb/converter/Repository.h>
#include <rsb/converter/ProtocolBufferConverter.h>
#include <converter/vecFloatConverter/main.hpp>

// RST Proto types
//#include <types/LocatedLaserScan.pb.h>        // already included in particlefilter.h
//#include <rst/geometry/Translation.pb.h>      // s.a.
#include <types/PoseEuler.pb.h>

// RSC
#include <rsc/misc/SignalWaiter.h>

// OpenCV for map loading
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>      // imread
#include <opencv2/imgproc/imgproc.hpp>      // cvtColor

#include "particlefilter.h"
#include "raycastingmodel.h"
#include "likelihoodfieldmodel.h"

#include "importancetype/addgaussians.h"

cv::Mat3b visualize(Map &map, ParticleFilter &particlefilter, bool visualizeImportance, float scale) {
    INFO_MSG("Preparing visualization");
    // Visualization
    // convert to RGB
    cv::Mat3b vis(map.size(), CV_8UC3);
    cv::cvtColor(map, vis, CV_GRAY2RGB, 3);

    // resize
    cv::resize(vis, vis, cv::Size(map.size().width * scale, map.size().height * scale));

    // find maximum importance
    sample_set_t *sampleSet = particlefilter.getSamplesSet();
    float maxImportance = 0;
    for (size_t i = 0; i < sampleSet->size; ++i) {
        maxImportance = max(maxImportance, sampleSet->samples[i].importance);
    }
    maxImportance /= sampleSet->totalWeight;

    float meanX = 0.0f, meanY = 0.0f;
    float sumThetaX = 0.0f, sumThetaY = 0.0f;

    // draw all samples
    for (size_t i = 0; i < sampleSet->size; ++i) {
        sample_t sample = sampleSet->samples[i];
        float importance = sample.importance / sampleSet->totalWeight;
        // accumulate data for mean
        meanX += importance * sample.pose.x;
        meanY += importance * sample.pose.y;
        sumThetaX += importance * cos(sample.pose.theta);
        sumThetaY += importance * sin(sample.pose.theta);

        cv::Point p(sample.pose.x / map.meterPerCell * scale, sample.pose.y / map.meterPerCell * scale);

        int radius = 5;
        int intensity = 0;
        if (visualizeImportance) {
            intensity = -255 / maxImportance * importance + 255;
        }
        cv::circle(vis, p, radius, cv::Scalar(255,intensity,intensity));

        // also indicate theta
        cv::Point q( p.x + cos(sample.pose.theta) * radius * 1.5f, p.y + sin(sample.pose.theta) * 1.5f * radius );
        cv::line(vis, p, q, cv::Scalar(255,intensity,intensity));
    }

    // overlay mean
    float meanTheta = atan2(sumThetaY, sumThetaX);

    cv::Point p(meanX / map.meterPerCell * scale, meanY / map.meterPerCell * scale);

    int radius = 5;
    cv::circle(vis, p, 5, cv::Scalar(0,0,255));

    cv::Point q( p.x + cos(meanTheta) * radius * 1.5f, p.y + sin(meanTheta) * 1.5f * radius );
    cv::line(vis, p, q, cv::Scalar(0,0,255));

    return vis;
}

rsb::Informer<rst::geometry::PoseEuler>::Ptr poseInformer;
void publishPose(const pose_t &pose) {
    rsb::Informer<rst::geometry::PoseEuler>::DataPtr data(new rst::geometry::PoseEuler);

    // convert
    data->mutable_translation()->set_x(pose.x);
    data->mutable_translation()->set_y(pose.y);
    data->mutable_translation()->set_z(0);
    data->mutable_rotation()->set_pitch(0);
    data->mutable_rotation()->set_roll(0);
    data->mutable_rotation()->set_yaw(pose.theta);

    poseInformer->publish(data);
}

int main(int argc, const char **argv) {
    /*
     * Handle program options
     */
    std::string lidarInScope = "/AMiRo_Hokuyo/lidar";
    std::string odomInScope = "/AMiRo_Hokuyo/gps";
    std::string debugImageOutScope = "/particlefilter/debugImage";
    std::string poseEstimateInScope = "/setPosition";
    std::string poseEstimateOutScope = "/pose";

    size_t sampleCount = 10;
    float meterPerPixel = 0.01f;
    std::string pathToMap = "map.png";
    int beamskip = 1;
    float newSampleProb = 0.1f;
    float maxFrequency = 10.0f;
    bool flip = false;
    float sigma = 1.0f;
    std::vector<float> initialPose;
    float samplingStdDev = 0.01f;

    namespace po = boost::program_options;

    po::options_description options("Allowed options");
    options.add_options()("help,h", "Display a help message.")
            ("lidarInScope", po::value < std::string > (&lidarInScope)->default_value(lidarInScope), "Scope for receiving lidar data")
            ("odomInScope", po::value < std::string > (&odomInScope)->default_value(odomInScope), "Scope for receiving odometry data")
            ("poseEstimateInScope", po::value< std::string > (&poseEstimateInScope), "Scope for receiving pose estimates (from the setPosition tool).")
            ("debugImageOutScope", po::value < std::string > (&debugImageOutScope), "Scope for sending the debug image.")
            ("poseEstimateOutScope", po::value< std::string > (&poseEstimateOutScope), "Scope for publishing the estimated pose.")
            ("sampleCount", po::value < std::size_t > (&sampleCount)->default_value(sampleCount), "Number of particles")
            ("meterPerPixel", po::value < float > (&meterPerPixel)->default_value(meterPerPixel), "resolution of the map in meter per pixel")
            ("pathToMap", po::value < std::string > (&pathToMap)->default_value(pathToMap), "Filesystem path to image that contains the map")
            ("flip", po::bool_switch(&flip), "Flip map around x axis.")
            ("kldsampling", "Switches on KLD sampling.")
            ("newSampleProb", po::value < float > (&newSampleProb)->default_value(newSampleProb), "Probability for generating a new sample (instead of roulette wheel selection)")
            ("beamskip", po::value < int > (&beamskip)->default_value(beamskip), "Take every n-th beam into account when calculating importance factor")
            ("maxFrequency", po::value < float > (&maxFrequency)->default_value(maxFrequency), "Maximum frequency at which new positon is published (1/s)")
            ("sigma", po::value< float > (&sigma)->default_value(sigma), "Sigma for the gaussians used for the beam noise.")
            ("initialPose", po::value< std::vector<float> >(&initialPose)->multitoken(), "Initial pose, when no pose is given, randomly distributed samples are used. (m, m, rad)")
            ("samplingStdDev", po::value< float >(&samplingStdDev)->default_value(samplingStdDev), "Standard deviation in the sampling step.");

    // allow to give the value as a positional argument
    po::positional_options_description p;
    p.add("value", 1);

    po::variables_map vm;
    po::store( po::command_line_parser(argc, argv).options(options).positional(p).run(), vm);

    // first, process the help option
    if (vm.count("help")) {
        std::cout << options << "\n";
        exit(1);
    }

    // afterwards, let program options handle argument errors
    po::notify(vm);

    INFO_MSG("lidarInScope: " << lidarInScope);
    INFO_MSG("odomInScope: " << odomInScope);
    INFO_MSG("poseEstimateInScope: " << poseEstimateInScope);
    INFO_MSG("poseEstimateOutScope: " << poseEstimateOutScope);

    INFO_MSG("meter per pixel: " << meterPerPixel);

    // do KLD sampling?
    bool doKLDSampling = false;
    if (vm.count("kldsampling")) {
        INFO_MSG("KLD sampling enabled");
        doKLDSampling = true;
    }

    // send debug image?
    bool sendDebugImage = false;
    if (vm.count("debugImageOutScope")) {
        INFO_MSG("Sending debug image on scope: " << debugImageOutScope);
        sendDebugImage = true;
    }

    // maybe create a initial sample set?
    sample_set_t *sampleSet = NULL;
    if (vm.count("initialPose") && initialPose.size() == 3) {
        INFO_MSG("Using initial pose " << initialPose.at(0) << "," << initialPose.at(1) << "," << initialPose.at(2));
        sample_t initialSample;
        initialSample.importance = 1;
        initialSample.pose.x = initialPose.at(0);
        initialSample.pose.y = initialPose.at(1);
        initialSample.pose.theta = initialPose.at(2);

        sampleSet = new sample_set_t;
        sampleSet->size = sampleCount;
        sampleSet->totalWeight = 1;
        sampleSet->samples = new sample_t[sampleCount];
        sampleSet->samples[0] = initialSample;

        for (size_t i = 1; i < sampleCount; i++) {
            sampleSet->samples[i].importance = 0;
            sampleSet->samples[i].pose = initialSample.pose;
        }
    }

    /*
     * RSB Informers and listeners
     */
    rsb::Factory& factory = rsb::getFactory();

    // Setup converters
    boost::shared_ptr< rsb::converter::ProtocolBufferConverter<rst::vision::LocatedLaserScan > > scanConverter(new rsb::converter::ProtocolBufferConverter<rst::vision::LocatedLaserScan >());
    rsb::converter::converterRepository<std::string>()->registerConverter(scanConverter);
    boost::shared_ptr< rsb::converter::ProtocolBufferConverter<rst::geometry::Pose > > odomConverter(new rsb::converter::ProtocolBufferConverter<rst::geometry::Pose >());
    rsb::converter::converterRepository<std::string>()->registerConverter(odomConverter);
    boost::shared_ptr<muroxConverter::vecFloatConverter> vecFloatConverter(new muroxConverter::vecFloatConverter());
    rsb::converter::converterRepository<std::string>()->registerConverter(vecFloatConverter);
    rsb::converter::ProtocolBufferConverter<rst::geometry::PoseEuler>::Ptr poseConverter(new rsb::converter::ProtocolBufferConverter<rst::geometry::PoseEuler>());
    rsb::converter::converterRepository<std::string>()->registerConverter(poseConverter);

    // Prepare RSB listener for incomming lidar scans
    rsb::ListenerPtr lidarListener = factory.createListener(lidarInScope);
    boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<rst::vision::LocatedLaserScan>>>lidarQueue(new rsc::threading::SynchronizedQueue<boost::shared_ptr<rst::vision::LocatedLaserScan>>(1));
    lidarListener->addHandler(rsb::HandlerPtr(new rsb::util::QueuePushHandler<rst::vision::LocatedLaserScan>(lidarQueue)));
    // Prepare RSB async listener for odometry messages
    rsb::ListenerPtr odomListener = factory.createListener(odomInScope);
    boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<rst::geometry::Pose>>>odomQueue(new rsc::threading::SynchronizedQueue<boost::shared_ptr<rst::geometry::Pose>>(1));
    odomListener->addHandler(rsb::HandlerPtr(new rsb::util::QueuePushHandler<rst::geometry::Pose>(odomQueue)));
    // Prepare RSB async listener for a pose estimate (with the setPosition tool)
    rsb::ListenerPtr poseEstimateListener = factory.createListener(poseEstimateInScope);
    boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr< std::vector<float> >>>poseEstimateQueue(new rsc::threading::SynchronizedQueue<boost::shared_ptr< std::vector<float> >>(5));
    poseEstimateListener->addHandler(rsb::HandlerPtr(new rsb::util::QueuePushHandler< std::vector<float> >(poseEstimateQueue)));

    // Informer for map
    rsb::Informer<std::string>::Ptr debugImageInformer = factory.createInformer<std::string>(debugImageOutScope);

    // Informer for current pose estimate
    poseInformer = factory.createInformer<rst::geometry::PoseEuler>(poseEstimateOutScope);

    /*
     * Setup the particle filter
     */
    // Load map
    cv::Mat1b tmp_mat = cv::imread(pathToMap, CV_LOAD_IMAGE_GRAYSCALE);
    if (!tmp_mat.data) {
        ERROR_MSG("Could not load map file: " << pathToMap);
        return 1;
    }
    if (flip) {
        INFO_MSG("Flipping map around x-axis.");
        cv::flip(tmp_mat, tmp_mat, 0); // 0 => around x-axis
    }
    Map map(tmp_mat);
    map.meterPerCell = meterPerPixel;

    // Blocks until first scan is received. It will be used as configuration for the particle filter
    INFO_MSG("Waiting for first laser scan...");
    boost::shared_ptr<rst::vision::LocatedLaserScan> scanPtr = lidarQueue->pop();
    // Get inital odometry
    INFO_MSG("Waiting for first odometry data...");
    boost::shared_ptr<rst::geometry::Pose> odomPtr = odomQueue->pop();

    // Finally set up the particle filter
    //RayCastingModel sensorModel(&map);
    LikelihoodFieldModel<AddGaussians> sensorModel(&map);
    sensorModel.importanceType.sigma = sigma;
    ParticleFilter particlefilter(sampleCount, *odomPtr, *scanPtr, &map, &sensorModel, newSampleProb, doKLDSampling, beamskip, sampleSet, samplingStdDev);

    /*
     * Main loop
     */
    boost::uint64_t minPeriod = 1000 / maxFrequency; // in ms
    rsc::misc::initSignalWaiter();
    bool running = true;
    // visualization parameters
    bool visualizeImportance = true;
    int height = min(map.size().height, 700);
    float scale = (float)height / map.size().height;

    while (rsc::misc::lastArrivedSignal() == rsc::misc::NO_SIGNAL && running) {
        boost::uint64_t loopStart = rsc::misc::currentTimeMillis();

        if (sendDebugImage) {
            cv::Mat3b vis = visualize(map, particlefilter, visualizeImportance, scale);

            // convert to jpeg
            std::vector<uchar> buf;
            std::vector<int> compression_params;
            compression_params.push_back(CV_IMWRITE_JPEG_QUALITY);
            compression_params.push_back(100);
            cv::imencode(".jpg", vis, buf, compression_params);

            // and publish
            rsb::Informer<std::string>::DataPtr frameJpg(new std::string(buf.begin(), buf.end()));
            debugImageInformer->publish(frameJpg);

#ifndef __arm__
            // alternatively show image in opencv window (when on x86)
            cv::flip(vis, vis, 0); // flip horizontally
            cv::imshow("visualization", vis);
            int key = cv::waitKey(100);
            switch (key) {
                case 27: // 27 == escape key
                    running = false;
                    break;

                case ' ': // space bar toogles importance visualization
                    visualizeImportance = !visualizeImportance;
                    break;
            }
#endif
        }

        // Update scan
        if (!lidarQueue->empty()) {
            scanPtr = lidarQueue->pop();
        }
        // Update odometry
        if (!odomQueue->empty()) {
            odomPtr = odomQueue->pop();
        }

        // Check for a new pose estimate
        if (!poseEstimateQueue->empty()) {
            std::vector<float> poseEstimate = *(poseEstimateQueue->pop());
            if (poseEstimate.size() == 3) {
                // pose estimates are given in cell/pixel coordinates, convert them
                sample_t newSample;
                newSample.pose.x = poseEstimate.at(0) / scale * map.meterPerCell;
                newSample.pose.y = poseEstimate.at(1) / scale * map.meterPerCell;
                newSample.pose.theta = poseEstimate.at(2);

                sample_set_t *sampleSet = particlefilter.getSamplesSet();
                newSample.importance = sampleSet->size * sampleSet->totalWeight;
                // first adapt the total weight
                sampleSet->totalWeight -= sampleSet->samples[0].importance;
                sampleSet->totalWeight += newSample.importance;
                // and then replace old sample
                sampleSet->samples[0] = newSample;

                for (size_t i = 1; i < sampleSet->size; i++) {
                    sampleSet->samples[i].importance = 0;
                }

                sampleSet->totalWeight = newSample.importance;

                // XXX: instead maybe search for an unimportant sample? or update multiple?
            } else {
                ERROR_MSG("Received a pose estimate vector of wrong size: " << poseEstimate.size());
            }
        }

        boost::uint64_t start = rsc::misc::currentTimeMillis();
        particlefilter.update(*scanPtr, *odomPtr, true);
        boost::uint64_t stop = rsc::misc::currentTimeMillis();
        INFO_MSG("Updating particle filter took " << (stop - start) << " ms");

        // publish the estimated pose
        pose_t meanPose = particlefilter.getWeightedMeanPose();
        publishPose(meanPose);

        boost::uint64_t loopPeriod = rsc::misc::currentTimeMillis() - loopStart;
        if (loopPeriod < minPeriod) {
            usleep((minPeriod - loopPeriod) * 1000);
        }

        INFO_MSG("==== END OF MAIN LOOP ====");
    }

    INFO_MSG("Terminating normally");
    return 0;
}
