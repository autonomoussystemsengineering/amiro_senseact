/**************
 * Namespaces *
 **************/

using namespace std;

/************
 * Includes *
 ************/

#include <MSG.h> // debug, info, etc. output

#include <boost/program_options.hpp> // handles program options
#include <boost/make_shared.hpp>

#include <rsc/misc/SignalWaiter.h> // for a clean exit

#include <rsb/Factory.h>
#include <rsb/Listener.h> // for receiving current pose
#include <rsb/Informer.h> // for sending new target pose
#include <types/TargetPoseEuler.pb.h> // RST type for the target pose
#include <rsb/converter/ProtocolBufferConverter.h>
#include <rsb/converter/Repository.h>
#include <converter/vecIntConverter/main.hpp>
#include <rsb/filter/TypeFilter.h>
#include <rst/kinematics/Twist.pb.h>
#include <types/loc.pb.h> // for TWB tracking
#include <extspread/extspread.hpp>

#include <rsb/util/QueuePushHandler.h> // for receiving tracking data async
#include <rsc/threading/SynchronizedQueue.h>

#include <queue> // for a queue of waypoints

#include <fstream> // for pose logging

/********************
 * Global variables *
 ********************/

rsb::Informer<rst::geometry::TargetPoseEuler>::Ptr targetPoseInformer;
rsb::Informer<vector<int> >::Ptr targetSpeedInformer;
rsb::Informer<rst::kinematics::Twist>::Ptr twistInformer;
queue<rst::geometry::PoseEuler> waypoints;

// parameters for navigation
double xy_tolerance = 0, yaw_tolerance = 0;

// how to control the motor
enum {
    setTargetSpeed = 0,
    setTargetPosition = 1,
    Twist = 2
} motorCommandMethod = setTargetPosition;

const string motorCommandStrings[] = {
    "setTargetSpeed",
    "setTargetPosition",
    "Twist"
};

// velocities
float rotationalSpeed = M_PI / 6.0f; // rad / second
float forwardSpeed = 0.1f; // meter / second

// ... and pre-computed in µm
int rotationalSpeed_um;
int forwardSpeed_um;

// marker id for external tracking
uint trackingId = 0;

// file handler for pose data log
bool logging = false;
ofstream poseLogStream;

/*************
 * Functions *
 *************/

inline double distance(const rst::geometry::PoseEuler &a, const rst::geometry::PoseEuler &b) {
    return sqrt(
                pow(a.translation().x() - b.translation().x(), 2) +
                pow(a.translation().y() - b.translation().y(), 2)
    );
}

inline double normalize_angle(double a) {
    // normalize to [-PI; PI)
    if (a < -M_PI || a >= M_PI) {
        a = fmod(a + M_PI, 2*M_PI);
        if (a < 0) {
            a += M_PI;
        } else {
            a -= M_PI;
        }
    }

    return a;
}

inline double yaw_difference(const rst::geometry::PoseEuler &a, const rst::geometry::PoseEuler &b) {
    double diff = a.rotation().yaw() - b.rotation().yaw();

    return normalize_angle(diff);
}

// relative angle between a's yaw and the vector from a to b
inline double relative_angle(const rst::geometry::PoseEuler &a, const rst::geometry::PoseEuler &b) {
    double abs_angle = atan2(
                b.translation().y() - a.translation().y(),
                b.translation().x() - a.translation().x()
    );

    return normalize_angle(abs_angle - a.rotation().yaw());
}

void publishTargetPosition(double dist, double yaw_diff, double alpha) {
    // update the target pose (is in relative coordinates, x and yaw)
    rsb::Informer<rst::geometry::TargetPoseEuler>::DataPtr targetPose(new rst::geometry::TargetPoseEuler);

    // firstly check if we are close to the waypoint
    if (dist > xy_tolerance) {
        // nope, rotate towards it
        if (abs(alpha) > yaw_tolerance) {
            DEBUG_MSG("Rotating towards waypoint, alpha is " << alpha);
            targetPose->mutable_target_pose()->mutable_translation()->set_x(0);
            targetPose->mutable_target_pose()->mutable_rotation()->set_yaw(alpha);
        } else {
            DEBUG_MSG("Going straight to waypoint, dist is " << dist);
            // we're facing it, go straight, with a small rotation
            targetPose->mutable_target_pose()->mutable_translation()->set_x(dist);
            targetPose->mutable_target_pose()->mutable_rotation()->set_yaw(alpha);
        }
    } else {
        DEBUG_MSG("Close to waypoint, rotating by " << yaw_diff);
        // we're close to the waypoint, rotate towards the expected yaw
        targetPose->mutable_target_pose()->mutable_translation()->set_x(0);
        targetPose->mutable_target_pose()->mutable_rotation()->set_yaw(yaw_diff);
    }

    // set the other necessary fields
    targetPose->mutable_target_pose()->mutable_translation()->set_y(0);
    targetPose->mutable_target_pose()->mutable_translation()->set_z(0);
    targetPose->mutable_target_pose()->mutable_rotation()->set_pitch(0);
    targetPose->mutable_target_pose()->mutable_rotation()->set_roll(0);
    targetPose->set_target_time(10); // speed limitation is set on microcontroller

    // publish new target pose
    targetPoseInformer->publish(targetPose);
}

void publishTargetSpeed(const double dist, const double yaw_diff, double alpha) {
    rsb::Informer<vector<int> >::DataPtr data(new vector<int>(2, 0)); // 0: forward velocity in µm/s; 1: rotational velocity in µrad/s

    // firstly check if we are close to the waypoint
    if (dist > xy_tolerance) {
        // nope, rotate towards it
        if (abs(alpha) > yaw_tolerance) {
            DEBUG_MSG("Rotating towards waypoint, alpha is " << alpha);

            data->at(0) = 0;
            if (alpha < 0) { // add a bias to one side
                data->at(1) = -rotationalSpeed_um;
            } else {
                data->at(1) = rotationalSpeed_um;
            }

        } else {
            DEBUG_MSG("Going straight to waypoint, dist is " << dist);
            // we're facing it, go straight, with a small rotation
            data->at(0) = forwardSpeed_um;
            // with a little curve
            double time = (dist * 1e6) / forwardSpeed_um; // seconds
            double speed = (alpha * 1e6) / time; // um / s
            if (speed > rotationalSpeed_um) { // limit positive speed
                speed = rotationalSpeed_um;
            } else if (speed < -rotationalSpeed_um) { // limit negative speed
                speed = -rotationalSpeed_um;
            }
            data->at(1) = speed;
        }
    } else {
        DEBUG_MSG("Reached waypoint, rotating to final yaw by " << yaw_diff);
        // we're close to the waypoint, rotate towards the expected yaw
        data->at(0) = 0;
        if (yaw_diff < 0) {
            data->at(1) = rotationalSpeed_um;
        } else {
            data->at(1) = -rotationalSpeed_um;
        }
    }

    DEBUG_MSG("Sending target speed: " << data->at(0) << " " << data->at(1));
    targetSpeedInformer->publish(data);
}

// !!untested!!
void publishTwist(const double dist, const double yaw_diff) {
    rsb::Informer<rst::kinematics::Twist>::DataPtr data(new rst::kinematics::Twist);

    if (abs(yaw_diff) < yaw_tolerance) {
        // rotate
        if (yaw_diff < 0) {
            data->mutable_angular()->set_c(rotationalSpeed);
        } else {
            data->mutable_angular()->set_c(-rotationalSpeed);
        }
    } else {
        // go straight
        data->mutable_linear()->set_x(forwardSpeed);
        // with a little curve
        double time = (dist * 1e6) / forwardSpeed; // seconds
        float speed = (yaw_diff * 1e6) / time;
        if (speed > 0 && speed > rotationalSpeed) { // limit positive speed
            speed = rotationalSpeed;
        } else if (speed < 0 && speed < -rotationalSpeed) { // limit negative speed
            speed = -rotationalSpeed;
        }

        data->mutable_angular()->set_c(speed);
    }

    DEBUG_MSG("Sending Twist: " << data->DebugString());
    // add other required fields
    data->mutable_angular()->set_a(0);
    data->mutable_angular()->set_b(0);
    data->mutable_linear()->set_y(0);
    data->mutable_linear()->set_z(0);

    twistInformer->publish(data);
}

void stop() {
    switch (motorCommandMethod) {
    case setTargetSpeed: {
        rsb::Informer<vector<int> >::DataPtr data(new vector<int>(2, 0));
        targetSpeedInformer->publish(data);
        break;
    }

    case setTargetPosition:
        ERROR_MSG("setTargetPosition stop not implemented yet!");
        exit(EXIT_FAILURE);
        break;

    case Twist:
        ERROR_MSG("Twist stop not implemented yet!");
        exit(EXIT_FAILURE);
        break;
    }
}

/**
 * @brief updateMotorCommand Updates the motor command according to the given new pose.
 * @param pose
 */
void updateMotorCommand(const boost::shared_ptr<rst::geometry::PoseEuler> pose) {
    DEBUG_MSG("Updating motor command, current pose estimate:\n" << pose->DebugString());
    DEBUG_MSG("current waypoint: " << waypoints.front().DebugString());

    double dist = distance(*pose, waypoints.front());
    double yaw_diff = yaw_difference(*pose, waypoints.front());

    // check if waypoint is close
    if (dist < xy_tolerance) {

        // check if rotation is right too
        if (abs(yaw_diff) < yaw_tolerance) {
            INFO_MSG("reached waypoint:" << waypoints.front().DebugString());
            INFO_MSG("estimated pose: " << pose->DebugString());

            // log that the waypoint was reached
            if (logging) {
                poseLogStream << "Reached waypoint: " << waypoints.front().translation().x() << "," << waypoints.front().translation().y() << "," << waypoints.front().rotation().yaw() << endl;
                poseLogStream << "Current pose: " << pose->translation().x() << "," << pose->translation().y() << "," << pose->rotation().yaw() << endl;
            }

            // remove the reached waypoint
            waypoints.pop();

            // exit if necessary
            if (waypoints.empty()) {
                INFO_MSG("waypoint queue is empty, exiting");
                stop();
                exit(EXIT_SUCCESS);
            }
        }
    }

    double alpha = relative_angle(*pose, waypoints.front());

    switch (motorCommandMethod) {
    case setTargetPosition:
        publishTargetPosition(dist, yaw_diff, alpha);
        break;

    case setTargetSpeed:
        publishTargetSpeed(dist, yaw_diff, alpha);
        break;

    case Twist:
        publishTwist(dist, yaw_diff);
        break;
    }
}

twbTracking::proto::Object getObjectByTrackingId(const boost::shared_ptr<twbTracking::proto::ObjectList> &objectList) {
    // find the corresponding object in the list
    twbTracking::proto::Object object;
    object.set_id(0);

    // find by id
    bool foundObjectId = false;
    for (int objIdx = 0; objIdx < objectList->object_size(); ++objIdx) {
        if (objectList->object(objIdx).id() == trackingId) {
            foundObjectId = true;
            object = objectList->object(objIdx);
            break;
        }
    }

    if (!foundObjectId) {
        WARNING_MSG("Tracking id " << trackingId << " not found!");
        return object;
    }

    DEBUG_MSG("Found marker " << trackingId << ": "
              << object.position().translation().x() << " "
              << object.position().translation().y() << " "
              << object.position().rotation().z());

    if (object.position().translation().x() != object.position().translation().x()
            || object.position().translation().y() != object.position().translation().y()
            || object.position().rotation().z() != object.position().rotation().z()) {
        WARNING_MSG("Detected NaN, rejecting tracking data!");
        object.set_id(0);
        return object;
    }

    return object;
}

/**
 * @brief processTrackingData Selects the correct tracking data by tracking ID and forwards it to euler pose version of updateTargetPosition.
 * @param objectList
 */
void processTrackingData(const boost::shared_ptr<twbTracking::proto::ObjectList> objectList) {
    //DEBUG_MSG("Processing tracking data: " << objectList->DebugString());

    twbTracking::proto::Object object = getObjectByTrackingId(objectList);
    if (object.id() != trackingId) {
        return;
    }

    // now convert tracking data to Euler pose
    boost::shared_ptr<rst::geometry::PoseEuler> eulerPose(new rst::geometry::PoseEuler);
    eulerPose->mutable_translation()->set_x(object.position().translation().x());
    eulerPose->mutable_translation()->set_y(object.position().translation().y());
    eulerPose->mutable_rotation()->set_yaw(object.position().rotation().z() * (M_PI / 180.0f));

    updateMotorCommand(eulerPose);
}

int main(int argc, const char **argv) {

    /*******************
     * Program options *
     *******************/

    // programm options variables
    string poseInScope;
    string motorCmdOutScope;
    vector<float> waypointsArg;
    string motorCommandArg = motorCommandStrings[setTargetPosition];
    bool useTwbTracking = false;
    string twbSpreadHost = "alpia.techfak.uni-bielefeld.de";
    string twbSpreadPort = "4803";
    string poseLog = "";
    uint sleepTime = 100; // ms
    bool relative = false;

    namespace po = boost::program_options;

    // register them and add descriptions
    po::options_description options("Allowed options");
    options.add_options()("help,h", "Display a help message.")
            ("poseInScope",     po::value<string>(&poseInScope),                "Scope for receiving the current pose from a localization algorithm.")
            ("motorCmdOutScope",po::value<string>(&motorCmdOutScope)->required(),"Scope for publishing motor commands.")
            ("motorCommand",    po::value<string>(&motorCommandArg)->default_value(motorCommandArg), "How to send commands to the motor (setTargetSpeed, setTargetPosition or Twist).")
            ("trackingId",      po::value<uint>(&trackingId)->default_value(trackingId), "Enables tracking via TWB. The id of the tracking marker must be given.")
            ("twbSpreadHost",   po::value<string>(&twbSpreadHost)->default_value(twbSpreadHost), "Host for the RSB spread config when TWB tracking is used.")
            ("twbSpreadPort",   po::value<string>(&twbSpreadPort)->default_value(twbSpreadPort), "Port for the RSB spread config when TWB tracking is used.")
            ("poseLog",         po::value<string>(&poseLog),                                     "Path to a log file where the current pose and waypoint are logged, when latter is reached.")
            ("sleepTime",       po::value<uint>(&sleepTime)->default_value(sleepTime),           "The minimum time between checking for new pose estimates/tracking data (in ms).")
            ("forwardSpeed",    po::value<float>(&rotationalSpeed)->default_value(forwardSpeed),    "Speed of forward movement in m/s.")
            ("rotationSpeed",   po::value<float>(&rotationalSpeed)->default_value(rotationalSpeed), "Speed of rotation in rad/s.")
            ("yawTolerance",    po::value<double>(&yaw_tolerance)->default_value(yaw_tolerance),    "Below this threshold a waypoint is regarded as reached.")
            ("xyTolerance",     po::value<double>(&xy_tolerance)->default_value(xy_tolerance),      "Below this threshold a waypoint is regarded as reached.")
            ("waypoint",        po::value<vector<float> >(&waypointsArg)->multitoken()->composing()->required(),
                "A waypoint position, may be specified multiple times.")
            ("relative",        po::bool_switch(&relative), "waypoints are seen as relative to start position");

    po::variables_map vm;
    po::store(po::command_line_parser(argc, argv).options(options).style(po::command_line_style::unix_style ^ po::command_line_style::allow_short).run(), vm);

    // first, process the help option
    if (vm.count("help")) {
        cout << options << endl;
        exit(EXIT_FAILURE);
    }

    // afterwards, let program options handle argument errors
    po::notify(vm);

    // process scopes
    if (vm.count("trackingId") && trackingId > 0) {
        useTwbTracking = true;

        if (!vm.count("poseInScope")) {
            poseInScope = "/tracking/merger";
        }
    }

    INFO_MSG("poseInScope: " << poseInScope);
    INFO_MSG("motorCmdOutScope:" << motorCmdOutScope << std::flush);

    // check for logging
    if (!poseLog.empty()) {
        INFO_MSG("Pose will be locked to " << poseLog);
        logging = true;
        poseLogStream.open(poseLog);
        if (!poseLogStream.is_open()) {
            ERROR_MSG("Failed opening file!");
            exit(EXIT_FAILURE);
        }
    }

    // pre-compute velocities in um
    rotationalSpeed_um = rotationalSpeed * 1e6;
    forwardSpeed_um = forwardSpeed * 1e6;

    // pre-compute µs sleep time
    const uint sleepTime_us = sleepTime * 1e3; // from ms to µs

    // verify waypoint arguments
    if (waypointsArg.size() % 3 != 0) {
        ERROR_MSG("A waypoint must consist of three elements (x (m), y (m), yaw (rad))");
        exit(EXIT_FAILURE);
    }

    // split waypoints
    if (!relative) { // if we have relative waypoints we need the starting position first
        vector<float>::const_iterator iter = waypointsArg.begin();
        while (iter < waypointsArg.end()) {
            // copy vector to pose
            rst::geometry::PoseEuler waypoint;
            waypoint.mutable_translation()->set_x(*iter);
            waypoint.mutable_translation()->set_y(*(iter + 1));
            waypoint.mutable_rotation()->set_yaw(*(iter + 2));
            waypoints.push(waypoint);

            INFO_MSG("waypoint:\n" << waypoint.DebugString());

            iter += 3;
        }
    }

    // determine motor command method
    if (motorCommandArg.compare(motorCommandStrings[setTargetPosition]) == 0) {
        motorCommandMethod = setTargetPosition;
    } else if (motorCommandArg.compare(motorCommandStrings[setTargetSpeed]) == 0) {
        motorCommandMethod = setTargetSpeed;
    } else if (motorCommandArg.compare(motorCommandStrings[Twist]) == 0) {
        motorCommandMethod = Twist;
    } else {
        ERROR_MSG("motorCommand \"" << motorCommandArg << "\" not available!");
        exit(EXIT_FAILURE);
    }

    /*********************************
     * Setup RSB Listener & Informer *
     *********************************/

    using namespace rsb;
    using namespace rsb::converter;
    using namespace rsb::filter;
    using namespace rsb::util;
    using namespace rst::geometry;
    using namespace rsc;
    using namespace rsc::threading;
    using namespace muroxConverter;

    Factory &factory = getFactory();

    if (motorCommandMethod == setTargetPosition) {
        // setup rsb converter for target pose
        ProtocolBufferConverter<TargetPoseEuler>::Ptr converter(new converter::ProtocolBufferConverter<TargetPoseEuler>);
        converterRepository<string>()->registerConverter(converter);
    } else if (motorCommandMethod == setTargetSpeed) {
        // or target speed
        vecIntConverter::Ptr converter(new vecIntConverter);
        converterRepository<string>()->registerConverter(converter);
    } else if (motorCommandMethod == Twist) {
        ProtocolBufferConverter<rst::kinematics::Twist>::Ptr converter(new converter::ProtocolBufferConverter<rst::kinematics::Twist>);
        converterRepository<string>()->registerConverter(converter);
    }
    // and current pose
    ProtocolBufferConverter<PoseEuler>::Ptr poseConverter(new converter::ProtocolBufferConverter<PoseEuler>);
    converterRepository<string>()->registerConverter(poseConverter);

    // create the informers
    targetPoseInformer = factory.createInformer<TargetPoseEuler>(motorCmdOutScope);
    targetSpeedInformer = factory.createInformer<vector<int> >(motorCmdOutScope);
    twistInformer = factory.createInformer<rst::kinematics::Twist>(motorCmdOutScope);

    // listener for the current pose estimate
    // but first register the converter and setup the RSB config
    ParticipantConfig poseRSBConfig = factory.getDefaultParticipantConfig();
    if (useTwbTracking) {
        ProtocolBufferConverter<twbTracking::proto::ObjectList>::Ptr converter(new converter::ProtocolBufferConverter<twbTracking::proto::ObjectList>);
        converterRepository<string>()->registerConverter(converter);

        poseRSBConfig = getextspreadconfig(factory, twbSpreadHost, twbSpreadPort);
    }
    ListenerPtr poseListener = factory.createListener(poseInScope, poseRSBConfig);

    // and now add filters and handler for the specific type we use
    boost::shared_ptr<SynchronizedQueue<boost::shared_ptr<PoseEuler>>> poseQueue;
    boost::shared_ptr<SynchronizedQueue<boost::shared_ptr<twbTracking::proto::ObjectList>>> trackingQueue;

    if (useTwbTracking) {
        poseListener->addFilter(FilterPtr(TypeFilter::createForType<twbTracking::proto::ObjectList>()));
        trackingQueue = boost::make_shared<SynchronizedQueue<boost::shared_ptr<twbTracking::proto::ObjectList>>>(1);
        poseListener->addHandler(HandlerPtr(new QueuePushHandler<twbTracking::proto::ObjectList>(trackingQueue)));
    } else {
        poseListener->addFilter(FilterPtr(TypeFilter::createForType<PoseEuler>())); // only respect euler poses
        poseQueue = boost::make_shared<SynchronizedQueue<boost::shared_ptr<PoseEuler>>>(1);
        poseListener->addHandler(HandlerPtr(new QueuePushHandler<PoseEuler>(poseQueue)));
    }

    // calculate absolute waypoints from relative
    if (relative) {
        INFO_MSG("Using relative waypoints, waiting for first pose...");

        if (useTwbTracking) {
            ERROR_MSG("Relative positions for TWB tracking not implemented yet!");
            exit(EXIT_FAILURE);
        } else {
            boost::shared_ptr<PoseEuler> startPose = poseQueue->pop();
            INFO_MSG("Start pose is " << startPose->DebugString());

            vector<float>::const_iterator iter = waypointsArg.begin();
            while (iter < waypointsArg.end()) {
                // copy vector to pose
                rst::geometry::PoseEuler waypoint;
                waypoint.mutable_translation()->set_x(*iter + startPose->translation().x());
                waypoint.mutable_translation()->set_y(*(iter + 1) + startPose->translation().y());
                double yaw = normalize_angle(*(iter + 2) + startPose->rotation().yaw());
                waypoint.mutable_rotation()->set_yaw(yaw);
                waypoints.push(waypoint);

                INFO_MSG("waypoint:\n" << waypoint.DebugString());

                iter += 3;
            }
        }
    }

    // log start position
    if (::logging) {
        if (useTwbTracking) {
            INFO_MSG("Waiting for initial pose to log...");

            twbTracking::proto::Object object = getObjectByTrackingId(trackingQueue->pop());
            while (object.id() != trackingId) {
                object = getObjectByTrackingId(trackingQueue->pop());
            }

            poseLogStream << "Start pose: " << object.position().translation().x() << ","
                          << object.position().translation().y() << ","
                          << object.position().rotation().z() * (M_PI / 180.0f) << endl;

        } else {
            boost::shared_ptr<PoseEuler> startPose = poseQueue->pop();
            INFO_MSG("Start pose is " << startPose->DebugString());
            poseLogStream << "Start pose: " << startPose->translation().x() << "," << startPose->translation().y() << "," << startPose->rotation().yaw() << endl;
        }
    }

    /*************
     * Main loop *
     *************/
    rsc::misc::initSignalWaiter();

    while (rsc::misc::lastArrivedSignal() == rsc::misc::NO_SIGNAL) {
        if (useTwbTracking) {
            processTrackingData(trackingQueue->pop());
        } else {
            updateMotorCommand(poseQueue->pop());
        }

        usleep(sleepTime_us);
        //getchar();
    }

    return rsc::misc::suggestedExitCode(rsc::misc::lastArrivedSignal());
}
