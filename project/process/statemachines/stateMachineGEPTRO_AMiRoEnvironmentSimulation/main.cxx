//============================================================================
// Name        : main.cxx
// Author      : mbarther <mbarther@techfak.uni-bielefeld.de>
// Description : This is the state machine with opposite communication to
//               the final GEPTRO state machine.
//============================================================================

// Messages
#define INFO_MSG_
#define DEBUG_MSG_
#define SUCCESS_MSG_
#define WARNING_MSG_
#define ERROR_MSG_
#include <MSG.h>

// CAN
#include <ControllerAreaNetwork.h>
#include <Types.h>
#include <types/twbTracking.pb.h>
#include <Constants.h>
types::position homingPosition;
ControllerAreaNetwork myCAN;

// For running system comands
#include <stdlib.h>

// For using kbhit
#include <unistd.h>

// For checking character pressed in the console
#include <kbhit.hpp>

// For using vector
#include <vector>

// For using string
#include <string>

// For program options
#include <boost/program_options.hpp>

// RSB
#include <rsb/Informer.h>
#include <converter/matConverter/matConverter.hpp>
#include <converter/vecIntConverter/main.hpp>
#include <boost/shared_ptr.hpp>
#include <rsb/Factory.h>
#include <rsb/converter/Repository.h>
#include <rsb/Event.h>
#include <rsb/Handler.h>
#include <rsb/filter/OriginFilter.h>
#include <rsc/threading/SynchronizedQueue.h>
#include <rsb/util/QueuePushHandler.h>

// RST
#include <rsb/converter/Repository.h>
#include <rsb/converter/ProtocolBufferConverter.h>

// RST Proto types
#include <rst/geometry/Pose.pb.h>



using namespace boost;
using namespace std;
using std::vector;
using namespace cv;
using namespace rsb;
using namespace muroxConverter;
using namespace rsb::converter;

// State machine states
#define NUM_STATES 7
enum states {
	idle,
	exploration,
	blobDetection,
	localPlanner,
	objectDetection,
	objectDelivery,
	objectTransport
};

states amiroState = idle;
states amiroStateL = exploration;

std::string statesString[NUM_STATES] {
	"idle",
	"exploration",
	"blobDetection",
	"localPlanner",
	"objectDetection",
	"objectDelivery",
	"objectTransport"
};
	



// Objects && Object detection communication
#define NUM_OBJECTS 6
enum objects { object1 , object2 , object3 , object4 , object5 , object6 };
std::string objectsString[NUM_OBJECTS] = {"object1","object2","object3","object4","object5","object6"};

// Exploration
std::string sExplorationCmdScope("/exploration/answer");
std::string sExplorationAnswerScope("/exploration/command");

// Server Scope
std::string serverScope = "/mapGenerator";

// Delivery
std::string sDeliveryCmdScope("/objectDelivery/answer");
std::string sDeliveryAnswerScope("/objectDelivery/command");

// Object transport
std::string sTransportCmdScope("/objectTransport/answer");
std::string sTransportAnswerScope("/objectTransport/command");

// Object detection
std::string sObjectDetCmdScope("/objectDetection/detected");
std::string sObjectDetAnswerScope("/objectDetection/command");

// Local planner
std::string sLocalPlannerCmdScope("/localplanner/answer");
std::string sLocalPlannerAnswerScope("/localplanner/command");

// Communication with ToBI
std::string sOutScopeTobi = "/tobiamiro";
std::string sInScopeTobi = "/amiro";
std::string sInScopeTobi2nd = "tobi";

// Sensor scopes
std::string sInScopeLidar = "/lidar";
std::string sInScopeOdometry = "/odo";


// RSB content
std::string outputRSBExploration = "start";
std::string inputRSBExploration = "finish";
std::string outputRSBBlobDetection = "start";
std::string inputRSBBlobDetection = "finish";
std::string outputRSBObjectDetection = "COMP";
std::string inputRSBObjectDetection = "finish";
std::string outputRSBLocalPlanner = "start";
std::string inputRSBLocalPlanner = "finish";
std::string outputRSBDelivery = "start";
std::string inputRSBDelivery = "finish";
std::string outputRSBTransport = "start";
std::string inputRSBTransport = "finish";

// RSB input recognizer
bool rsbInputExploration = false;
bool rsbInputBlobDetection = false;
bool rsbInputObjectDetection = false;
bool rsbInputDelivery = false;
bool rsbInputTransport = false;
bool rsbInputLocalPlanner = false;

int processSM(void);

twbTracking::proto::Pose2D objectPos;

int objectCount = 0;
bool objectDetected[] = {false, false};

bool skipExplo = false;
bool skipLP = false;
bool skipDet = false;
bool skipTrans = false;
bool skipDeli = false;

// Object detection
double minimalAngle = 0;
double minimalValue = 99999;

/*
class objectsCallback: public LocalServer::Callback<bool, twbTracking::proto::Pose2DList> {
	boost::shared_ptr<twbTracking::proto::Pose2DList> call(const std::string&, boost::shared_ptr<bool> draw_debug) {

		boost::shared_ptr<twbTracking::proto::Pose2DList> pose2DList(new twbTracking::proto::Pose2DList);

		for(int i = 1; i <= 2; ++i) {
			// Add object as pose
			twbTracking::proto::Pose2D *pose2D1 = pose2DList->add_pose();
			pose2D1->set_x(i*0.2);
			pose2D1->set_y(i*0.2);
			pose2D1->set_orientation(i*5);
			pose2D1->set_id(i);
		}
		return pose2DList;
	}
};
*/

int main(int argc, char **argv) {
    namespace po = boost::program_options;

    po::options_description options("Allowed options");
    options.add_options()("help,h", "Display a help message.")
        ("skipExploration", "Skipping all parts for Exploration.")
        ("skipBlobbing", "Skipping all parts for Blob Detection.")
        ("skipLocalPlanner", "Skipping all parts for Local Planner.")
        ("skipDetection", "Skipping all parts for Object Detection.")
        ("skipTransport", "Skipping all parts for Object Transport.")
        ("skipDelivery", "Skipping all parts for Object Delivery.");


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

    // save skipping flags
    skipExplo = vm.count("skipExploration");
    skipLP = vm.count("skipLocalPlanner");
    skipDet = vm.count("skipDetection");
    skipTrans = vm.count("skipTransport");
    skipDeli = vm.count("skipDelivery");

    // print all scopes
    INFO_MSG("List of all RSB scopes:");
    INFO_MSG(" - Exploration cmd: " << sExplorationCmdScope);
    INFO_MSG(" - Exploration ans: " << sExplorationAnswerScope);
    INFO_MSG(" - Detection cmd:   " << sObjectDetCmdScope);
    INFO_MSG(" - Detection ans:   " << sObjectDetAnswerScope);
    INFO_MSG(" - Delivery cmd:    " << sDeliveryCmdScope);
    INFO_MSG(" - Delivery ans:    " << sDeliveryAnswerScope);
    INFO_MSG(" - Transport cmd:   " << sTransportCmdScope);
    INFO_MSG(" - Transport ans:   " << sTransportAnswerScope);

    // use camera stream for testing
    return processSM();
}


int processSM(void) {

    // Create the factory
    rsb::Factory &factory = rsb::getFactory();

    //////////////////// REGISTRATION OF OTHER TYPES //////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////

    // Register new converter for Pose2D
    boost::shared_ptr<rsb::converter::ProtocolBufferConverter<twbTracking::proto::Pose2D> > converterlocalization(new rsb::converter::ProtocolBufferConverter<twbTracking::proto::Pose2D>());
    rsb::converter::converterRepository<std::string>()->registerConverter(converterlocalization);

    /////////////////// LOCAL SCOPES //////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////

/*    // Object Detection: Listener and Informer
    rsb::ListenerPtr listenerObjectDetAnswerScope = factory.createListener(sObjectDetAnswerScope);
    boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<std::string> > > queueObjectDetAnswerScope(
            new rsc::threading::SynchronizedQueue<boost::shared_ptr<std::string> >(1));
    listenerObjectDetAnswerScope->addHandler(rsb::HandlerPtr(new rsb::QueuePushHandler<std::string>(queueObjectDetAnswerScope)));

    rsb::Informer< std::string >::Ptr informerObjectDetScope = factory.createInformer< std::string > (sObjectDetCmdScope);
*/
    // Local Planner: Listener and Informer
    rsb::ListenerPtr listenerLocalPlannerAnswerScope = factory.createListener(sLocalPlannerAnswerScope);
    boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<std::string> > > queueLocalPlannerAnswerScope(
            new rsc::threading::SynchronizedQueue<boost::shared_ptr<std::string> >(1));
    listenerLocalPlannerAnswerScope->addHandler(rsb::HandlerPtr(new rsb::util::QueuePushHandler<std::string>(queueLocalPlannerAnswerScope)));

    rsb::Informer< std::string >::Ptr informerLocalPlannerScope = factory.createInformer< std::string > (sLocalPlannerCmdScope);

    // Exploration: Listener and Informer
    rsb::ListenerPtr listenerExplorationScope = factory.createListener(sExplorationAnswerScope);
    boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<std::string> > > queueExplorationAnswerScope(
            new rsc::threading::SynchronizedQueue<boost::shared_ptr<std::string> >(1));
    listenerExplorationScope->addHandler(rsb::HandlerPtr(new rsb::util::QueuePushHandler<std::string>(queueExplorationAnswerScope)));

    rsb::Informer< std::string >::Ptr informerExplorationScope = factory.createInformer< std::string > (sExplorationCmdScope);

    // Object delivery: Listener and Informer
    rsb::ListenerPtr listenerDeliveryScope = factory.createListener(sDeliveryAnswerScope);
    boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<twbTracking::proto::Pose2D>>> queueDeliveryAnswerScope(
            new rsc::threading::SynchronizedQueue<boost::shared_ptr<twbTracking::proto::Pose2D>>(1));
    listenerDeliveryScope->addHandler(rsb::HandlerPtr(new rsb::util::QueuePushHandler<twbTracking::proto::Pose2D>(queueDeliveryAnswerScope)));

    rsb::Informer< std::string >::Ptr informerDeliveryScope = factory.createInformer< std::string > (sDeliveryCmdScope);

    // Object transport: Listener and Informer
    rsb::ListenerPtr listenerTransportScope = factory.createListener(sTransportAnswerScope);
    boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<std::string> > > queueTransportAnswerScope(
            new rsc::threading::SynchronizedQueue<boost::shared_ptr<std::string> >(1));
    listenerTransportScope->addHandler(rsb::HandlerPtr(new rsb::util::QueuePushHandler<std::string>(queueTransportAnswerScope)));

    rsb::Informer< std::string >::Ptr informerTransportScope = factory.createInformer< std::string > (sTransportCmdScope);

/*    /////////////////////// RSB SERVER ////////////////////////////////
    ///////////////////////////////////////////////////////////////////

    LocalServerPtr server = factory.createLocalServer(serverScope);

    // Register method with name and implementing callback object.
    server->registerMethod("getObjectsList", LocalServer::CallbackPtr(new objectsCallback()));
*/

    INFO_MSG("All RSB connections built. Starting statemachine now.");

    boost::shared_ptr<std::string> stringPublisher(new std::string);

    // run through statemachine
    bool runningStatemachine = true;
    while(runningStatemachine) {

        // Check RSB input first
        std::string sRSBInput = "";
        rsbInputExploration = false;
        rsbInputBlobDetection = false;
        rsbInputObjectDetection = false;
        rsbInputDelivery = false;
        rsbInputTransport = false;
        rsbInputLocalPlanner = false;

        // Check input from outside
        if (!skipExplo && !queueExplorationAnswerScope->empty()) {
            sRSBInput = *queueExplorationAnswerScope->pop();
            if (sRSBInput.compare(outputRSBExploration) == 0) {
                rsbInputExploration = true;
            }
        }
/*        if (!skipDet && !queueObjectDetAnswerScope->empty()) {
            sRSBInput = *queueObjectDetAnswerScope->pop();
            if (sRSBInput.compare(outputRSBObjectDetection) == 0) {
                rsbInputObjectDetection = true;
            }
        }*/
        if (!skipLP && !queueLocalPlannerAnswerScope->empty()) {
            sRSBInput = *queueLocalPlannerAnswerScope->pop();
            if (sRSBInput.compare(outputRSBLocalPlanner) == 0) {
                rsbInputLocalPlanner = true;
            }
        }
        if (!skipDeli && !queueDeliveryAnswerScope->empty()) {
            rsbInputDelivery = true;
            objectPos = *queueDeliveryAnswerScope->pop();
        }
        if (!skipTrans && !queueTransportAnswerScope->empty()) {
            sRSBInput = *queueTransportAnswerScope->pop();
            if (sRSBInput.compare(outputRSBTransport) == 0) {
                rsbInputTransport = true;
            }
        }

        if (amiroState != amiroStateL) {
            INFO_MSG("STATE: " << statesString[amiroState]);
        }
        amiroStateL = amiroState;

        switch (amiroState) {
            case idle:
                if (rsbInputExploration && !skipExplo) {
                    amiroState = exploration;
                } else if (rsbInputLocalPlanner && !skipLP) {
                    amiroState = localPlanner;
                } else if (rsbInputObjectDetection && !skipDet) {
                    amiroState = objectDetection;
                } else if (rsbInputDelivery && !skipDeli) {
                    amiroState = objectDelivery;
                } else if (rsbInputTransport && !skipTrans) {
                    amiroState = objectTransport;
                }
                break;
            case exploration:
                INFO_MSG("EXPLORING");
                sleep(3);
                *stringPublisher = inputRSBExploration;
                informerExplorationScope->publish(stringPublisher);
                amiroState = idle;
                break;
            case localPlanner:
                INFO_MSG("DRIVING");
                sleep(3);
                *stringPublisher = inputRSBLocalPlanner;
                informerLocalPlannerScope->publish(stringPublisher);
                amiroState = idle;
                break;
            case objectDetection:
/*                INFO_MSG("DETECTING");
                sleep(3);
                inputRSBObjectDetection = std::to_string(objectCount+3);
                objectCount++;
                *stringPublisher = inputRSBObjectDetection;
                informerObjectDetScope->publish(stringPublisher);*/
                amiroState = idle;
                break;
            case objectDelivery:
                INFO_MSG("DELIVERING of object at " << objectPos.x() << "/" << objectPos.y() << " with radius " << objectPos.orientation());
                sleep(3);
                *stringPublisher = inputRSBDelivery;
                informerDeliveryScope->publish(stringPublisher);
                amiroState = idle;
                break;
            case objectTransport:
                INFO_MSG("TRANSPORTING");
                sleep(3);
                *stringPublisher = inputRSBTransport;
                informerTransportScope->publish(stringPublisher);
                amiroState = idle;
                break;
            default:
                ERROR_MSG("Unknown state in statemachine!");
                return -1;
        }

        usleep(500000);

    }

    INFO_MSG("Statemachine has been closed.");

    return 0;
}
