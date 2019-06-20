//============================================================================
// Name        : main.cxx
// Author      : mbarther <mbarther@techfak.uni-bielefeld.de>
// Description : Statemachine of the GEPTRO table application.
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

#include <rst/geometry/Pose.pb.h>
#include <rst/geometry/Translation.pb.h>
#include <rst/geometry/Rotation.pb.h>
using namespace rst::geometry;

using namespace boost;
using namespace std;
using std::vector;
using namespace cv;
using namespace rsb;
using namespace muroxConverter;
using namespace rsb::converter;
using namespace rsb::patterns;

// State machine states
enum states {
	idle,
	init,
	explorationStart,
	exploration,
	blobDetection,
	objectDetectionStart,
	objectDetectionInit,
	objectDetectionMain,
	initDone,
	waiting,
	objectDeliveryStart,
	objectDelivery,
        objectDeliveryFinish,
	objectTransportStart,
	objectTransport,
	objectTransportFinish
};

states amiroState = idle;
states amiroStateL = init;

std::string statesString[] {
	"idle",
	"initialization",
	"starting exploration",
	"exploration",
	"blob detection",
	"starting object detection",
	"init object detection",
	"object detection",
	"initialization done",
	"waiting",
	"starting object delivery",
	"object delivery",
        "object delivery finished",
	"starting object transport",
	"object transport",
        "object transport finished"
};
	



// Objects && Object detection communication
#define NUM_OBJECTS 6
enum objects { object1 , object2 , object3 , object4 , object5 , object6 };
std::string objectsString[NUM_OBJECTS] = {"object1","object2","object3","object4","object5","object6"};

// RSB informer
rsb::Informer<twbTracking::proto::Pose2DList>::Ptr informerObjectDetScope;
rsb::Informer<twbTracking::proto::Pose2DList>::Ptr informerObjectDetRectScope;
rsb::Informer<std::string>::Ptr                    informerLocalPlannerScope;
rsb::Informer<std::string>::Ptr                    informerBlobDetectionScope;
rsb::Informer<std::string>::Ptr                    informerExplorationScope;
rsb::Informer<std::string>::Ptr                    informerPositionScope;
rsb::Informer<twbTracking::proto::Pose2D>::Ptr     informerDeliveryScope;
rsb::Informer<std::string>::Ptr                    informerTransportScope;
rsb::Informer<std::string>::Ptr                    informerOutsideScope;

// Localization
//std::string mapServerScope = "/CoreSlamServer";
std::string mapServerScope = "/mapGenerator";

// Exploration
std::string sExplorationCmdScope("/exploration/command");
std::string sExplorationAnswerScope("/exploration/answer");
std::string positionOutscope = "/rectposition/command";
std::string positionInscope = "/rectposition/answer";

// Delivery
std::string sDeliveryCmdScope("/objectDelivery/command");
std::string sDeliveryAnswerScope("/objectDelivery/answer");

// Object transport
std::string sTransportCmdScope("/objectTransport/command");
std::string sTransportAnswerScope("/objectTransport/answer");

// Object detection
std::string sObjectDetCmdScope("/objectDetectionMain/command");
std::string sObjectDetAnswerScope("/objectDetectionMain/answer");
std::string sObjectDetRectOutscope = "/rectangledata";

// Local planner
std::string sLocalPlannerCmdScope("/localplanner/command");
std::string sLocalPlannerAnswerScope("/localplanner/answer");

// Blob Detection
std::string sBlobDetectionCmdScope("/objectsReq/request");
std::string sBlobDetectionAnswerScope("/objectsReq/answer");

// Communication with ToBI
std::string sInScopeTobi = "/tobiamiro";
std::string sOutScopeTobi = "/amiro";
std::string sOutScopeTobi2nd = "tobi";

// Output for debugging
std::string sOutScopeState = "/amiroState";


// RSB content
std::string outputRSBOutsideInitDone = "initdone";
std::string outputRSBOutsideObjectDet = "object";
std::string outputRSBOutsideDeliveryAPP = "finish";
std::string outputRSBOutsideTransportAPP = "finish";
std::string inputRSBOutsideInit = "init";
std::string inputRSBOutsideDelivery = "object";
std::string inputRSBOutsideTransport = "transport";
std::string outputRSBExploration = "start";
std::string inputRSBExploration = "finish";
std::string outputRSBObjectDetection = "COMP";
std::string inputRSBObjectDetection = "finish";
std::string outputRSBLocalPlanner = "start";
std::string inputRSBLocalPlanner = "finish";
std::string outputRSBDelivery = "start";
std::string inputRSBDelivery = "finish";
std::string outputRSBTransport = "start";
std::string inputRSBTransport = "finish";

std::string obstacleServerReq = "getObjectsList";

// publisher variables
boost::shared_ptr<std::string> stringPublisher(new std::string);
boost::shared_ptr<twbTracking::proto::Pose2D> positionPublisher(new twbTracking::proto::Pose2D());
boost::shared_ptr<twbTracking::proto::Pose2DList> objects(new twbTracking::proto::Pose2DList());

// RSB input recognizer
bool rsbInputOutsideInit = false;
bool rsbInputOutsideDeliver = false;
bool rsbInputOutsideTransport = false;
bool rsbInputExploration = false;
bool rsbInputDelivery = false;
bool rsbInputTransport = false;
bool rsbInputObjectDetection = false;
bool rsbInputLocalPlanner = false;

// RSB output resend flags
bool rsbRecObjectDet[] = {false, false, false, false, false, false};
bool rsbRecAllObjects = false;
bool rsbRecObjectDelivery = false;
bool rsbRecTransport = false;

int objectCount = 0;
const int objectCountMax = 6;
int objectOffsetForToBI = 2;
bool objectDetected[] = {false, false, false, false, false, false};
boost::shared_ptr<twbTracking::proto::Pose2D> objectDetectionAnswer;
float objectPos[objectCountMax][3];

int deliverObjectId = 0;

int robotID = 0;
std::string colorInit = "";
bool blinkerRight = false;
amiro::Color mainColor = amiro::Color::WHITE;

std::string sRemoteServerPort = "4823";
std::string sRemoteServer = "localhost";

bool testWithAnswerer = false;
bool mapServerIsRemote = false;
bool isBigMap = false;

boost::shared_ptr<twbTracking::proto::Pose2DList> objectPosList(new twbTracking::proto::Pose2DList());

// functions
bool readInitInput(std::string inputData);
bool readRecObjectDetection(std::string inputData);
bool readDeliveryFinished(std::string inputData);
bool readTransportFinished(std::string inputData);
void setMainLight(void);
void setLightcolor(void);
void idleBlink(void);
int processSM(void);
int ssmObjectDetection(void);

int main(int argc, char **argv) {
    namespace po = boost::program_options;

    po::options_description options("Allowed options");
    options.add_options()("help,h", "Display a help message.")
        ("spread,s", po::value <std::string> (&sRemoteServer), "IP of remote spread server.")
        ("spreadPort,p", po::value <std::string> (&sRemoteServerPort), "Port of remote spread server.")
        ("outscopeTobi,o", po::value <std::string> (&sOutScopeTobi), "Scope for sending the current state to tobi.")
        ("outscopeState,s", po::value <std::string> (&sOutScopeState), "Scope for sending the current state internaly.")
        ("inscopeTobi,i", po::value <std::string> (&sInScopeTobi), "Scope for recieving Tobis messages.")
        ("mapServerScope", po::value <std::string> (&mapServerScope), "Scope for map server.")
        ("obstacleServerReq", po::value <std::string> (&obstacleServerReq), "Function name of map server for obstacle map.")
        ("sObjectDetRectOutscope", po::value <std::string> (&sObjectDetRectOutscope), "Scope for sending rectangle data.")
        ("robotID,d", po::value <int> (&robotID), "Robot ID.")
        ("objectOffset", po::value <int> (&objectOffsetForToBI), "Object offset for ToBI (default: 2).")
	("bigMap", "Flag if the map is very big (the pathplanner needs more than 25 seconds).")
        ("mapServerIsRemote", "Flag, if the map server is a remote server (otherwise it uses local connection).")
        ("testWithAnswerer", "Prepares some constants for test with answerer.");

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

    if (objectOffsetForToBI < 0) {
        objectOffsetForToBI = 0;
    }

    testWithAnswerer = vm.count("testWithAnswerer");
    mapServerIsRemote = vm.count("mapServerIsRemote");
    isBigMap = vm.count("bigMap");

    // prepare scopes for ToBI-AMIRo communication
    sInScopeTobi.append(std::to_string(robotID));
    sOutScopeTobi.append(std::to_string(robotID)).append(sOutScopeTobi2nd);
    if (vm.count("robotID")) {
        mapServerScope.append(std::to_string(robotID));
    }

    // print all scopes
    INFO_MSG("List of all RSB scopes:");
    INFO_MSG(" - ToBI to AMiRo:   " << sInScopeTobi);
    INFO_MSG(" - AMiRo to ToBI:   " << sOutScopeTobi);
    INFO_MSG(" - State debugging: " << sOutScopeState);
    INFO_MSG(" - Exploration cmd: " << sExplorationCmdScope);
    INFO_MSG(" - Exploration ans: " << sExplorationAnswerScope);
    INFO_MSG(" - Detection cmd:   " << sObjectDetCmdScope);
    INFO_MSG(" - Detection ans:   " << sObjectDetAnswerScope);
    INFO_MSG(" - Delivery cmd:    " << sDeliveryCmdScope);
    INFO_MSG(" - Delivery ans:    " << sDeliveryAnswerScope);
    INFO_MSG(" - Transport cmd:   " << sTransportCmdScope);
    INFO_MSG(" - Transport ans:   " << sTransportAnswerScope);

    // reset odometry
    types::position robotPosition;
    robotPosition.x = 0;
    robotPosition.y = 0;
    robotPosition.f_z = 0;
    myCAN.setOdometry(robotPosition);

    // do statemachine now
    return processSM();
}


int processSM(void) {

    // Create the factory
    rsb::Factory &factory = rsb::getFactory();

    //////////////////// REGISTRATION OF OTHER TYPES //////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////

    // Register converter for the pose list
    boost::shared_ptr<rsb::converter::ProtocolBufferConverter<twbTracking::proto::Pose2DList> > pose2DListConverter(new rsb::converter::ProtocolBufferConverter<twbTracking::proto::Pose2DList>());
    rsb::converter::converterRepository<std::string>()->registerConverter(pose2DListConverter);

    // Register new converter for Pose2D
    boost::shared_ptr<rsb::converter::ProtocolBufferConverter<twbTracking::proto::Pose2D> > converterlocalization(new rsb::converter::ProtocolBufferConverter<twbTracking::proto::Pose2D>());
    rsb::converter::converterRepository<std::string>()->registerConverter(converterlocalization);


    //////////////////// CREATE A CONFIG TO COMMUNICATE WITH ANOTHER SERVER ////////
    ///////////////////////////////////////////////////////////////////////////////
        // Get the global participant config as a template
        rsb::ParticipantConfig tmpPartConf = factory.getDefaultParticipantConfig();
              {
                // Get the options for socket transport, because we want to change them
                rsc::runtime::Properties tmpProp  = tmpPartConf.mutableTransport("socket").getOptions();

                // disable socket transport
                std::string enabled = "0";
                tmpProp["enabled"] = boost::any(enabled);

                // Write the socket tranport properties back to the participant config
                tmpPartConf.mutableTransport("socket").setOptions(tmpProp);
              }
              {
                // Get the options for spread transport, because we want to change them
                rsc::runtime::Properties tmpPropSpread = tmpPartConf.mutableTransport("spread").getOptions();

                // enable socket transport
                std::string enabled = "1";
                tmpPropSpread["enabled"] = boost::any(enabled);

                // the port of the server
                tmpPropSpread["port"] = boost::any(sRemoteServerPort);

                // Change the server
                tmpPropSpread["host"] = boost::any(sRemoteServer);

                // Write the tranport properties back to the participant config
                tmpPartConf.mutableTransport("spread").setOptions(tmpPropSpread);

                INFO_MSG("Remote Spread Configuration done: " << tmpPropSpread);
              }
    ///////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////

    /////////////////// LOCAL SCOPES///////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////

    // Object Detection: Listener and Informer
    rsb::ListenerPtr listenerObjectDetAnswerScope = factory.createListener(sObjectDetAnswerScope);
    boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<twbTracking::proto::Pose2D> > > queueObjectDetAnswerScope(
            new rsc::threading::SynchronizedQueue<boost::shared_ptr<twbTracking::proto::Pose2D> >(1));
    listenerObjectDetAnswerScope->addHandler(rsb::HandlerPtr(new rsb::util::QueuePushHandler<twbTracking::proto::Pose2D>(queueObjectDetAnswerScope)));

    informerObjectDetScope = factory.createInformer<twbTracking::proto::Pose2DList> (sObjectDetCmdScope);
    informerObjectDetRectScope = factory.createInformer<twbTracking::proto::Pose2DList> (sObjectDetRectOutscope);
//    informerObjectDetScope = factory.createInformer< std::string > (sObjectDetCmdScope);

    // Local Planner: Listener and Informer
    rsb::ListenerPtr listenerLocalPlannerAnswerScope = factory.createListener(sLocalPlannerAnswerScope);
    boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<std::string> > > queueLocalPlannerAnswerScope(
            new rsc::threading::SynchronizedQueue<boost::shared_ptr<std::string> >(1));
    listenerLocalPlannerAnswerScope->addHandler(rsb::HandlerPtr(new rsb::util::QueuePushHandler<std::string>(queueLocalPlannerAnswerScope)));

    informerLocalPlannerScope = factory.createInformer< std::string > (sLocalPlannerCmdScope);

    // Blob Detection: Listener and Informer
    rsb::ListenerPtr listenerBlobDetectionAnswerScope = factory.createListener(sBlobDetectionAnswerScope);
    boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<std::string> > > queueBlobDetectionAnswerScope(
            new rsc::threading::SynchronizedQueue<boost::shared_ptr<std::string> >(1));
    listenerBlobDetectionAnswerScope->addHandler(rsb::HandlerPtr(new rsb::util::QueuePushHandler<std::string>(queueBlobDetectionAnswerScope)));

    informerBlobDetectionScope = factory.createInformer< std::string > (sBlobDetectionCmdScope);

    // Exploration: Listener and Informer
    rsb::ListenerPtr listenerExplorationScope = factory.createListener(sExplorationAnswerScope);
    boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<std::string> > > queueExplorationAnswerScope(
            new rsc::threading::SynchronizedQueue<boost::shared_ptr<std::string> >(1));
    listenerExplorationScope->addHandler(rsb::HandlerPtr(new rsb::util::QueuePushHandler<std::string>(queueExplorationAnswerScope)));

    informerExplorationScope = factory.createInformer< std::string > (sExplorationCmdScope);

    rsb::ListenerPtr positionListener = factory.createListener(positionInscope);
    boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<twbTracking::proto::Pose2DList>>>positionQueue(new rsc::threading::SynchronizedQueue<boost::shared_ptr<twbTracking::proto::Pose2DList>>(1));
    positionListener->addHandler(rsb::HandlerPtr(new rsb::util::QueuePushHandler<twbTracking::proto::Pose2DList>(positionQueue)));

    informerPositionScope = factory.createInformer<std::string>(positionOutscope);

    // Object delivery: Listener and Informer
    rsb::ListenerPtr listenerDeliveryScope = factory.createListener(sDeliveryAnswerScope);
    boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<std::string> > > queueDeliveryAnswerScope(
            new rsc::threading::SynchronizedQueue<boost::shared_ptr<std::string> >(1));
    listenerDeliveryScope->addHandler(rsb::HandlerPtr(new rsb::util::QueuePushHandler<std::string>(queueDeliveryAnswerScope)));

    informerDeliveryScope = factory.createInformer<twbTracking::proto::Pose2D> (sDeliveryCmdScope);

    // Object transport: Listener and Informer
    rsb::ListenerPtr listenerTransportScope = factory.createListener(sTransportAnswerScope);
    boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<std::string> > > queueTransportAnswerScope(
            new rsc::threading::SynchronizedQueue<boost::shared_ptr<std::string> >(1));
    listenerTransportScope->addHandler(rsb::HandlerPtr(new rsb::util::QueuePushHandler<std::string>(queueTransportAnswerScope)));

    informerTransportScope = factory.createInformer< std::string > (sTransportCmdScope);


    /////////////////// REMOTE SCOPES///////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////

    rsb::ListenerPtr listenerOutsideScope;
    boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<std::string> > > queueOutsideScope(
            new rsc::threading::SynchronizedQueue<boost::shared_ptr<std::string> >(1));

    try {
        listenerOutsideScope = factory.createListener(sInScopeTobi, tmpPartConf);
        listenerOutsideScope->addHandler(rsb::HandlerPtr(new rsb::util::QueuePushHandler<std::string>(queueOutsideScope)));

        informerOutsideScope = factory.createInformer< std::string > (sOutScopeTobi, tmpPartConf);
    }
    catch(std::exception& e) {
        ERROR_MSG("Remote connection not established");
        return -1;
    }

    /////////////////// SERVER REQUESTS ///////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////

    // map server
    RemoteServerPtr mapServer;
    if (mapServerIsRemote) {
        mapServer = factory.createRemoteServer(mapServerScope, tmpPartConf, tmpPartConf);
    } else {
        mapServer = factory.createRemoteServer(mapServerScope);
    }

    INFO_MSG("All RSB connections built. Starting statemachine now.");

    // run through statemachine
    bool runningStatemachine = true;
    while(runningStatemachine) {

        // Check RSB input first
        std::string sRSBInput = "";
        rsbInputOutsideInit = false;
        rsbInputOutsideDeliver = false;
        rsbInputOutsideTransport = false;
        rsbInputExploration = false;
        rsbInputObjectDetection = false;
        rsbInputDelivery = false;
        rsbInputTransport = false;
        rsbInputLocalPlanner = false;

        rsbRecObjectDelivery = false;
        rsbRecTransport = false;

        // Check input from outside
        if (!queueOutsideScope->empty()) {
            sRSBInput = *queueOutsideScope->pop();
            if (readInitInput(sRSBInput)) {
                rsbInputOutsideInit = true;
            } else if (sRSBInput.compare(inputRSBOutsideTransport) == 0) {
                rsbInputOutsideTransport = true;
            } else {
                std::string mainPart;
                mainPart.append(sRSBInput, 0, inputRSBOutsideDelivery.size());
                rsbRecObjectDelivery = readDeliveryFinished(sRSBInput);
                rsbRecTransport = readTransportFinished(sRSBInput);
                if (!readRecObjectDetection(sRSBInput) && !rsbRecObjectDelivery
                    && !rsbRecTransport && mainPart.compare(inputRSBOutsideDelivery) == 0) {
                    rsbInputOutsideDeliver = true;
                    std::string sNum = "";
                    sNum.append(sRSBInput, mainPart.size(), sRSBInput.size());
                    deliverObjectId = std::stoi(sNum);
                }
            }
        }
        if (!queueExplorationAnswerScope->empty()) {
            sRSBInput = *queueExplorationAnswerScope->pop();
            if (sRSBInput.compare(inputRSBExploration) == 0) {
                rsbInputExploration = true;
            }
        }
        if (!queueObjectDetAnswerScope->empty()) {
            objectDetectionAnswer = queueObjectDetAnswerScope->pop();
            rsbInputObjectDetection = true;
        }
        if (!queueLocalPlannerAnswerScope->empty()) {
            sRSBInput = *queueLocalPlannerAnswerScope->pop();
            if (sRSBInput.compare(inputRSBLocalPlanner) == 0) {
                rsbInputLocalPlanner = true;
            }
        }
        if (!queueDeliveryAnswerScope->empty()) {
            sRSBInput = *queueDeliveryAnswerScope->pop();
            if (sRSBInput.compare(inputRSBDelivery) == 0) {
                rsbInputDelivery = true;
            }
        }
        if (!queueTransportAnswerScope->empty()) {
            sRSBInput = *queueTransportAnswerScope->pop();
            if (sRSBInput.compare(inputRSBTransport) == 0) {
                rsbInputTransport = true;
            }
        }

        // print actual state
        if (amiroState != amiroStateL) {
            INFO_MSG("STATE: " << statesString[amiroState]);
            amiroStateL = amiroState;
        }

        // check states
        switch (amiroState) {
            case idle:
                idleBlink();
                if (rsbInputOutsideInit) {
                    amiroState = init;
                }
                break;
            case init:
                // TODO initalization parts
                setLightcolor();
                setMainLight();
                *stringPublisher = inputRSBOutsideInit;
                informerOutsideScope->publish(stringPublisher);
                amiroState = explorationStart;
                break;
            case explorationStart:
                *stringPublisher = outputRSBExploration;
                informerExplorationScope->publish(stringPublisher);
                amiroState = exploration;
                break;
            case exploration:
                if (rsbInputExploration) {
                    //informerOutsideScope->publish(???);
                    setMainLight();
                    *stringPublisher = outputRSBExploration;
                    if (!positionQueue->empty()) {
                        positionQueue->pop();
                    }
                    informerPositionScope->publish(stringPublisher);
                    boost::shared_ptr<twbTracking::proto::Pose2DList> rectPositionsPtr = positionQueue->pop();
                    informerObjectDetRectScope->publish(rectPositionsPtr);
                    amiroState = blobDetection;
                }
                break;
            case blobDetection:
                if (!testWithAnswerer) {
                    boost::shared_ptr<bool> draw_debug(new bool(false));
                    if (isBigMap) {
                        if (!queueBlobDetectionAnswerScope->empty()) {
                            queueBlobDetectionAnswerScope->pop();
                        }
                        informerBlobDetectionScope->publish(stringPublisher);
                        while(queueBlobDetectionAnswerScope->empty()) {
                            INFO_MSG("Waiting for Blob Detection Answer.");
                            sleep(1);
                        }
                        queueBlobDetectionAnswerScope->pop();
                    }
                    objectPosList = mapServer->call<twbTracking::proto::Pose2DList>(obstacleServerReq, draw_debug);
                    INFO_MSG(" -> Object count of Blob Detection: " << objectPosList->pose_size());
                    amiroState = objectDetectionStart;
                } else {
/*                    for(int i = 0; i < 2; ++i) {
                        // Add object as pose
                        twbTracking::proto::Pose2D *pose2D1 = objectPosList->add_pose();
                        pose2D1->set_x(1.2893);
                        pose2D1->set_y();
                        pose2D1->set_orientation(i*0.05);
                        pose2D1->set_id(i);
                    }*/
                    objectPos[0][0] = 1.25638;
                    objectPos[0][1] = 1.42867;
                    objectPos[0][2] = 0.139431;
                    objectPos[1][0] = 1.2893;
                    objectPos[1][1] = 0.536911;
                    objectPos[1][2] = 0.137255;
                    objectDetected[0] = true;
                    objectDetected[1] = true;
                    amiroState = initDone;
		}
                break;
            case objectDetectionStart:
/*                if (objectCount > 0) {
                    positionPublisher->set_x(0);
                    positionPublisher->set_y(0);
                    positionPublisher->set_orientation(100000);
                    informerObjectDetScope->publish(positionPublisher);
                }*/
                informerObjectDetScope->publish(objectPosList);
                amiroState = objectDetectionInit;
                break;
            case objectDetectionInit:
                if (rsbInputObjectDetection) {
                    objectCount = objectDetectionAnswer->id();
                    INFO_MSG(" -> Object count set to " << objectCount);
                    amiroState = objectDetectionMain;
                }
                break;
            case objectDetectionMain:
                rsbRecAllObjects = true;
                for (int objIdx=0; objIdx<objectCountMax; objIdx++) {
                    if (objectDetected[objIdx] && !rsbRecObjectDet[objIdx]) {
//                        INFO_MSG("Object " << objIdx << " detected, but not recognized yet.");
                        rsbRecAllObjects = false;
                        std::string objOutput = "";
                        objOutput.append(outputRSBOutsideObjectDet).append(std::to_string(objIdx+1+objectOffsetForToBI));
                        *stringPublisher = objOutput;
                        informerOutsideScope->publish(stringPublisher);
                    }
                }
                if (rsbInputObjectDetection) {
                    // TODO check if object has been detected and reduce #object afterwards
//                    if (testWithAnswerer) {
//                        objectCount--;
//                    } else 
//                    if (objectDetectionAnswer.compare("null") == 0) {
//                        WARNING_MSG(" -> Doesn't know the object.");
//                    } else {
                        objectCount--;
                        INFO_MSG(" -> Object " << (objectDetectionAnswer->id()+objectOffsetForToBI) << " found! " << objectCount << " objects left.");
                        objectDetected[objectDetectionAnswer->id()-1] = true;
                        objectPos[objectDetectionAnswer->id()-1][0] = objectDetectionAnswer->x();
                        objectPos[objectDetectionAnswer->id()-1][1] = objectDetectionAnswer->y();
                        objectPos[objectDetectionAnswer->id()-1][2] = objectDetectionAnswer->orientation();
                        std::string objOutput = "";
                        objOutput.append(outputRSBOutsideObjectDet).append(std::to_string(objectDetectionAnswer->id()+objectOffsetForToBI));
                        *stringPublisher = objOutput;
                        informerOutsideScope->publish(stringPublisher);
//                        amiroState = objectDetectionStart;
//                    }
                }
                if (objectCount > 0) {
//                    ssmObjectDetection();
                } else if (rsbRecAllObjects) {
                    amiroState = initDone;
                }
                break;
            case initDone:
                setMainLight();
                *stringPublisher = outputRSBOutsideInitDone;
                informerOutsideScope->publish(stringPublisher);
                amiroState = waiting;
                break;
            case waiting:
                if (rsbInputOutsideDeliver) {
                    amiroState = objectDeliveryStart;
                } else if (rsbInputOutsideTransport) {
                    amiroState = objectTransportStart;
                }
                break;
            case objectDeliveryStart:
                INFO_MSG(" -> Delivering object " << deliverObjectId);
                if (deliverObjectId > objectOffsetForToBI && objectDetected[deliverObjectId-objectOffsetForToBI-1]) {
                    positionPublisher->set_x(objectPos[deliverObjectId-objectOffsetForToBI-1][0]);
                    positionPublisher->set_y(objectPos[deliverObjectId-objectOffsetForToBI-1][1]);
                    positionPublisher->set_orientation(objectPos[deliverObjectId-objectOffsetForToBI-1][2]);
                    informerDeliveryScope->publish(positionPublisher);
                    amiroState = objectDelivery;
                } else {
                    ERROR_MSG("Object " << deliverObjectId << " hasn't been found by the robot yet!");
                    amiroState = waiting;
                }
                break;
            case objectDelivery:
                if (rsbInputOutsideDeliver) {
                    std::string sOutput = "";
                    sOutput.append(inputRSBOutsideDelivery).append(std::to_string(deliverObjectId)).append("start");
                    *stringPublisher = sOutput;
                    informerOutsideScope->publish(stringPublisher);
                }
                if (rsbInputDelivery) {
                    amiroState = objectDeliveryFinish;
                }
                break;
            case objectDeliveryFinish:
                if (rsbRecObjectDelivery) {
                    setMainLight();
                    amiroState = waiting;
                } else {
                    std::string sOutput = "";
                    sOutput.append(inputRSBOutsideDelivery).append(std::to_string(deliverObjectId)).append(outputRSBOutsideDeliveryAPP);
                    *stringPublisher = sOutput;
                    informerOutsideScope->publish(stringPublisher);
                }
                break;
            case objectTransportStart:
                *stringPublisher = outputRSBTransport;
                informerTransportScope->publish(stringPublisher);
                amiroState = objectTransport;
                break;
            case objectTransport:
                if (rsbInputOutsideTransport) {
                    std::string sOutput = "";
                    sOutput.append(inputRSBOutsideTransport).append("start");
                    *stringPublisher = sOutput;
                    informerOutsideScope->publish(stringPublisher);  
                }
                if (rsbInputTransport && !rsbRecTransport) {
                    amiroState = objectTransportFinish;
                }
                break;
            case objectTransportFinish:
                if (rsbRecTransport) {
                    setMainLight();
                    amiroState = waiting;
                } else {
                    std::string sOutput = "";
                    sOutput.append(inputRSBOutsideTransport).append(outputRSBOutsideTransportAPP);
                    *stringPublisher = sOutput;
                    informerOutsideScope->publish(stringPublisher);
                }
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

bool readInitInput(std::string inputData) {
    int expectedLength = inputRSBOutsideInit.size();
    if (inputData.size() < expectedLength) {
        return false;
    }
    std::string justInit;
    justInit.append(inputData, 0, expectedLength);
    if (justInit.compare(inputRSBOutsideInit) == 0) {
        colorInit = "";
    	colorInit.append(inputData, expectedLength, inputData.size()-expectedLength);
        return true;
    } else {
        return false;
    }
}

bool readRecObjectDetection(std::string inputData) {
    if (inputData.size() > 9) {
        std::string mainPart;
        mainPart.append(inputData, 0, outputRSBOutsideObjectDet.size());
        std::string lastPart;
        lastPart.append(inputData, inputData.size()-3, 3);
        if (mainPart.compare(outputRSBOutsideObjectDet) == 0 && lastPart.compare("rec") == 0) {
            std::string sNum;
            sNum.append(inputData, outputRSBOutsideObjectDet.size(), inputData.size()-3-outputRSBOutsideObjectDet.size());
            int objNum = std::stoi(sNum);
            if (objNum > objectOffsetForToBI) {
                rsbRecObjectDet[objNum-objectOffsetForToBI-1] = true;
                return true;
            }
        }
    }
    return false;
}

bool readDeliveryFinished(std::string inputData) {
    std::string expected = "";
    expected.append(inputRSBOutsideDelivery).append(std::to_string(deliverObjectId)).append(outputRSBOutsideDeliveryAPP).append("rec");
    if (inputData.compare(expected) == 0) {
        return true;
    }
    return false;
}

bool readTransportFinished(std::string inputData) {
    std::string expected = "";
    expected.append(inputRSBOutsideTransport).append(outputRSBOutsideTransportAPP).append("rec");
    if (inputData.compare(expected) == 0) {
        return true;
    }
    return false;
}

void setMainLight(void) {
    for (int ledIdx=0; ledIdx<8; ledIdx++) {
        myCAN.setLightColor(ledIdx, mainColor);
    }
}

void setLightcolor(void) {
    if (robotID < colorInit.size()) {
        switch (colorInit[robotID]) {
            case 'r':
//                INFO_MSG("Chosen color is red.");
                mainColor = amiro::Color::RED;
                break;
            case 'b':
//                INFO_MSG("Chosen color is blue.");
                mainColor = amiro::Color::BLUE;
                break;
            case 'g':
//                INFO_MSG("Chosen color is green.");
                mainColor = amiro::Color::GREEN;
                break;
            case 'y':
//                INFO_MSG("Chosen color is yellow.");
                mainColor = amiro::Color::YELLOW;
                break;
            default:
                WARNING_MSG("Color '" << colorInit[robotID] << "' is unknown!");
                mainColor = amiro::Color::WHITE;
        }
    } else if (colorInit.size() > 0) {
        WARNING_MSG("For id " << robotID << " isn't any color defined, only for ids until " << colorInit.size()-1);
        mainColor = amiro::Color::WHITE;
    } else {
        WARNING_MSG("There aren't any colors defined.");
        mainColor = amiro::Color::WHITE;
    }
}

void idleBlink(void) {
    blinkerRight = !blinkerRight;
    for (int ledIdx=0; ledIdx<8; ledIdx++) {
        if (blinkerRight && ledIdx < 4 || !blinkerRight && ledIdx >= 4) {
            myCAN.setLightColor(ledIdx, amiro::Color(amiro::Color::WHITE));
        } else {
            myCAN.setLightColor(ledIdx, amiro::Color(amiro::Color::BLACK));
        }
    }
}


