//============================================================================
// Name        : main.cxx
// Author      : albauer <albauer@techfak.uni-bielefeld.de>,
//               mbarther <mbarther@techfak.uni-bielefeld.de>
// Description : Statemachine of the RoboCup@Home 2015 Final.
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
#include <Constants.h>
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
#include <rsb/converter/ProtocolBufferConverter.h>
#include <rsb/Event.h>
#include <rsb/Handler.h>
#include <rsb/filter/OriginFilter.h>
#include <rsc/threading/SynchronizedQueue.h>
#include <rsb/util/QueuePushHandler.h>
#include <rst/geometry/Pose.pb.h>

// IR sensor model
#include <sensorModels/VCNL4020Models.h>

using namespace boost;
using namespace std;
using std::vector;
using namespace cv;
using namespace rsb;
using namespace muroxConverter;
using namespace rsb::converter;

// State machine states
enum states                  { idle , turn, init , following , waypoint, stopped, homing, goToRoom, reachedroom, failedroom};
std::string statesString[] = {"idle","turn","init","following","waypoint","stopped","homing", "goToRoom", "reachedroom", "failedroom"};
states amiroState;

bool personEntered = false;

// Following of the leader
// We send a command "start" string  for start of the following
const std::string sFollowingCmdScope("/following");
const std::string sWaypointCmdScope("/waypoint/command");
const std::string sHomingCmdScope("/homing");
const std::string sHomingAnswerScope("/homing/response");
// We assume a "finish" string, which indicates the current state of the following program
const std::string sFollowingAnswerScope("/followingState");
const std::string sWaypointAnswerScope("/waypoint/state");
std::string poseScope("/amiro0/pose");

int processSM(void);
void setAMiRoColor(int r, int g, int b);
void set_state_idle();
void set_state_turn();
void set_state_homing();
void set_state_init();
void set_state_following();
void stop_following();
void set_state_waypoint();
void set_state_waypoint_entered();
void set_state_stopped();
void set_state_goToRoom();
void set_state_reachedroom();
void set_state_failedroom();
void stopWaypoint();
void motorActionMilli(int speed, int turn);

std::string g_sInScopeTobi = "/tobiamiro";
std::string g_sOutScopeStateTobi = "/amiro";
std::string g_sSubscopeState = "/state";
std::string g_sSubscopeWaypoint = "/waypoint";
std::string g_sRemoteServerPort = "4823";
std::string g_sRemoteServer = "localhost";
std::string amiro_id = "0";
int turn_degree = 180;
int turn_follow = 0;
int move_follow = 0;


float poseXThreshold = 0;
bool followPerson = true;

// IR triggering related
float irDetectionDist = 0.01; // m
std::string proxSensorInscopeObstacle = "/rir_prox/obstacle";

bool gotIRCommand(boost::shared_ptr<std::vector<int>> sensorValues, int sensorIdxStart = 0, int sensorIdxEnd = 7) {
    float distBefore = VCNL4020Models::obstacleModel(0, sensorValues->at(ringproximity::SENSOR_COUNT-1));
    for (int i=sensorIdxStart; i != (sensorIdxEnd + 1) % ringproximity::SENSOR_COUNT; i = (i + 1) % ringproximity::SENSOR_COUNT) {
        float distCur = VCNL4020Models::obstacleModel(0, sensorValues->at(i));
        //DEBUG_MSG("dist " << i << ": " << distCur);
        if (distCur < irDetectionDist && distBefore < irDetectionDist) {
            return true;
        }
        distBefore = distCur;
    }
    return false;
}

int main(int argc, char **argv) {
  namespace po = boost::program_options;

  po::options_description options("Allowed options");
  options.add_options()("help,h", "Display a help message.")
            ("spread,s", po::value<std::string> (&g_sRemoteServer), "IP of remote spread server.")
            ("spreadPort", po::value<std::string> (&g_sRemoteServerPort), "Port of remote spread server.")
            ("outscopeStateTobi", po::value<std::string> (&g_sOutScopeStateTobi), "Scope for sending the current state to Tobi.")
            ("inscopeTobi,i", po::value<std::string> (&g_sInScopeTobi), "Scope for receiving Tobi's messages.")
            ("id", po::value<std::string> (&amiro_id), "ID of AMiRo.")
        ("turn", po::value<int> (&turn_degree), "Angle to turn in turning state.")
        ("turnAfterFollow", po::value<int> (&turn_follow), "Angle to turn after following (in grad, positiv: counter clockwise).")
        ("moveAfterFollow", po::value<int> (&move_follow), "Distance to move after following after turning (in mm, positiv: forward).")
        ("proxObstacleInscope,o", po::value<std::string>(&proxSensorInscopeObstacle), "Inscope for receiving proximity sensor values for obstacle model.")
        ("irDetectionDist,i", po::value<float>(&irDetectionDist), "Maximal distance for command detection by the proximity sensors in m.")
        ("poseXThreshold", po::value<float>(&poseXThreshold), "poseXThreshold in meter")
        ("poseScope", po::value<std::string>(&poseScope), "Scope for receiving pose.");

  // Allow to give the value as a positional argument
  po::positional_options_description p;
  p.add("value", 1);

  po::variables_map vm;
  po::store(po::command_line_parser(argc, argv).options(options).positional(p).run(), vm);

  // First, process the help option
  if (vm.count("help")) {
    std::cout << options << "\n";
    exit(1);
  }

  // Afterwards, let program options handle argument errors
  po::notify(vm);

  return processSM();
}

int processSM(void) {

  // Build scopes
  g_sInScopeTobi.append(amiro_id);
  g_sOutScopeStateTobi.append(amiro_id).append("tobi");
  std::string g_sStateInScope = "";
  g_sStateInScope.append(g_sInScopeTobi).append(g_sSubscopeState);
  std::string g_sStateOutScope = "";
  g_sStateOutScope.append(g_sOutScopeStateTobi).append(g_sSubscopeState);
  std::string g_sWaypointOutScope = "";
  g_sWaypointOutScope.append(g_sOutScopeStateTobi).append(g_sSubscopeWaypoint);

  INFO_MSG("Scopes of remote RSB communication to ToBI:")
  INFO_MSG(" - Inscope for internal states: " << g_sStateInScope)
  INFO_MSG(" - Outscope for internal states: " << g_sStateOutScope)
  INFO_MSG(" - Outscope for waypoint status: " << g_sWaypointOutScope)
  INFO_MSG("Scopes of local communication:")
  INFO_MSG(" - Inscope for Following: " << sFollowingAnswerScope)
  INFO_MSG(" - Outscope for Following: " << sFollowingCmdScope)
  INFO_MSG(" - Inscope for Waypoint: " << sWaypointAnswerScope)
  INFO_MSG(" - Outscope for Waypoint: " << sWaypointCmdScope)

  // Create the factory
  rsb::Factory &factory = rsb::getFactory();

  //////////////////// CREATE A CONFIG TO COMMUNICATE WITH ANOTHER SERVER ////////
  ////////////////////////////////////////////////////////////////////////////////
  // Get the global participant config as a template
  rsb::ParticipantConfig tmpPartConf = factory.getDefaultParticipantConfig();
  {
    // Get the options for socket transport, because we want to change them
    rsc::runtime::Properties tmpProp  = tmpPartConf.mutableTransport("socket").getOptions();

    // Disable socket transport
    std::string enabled = "0";
    tmpProp["enabled"] = boost::any(enabled);

    // Write the socket tranport properties back to the participant config
    tmpPartConf.mutableTransport("socket").setOptions(tmpProp);
  }
  {
    // Get the options for spread transport, because we want to change them
    rsc::runtime::Properties tmpPropSpread = tmpPartConf.mutableTransport("spread").getOptions();

    // Enable socket transport
    std::string enabled = "1";
    tmpPropSpread["enabled"] = boost::any(enabled);

    // The port of the server
    tmpPropSpread["port"] = boost::any(g_sRemoteServerPort);

    // Change the server
    tmpPropSpread["host"] = boost::any(g_sRemoteServer);

    // Write the tranport properties back to the participant config
    tmpPartConf.mutableTransport("spread").setOptions(tmpPropSpread);

    std:: cout << tmpPropSpread << std::endl;
  }
  ///////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////

  // ------------ Converters ----------------------

  // Register new converter for std::vector<int>
  boost::shared_ptr<vecIntConverter> converterVecInt(new vecIntConverter());
  rsb::converter::converterRepository<std::string>()->registerConverter(converterVecInt);
  boost::shared_ptr< rsb::converter::ProtocolBufferConverter<rst::geometry::Pose> >
          converter(new rsb::converter::ProtocolBufferConverter<rst::geometry::Pose>());
  rsb::converter::converterRepository<std::string>()->registerConverter(converter);

  /////////////////// LOCAL SCOPES///////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////

  // Following: Listener and Informer
  rsb::ListenerPtr listenerFollowing = factory.createListener(sFollowingAnswerScope);
  boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<std::string> > > queueFollowingAnswer(
      new rsc::threading::SynchronizedQueue<boost::shared_ptr<std::string> >(1));
  listenerFollowing->addHandler(rsb::HandlerPtr(new rsb::util::QueuePushHandler<std::string>(queueFollowingAnswer)));

  rsb::Informer< std::string >::Ptr informerFollowing = factory.createInformer< std::string > (sFollowingCmdScope);

  // Waypoint: Listener and Informer
  rsb::ListenerPtr listenerWaypoint = factory.createListener(sWaypointAnswerScope);
  boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<std::string> > > queueWaypointAnswer(
      new rsc::threading::SynchronizedQueue<boost::shared_ptr<std::string> >(1));
  listenerWaypoint->addHandler(rsb::HandlerPtr(new rsb::util::QueuePushHandler<std::string>(queueWaypointAnswer)));

  rsb::Informer< std::string >::Ptr informerWaypoint = factory.createInformer< std::string > (sWaypointCmdScope);
  
  rsb::Informer< std::string >::Ptr informerHoming = factory.createInformer< std::string > (sHomingCmdScope);

  rsb::ListenerPtr listenerHoming = factory.createListener(sHomingAnswerScope);
  boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<std::string> > > queueHomingAnswer(
      new rsc::threading::SynchronizedQueue<boost::shared_ptr<std::string> >(1));
  listenerHoming->addHandler(rsb::HandlerPtr(new rsb::util::QueuePushHandler<std::string>(queueHomingAnswer)));

  // prepare RSB listener for the IR data (obstacles)
  rsb::ListenerPtr proxListenerObstacle = factory.createListener(proxSensorInscopeObstacle);
  boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<std::vector<int>>> >proxQueueObstacle(new rsc::threading::SynchronizedQueue<boost::shared_ptr<std::vector<int>>>(1));
  proxListenerObstacle->addHandler(rsb::HandlerPtr(new rsb::util::QueuePushHandler<std::vector<int>>(proxQueueObstacle)));

  /////////////////// REMOTE SCOPES///////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////

  // Prepare RSB informer and listener
  rsb::ListenerPtr listenerRemoteTobiState = factory.createListener(g_sStateInScope, tmpPartConf);
  boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<std::string> > > queueRemoteTobiState(
      new rsc::threading::SynchronizedQueue<boost::shared_ptr<std::string> >(20));
  listenerRemoteTobiState->addHandler(rsb::HandlerPtr(new rsb::util::QueuePushHandler<std::string>(queueRemoteTobiState)));

  rsb::Informer< std::string >::Ptr informerRemoteState = factory.createInformer< std::string > (g_sStateOutScope, tmpPartConf);

  rsb::Informer< std::string >::Ptr informerRemoteWaypoint = factory.createInformer< std::string > (g_sWaypointOutScope, tmpPartConf);

  // Listener for position
  INFO_MSG("listening for pose on " << poseScope);
  rsb::ListenerPtr listenerPosition = factory.createListener(poseScope, tmpPartConf);
  boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<rst::geometry::Pose> > > queuePosition(
      new rsc::threading::SynchronizedQueue<boost::shared_ptr<rst::geometry::Pose> >(1));
  listenerPosition->addHandler(rsb::HandlerPtr(new rsb::util::QueuePushHandler<rst::geometry::Pose>(queuePosition)));

/*  try {
    for (int idx = 0; idx < NUM_STATES; ++idx) {
      std::string sOutScopeStateTobiTmp(g_sOutScopeStateTobi);
      informerRemoteState[idx] = factory.createInformer< std::string > (sOutScopeStateTobiTmp.append("/").append(statesString[idx]), tmpPartConf);
    }

    // Create the listener for the standby task
    listenerRemoteTobiState = factory.createListener(g_sInScopeTobi, tmpPartConf);
    listenerRemoteTobiState->addHandler(rsb::HandlerPtr(new rsb::util::QueuePushHandler<std::string>(queueRemoteTobiState)));
  }
  catch(std::exception& e) {
    ERROR_MSG("Remote connection not established!");
    return -1;
  }*/

  // When we reached this point, everything should be fine

/*  // Local variables
  bool bGotFromTobi_something = false;
  bool bGotFromTobi_init = false;  // True if a state from tobi was received
  bool bGotFromTobi_initfollowing = false;
  bool bGotFromTobi_stopfollowing = false;
  bool bGotFromTobi_initwaypoint = false;
  bool bGotFromTobi_stopwaypoint = false;
  bool bGotFromTobi_turn = false;*/

//  // Variables for the keypress
//  // Check for keypress in window
//  int KB_codeCV = 0;
//  // Check for keypress in terminal
//  int KB_code = 0;

  boost::shared_ptr<std::string> signal_state(new std::string);
  boost::shared_ptr<std::string> stringPublisher(new std::string);
  boost::shared_ptr<std::string> signal_init(new std::string("init"));
  boost::shared_ptr<std::string> signal_stop(new std::string("stop"));
  boost::shared_ptr<std::string> signal_savePosition(new std::string("save"));
  boost::shared_ptr<std::string> signal_homingPosition(new std::string("homing"));
  boost::shared_ptr<std::string> signal_targetPosition(new std::string("target"));
  boost::shared_ptr<std::string> signal_abortPosition(new std::string("abort"));

  set_state_idle();

  // Process the behavior
  while (true) {

    // Local variables
    bool bGotFromTobi_something = false;
    bool bGotFromTobi_init = false;  // True if a state from tobi was received
    bool bGotFromTobi_initfollowing = false;
    bool bGotFromTobi_stopfollowing = false;
    bool bGotFromTobi_initwaypoint = false;
    bool bGotFromTobi_stopwaypoint = false;
    bool bGotFromTobi_turn = false;
    bool bGotFromTobi_savePosition = false;
    bool bGotFromTobi_homing = false;
    bool bGotFromTobi_gotoroom = false;

    bool bGotFromWaypoint_entered = false; // True if something entered the waypoint's area
    bool bGotFromWaypoint_left = false; // True if something left the waypoint's area

    bool gotFromIRgoToRoom = false;
    bool gotFromIRinitWaypoint = false;
    bool gotFromIRstopWaypoint = false;
    bool gotFromIRhoming = false;

    bool bGotFromTobi_reachedroomrec = false;
    bool bGotFromTobi_failedroomrec = false;

    // --- Check for incoming messaturn_degreeges/triggers/signals ---

    // Get messages from Tobi
    if (!queueRemoteTobiState->empty()) {
      bGotFromTobi_something = true;
      INFO_MSG("Received a state from Tobi.")
      std::string msg(*queueRemoteTobiState->pop());
      if (msg.compare("init") == 0) {
        INFO_MSG("Initialising...")
        bGotFromTobi_init = true;
      } else if (msg.compare("gotoroom") == 0) {
          INFO_MSG("Go to room");
          bGotFromTobi_gotoroom = true;
          std::string output = "gotoroomrec";
          *stringPublisher = output;
          informerRemoteState->publish(stringPublisher);
      } else if (msg.compare("initfollow")== 0) {
        INFO_MSG("Init following")
        bGotFromTobi_initfollowing = true;
        std::string output = "initfollowrec";
        *stringPublisher = output;
        informerRemoteState->publish(stringPublisher);
      } else if (msg.compare("stopfollow") == 0) {
        INFO_MSG("Stop following")
        // HACK Send the REC at the processing of the state
        bGotFromTobi_stopfollowing = true;
      } else if (msg.compare("initwaypoint") == 0) {
        INFO_MSG("Init waypoint")
        std::string output = "initwaypointrec";
        *stringPublisher = output;
        informerRemoteState->publish(stringPublisher);
        bGotFromTobi_initwaypoint = true;
      } else if (msg.compare("stopwaypoint") == 0) {
        INFO_MSG("Stop waypoint")
        bGotFromTobi_stopwaypoint = true;
        std::string output = "stopwaypointrec";
        *stringPublisher = output;
        informerRemoteState->publish(stringPublisher);
      } else if (msg.compare("turn") == 0) {
        INFO_MSG("Turn")
        bGotFromTobi_turn = true;
        std::string output = "turnrec";
        *stringPublisher = output;
        informerRemoteState->publish(stringPublisher);
      } else if (msg.compare("homing") == 0) {
        INFO_MSG("Homing")
        bGotFromTobi_homing = true;
        std::string output = "homingrec";
        *stringPublisher = output;
        informerRemoteState->publish(stringPublisher);
      } else if (msg.compare("reachedroomrec") == 0) {
        INFO_MSG("Reachedroomrec");
        bGotFromTobi_reachedroomrec = true;
      } else if (msg.compare("failedroomrec") == 0) {
        INFO_MSG("Failedroomrec");
        bGotFromTobi_failedroomrec = true;
      }

      if (!bGotFromTobi_reachedroomrec && !bGotFromTobi_failedroomrec) {
        queueRemoteTobiState->clear();
      }
    } else {
      bGotFromTobi_something = false;
      bGotFromTobi_init = false;
      bGotFromTobi_initfollowing = false;
      bGotFromTobi_stopfollowing = false;
      bGotFromTobi_initwaypoint = false;
      bGotFromTobi_stopwaypoint = false;
      bGotFromTobi_savePosition = false;
      bGotFromTobi_homing = false;
      bGotFromTobi_reachedroomrec = false;
      bGotFromTobi_failedroomrec = false;
    }
    // HACK Ignoring TURN commands, so that after a stopFollow, the turning is not initialized, but the homing to a save position
    bGotFromTobi_turn = false;


    if (!queuePosition->empty()) {
        rst::geometry::Pose pose = *(queuePosition->pop());

        DEBUG_MSG("X: " << pose.translation().x());
        if (pose.translation().x() > poseXThreshold) {
            bGotFromTobi_stopfollowing = true;
        }
    }

//    // Get messages from following
//    if (!queueFollowingAnswer->empty()) {
//      INFO_MSG("Received a message from following.")
//          std::string msg(*queueFollowingAnswer->pop());
//      if (msg.compare("undock") == 0) {
//        INFO_MSG("Following stopped.")
//            bGotFollowing_undock = true;
//      }
////      else if (msg.compare("false") == 0){
////        ERROR_MSG("Exploration failed")
////      }
//    } else {
//      bGotFollowing_undock = false;
//    }

    // check for IR sensor trigger
    gotFromIRgoToRoom = false;
    gotFromIRinitWaypoint = false;
    gotFromIRstopWaypoint = false;
    gotFromIRhoming = false;
    if (!proxQueueObstacle->empty()) {
        boost::shared_ptr<std::vector<int>> sensorValuesObstacle = boost::static_pointer_cast<std::vector<int>>(proxQueueObstacle->pop());

        // check side sensors
        if (gotIRCommand(sensorValuesObstacle, 5, 6) && gotIRCommand(sensorValuesObstacle, 1, 2)) {
            // abort navigation
            informerHoming->publish(signal_abortPosition);
        } else if (gotIRCommand(sensorValuesObstacle, 3, 4)) { // front sensors
            gotFromIRgoToRoom = true;
        } else if (gotIRCommand(sensorValuesObstacle, 5, 6)) { // right sensors
            gotFromIRinitWaypoint = true;
        } else if (gotIRCommand(sensorValuesObstacle, 7, 0)) { // check back sensors
            gotFromIRstopWaypoint = true;
        } else if (gotIRCommand(sensorValuesObstacle, 1, 2)) { // left sensors
            gotFromIRhoming = true;
        }
    }

    // Get messages from waypoint
    if (!queueWaypointAnswer->empty()) {
      INFO_MSG("Received a message from waypoint.")
        std::string msg(*queueWaypointAnswer->pop());
      if (msg.compare("entered") == 0) {
        INFO_MSG("Something entered the waypoint's area.")
            bGotFromWaypoint_entered = true;
      } else if (msg.compare("left") == 0) {
        INFO_MSG("Something left the waypoint's area.")
            bGotFromWaypoint_left = true;
      }
    } else {
      bGotFromWaypoint_entered = false;
      bGotFromWaypoint_left = false;
    }

//    // Check for keypress in terminal
//    KB_code = 0;
//    if (kbhit()) {
//      KB_code = getchar();
//      INFO_MSG("KB_code = " << KB_code)
//      // Do an object detection if "d" button was pressed
//      if (KB_code == 1000) {
//        INFO_MSG("Do object detection")
//                amiroState = objectDetection;
//      }
//    }

    INFO_MSG("STATE: " << statesString[amiroState])

    // --- Produce current state's behavior and do transition ---
    switch (amiroState) {
    case idle:
      setAMiRoColor(255,255,255);
//      if(bGotFromTobi_init) {
//        set_state_init();
      set_state_waypoint();
//      }
      break;

    case init:
      // Clear all variables
      bGotFromTobi_init = false;
      bGotFromTobi_stopfollowing = false;
      bGotFromTobi_stopwaypoint = false;
      bGotFromTobi_homing = false;
      bGotFromTobi_savePosition = false;
      bGotFromWaypoint_entered = false;
      bGotFromWaypoint_left = false;
      set_state_stopped();
      if(bGotFromTobi_initfollowing) {
        informerFollowing->publish(signal_init);
        set_state_following();
      } else if(bGotFromTobi_initwaypoint || gotFromIRinitWaypoint) {
        informerWaypoint->publish(signal_init);
        set_state_waypoint();
      } else if (bGotFromTobi_gotoroom || gotFromIRgoToRoom) {
        informerHoming->publish(signal_targetPosition);
        set_state_goToRoom();
      }
//      bGotFromTobi_initfollowing = false;
//      bGotFromTobi_initwaypoint = false;
      break;

    case following:
      if (bGotFromTobi_stopfollowing) {
        informerFollowing->publish(signal_stop);
        // HACK START: We just wait a few seconds, so that AMiRo can drive to a save homing position
        //             and after that, the "rec" message is send to Tobi, so that he go on
        set_state_homing();
        informerHoming->publish(signal_homingPosition);
        boost::shared_ptr<std::string> tmp(new std::string("left"));
        informerRemoteWaypoint->publish(tmp);
        informerWaypoint->publish(signal_stop);
//        usleep(1 /*seconds*/ * 1000000);
//        static bool onceGotHoming = false;
//        if (!onceGotHoming) {
//          informerHoming->publish(signal_savePosition);
//          onceGotHoming = true;
//        }
//        usleep(1 /*seconds*/ * 1000000);
//        std::string output = "stopfollowrec"; *stringPublisher = output; informerRemoteState->publish(stringPublisher);
        //HACK END
        //set_state_stopped();
      } else if (bGotFromTobi_initwaypoint) {
        informerFollowing->publish(signal_stop);
        informerWaypoint->publish(signal_init);
        stop_following();
        set_state_waypoint();
      } else if (bGotFromTobi_turn) {
        informerFollowing->publish(signal_stop);
        stop_following();
        set_state_turn();
      }
      break;

    case goToRoom:
        // got a response from CoreSLAM, it must have reached the target position
        if (!queueHomingAnswer->empty()) {
            boost::shared_ptr<std::string> response = queueHomingAnswer->pop();
            if ((*response).compare("done") == 0) {
                set_state_reachedroom();
            } else if ((*response).compare("failed") == 0) {
                set_state_failedroom();
            }
        }
        break;


    case reachedroom:
        informerRemoteState->publish(boost::shared_ptr<std::string>(new std::string("reachedroom")));

        if (bGotFromTobi_reachedroomrec) {
            set_state_stopped();
        } else if (gotFromIRinitWaypoint) {
            informerWaypoint->publish(signal_init);
            set_state_waypoint();
        } else if (gotFromIRhoming) {
            informerHoming->publish(signal_homingPosition);
            informerWaypoint->publish(signal_stop);
            set_state_homing();
        }
        break;


    case failedroom:
        informerRemoteState->publish(boost::shared_ptr<std::string>(new std::string("failedroom")));

        if (bGotFromTobi_failedroomrec) {
            set_state_stopped();
        } else if (gotFromIRinitWaypoint) {
            informerWaypoint->publish(signal_init);
            set_state_waypoint();
        } else if (gotFromIRhoming) {
            informerHoming->publish(signal_homingPosition);
            informerWaypoint->publish(signal_stop);
            set_state_homing();
        }
        break;

    case waypoint:
      DEBUG_MSG("Entered=" << bGotFromWaypoint_entered << ", Left=" << bGotFromWaypoint_left << ", personEntered=" << personEntered);
      if(bGotFromTobi_stopwaypoint || gotFromIRstopWaypoint) {
        informerWaypoint->publish(signal_stop);
        set_state_stopped();
      } else if (bGotFromWaypoint_entered && !personEntered) {
        DEBUG_MSG("Switched to entered in waypoint!");
        set_state_waypoint_entered();
        boost::shared_ptr<std::string> tmp(new std::string("entered"));
        informerRemoteWaypoint->publish(tmp);

        // stop waypoint and start following
        informerWaypoint->publish(signal_stop);
        if (followPerson) {
          informerFollowing->publish(signal_init);
          set_state_following();
        }

      } else if (bGotFromWaypoint_left && personEntered) {
        DEBUG_MSG("Switched to left in waypoint!");
        set_state_waypoint();
        boost::shared_ptr<std::string> tmp(new std::string("left"));
        informerRemoteWaypoint->publish(tmp);

        // stop waypoint and start following
        informerWaypoint->publish(signal_stop);
        if (followPerson) {
          informerFollowing->publish(signal_init);
          set_state_following();
        }

      } else if (bGotFromTobi_initfollowing) {
        informerWaypoint->publish(signal_stop);
        if (followPerson) {
          informerFollowing->publish(signal_init);
          set_state_following();
        }
      } else if (bGotFromTobi_turn) {
        informerWaypoint->publish(signal_stop);
        set_state_turn();
      } else if (bGotFromTobi_homing || gotFromIRhoming) {
        informerHoming->publish(signal_homingPosition);
        informerWaypoint->publish(signal_stop);
        set_state_homing();
      }
      break;

    case homing:
        // got a response from CoreSLAM, it must have reached the home position
        if (!queueHomingAnswer->empty()) {
            // only follow the person once
            followPerson = false;

            boost::shared_ptr<std::string> response = queueHomingAnswer->pop();
            if ((*response).compare("done") == 0) {
                //set_state_stopped();
                set_state_idle();
            } else if ((*response).compare("failed") == 0) {
                set_state_stopped();
            }
        }
        break;
    case stopped:
      if (bGotFromTobi_initwaypoint || gotFromIRinitWaypoint) {
        informerWaypoint->publish(signal_init);
        set_state_waypoint();
      } else if (bGotFromTobi_initfollowing) {
        if (followPerson) {
          informerFollowing->publish(signal_init);
          set_state_following();
        }
      } else if (bGotFromTobi_turn) {
        set_state_turn();
      } else if (bGotFromTobi_homing || gotFromIRhoming) {
        informerHoming->publish(signal_homingPosition);
        informerWaypoint->publish(signal_homingPosition);
        set_state_homing();
      } else if (bGotFromTobi_gotoroom || gotFromIRgoToRoom) {
        informerHoming->publish(signal_targetPosition);
        set_state_goToRoom();
      }
      break;
      
    default:
      WARNING_MSG("Unknown state of AMiRo's state machine.");
    }

//    *signal_state = statesString[amiroState];
//    INFO_MSG("SENDING STATE SCOPE: " << g_sOutScopeStateTobi << "/" << *signal_state)

//    informerRemoteState[amiroState]->publish(signal_state);
    // informerAmiroState->publish(signal_state);
//    boost::this_thread::sleep(boost::posix_time::seconds(1));
    usleep(100000);

  }
  return 0;
}

void setAMiRoColor(int r, int g, int b) {
  for (int l = 0; l < 8; ++l) {
    myCAN.setLightColor(l,amiro::Color(r,g,b));
  }
}

void set_state_idle() {
  amiroState = idle;
  setAMiRoColor(0,0,0);
}

void set_state_turn() {
  amiroState = turn;
  setAMiRoColor(127,127,255);
}

void set_state_homing() {
  amiroState = homing;
  setAMiRoColor(255,255,255);
}

void set_state_goToRoom() {
    amiroState = goToRoom;
    setAMiRoColor(0,255,255);
}

void set_state_reachedroom() {
    amiroState = reachedroom;
    setAMiRoColor(0,255,100);
}

void set_state_failedroom() {
    amiroState = failedroom;
    setAMiRoColor(100,0,100);
}

void set_state_init() {
  amiroState = init;
  setAMiRoColor(127,127,127);
}

void set_state_following() {
  amiroState = following;
  setAMiRoColor(255,255,0);
}

void stop_following() {
  if (turn_follow != 0 || move_follow != 0) {
    //usleep(500000);
  }
  if (turn_follow != 0) {
    float vel = M_PI/4.0;
    float angle = turn_follow * M_PI/180.0;
    if (angle < 0) {
      vel *= -1;
    }
    float waiting_us = angle/vel * 1000000.0;
    motorActionMilli(0, (int)(vel*1000.0));
    //usleep((int)waiting_us);
    motorActionMilli(0, 0);
  }
  if (move_follow != 0) {
    float vel = 0.1;
    float dist = move_follow / 1000.0;
    if (dist < 0) {
      vel *= -1;
    }
    float waiting_us = dist/vel * 1000000.0;
    motorActionMilli((int)(vel*1000.0), 0);
    //usleep((int)waiting_us);
    motorActionMilli(0, 0);
  }
}

void set_state_waypoint() {
  personEntered = false;
  amiroState = waypoint;
//  setAMiRoColor(255,0,127);
  setAMiRoColor(255,0,0);
}

void set_state_waypoint_entered() {
  personEntered = true;
  amiroState = waypoint;
  setAMiRoColor(0,255,0);
}

void set_state_stopped() {
  amiroState = stopped;
  setAMiRoColor(0,0,255);
}

void motorActionMilli(int speed, int turn) {
  myCAN.setTargetSpeed(speed*1000, turn*1000);
}
