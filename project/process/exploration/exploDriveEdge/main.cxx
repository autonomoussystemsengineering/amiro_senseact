//============================================================================
// Name        : main.cxx
// Author      : mbarther <mbarther@techfak.uni-bielefeld.de>
// Description : The robot drives forward to the next edge and turns to the
//               right. This procedure is repeated as often as set (on
//               default it's done twice).
//============================================================================

//#define TRACKING
#define INFO_MSG_
#define DEBUG_MSG_
// #define SUCCESS_MSG_
#define WARNING_MSG_
// #define ERROR_MSG_
#include <MSG.h>
#include <functional>
#include <algorithm>
#include <math.h>

#include <iostream>
#include <boost/thread.hpp>
#include <boost/program_options.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/date_time.hpp>


#include <rsb/filter/OriginFilter.h>
#include <rsb/Informer.h>
#include <rsb/Factory.h>
#include <rsb/Version.h>
#include <rsb/Event.h>
#include <rsb/Handler.h>
#include <rsb/converter/Repository.h>
#include <rsb/converter/ProtocolBufferConverter.h>
#include <rsc/threading/SynchronizedQueue.h>
#include <rsb/util/QueuePushHandler.h>


using namespace rsb;

#include <rst/geometry/Pose.pb.h>
#include <rst/geometry/Translation.pb.h>
#include <rst/geometry/Rotation.pb.h>
using namespace rst::geometry;

#include <converter/vecIntConverter/main.hpp>
#include <converter/matConverter/matConverter.hpp>
using namespace muroxConverter;
using namespace std;

#include <Types.h>

#include <types/twbTracking.pb.h>
#include <types/PoseEuler.pb.h>

#include <stdint.h>  // int32

#include <ControllerAreaNetwork.h>
#include <Constants.h>
#include <sensorModels/VCNL4020Models.h>
using namespace amiro;

using namespace rsb;
using namespace rsb::patterns;


// margins
#define OBSTACLE_MARGIN 100
#define OBSTACLE_MARGIN_SIDE 7500
#define GROUND_MARGIN 0.06
#define GROUND_MARGIN_DANGER 0.05
#define EDGE_DIFF 0.004

// velocities
#define VEL_FORWARD 8
#define VEL_FORWARD_SLOW 4
#define VEL_TURNING 40
#define VEL_TURNING_SLOW 20


// scopenames for rsb
std::string proxSensorInscopeObstacle = "/rir_prox/obstacle";
std::string proxSensorInscopeGround = "/rir_prox/ground";
std::string odometryInscope = "/localization";
std::string commandInscope = "/exploration/command";
std::string answerOutscope = "/exploration/answer";
std::string positionInscope = "/rectposition/command";
std::string positionOutscope = "/rectposition/answer";

// rsb messages
std::string cmdStart = "start";
std::string ansFinish = "finish";
std::string ansProblem = "broken";

// edge count
int edgeNum = 0;
int edgeCount = 2;

enum stateType {
  STturnEdge,
  STfindDirection,
  STroundScan,
  STdriveEdge,
  STcheckEdge,
  STcorrectEdge,
  STturn,
  STfinalize
};

std::string stateTypeString[] {
  "turn ortho to edge",
  "find direction",
  "round scan",
  "driving edge",
  "check edge",
  "correct edge",
  "turn",
  "finalize"
};

void sendMotorCmd(int speed, int angle, ControllerAreaNetwork &CAN) {
  
  CAN.setTargetSpeed(speed, angle);
  DEBUG_MSG( "v: " << speed << "w: " << angle);
}

int mymcm(int mym) {
  return mym*10000;
}

int main(int argc, char **argv) {

  // Handle program options
  namespace po = boost::program_options;

  po::options_description options("Allowed options");
  options.add_options()("help,h", "Display a help message.")
      ("edgeCount,e", po::value <int> (&edgeCount), "Count of edges the robot should drive (default = 2).")
      ("positionInscope", po::value <std::string> (&odometryInscope), "Inscope for position data of SLAM localization.");

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

  INFO_MSG("Initialize RSB");

  // Get the RSB factory
  rsb::Factory& factory = rsb::getFactory();

  // ------------ Converters ----------------------

  // Register new converter for std::vector<int>
  boost::shared_ptr<vecIntConverter> converterVecInt(new vecIntConverter());
  rsb::converter::converterRepository<std::string>()->registerConverter(converterVecInt);

  // Register converter for the pose list
  boost::shared_ptr<rsb::converter::ProtocolBufferConverter<twbTracking::proto::Pose2DList> > pose2DListConverter(new rsb::converter::ProtocolBufferConverter<twbTracking::proto::Pose2DList>());
  rsb::converter::converterRepository<std::string>()->registerConverter(pose2DListConverter);

  // Register converter for Pose2D
  boost::shared_ptr<rsb::converter::ProtocolBufferConverter<twbTracking::proto::Pose2D> > converterlocalization(new rsb::converter::ProtocolBufferConverter<twbTracking::proto::Pose2D>());
  rsb::converter::converterRepository<std::string>()->registerConverter(converterlocalization);

  // Register converter for PoseEuler
  boost::shared_ptr< rsb::converter::ProtocolBufferConverter<rst::geometry::PoseEuler > > odomEulerConverter(new rsb::converter::ProtocolBufferConverter<rst::geometry::PoseEuler >());
  rsb::converter::converterRepository<std::string>()->registerConverter(odomEulerConverter);

  // ------------ Listener ----------------------

  // prepare RSB listener for the IR data
  rsb::ListenerPtr proxListenerObstacle = factory.createListener(proxSensorInscopeObstacle);
  boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<std::vector<int>>> >proxQueueObstacle(new rsc::threading::SynchronizedQueue<boost::shared_ptr<std::vector<int>>>(1));
  proxListenerObstacle->addHandler(rsb::HandlerPtr(new rsb::util::QueuePushHandler<std::vector<int>>(proxQueueObstacle)));

  rsb::ListenerPtr proxListenerGround = factory.createListener(proxSensorInscopeGround);
  boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<std::vector<int>>> >proxQueueGround(new rsc::threading::SynchronizedQueue<boost::shared_ptr<std::vector<int>>>(1));
  proxListenerGround->addHandler(rsb::HandlerPtr(new rsb::util::QueuePushHandler<std::vector<int>>(proxQueueGround)));

  // prepare RSB listener for commands
  rsb::ListenerPtr cmdListener = factory.createListener(commandInscope);
  boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<std::string>>> cmdQueue(new rsc::threading::SynchronizedQueue<boost::shared_ptr<std::string>>(1));
  cmdListener->addHandler(rsb::HandlerPtr(new rsb::util::QueuePushHandler<std::string>(cmdQueue)));

  // prepare RSB listener for position requests
  rsb::ListenerPtr posListener = factory.createListener(positionInscope);
  boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<std::string>>> positionQueue(new rsc::threading::SynchronizedQueue<boost::shared_ptr<std::string>>(1));
  posListener->addHandler(rsb::HandlerPtr(new rsb::util::QueuePushHandler<std::string>(positionQueue)));

  // Prepare RSB async listener for localization messages
  rsb::ListenerPtr odomListener = factory.createListener(odometryInscope);
  boost::shared_ptr<rsc::threading::SynchronizedQueue<rsb::Informer<rst::geometry::PoseEuler>::DataPtr>>odomQueue(new rsc::threading::SynchronizedQueue<rsb::Informer<rst::geometry::PoseEuler>::DataPtr>(1));
  odomListener->addHandler(rsb::HandlerPtr(new rsb::util::QueuePushHandler<rst::geometry::PoseEuler>(odomQueue)));

  // prepare RSB informer for answers
  rsb::Informer<std::string>::Ptr informerAnswer = factory.createInformer<std::string> (answerOutscope);

  // prepare RSB informer for position requests
  rsb::Informer<twbTracking::proto::Pose2DList>::Ptr positionInformer = factory.createInformer<twbTracking::proto::Pose2DList>(positionOutscope);

  // Init the CAN interface
  ControllerAreaNetwork CAN;

  boost::shared_ptr<std::string> stringPublisher(new std::string);
  boost::shared_ptr<twbTracking::proto::Pose2DList> rectPositionsPtr(new twbTracking::proto::Pose2DList());        
  twbTracking::proto::Pose2D *pose2D;
  rst::geometry::PoseEuler odomInput;

  uint8_t sensorIdx = 0;
  bool ok = true;
  bool turn = 0;
  int oldValues[2];

  int counter = 0;
  while(true) {

    if (!cmdQueue->empty()) {
      // Read command
      std::string cmd = *cmdQueue->pop();

      ok = true;

      stateType state = STturnEdge;
      stateType stateL = STturn;

      edgeNum = 0;

      while(ok) {
        if (!proxQueueObstacle->empty() && !proxQueueGround->empty()) {
          counter = 0;
      
          // Read the proximity data
          boost::shared_ptr<std::vector<int>> sensorValuesObstacle = boost::static_pointer_cast<std::vector<int>>(proxQueueObstacle->pop());
          boost::shared_ptr<std::vector<int>> sensorValuesGround = boost::static_pointer_cast<std::vector<int>>(proxQueueGround->pop());

          if (state != stateL) {
            INFO_MSG("Switched to new state '" << stateTypeString[state] << "'");
            stateL = state;
          }

          float edgeDistL, edgeDistR, edgeDistFL, edgeDistFR;
          int minEdgeIdx = 0;
          float minEdgeDist = 10;
          int waitingTime_us = 0;
          switch (state) {
            case STturnEdge:
              // Search table edge
              for (int senIdx=0; senIdx<ringproximity::SENSOR_COUNT; senIdx++) {
                float ed = VCNL4020Models::edgeModel(sensorValuesGround->at(senIdx));
                if (ed < minEdgeDist) {
                  minEdgeIdx = senIdx;
                  minEdgeDist = ed;
                }
              }
/*              if (turn == 0 && minEdgeIdx < 7 && minEdgeIdx > 3) {
                turn = 1;
                sendMotorCmd(0, mymcm(VEL_TURNING_SLOW), CAN);
              } else if (turn == 0 && minEdgeIdx > 0 && minEdgeIdx < 4) {
                turn = 2;
                sendMotorCmd(0, mymcm(-VEL_TURNING_SLOW), CAN);
              }*/
              state = STfindDirection;
              break;
            case STfindDirection:
              edgeDistL = VCNL4020Models::edgeModel(sensorValuesGround->at(7));
              edgeDistR = VCNL4020Models::edgeModel(sensorValuesGround->at(0));
/*              if (turn == 0 && edgeDistL < edgeDistR - EDGE_DIFF) {
                turn = 1;
                sendMotorCmd(0, mymcm(VEL_TURNING_SLOW), CAN);
              } else if (turn == 0 && edgeDistR < edgeDistL - EDGE_DIFF) {
                turn = 2;
                sendMotorCmd(0, mymcm(-VEL_TURNING_SLOW), CAN);
              } else if (abs(edgeDistR-edgeDistL) <= EDGE_DIFF && edgeDistR < GROUND_MARGIN && edgeDistL < GROUND_MARGIN) {
*/                turn = 0;
//                sendMotorCmd(mymcm(VEL_FORWARD), 0, CAN);
//                state = STdriveEdge;
                sendMotorCmd(0, 0, CAN);
                state = STroundScan;
//              }
              break;
            case STroundScan:
//              waitingTime_us = (int)(((2.0*M_PI*1000.0) / ((float)VEL_TURNING*10.0)) * 1000000.0) - 200000; // us
//              sendMotorCmd(0, mymcm(VEL_TURNING), CAN);
//              usleep(waitingTime_us);
              // Save position
              pose2D = rectPositionsPtr->add_pose();
              pose2D->set_x(0);
              pose2D->set_y(0);
              pose2D->set_orientation(0);
              pose2D->set_id(0);
              sendMotorCmd(mymcm(VEL_FORWARD), 0, CAN);
              state = STdriveEdge;
            case STdriveEdge:
              edgeDistL = VCNL4020Models::edgeModel(sensorValuesGround->at(3));
              edgeDistR = VCNL4020Models::edgeModel(sensorValuesGround->at(4));
              if (edgeDistL < GROUND_MARGIN || edgeDistR < GROUND_MARGIN) {
                sendMotorCmd(0, 0, CAN);
                usleep(500000);
                state = STcheckEdge;
              }
              break;
            case STcheckEdge:
              edgeDistL = VCNL4020Models::edgeModel(sensorValuesGround->at(3));
              edgeDistR = VCNL4020Models::edgeModel(sensorValuesGround->at(4));
              if (edgeDistL < GROUND_MARGIN || edgeDistR < GROUND_MARGIN) {
                INFO_MSG("Edge detected");
                state = STcorrectEdge;
              } else {
                sendMotorCmd(mymcm(VEL_FORWARD), 0, CAN);
                state = STdriveEdge;
              }
              break;
            case STcorrectEdge:
              edgeDistR = VCNL4020Models::edgeModel(sensorValuesGround->at(3));
              edgeDistL = edgeDistL*cos(ringproximity::SENSOR_ANGULAR_FRONT_OFFSET);
              edgeDistR = (GROUND_MARGIN_DANGER - edgeDistR) * 10;
              INFO_MSG("Distance to edge: " << edgeDistL << " m (" << edgeDistR << " cm too close) -> Driving with " << VEL_FORWARD_SLOW << " cm/s for " << (int)(edgeDistR/((float)VEL_FORWARD_SLOW)*1000000) << " us");
              if (edgeDistR > 0) {
                sendMotorCmd(mymcm(-VEL_FORWARD_SLOW), 0, CAN);
                usleep((int)(edgeDistR/((float)VEL_FORWARD_SLOW)*1000000));
                sendMotorCmd(0,0,CAN);
              }
              sleep(1);
              if (odomQueue->empty()) {
                WARNING_MSG("No odometry available!");
              }
              odomInput = *odomQueue->pop();
              pose2D = rectPositionsPtr->add_pose();
              pose2D->set_x(odomInput.mutable_translation()->x());
              pose2D->set_y(odomInput.mutable_translation()->y());
              pose2D->set_orientation(odomInput.mutable_rotation()->yaw());
              pose2D->set_id(edgeNum);
              turn = 2;
              sendMotorCmd(0, mymcm(-VEL_TURNING), CAN);
              if (edgeNum >= edgeCount-1) {
                state = STfinalize;
              } else {
                state = STturn;
              }
              edgeNum++;
              DEBUG_MSG("#Edges driven: " << (edgeNum) << " of " << edgeCount);
              DEBUG_MSG("Saved position " << pose2D->x() << "/" << pose2D->y() << " with " << pose2D->orientation() << " rad and ID " << pose2D->id());
              break;
            case STturn:
              edgeDistL = VCNL4020Models::edgeModel(sensorValuesGround->at(1));
              edgeDistR = VCNL4020Models::edgeModel(sensorValuesGround->at(2));
              edgeDistFL = VCNL4020Models::edgeModel(sensorValuesGround->at(3));
              edgeDistFR = VCNL4020Models::edgeModel(sensorValuesGround->at(4));
              if (edgeDistL < GROUND_MARGIN && edgeDistR < GROUND_MARGIN && abs(edgeDistR-edgeDistL) < EDGE_DIFF
                  && edgeDistFL > GROUND_MARGIN && edgeDistFR > GROUND_MARGIN) {
                turn = 0;
                sendMotorCmd(mymcm(VEL_FORWARD), 0, CAN);
                state = STdriveEdge;
              }
              break;
            case STfinalize:
              waitingTime_us = (int)(((1.25*M_PI*1000.0) / ((float)VEL_TURNING*10.0)) * 1000000.0); // us
              sendMotorCmd(0, mymcm(VEL_TURNING), CAN);
              usleep(waitingTime_us);
              sendMotorCmd(0, 0, CAN);
              INFO_MSG("All steps done.");
              ok = false;
              *stringPublisher = ansFinish;
              informerAnswer->publish(stringPublisher);
              break;
            default:
              WARNING_MSG("Unknown state!");
              *stringPublisher = ansProblem;
              informerAnswer->publish(stringPublisher);
              return -1;
          }

        } else if (counter < 4) {
          counter++;
          usleep(50000);
        } else {
          sendMotorCmd(0, 0, CAN);
          WARNING_MSG("Didn't received any sensor data for more than 200 ms. Just stopping!");
          *stringPublisher = ansProblem;
          informerAnswer->publish(stringPublisher);
          ok = false;
        }
      }

    } else if (!positionQueue->empty()) {
      positionQueue->pop();
      positionInformer->publish(rectPositionsPtr);
    } else {
      usleep(500000);
    }
  }

  return EXIT_SUCCESS;
}
