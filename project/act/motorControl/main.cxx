//============================================================================
// Name        : main.cxx
// Author      : tkorthals <tkorthals@cit-ec.uni-bielefeld.de>
// Description : Send controls to the motor of AMiRo
//               Velocity are received in mm/s via RSB
//               Angular velocity are received in °/s via RSB
// Edited by   : jhomburg <jhomburg@techfak.uni-bielefeld.de>
//============================================================================

#define INFO_MSG_
// #define DEBUG_MSG_
// #define SUCCESS_MSG_
#define WARNING_MSG_
// #define ERROR_MSG_
#include <MSG.h>

#include <iostream>
#include <boost/thread.hpp>
#include <boost/program_options.hpp>
#include <boost/shared_ptr.hpp>

#include <rsb/filter/OriginFilter.h>
#include <rsb/Informer.h>
#include <rsb/Factory.h>
#include <rsb/Event.h>
#include <rsb/MetaData.h>
#include <rsb/Handler.h>
#include <rsb/converter/Repository.h>
#include <rsb/converter/ProtocolBufferConverter.h>

#include <stdint.h>  // int32

#include <ControllerAreaNetwork.h>

#include <mutex>
#include <rsc/threading/SynchronizedQueue.h>
#include <rsb/util/EventQueuePushHandler.h>

//RST
#include <rst/generic/Value.pb.h>

using namespace std;

struct MotorCmd {
	bool valid;
	int forward_vel;
	int angular_vel;
	boost::uint64_t expiration_time;
};

MotorCmd rankedMotorCmdTable[100];
int idxNewCmd = 0, currIdx = -1;
bool newLowerCmd = false;
std::mutex table_mutex;
int verbose = 0;

void printRankedCmdTalbe(string reason) {
	INFO_MSG("========= " << left << setw(40) << setfill('=') << reason);
	INFO_MSG(" Rank | Valid |   Forward   |    Angular   | Remaining time");
	int maxRow = 5;
	for(int i = 0; i <= maxRow || i == 99; ++i) {
		stringstream ssExpirationTime;
		if (rankedMotorCmdTable[i].valid) {
			switch (rankedMotorCmdTable[i].expiration_time) {
			case 0 : ssExpirationTime << "forever"; break;
			case 1 : ssExpirationTime << 0 << " ms"; break;
			default: ssExpirationTime << int(rankedMotorCmdTable[i].expiration_time - rsc::misc::currentTimeMicros()) / 1000 << " ms";}
		} else ssExpirationTime << "expired";
		INFO_MSG(
		   (i == currIdx  ? "->" : "  ")
		<< " " << (i < 10 ? "0" : "") << i
		<< " |   " << (rankedMotorCmdTable[i].valid ? 1 : 0)
		<< "   | " << setw(7) << setfill(' ') << rankedMotorCmdTable[i].forward_vel << "  µm"
		<< " | " << setw(7) << setfill(' ') << rankedMotorCmdTable[i].angular_vel << " µrad"
		<< " | " << ssExpirationTime.str());
		if (i == maxRow) {
			INFO_MSG(" ...  |  ...  | ...         | ...          | " << "...");
			i = 98;
		}
	}
}

void insertMotorCmdFromEvent(rsb::EventPtr motorCmdEvent) {
  if (motorCmdEvent->getType()!="rst::generic::Value"){return;}
  boost::shared_ptr<rst::generic::Value> motorCmdValueArray = boost::static_pointer_cast<rst::generic::Value> (motorCmdEvent->getData());
  if (motorCmdValueArray->type()!=rst::generic::Value::ARRAY){return;}
  std::vector<int> motorCmdVec(0,0);
  rst::generic::Value entry;
  for (int i=0; i<motorCmdValueArray->array_size();i++){
    entry=motorCmdValueArray->array(i);
    if (entry.type()!=rst::generic::Value::INT){return;}
    motorCmdVec.push_back(entry.int_());
  }
  if (motorCmdVec.size()<3){if (verbose>2){INFO_MSG("array too short\n");} return;}
	boost::uint64_t duration = motorCmdVec.at(2);
	struct MotorCmd newMotorCmd = {true, motorCmdVec.at(0), motorCmdVec.at(1), duration == 0 ? 1 : duration + motorCmdEvent->getMetaData().getReceiveTime()};
	string behaviorScope = motorCmdEvent->getScopePtr()->getComponents().back();
	int idx = (int(behaviorScope[0])-48) * 10 + (int(behaviorScope[1]) - 48);
  if (verbose>0){
    INFO_MSG("IDX: " << idx << "duration: " <<newMotorCmd.expiration_time -motorCmdEvent->getMetaData().getReceiveTime()<< '\n');
  }
	rankedMotorCmdTable[idx] = newMotorCmd;
	newLowerCmd = idx <= currIdx;
	idxNewCmd = idx;
}

int main (int argc, const char **argv){

  // Handle program options
  namespace po = boost::program_options;

  std::string rsbInScope = "/motor";

  po::options_description options("Allowed options");
  options.add_options()("help,h", "Display a help message.")
    ("inscope,i", po::value < std::string > (&rsbInScope), "Scope for receiving the motor steering commands");

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

  // Create the CAN interface
  ControllerAreaNetwork CAN;

  // Get the RSB factory
  rsb::Factory& factory = rsb::getFactory();

  boost::shared_ptr< rsb::converter::ProtocolBufferConverter<rst::generic::Value> >
    converter(new rsb::converter::ProtocolBufferConverter<rst::generic::Value>());
  rsb::converter::converterRepository<std::string>()->registerConverter(converter);

  rsb::ListenerPtr motorCmdListener = factory.createListener(rsbInScope);
  INFO_MSG("Listening on Scope: "<<rsbInScope<<endl)
  boost::shared_ptr<rsc::threading::SynchronizedQueue<rsb::EventPtr> > cmdQueue(new rsc::threading::SynchronizedQueue<rsb::EventPtr>(1));
  motorCmdListener->addHandler(rsb::HandlerPtr(new rsb::util::EventQueuePushHandler(cmdQueue)));
  rsb::EventPtr newMotorCmdEvent;

  // Init with stop cmd
  currIdx = 99;
  struct MotorCmd stopCmd = {true, 0, 0, 0};
  rankedMotorCmdTable[currIdx--] = stopCmd; // -1 because of increment at beginning of loop

  boost::uint32_t remainingExecTime;
  while(true) {
	  ++currIdx;
	  // Skip cmd if [not valid] or [valid but not valid after update] (additionally some assignments)
	  if (!rankedMotorCmdTable[currIdx].valid)continue;
	  if ((rankedMotorCmdTable[currIdx].valid = (rankedMotorCmdTable[currIdx].expiration_time > rsc::misc::currentTimeMicros()))) {
		  remainingExecTime = (rankedMotorCmdTable[currIdx].expiration_time - rsc::misc::currentTimeMicros())/1000;
	  } else {
		  if (rankedMotorCmdTable[currIdx].expiration_time < 2) { // Special meaning of expiration_time (see below)
			  rankedMotorCmdTable[currIdx].valid = true;
			  remainingExecTime = rankedMotorCmdTable[currIdx].expiration_time; // 0 -> wait forever, 1 -> immediately expire
		  } else continue;
	  }
	  // Start execution of currently lowest level and valid cmd
	  CAN.setTargetSpeed(rankedMotorCmdTable[currIdx].forward_vel, rankedMotorCmdTable[currIdx].angular_vel);
	  try { // ...to pop new cmd from queue (if no new cmd available -> blocks until currently active cmd is expired)
		  if (remainingExecTime == 1)
			  throw rsc::threading::QueueEmptyException(); // immediately expire
		  else
			  newMotorCmdEvent = cmdQueue->pop(remainingExecTime);
	  } catch(const rsc::threading::QueueEmptyException& timeoutException){
		  // Popping timed out -> currently active cmd expired
		  rankedMotorCmdTable[currIdx].valid = false;
      if (verbose>1){
        printRankedCmdTalbe("Current command expired ");
      }
		  // Continue with execution of next higher level cmd
		  continue;
	  }
	  // No exception -> popping succeeded -> new cmd came in -> insert in table and check priority
	  insertMotorCmdFromEvent(newMotorCmdEvent);
	  if (newLowerCmd) { // if new cmd has lower priority -> jump to that cmd
		  currIdx = idxNewCmd;
		  newLowerCmd = false;
	  }
    if (verbose>1){
      printRankedCmdTalbe("Insertion ");
    }
	  --currIdx; // -1 because of increment at beginning
	  // otherwise just continue with execution of previously active cmd:
	  // -> currIdx not changed and cmd still valid
	  // -> remaining execution time gets adjusted at beginning of loop
  }

  return EXIT_SUCCESS;
}
