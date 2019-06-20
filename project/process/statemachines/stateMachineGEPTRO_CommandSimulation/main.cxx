//============================================================================
// Name        : main.cxx
// Author      : mbarther <mbarther@techfak.uni-bielefeld.de>
// Description : This is the ToBI state machine simulation to the
//               openChallenge state machine.
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
#define NUM_STATES 9
enum states {
	init,
	initWait,
	objectWait,
	initDoneWait,
	objectDeliveryStart,
	objectDeliveryWait,
	objectTransportStart,
	objectTransportWait,
	finishAnswerer
};

states amiroState = init;
states amiroStateL = initWait;

std::string statesString[NUM_STATES] {
	"init",
	"initWait",
	"objectWait",
	"initDoneWait",
	"objectDeliveryStart",
	"objectDeliveryWait",
	"objectTransportStart",
	"objectTransportWait",
	"finishAnswerer"
};
	



// Objects && Object detection communication
#define NUM_OBJECTS 6
enum objects { object1 , object2 , object3 , object4 , object5 , object6 };
std::string objectsString[NUM_OBJECTS] = {"object1","object2","object3","object4","object5","object6"};

// Communication with ToBI
std::string sOutScopeTobi = "/tobiamiro";
std::string sInScopeTobi = "/amiro";
std::string sInScopeTobi2nd = "tobi";

// Output for debugging
std::string sOutScopeState = "/amiroState";


// RSB content
std::string inputRSBInit = "init";
std::string inputRSBInitDone = "initdone";
std::string inputRSBObject = "object";
std::string inputRSBDeliverySufStart = "start";
std::string inputRSBDeliverySufFin = "finish";
std::string inputRSBTransportSufStart = "start";
std::string inputRSBTransportSufFin = "finish";

std::string outputRSBInit = "init";
std::string outputRSBColor = "rbgyx";
std::string outputRSBDelivery = "object";
std::string outputRSBTransport = "transport";
std::string outputRSBRec = "rec";

// RSB input recognizer
bool rsbInputInit = false;
bool rsbInputInitDone = false;
bool rsbInputDelivery = false;
bool rsbInputTransport = false;

// RSB output resend flags
bool deliveryRec = false;
bool transportRec = false;

int processSM(void);

int robotID = 0;
int objectID = 3;

int objectCount = 2;
int objectOffsetForToBI = 2;
bool objectDetected[] = {false, false};
bool objectDetectedRec[] = {false, false};

std::string sRemoteServerPort = "4823";
std::string sRemoteServer = "localhost";

// Object detection
double minimalAngle = 0;
double minimalValue = 99999;

bool readDelivery(std::string inputData) {
    std::string prefix = "";
    prefix.append(inputData, 0, outputRSBDelivery.size());
    if (prefix.compare(outputRSBDelivery) == 0) {
        std::string suffix = "";
        if (inputData.size() >= outputRSBDelivery.size() + inputRSBDeliverySufStart.size()) {
            suffix.append(inputData, outputRSBDelivery.size(), inputRSBDeliverySufStart.size());
            if (suffix.compare(inputRSBDeliverySufStart) == 0) {
                deliveryRec = true;
                INFO_MSG(" -> delivery start has been received.");
                return true;
            }
        }
        suffix = "";
        if (inputData.size() >= outputRSBDelivery.size() + inputRSBDeliverySufFin.size()) {
            suffix.append(inputData, outputRSBDelivery.size(), inputRSBDeliverySufFin.size());
            if (suffix.compare(inputRSBDeliverySufFin) == 0) {
                rsbInputDelivery = true;
                INFO_MSG(" -> delivery finished.");
                return true;
            }
        }
    }
    return false;
}

bool readObjectDetection(std::string inputData) {
    std::string prefix = "";
    prefix.append(inputData, 0, inputRSBObject.size());
    if (prefix.compare(inputRSBObject) == 0) {
        std::string sNum = "";
        sNum.append(inputData, prefix.size(), inputData.size()-prefix.size());
        int objId = std::stoi(sNum);
        if (objId > objectOffsetForToBI) {
            INFO_MSG(" -> Object " << objId << " has been detected.");
            objectDetected[objId-objectOffsetForToBI-1] = true;
            return true;
        }
    }
    return false;
}

bool readTransport(std::string inputData) {
    std::string prefix = "";
    prefix.append(inputData, 0, outputRSBTransport.size());
    if (prefix.compare(outputRSBTransport) == 0) {
        std::string suffix = "";
        if (inputData.size() >= outputRSBTransport.size() + inputRSBTransportSufStart.size()) {
            suffix.append(inputData, outputRSBTransport.size(), inputRSBTransportSufStart.size());
            if (suffix.compare(inputRSBTransportSufStart) == 0) {
                transportRec = true;
                INFO_MSG(" -> transport start has been received.");
                return true;
            }
        }
        suffix = "";
        if (inputData.size() >= outputRSBTransport.size() + inputRSBTransportSufFin.size()) {
            suffix.append(inputData, outputRSBTransport.size(), inputRSBTransportSufFin.size());
            if (suffix.compare(inputRSBTransportSufFin) == 0) {
                rsbInputTransport = true;
                INFO_MSG(" -> transport finished.");
                return true;
            }
        }
    }
    return false;
}

int main(int argc, char **argv) {
    namespace po = boost::program_options;

    po::options_description options("Allowed options");
    options.add_options()("help,h", "Display a help message.")
        ("spread,s", po::value < std::string > (&sRemoteServer), "IP of remote spread server.")
        ("spreadPort,p", po::value < std::string > (&sRemoteServerPort), "Port of remote spread server.")
        ("outscopeTobi,o", po::value < std::string > (&sOutScopeTobi), "Scope for sending the current state to tobi.")
        ("outscopeState,s", po::value < std::string > (&sOutScopeState), "Scope for sending the current state internaly.")
        ("inscopeTobi,i", po::value < std::string > (&sInScopeTobi), "Scope for recieving Tobis messages.")
        ("objectID", po::value < int > (&objectID), "Object ID of the object, which shall be delivered (default: 3).")
        ("objectOffset", po::value < int > (&objectOffsetForToBI), "Object count offset for ToBI (default: 2).")
        ("robotID,d", po::value < int > (&robotID), "Robot ID (default: 0).");


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

    if (objectID <= objectOffsetForToBI) {
        ERROR_MSG("The object ID for delivery has to be greater than the object offset for ToBI (" << objectOffsetForToBI << ")");
        exit(0);
    }

    // prepare scopes for ToBI-AMIRo communication
    sOutScopeTobi.append(std::to_string(robotID));
    sInScopeTobi.append(std::to_string(robotID)).append(sInScopeTobi2nd);
    outputRSBDelivery.append(std::to_string(objectID));

    // print all scopes
    INFO_MSG("List of all RSB scopes:");
    INFO_MSG(" - ToBI to AMiRo:   " << sInScopeTobi);
    INFO_MSG(" - AMiRo to ToBI:   " << sOutScopeTobi);
    INFO_MSG(" - State debugging: " << sOutScopeState);

    // use camera stream for testing
    return processSM();
}


int processSM(void) {

    // Create the factory
    rsb::Factory &factory = rsb::getFactory();


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

    /////////////////// REMOTE SCOPES///////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////

    rsb::ListenerPtr listenerOutsideScope;
    boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<std::string> > > queueOutsideScope(
            new rsc::threading::SynchronizedQueue<boost::shared_ptr<std::string> >(1));
    rsb::Informer< std::string >::Ptr informerOutsideScope;

    try {
        listenerOutsideScope = factory.createListener(sInScopeTobi, tmpPartConf);
        listenerOutsideScope->addHandler(rsb::HandlerPtr(new rsb::util::QueuePushHandler<std::string>(queueOutsideScope)));

        informerOutsideScope = factory.createInformer< std::string > (sOutScopeTobi, tmpPartConf);
    }
    catch(std::exception& e) {
        ERROR_MSG("Remote connection not established");
        return -1;
    }

    INFO_MSG("All RSB connections built. Starting statemachine now.");

    boost::shared_ptr<std::string> stringPublisher(new std::string);

    // run through statemachine
    bool runningStatemachine = true;
    while(runningStatemachine) {

        // Check RSB input first
        std::string sRSBInput = "";
        std::string sOutput = "";

        rsbInputInit = false;
        rsbInputInitDone = false;
        deliveryRec = false;
        transportRec = false;

        // Check input from outside
        if (!queueOutsideScope->empty()) {
            sRSBInput = *queueOutsideScope->pop();
            if (sRSBInput.compare(inputRSBInit) == 0) {
                rsbInputInit = true;
            } else if (sRSBInput.compare(inputRSBInitDone) == 0) {
                rsbInputInitDone = true;
            } else if (!readDelivery(sRSBInput) && !readTransport(sRSBInput)) {
                readObjectDetection(sRSBInput);
            }
        }

        if (amiroState != amiroStateL) {
            INFO_MSG("STATE: " << statesString[amiroState]);
        }
        amiroStateL = amiroState;

        switch (amiroState) {
            case init:
                if (rsbInputInit) {
                    amiroState = objectWait;
                } else {
                    sOutput = "";
                    sOutput.append(outputRSBInit).append(outputRSBColor);
                    *stringPublisher = sOutput;
                    informerOutsideScope->publish(stringPublisher);
                }
                break;
            case objectWait:
                if (objectDetected[objectID-objectOffsetForToBI-1]) {
                    INFO_MSG("Sending rec message of object " << objectID << ".");
                    sOutput = "";
                    sOutput.append(inputRSBObject).append(std::to_string(objectID)).append("rec");
                    *stringPublisher = sOutput;
                    informerOutsideScope->publish(stringPublisher);
                    objectDetected[objectID-objectOffsetForToBI-1] = false;
                    objectDetectedRec[objectID-objectOffsetForToBI-1] = true;
                }
                if (objectDetectedRec[objectID-objectOffsetForToBI-1]) {
                    amiroState = initDoneWait;
                }
                break;
            case initDoneWait:
                if (rsbInputInitDone) {
                    amiroState = objectDeliveryStart;
                }
                break;
            case objectDeliveryStart:
                if (!deliveryRec) {
                    *stringPublisher = outputRSBDelivery;
                    informerOutsideScope->publish(stringPublisher);
                } else {
                    amiroState = objectDeliveryWait;
                }
                break;
            case objectDeliveryWait:
                if (rsbInputDelivery) {
                    sOutput = "";
                    sOutput.append(outputRSBDelivery).append(inputRSBDeliverySufFin).append(outputRSBRec);
                    *stringPublisher = sOutput;
                    informerOutsideScope->publish(stringPublisher);
                    amiroState = objectTransportStart;
                }
                break;
            case objectTransportStart:
                if (!transportRec) {
                    *stringPublisher = outputRSBTransport;
                    informerOutsideScope->publish(stringPublisher);
                } else {
                    amiroState = objectTransportWait;
                }
                break;
            case objectTransportWait:
                if (rsbInputTransport) {
                    sOutput = "";
                    sOutput.append(outputRSBTransport).append(inputRSBTransportSufFin).append(outputRSBRec);
                    *stringPublisher = sOutput;
                    informerOutsideScope->publish(stringPublisher);
                    amiroState = finishAnswerer;
                }
                break;
            case finishAnswerer:
                INFO_MSG("Statemachine of Answerer finished.");
                INFO_MSG("If the AMiRo is waiting now, everything is fine!")
		return 0;
            default:
                ERROR_MSG("Unknown state in statemachine!");
                return -1;
        }

        usleep(500000);

    }

    INFO_MSG("Statemachine has been closed.");

    return 0;
}
