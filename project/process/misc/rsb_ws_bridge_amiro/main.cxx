#include <string.h>
#include <time.h>
#include <stdlib.h>
#include <stdio.h>
#include <mongoose.h>
#include <vector>
#include <iostream>
#include <unistd.h>
#include <pthread.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <assert.h>
#include <signal.h>
#include <mutex>
#include <syslog.h>
#include <sys/time.h>
#include <unistd.h>
#include <sys/wait.h>

// RSB
#include <rsb/Informer.h>
#include <rsb/Factory.h>
#include <rsb/Event.h>
#include <rsb/Handler.h>
#include <rsb/converter/Repository.h>
#include <rsb/converter/PredicateConverterList.h>
#include <rsb/converter/ByteArrayConverter.h>

// RST
#include <rsb/converter/ProtocolBufferConverter.h>

typedef unsigned char uchar;

struct popen2 {
    pid_t child_pid;
    int   from_child, to_child;
};

namespace rsb_ws_bridge {
// For debugging
static int currentTimer, lastTimerCam = 0;  //currentTimer holds the actual Time in microseconds
static struct timeval tv;
// RSB
static rsb::ListenerPtr listenerPtr;
static rsb::HandlerPtr  handlerPtr;
static rsb::ParticipantConfig config;
static rsb::Factory& factory = rsb::getFactory();
// RSB WS interaction
static bool newRsbData = false;
static std::string newRsbPayload;
static std::string futureScope;
static std::mutex mutex;
// Execution of program
static std::mutex mutexExec;
static bool isExec = false;
static popen2 program;
}

#define WS_PREAMBLE_EXEC      "exec"
#define WS_PREAMBLE_RSB_SCOPE '/'

int getCurrentTime() {
  gettimeofday(&rsb_ws_bridge::tv, 0);
  return rsb_ws_bridge::tv.tv_sec * 1000
         + rsb_ws_bridge::tv.tv_usec / 1000;
}

#include <sys/types.h>
#include <unistd.h>

int popen2(const char *cmdline, struct popen2 *childinfo) {
    pid_t p;
    int pipe_stdin[2], pipe_stdout[2];

    if(pipe(pipe_stdin)) return -1;
    if(pipe(pipe_stdout)) return -1;

//    printf("pipe_stdin[0] = %d, pipe_stdin[1] = %d\n", pipe_stdin[0], pipe_stdin[1]);
//    printf("pipe_stdout[0] = %d, pipe_stdout[1] = %d\n", pipe_stdout[0], pipe_stdout[1]);

    p = fork();
    if(p < 0) return p; /* Fork failed */
    if(p == 0) { /* child */
        close(pipe_stdin[1]);
        dup2(pipe_stdin[0], 0);
        close(pipe_stdout[0]);
        dup2(pipe_stdout[1], 1);
        execl("/bin/sh", "sh", "-c", cmdline, 0);
        perror("execl"); exit(99);
    }
    childinfo->child_pid = p;
    childinfo->to_child = pipe_stdin[1];
    childinfo->from_child = pipe_stdout[0];
    return 0;
}

void receiveRsbData(boost::shared_ptr<std::string> e) {
  rsb_ws_bridge::mutex.lock();
  rsb_ws_bridge::newRsbData = true;
  std::cout << "Received RSB event with " << e->size() << " bytes" << std::endl;
//  std::cout << "Payload: " << *e << std::endl;
  rsb_ws_bridge::newRsbPayload = *e;
  rsb_ws_bridge::mutex.unlock();
}

static int process_request(struct mg_connection *conn) {

  // If connection is a websocket connection -> all received signals are processed here
  std::cout << "process_request" << std::endl;
  if (conn->is_websocket) {
    std::cout << "MG_REQUEST: conn->is_websocket" << std::endl;
    // Process the received signal
    rsb_ws_bridge::futureScope = std::string(conn->content, conn->content_len);

    std::string input = std::string(rsb_ws_bridge::futureScope);
    std::cout << "Input: " << input << " (" << input.length() << ")\n";
      
    if (rsb_ws_bridge::futureScope[0] == WS_PREAMBLE_RSB_SCOPE) {
      std::cout << "Set scope to: " << rsb_ws_bridge::futureScope << std::endl;

      // Set the new listener configuration
      rsb_ws_bridge::listenerPtr = rsb_ws_bridge::factory.createListener(
          rsb::Scope(rsb_ws_bridge::futureScope), rsb_ws_bridge::config);
      rsb_ws_bridge::handlerPtr = rsb::HandlerPtr(
          new rsb::DataFunctionHandler<std::string>(&receiveRsbData));
      rsb_ws_bridge::listenerPtr->addHandler(
          rsb_ws_bridge::handlerPtr);
    } else if (std::string(rsb_ws_bridge::futureScope).find(std::string(WS_PREAMBLE_EXEC)) == 0) { // Check if the exec preamble exists
      // Close any program first
      if (rsb_ws_bridge::isExec) {
        int killRes = kill(rsb_ws_bridge::program.child_pid, 0);
        printf("kill(%d, 0) -> %d\n", rsb_ws_bridge::program.child_pid, killRes);
        // TODO: here it has to wait!
//        waitpid(rsb_ws_bridge::program.child_pid, NULL, 0);
//        printf(" -> waited for child %d\n", rsb_ws_bridge::program.child_pid);
      }
      // Run the program
      int result = popen2(rsb_ws_bridge::futureScope.erase(0,4).c_str(), &rsb_ws_bridge::program); // Delete the exec preamble and run
      rsb_ws_bridge::isExec = true;
    	//std::cout << "Logger Data transmitted to Server" << "\n";
      if (result >= 0) {
        std::cout << "EXECUTION STARTED\n";
      } else {
        std::cout << "ERROR: EXECUTION FAILED !!!\n";
      }
    } else {
      std::cout << "Socket input couldn't be interpretated.\n";
    }

    // Getting connection closed Signal by Client and remove the handler for the rsb listener
    if ((conn->wsbits & 0x000F) == WEBSOCKET_OPCODE_CONNECTION_CLOSE) {
      std::cout << "socket closed" << "/" << conn->wsbits << std::endl;
      if ( rsb_ws_bridge::listenerPtr.get() != 0 ) {
        if ( rsb_ws_bridge::handlerPtr.get() != 0 ) {
          rsb_ws_bridge::listenerPtr->removeHandler(rsb_ws_bridge::handlerPtr);
          rsb_ws_bridge::handlerPtr.reset();
        }
        rsb_ws_bridge::listenerPtr.reset();
      }
      if (rsb_ws_bridge::isExec) {
        printf("kill(%d, 0) -> %d\n", rsb_ws_bridge::program.child_pid, kill(rsb_ws_bridge::program.child_pid, 0));
        rsb_ws_bridge::isExec = false;
      }
    }

    return
        conn->content_len == 4 && !memcmp(conn->content, "exit", 4) ?
            MG_FALSE : MG_TRUE;
  } else {
    //load everything that belongs to the html site automatically
    std::cout << "nein, doch nicht ;-)" << std::endl;
    return MG_FALSE;
  }
}

static int ev_handler(struct mg_connection *conn, enum mg_event ev) {

  // If a new CLient connects
  if (ev == MG_WS_HANDSHAKE) {
    std::cout << "MG_WS_HANDSHAKE" << std::endl;
  }

  // If a Client leaves
  if (ev == !MG_REPLY) {
    std::cout << "!MG_REPLY" << std::endl;
  }

  // Handle client request (and websocket receives)
  if (ev == MG_REQUEST) {
    std::cout << "MG_REQUEST" << std::endl;
    return process_request(conn);
  }

  if (ev == MG_AUTH) {  // If not included you have to authenticate to load the homepage
    std::cout << "MG_AUTH" << std::endl;
    return MG_TRUE;
  }

  // Handle websocket sending
  if (ev == MG_POLL && conn->is_websocket) {
    //std::cout << "MG_POLL" << std::endl;

    // Send byte package from rsb
    rsb_ws_bridge::mutex.lock();
    if (rsb_ws_bridge::newRsbData) {
      std::cout << "Send WS event with " << rsb_ws_bridge::newRsbPayload.size()
                << " bytes" << std::endl;
//      std::cout << "Payload: " << rsb_ws_bridge::newRsbPayload << std::endl;
      mg_websocket_write(conn, WEBSOCKET_OPCODE_BINARY,
                         (const char*) rsb_ws_bridge::newRsbPayload.c_str(),
                         rsb_ws_bridge::newRsbPayload.size());
      rsb_ws_bridge::newRsbData = false;
    }
    rsb_ws_bridge::mutex.unlock();

    // Send program output
    if (rsb_ws_bridge::isExec) {
      const int MAX_BUFFER_SIZE = 100000;
      char buf[MAX_BUFFER_SIZE];
      fd_set set;
      FD_ZERO (&set);
      FD_SET (rsb_ws_bridge::program.from_child, &set);
      struct timeval timeout; timeout.tv_sec = 0; timeout.tv_usec = 0;
      if ( select(rsb_ws_bridge::program.from_child + 1, &set, NULL, NULL, &timeout) > 0) {
        memset(buf, '\0', MAX_BUFFER_SIZE);
        read(rsb_ws_bridge::program.from_child, buf, MAX_BUFFER_SIZE);
        std::string stdout(buf);
        mg_websocket_write(conn, WEBSOCKET_OPCODE_BINARY,
                                 (const char*) stdout.c_str(),
                                 stdout.size());
        if ( MAX_BUFFER_SIZE <= stdout.size() ) {
          std::cout << "Error copying stdout of program execution: Buffer to small" << std::endl << std::flush;
        }
        // Sleep a while, so that new stdout may arrive
        usleep(100000 /*100 ms*/);
      }
    }
    return MG_FALSE;
  }
  return MG_FALSE;
}

int main(int argc, char **argv) {

  // Create a config for anonymous deserialization
  {
    std::list<
        std::pair<rsb::converter::ConverterPredicatePtr,
            typename rsb::converter::Converter<std::string>::Ptr> > converters;
    // Use a converter which can handle everything.
    converters.push_back(
        std::make_pair(
            rsb::converter::ConverterPredicatePtr(
                new rsb::converter::AlwaysApplicable()),
            typename rsb::converter::Converter<std::string>::Ptr(
                new rsb::converter::ByteArrayConverter())));
    rsb::converter::ConverterSelectionStrategy<std::string>::Ptr css =
        rsb::converter::ConverterSelectionStrategy<std::string>::Ptr(
            new rsb::converter::PredicateConverterList<std::string>(
                converters.begin(), converters.end()));

    // Configure a Listener object.
    rsb_ws_bridge::config =
        rsb_ws_bridge::factory.getDefaultParticipantConfig();

    std::set<rsb::ParticipantConfig::Transport> transports =
        rsb_ws_bridge::config.getTransports();
    for (std::set<rsb::ParticipantConfig::Transport>::const_iterator it =
        transports.begin(); it != transports.end(); ++it) {
      rsb::ParticipantConfig::Transport& transport = rsb_ws_bridge::config
          .mutableTransport(it->getName());
      rsc::runtime::Properties options = transport.getOptions();
      options["converters"] = css;
      transport.setOptions(options);
    }
  }

  // Create the mongoose server
  struct mg_server *server = mg_create_server(NULL, ev_handler);
  mg_set_option(server, "listening_port", "80");
  mg_set_option(server, "document_root", "./www");
  std::cout << "Started on port " << mg_get_option(server, "listening_port")
            << std::endl;

  // Begin of interation loop
  rsb_ws_bridge::lastTimerCam = getCurrentTime();
  unsigned int counter = 0;
  for (;;) {

    // Handle the server
    mg_poll_server(server, 100 /*ms*/);

    // Print out the pollings per seconds, the server has to process
    rsb_ws_bridge::currentTimer = getCurrentTime();
    if (rsb_ws_bridge::currentTimer - rsb_ws_bridge::lastTimerCam
        >= 1000 /*ms*/) {
      std::cout << "PPS: " << counter << " polls / "
                << rsb_ws_bridge::currentTimer - rsb_ws_bridge::lastTimerCam
                << " ms" << std::endl;
      rsb_ws_bridge::lastTimerCam = rsb_ws_bridge::currentTimer;
      counter = 0;
    }
    ++counter;
  }

  mg_destroy_server(&server);
  return 0;
}

