//============================================================================
// Name        : main.cxx
// Author      : tkorthals <tkorthals@cit-ec.uni-bielefeld.de>
// Description : Reads the IR sensor data
//============================================================================




#define INFO_MSG_
#define DEBUG_MSG_
#define SUCCESS_MSG_
#define WARNING_MSG_
#define ERROR_MSG_
#include <MSG.h>

// Reading inScope and outScope
// #include <ioFlags.hpp>

// Reading inScope, outScope and freq
// #include <ofFlags.hpp>

#include <boost/thread.hpp>
#include <boost/program_options.hpp>
#include <boost/shared_ptr.hpp>


#include <rsb/Informer.h>
#include <rsb/Factory.h>
#include <rsb/Event.h>
#include <rsb/Handler.h>
#include <rsb/converter/Repository.h>

// Include own converter
// #include <converter/vecIntConverter/main.hpp>

using namespace std;
using namespace rsb;
using namespace boost;
// using namespace muroxConverter;

// void captureMic (void);

// int main (int argc, const char **argv){

  
//       INFO_MSG( "Standard values " )
//       
//       INFO_MSG( "Outscope: " << ofFlags::outScope)
//       INFO_MSG( "Period: " << ofFlags::uiPeriod_ms)
//       
//       ofFlags::handleCommandline(argc, argv);
//       
//       INFO_MSG( "New values " )
//       
//       INFO_MSG( "Outscope: " << ofFlags::outScope)
//       INFO_MSG( "Period: " << ofFlags::uiPeriod_ms)
//       // Prepare RSB informer
//       rsb::Factory& factory = rsb::Factory::getInstance();
//       rsb::Informer< std::vector<int> >::Ptr informer_vec = factory.createInformer< std::vector<int> > ("/mic");
// 
//       captureMic();
//       
// 
//       return EXIT_SUCCESS;
// }


#include <stdio.h>
#include <stdlib.h>
#include <alsa/asoundlib.h>
int main(int argc, char **argv) { //argv[1] = "hw:0,0"
  int i; 
  int err;
  short buf[128];
  unsigned int exact_rate = 8000;
  int time_s = 3;
  static unsigned int channels = 2; 
  
  snd_pcm_t *playback_handle;
  snd_pcm_hw_params_t *hw_params; 
  char *pcm_name = strdup("plughw:0,0"); 
  snd_pcm_t *capture_handle; 

  snd_pcm_open (&capture_handle, argv[1], SND_PCM_STREAM_CAPTURE, 0);
  snd_pcm_hw_params_malloc (&hw_params);
  snd_pcm_hw_params_any (capture_handle, hw_params);
  snd_pcm_hw_params_set_access(capture_handle,hw_params,SND_PCM_ACCESS_RW_INTERLEAVED);
//   snd_pcm_hw_params_set_format (capture_handle, hw_params, SND_PCM_FORMAT_S16_LE);
  snd_pcm_hw_params_set_format (capture_handle, hw_params, SND_PCM_FORMAT_U8);
  int dir = 0;
  snd_pcm_hw_params_set_rate_near(capture_handle, hw_params, &exact_rate, &dir);
  snd_pcm_hw_params_set_channels (capture_handle, hw_params, channels);
  snd_pcm_hw_params (capture_handle, hw_params);

  snd_pcm_hw_params_free (hw_params);
  snd_pcm_prepare (capture_handle);



  
  
// printf("blah blub\n");
// snd_pcm_readi(capture_handle, buf, 8000);

  for (i = 0; i < (int)((float)(time_s*exact_rate) / 128.0); i++)
  { 
//     printf("file didn't open\n");
       snd_pcm_readi(capture_handle, buf, 128); 
//      printf("file didn't open\n");
//     write(fd,buf,128);
//     fwrite ( (void *) buf, sizeof(short), sizeof(buf), pFile);
  } 

// printf("blah blub\n");
    
//   for (int idx = 0; idx < 128; idx++) {
//     std::cout << idx << " " << buf[idx] << std::endl;
//   }
// FILE * pFile = fopen ("rec.rec","wb");
//     std::cout << "File ID " << pFile << std::endl;
//     std::cout << "File ID " << pFile << std::endl;
//     std::cout << "File ID " << pFile << std::endl;
//     std::cout << "File ID " << pFile << std::endl;
//     std::cout << "File ID " << pFile << std::endl;
//     fwrite ( (const void *) buf, sizeof(unsigned char), sizeof(buf), pFile);
//     printf("blah blub\n");
//   for (i = 0; i < (int)((float)(time_s*exact_rate) / 128.0); i++)
//   { 
// //     printf("file didn't open\n");
//        snd_pcm_readi(capture_handle, buf, 128); 
//      printf("file didn't open\n");
// //     write(fd,buf,128);
//     fwrite ( (void *) buf, sizeof(short), sizeof(buf), pFile);
//   } 
//   close(fd); 
  
//   fclose (pFile);
  
  
  
  /////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////
//   printf("blah blub\n");
//   // First get a factory instance that is used to create RSB domain
//     // objects.
//     Factory& factory = getFactory();
// printf("blah blub\n");
//     // Create an informer that is capable of sending events containing
//     // string data on the scope "/example/informer".
//     Informer<string>::Ptr informer
//         = factory.createInformer<string> ("/sense_mic");
// 	printf("blah blub\n");
//     shared_ptr<string> message(new string((const char *)buf));
//     printf("blah blub\n");
//     informer->publish(message);
// 	
//     printf("blah blub\n");
    snd_pcm_close (capture_handle);
  exit (0);
}
