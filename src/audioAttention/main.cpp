#include <yarp/dev/AudioGrabberInterfaces.h>
#include <yarp/sig/Sound.h>
#include <yarp/dev/PolyDriver.h>

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;

int main(){
  PolyDriver dev("portaudio");
  IAudioGrabberSound	*grabber;
  dev.view(grabber);
  Sound sound;
  
  if (grabber!=NULL) {
      
    if (grabber->getSound(sound)) {
      
    }
  }
  
  return 0;
}
