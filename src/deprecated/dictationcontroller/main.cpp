#include <yarp/sig/Image.h>
//#include <yarp/os/RFModule.h>
//#include <yarp/os/Module.h>
//#include <yarp/os/Network.h>
#include <yarp/os/all.h>
//#include <yarp/sig/Vector.h>
#include <yarp/sig/all.h>
#include <yarp/math/Math.h>
#include <yarp/dev/all.h>
#include <yarp/os/Time.h>
#include <iCub/ctrl/math.h>


#include <cv.h>
#include <highgui.h>
#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/calib3d/calib3d.hpp"

#include <iostream>
#include <algorithm>
#include <stdio.h>
#include <stdlib.h>
#include <cstdio>
#include <sstream>

//using namespace std;
//using namespace yarp::os;
//using namespace yarp::sig;

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::ctrl;

YARP_DECLARE_DEVICES(icubmod)


class MyModule:public RFModule
{
    PolyDriver      clientGaze;
    IGazeControl   *igaze;
    int prevyes;
    Port handlerPort; //a port to handle messages
    BufferedPort<Bottle>    gazeIn;
    BufferedPort<Bottle>    speechOut;
    BufferedPort<Bottle>    logOut;
	Port imgLPortOut;
	int online;
    int frame_counter;
    double prev;
    double prevgazetime;
    int speech_counter;
    vector<string> list;
    int gazeCount;
    float waittime;
    int withgaze;
    int order; // 1 - ABBA; 2 - BAAB
    int state; // 1 - init first; 2 - speech first; 3 - end first
    int initstate;
    double now;
    RpcClient speechStatusPort;
    string prevStr;
    int donespeaking;
    size_t wordnumber;
    size_t charnumber;
    int prevgaze;
    int english;
    int facecounter;
    float gazelength;
    float gazelen;
    double gazestart;
    int motorson;
    int firstspeech;
public:

    double getPeriod()
    {
        return 0.01; //module periodicity (seconds)
    }

    void log(double time, string strng)
    {
        stringstream logstream;
        logstream << setprecision(15) << time << " " << strng.c_str();
        cout << logstream.str() << endl;
        Bottle& b=logOut.prepare();
        b.clear();
        b.add(logstream.str().c_str());
        logOut.writeStrict();
    }

    void saythis(std::string text)
    {
        Bottle& b=speechOut.prepare();
        b.clear();
        b.add(text.c_str());
        speechOut.writeStrict();
        log(now, text.c_str());
        return;
    }

    bool updateModule()
    {
        now = Time::now();
        double dT = now - prev;

        Bottle ispeakreply, ispeakcmd;
        ispeakcmd.add("stat");
        speechStatusPort.write(ispeakcmd, ispeakreply);
        if (prevStr=="quiet" && ispeakreply.toString()=="speaking")         // starting to speak
        {
            if(firstspeech)     // there was no previous writing
            {
                gazelength=0.0;
             //   prevgazetime = Time::now();
            }
            else  // sum up previous writing
            {
                if(prevgaze)
                {
                    gazelength = gazelength + Time::now() - gazestart;
                    gazestart = Time::now();
//                    gazelength = gazelength + Time::now() - prevgazetime;
//                    prevgazetime = Time::now();
                }
                stringstream logstream1;
                logstream1 << "Gaze time is: " << gazelength;
                log(now, logstream1.str());
                gazelength=0.0;
            }
            log(now,"Starting to speak.");
            firstspeech=0;
        }
        if (prevStr=="speaking" && ispeakreply.toString()=="quiet")         // starting to write
        {
            if(prevgaze)
            {
                gazelength = gazelength + Time::now() - gazestart;
                gazestart = Time::now();
                //                gazelength = gazelength + Time::now() - prevgazetime;
            }
            stringstream logstream1;
            logstream1 << "Gaze time is: " << gazelength;
            log(now, logstream1.str());
//            log(now,"Done speaking.");
            log(now,"Starting to write.");
            gazelength=0.0;
        }

        Bottle* out=gazeIn.read(false);
        Bottle mutgaze;
        if(out!=NULL)                               // is there gaze information
        {
            //cerr << out->toString() << endl;
            Bottle& faces = out->findGroup("faces");
            //cerr << faces.toString() << endl;
            Bottle& face = faces.findGroup("face");
            mutgaze = face.findGroup("mutualgaze");
            Bottle& facerect = face.findGroup("facerect");
            int facex = facerect.get(1).asInt();
            int facey = facerect.get(2).asInt();
            int faceheight = facerect.get(3).asInt();
            int facewidth = facerect.get(4).asInt();
            int facecenterx = facex + facewidth+10;
            int facecentery = facey + faceheight;

            if(faceheight>0)
            {
                facecounter++;
                if(facecounter==30)                // the robot updates its target of gaze every second (facecounter==100)
                {
                    facecounter=0;
                    Vector face_center;
                    face_center.resize(2);
                    face_center[0]=facecenterx;
                    face_center[1]=facecentery-60;
                    if(frame_counter % 20 == 0)
                    {
                        if(motorson)
                        {
				cout << "Face center: " << face_center[0] << " " << face_center[1] << endl;
                            igaze->lookAtMonoPixel(0,face_center);
                        }
                    }
                }
            }
        }

        if(state==1 || state==4 || state==7 || state==10)
        {
            if(dT>13.0 && initstate==1)
            {
	    	if(english)
		{
			saythis("Hello I'm eye cub.");
                	initstate=1;
                	prev = Time::now();
                	state++;
                	gazelength=0.0;
		}
///			saythis("Hello I'm eye cub. Today we will perform a dictation.");
		else
			saythis("Ciao, sono aicab. Oggi faremo un dettato.");
                if(state==1 || state==10)
                {
                    if(order==1)
                    {
			if(english)
                        	
                        	saythis("This is procedure Alpha.");
			
			else			
			        saythis("Questa è la procedura Alpha.");
                        withgaze=0;
                    }
                    else
                    {
			if(english)
                        	saythis("This is procedure Beta.");
			else                        
				saythis("Questa è la procedura Beta.");
                        withgaze=1;
                    }

                }
                if(state==4 || state == 7)
                {
                    if(order==1)
                    {
			if(english)
	                        saythis("This is procedure Beta.");
			else
				saythis("Questa è la procedura Beta.");
                        withgaze=1;
                    }
                    else
                    {
                        if(english)
				saythis("This is procedure Alpha.");
			else
			        saythis("Questa è la procedura Alpha.");
	
                        withgaze=0;
                    }
                }
                initstate++;
                gazelength=0.0;
            }
            if(dT>22.0 && initstate==2)
            {
/*		if(english)
                	saythis("Please; write on the board the following sentences.");
		else                
			saythis("Per favore scriva sulla lavagna le seguenti frasi."); */
		if(english)
			saythis("Please, take your pen and be ready to write down the following sentences.");
		else
			saythis("Per favore, prendi il pennarello e scrivi le frasi che ti detto.");
                initstate++;
                gazelength=0.0;
            }
            if(dT>32.0 && initstate==3)
            {
                initstate=1;
                prev = Time::now();
                state++;
                gazelength=0.0;
            }
            firstspeech=1;
        }

        if(state==3 || state==6 || state==9)        // waiting for experimenter's button press to continue with next stage
        {
            log(now, "Waiting.");
            std::cin.clear();
            std::fflush(stdin);
            std::cin.get ();    // get c-string
            log(now, "Done waiting.");
            prev = Time::now();
            state++;
            gazelength=0.0;
	    yarp::sig::Vector azelr(3);
///	    azelr[0] = -30.0;
///	    azelr[1] = 0.0;
	    azelr[0] = 0.0;
	    azelr[1] = 20.0;
	    azelr[2] = 0.0;
            igaze->lookAtAbsAngles(azelr);
        }

        if(state==2 || state==5 || state==8 || state==11)
        {
            if(withgaze)                            // procedure Beta - with gaze
            {
                if(prevyes || initstate==1)
                {
                    initstate++;

/*                    stringstream logstream2;
                    logstream2 << "Gaze time is: " << gazelength;
                    log(now, logstream2.str());
*/
                    std::string hullo;
                    hullo = list.back();
                    list.pop_back();
                    saythis(hullo);
///		    saythis("Hi. How are you?");
                    prev = Time::now();
                    prevyes=0;
                }
            }
            else                                // procedure Alpha - without gaze
            {
                if (prevStr=="speaking" && ispeakreply.toString()=="quiet")
                {
                    donespeaking=1;
//                    waittime = 2.6*(wordnumber);
//                    waittime = 2.1*(wordnumber);
		    waittime = 0.55 * charnumber;          // 0.55 = 2.4
                    stringstream logstream;
                    logstream << "Wait time is: " << waittime;
                    log(now, logstream.str());
                    prev = Time::now();
                    dT = now - prev;
                }
                std::string hullo;
                if((donespeaking && dT>waittime) || initstate==1)
                {
                    initstate++;

/*                    stringstream logstream1;
                    logstream1 << "Gaze time is: " << gazelength;
                    log(now, logstream1.str());
*/

                    if(initstate==10)
                     {
                         state++;

                         stringstream logstream1;
                         logstream1 << "Gaze time is: " << gazelength;
                         log(now, logstream1.str());

                         gazelength=0.0;
			 if(english)
                         	saythis("End of procedure Alpha.");
			 else
				saythis("Fine della procedura Alpha.");
                         initstate=1;
                         prev = Time::now();
                     }
                     else
                     {
                        hullo = list.back();
                        wordnumber = std::count(hullo.begin(), hullo.end(), ' ')+1;
 			charnumber = hullo.length() - std::count(hullo.begin(), hullo.end(), '.');
                        list.pop_back();
                        saythis(hullo);
                        donespeaking=0;
                     }
                }
            }

            if(out!=NULL)                   // if message received from gaze module
            {
                int ismutualgaze = mutgaze.get(1).asInt();
                if(ismutualgaze==1 && prevgaze==0)
                {
                    log(now, "glance on");
                    gazestart = Time::now();
                }
                if(ismutualgaze==0 && prevgaze==1)
                {

                    log(now, "glance off");
                    gazelength = gazelength + Time::now() - gazestart;
                }
                prevgaze=ismutualgaze;
                if (ismutualgaze)
                {
                    if(withgaze && prevyes==0 && dT>3.0)
                    {
                        gazeCount++;
//                        if(gazeCount==10 && prevyes==0)
			gazelen = Time::now() - gazestart;
                        if(gazelen>0.15 && prevyes==0)
                        {
//                            log(now, "Gaze event.");
                            stringstream logstream1;
                            logstream1 << "Trigger gaze length: " << gazelen;
                            log(now, logstream1.str());

                            gazeCount=0;
                            prevyes = 1;
                            if(initstate==9)
                            {
//                                state++;

                                stringstream logstream1;
                                logstream1 << "Gaze time is: " << gazelength;
                                log(now, logstream1.str());

                                gazelength=0.0;
                                initstate=1;
				cout << "Blah" << endl; 
		list.push_back("Benvenuti.");
		list.push_back("Io sono AICAB.");
		list.push_back("Sono felice di vederti.");
		list.push_back("Ciao.");
		list.push_back("Come va?");
		list.push_back("Questa e casa mia.");
		list.push_back("Io sono un robot.");
		list.push_back("Ciao.");
/*				if(english)
                                	saythis("End of procedure Beta");
				else
					saythis("Fine della procedura Beta.");
                                prev = Time::now();
                                if(order==1)
                                    withgaze=0;
                                else
                                    withgaze=1;*/
                            }
                        }
                    }
                }
            }

        }
        prevStr = ispeakreply.toString();
        return true;
    }


    bool respond(const Bottle& command, Bottle& reply) 
    {
        cout<<"Got something, echo is on"<<endl;
        if (command.get(0).asString()=="quit")
            return false;
        else
            reply=command;
        return true;
    }

    /* 
    * Configure function. Receive a previously initialized
    * resource finder object. Use it to configure your module.
    * Open port and attach it to message handler.
    */
    bool configure(yarp::os::ResourceFinder &rf)
    {
/*
        list.push_back("The market is open.");          // 4
        list.push_back("The orange is sweet.");         // 4
        list.push_back("I love to play guitar.");       // 5
        list.push_back("He is in the kitchen.");        // 5
        list.push_back("I can speak English.");         // 4
        list.push_back("He is going home.");            // 4
        list.push_back("Tom is a funny man.");          // 5
        list.push_back("I have three apples.");         // 4

        list.push_back("The music; was good.");         // 4
        list.push_back("I am not a robot.");            // 5
        list.push_back("She is very pretty.");          // 4
        list.push_back("This song is great.");          // 4
        list.push_back("My friend has a horse.");       // 5
        list.push_back("The apple tasted good.");       // 4
        list.push_back("I like red flowers.");          // 4
        list.push_back("My horse runs very fast.");     // 5

        list.push_back("I have a nice house.");         // 5
        list.push_back("Jill wants a doll.");           // 4
        list.push_back("I have a computer.");           // 4
        list.push_back("Tom is stronger than Dan.");    // 5
        list.push_back("We, sing; a song.");            // 4
        list.push_back("I was very happy yesterday.");  // 5
        list.push_back("He eats white bread.");         // 4
        list.push_back("Jack wants a toy.");            // 4

        list.push_back("She is in the shower.");        // 5
        list.push_back("The baby plays with toys.");    // 5
        list.push_back("She is a teacher.");            // 4
        list.push_back("I have a nice box.");           // 5
        list.push_back("You look very happy.");         // 4
        list.push_back("The baby fell asleep.");        // 4
        list.push_back("They are; my friends.");        // 4
        list.push_back("I went to school.");            // 4
*/

	english = 1;
	if(english)
	{
		list.push_back("Benvenuti.");
		list.push_back("Io sono AICAB.");
		list.push_back("Sono felice di vederti.");
		list.push_back("Ciao.");
		list.push_back("Come va?");
		list.push_back("Questa e casa mia.");
		list.push_back("Io sono un robot.");
		list.push_back("Ciao.");

/*		list.push_back("Hi, how are you?");
		list.push_back("Hello.");
		list.push_back("I'm looking at you.");
		list.push_back("How is it going?");
		list.push_back("Hello, I'm eye cub.");
		list.push_back("Welcome to my home.");
		list.push_back("Hi there.");
		list.push_back("Hello.");
*/
/*		list.push_back("We sing a song.");
		list.push_back("I was very happy yesterday.");
		list.push_back("I have a computer.");
		list.push_back("Jill wants a doll.");
		list.push_back("He eats white bread.");
		list.push_back("I love to play guitar.");
		list.push_back("He is in the kitchen.");
		list.push_back("She has a nice bike.");

		list.push_back("I went to school.");
		list.push_back("Tom is stronger than Dan.");
		list.push_back("She is a teacher.");
		list.push_back("I like red flowers.");
		list.push_back("The music was good.");
		list.push_back("The apple tasted good.");
		list.push_back("You look very happy.");
		list.push_back("I have three apples.");

		list.push_back("Jack wants a toy.");
		list.push_back("The baby plays with toys.");
		list.push_back("I have a nice box.");
		list.push_back("This song is great.");
		list.push_back("Tom is a funny man.");
		list.push_back("My friend has a horse.");
		list.push_back("The baby fell asleep.");
		list.push_back("I can speak English.");

		list.push_back("I am not a robot.");
		list.push_back("My horse runs very fast.");
		list.push_back("They are going home.");
		list.push_back("She is very pretty.");
		list.push_back("The market is open.");
		list.push_back("She is in the shower.");
		list.push_back("They are; my friends.");
		list.push_back("The apple is sweet.");*/
	}
	else
	{
		list.push_back("Il bar è aperto.");
		list.push_back("Mi piace molto ballare."); 
		list.push_back("Non bevo il caffè.");
		list.push_back("Marco ha tanti cani.");
		list.push_back("Il mare è calmo.");
		list.push_back("Il cane abbaia spesso.");
		list.push_back("Sono allergico al latte.");
		list.push_back("La sedia è in camera.");

		list.push_back("La luce è rossa.");
		list.push_back("Lui ha sempre ragione.");
		list.push_back("Giovedí scorso era festa.");
		list.push_back("Oggi splende il sole.");
		list.push_back("Ho comprato il pane.");
		list.push_back("Guido tutti i giorni.");
		list.push_back("Vado spesso al mare.");
		list.push_back("La notte è buia.");

		list.push_back("Mi piace il gelato.");
		list.push_back("Devo scrivere un tema.");
		list.push_back("Lei ama cucire.");
		list.push_back("Io ballo la polka.");
		list.push_back("Gino canta molto bene.");
		list.push_back("Vado a dormire presto.");
		list.push_back("La lettera è firmata.");
		list.push_back("Gioco spesso a carte.");

		list.push_back("Mi piace l'opera.");
		list.push_back("Ho mangiato le mele.");
		list.push_back("Sua nonna sta bene.");
		list.push_back("Ho una maglia blu.");
		list.push_back("Il piatto è sul tavolo.");
		list.push_back("La giornata è piovosa.");
		list.push_back("Il film dura due ore.");
		list.push_back("La bottiglia è piena.");

	}
        gazeCount=0;
        speech_counter=0;
		online=1;
        prev = Time::now();
        waittime=5.0;
        
        withgaze=1;
        prevyes=0;
        state=1;
        order=2;
        initstate=1;
        prevStr = "quiet";
        donespeaking=1;
        wordnumber=0;
	charnumber=0;
        prevgaze=0;
        facecounter=0;
        gazelength=0.0;
	gazelen=0.0;
        motorson=1;
        firstspeech=0;

        if(motorson==1)
        {
            Property optGaze("(device gazecontrollerclient)");
            optGaze.put("remote","/iKinGazeCtrl");
            optGaze.put("local","/icub_eyetrack/gaze");
                printf("\nHello.\n");
            if (!clientGaze.open(optGaze))
            {
                printf("\nGAZE FAILED\n");
                return false;
            }
            else
                printf("\nGAZE OPEN\n");

            clientGaze.view(igaze);
	    igaze->blockNeckRoll(0.0);
            igaze->setNeckTrajTime(0.8);
            igaze->setEyesTrajTime(0.4);
	    yarp::sig::Vector azelr(3);
///	    azelr[0] = -30.0;
///	    azelr[1] = 0.0;
	    azelr[0] = 0.0;
	    azelr[1] = 20.0;
	    azelr[2] = 0.0;
            igaze->lookAtAbsAngles(azelr);
        }
        handlerPort.open("/dictationcontroller");
        gazeIn.open( "/dictationcontroller/gaze:i" );
        speechOut.open( "/dictationcontroller/speech:o" );
        logOut.open( "/dictationcontroller/log:o" );
        attach(handlerPort);
        Network::connect("/dlibgazer/out", "/dictationcontroller/gaze:i");
        Network::connect("/dictationcontroller/speech:o", "/iSpeak");
        speechStatusPort.open("/dictationcontroller/iSpeakrpc");
        Network::connect("/dictationcontroller/iSpeakrpc", "/iSpeak/rpc");

	frame_counter=0;
        cout<<"Done configuring!" << endl;
        return true;
    }

    /*
    * Interrupt function.
    */
    bool interruptModule()
    {
        cout<<"Interrupting your module, for port cleanup"<<endl;
        return true;
    }

    /*
    * Close function, to perform cleanup.
    */
    bool close()
    {
        cout<<"Calling close function\n";
        handlerPort.close();
        gazeIn.interrupt();
        gazeIn.close();
        speechOut.close();
        return true;
    }
};

int main(int argc, char * argv[])
{
    YARP_REGISTER_DEVICES(icubmod)

    Network yarp;
    MyModule module;
    ResourceFinder rf;
    rf.configure(argc, argv);
    rf.setVerbose(true);

    cout<<"Configure module..."<<endl;
    module.configure(rf);
    cout<<"Start module..."<<endl;
    module.runModule();
    cout<<"Main returning..."<<endl;
    return 0;
}



