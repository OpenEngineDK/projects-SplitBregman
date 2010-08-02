// includes from OpenEngine base
#include <Core/Thread.h>
#include <Logging/Logger.h>
#include <Logging/StreamLogger.h>
#include <Resources/Texture2D.h>
#include <Resources/Tex.h>
#include <Resources/ResourceManager.h>
#include <Resources/DirectoryManager.h>
#include <Utils/Timer.h>
#include <Utils/TexUtils.h>

// SimpleSetup
#include <Utils/SimpleSetup.h>
#include <Display/SDLEnvironment.h>
#include <Core/IListener.h>
#include <Core/EngineEvents.h>
#include <Renderers/IRenderer.h>

// includes from OpenEngine extensions
#include <Resources/FreeImage.h>
#include <Utils/TextureTool.h>

// other includes
#include <stdlib.h>

#include <Core/Mutex.h>
#include <Geometry/Line.h>

using namespace OpenEngine;
using namespace OpenEngine::Resources;
using namespace OpenEngine::Renderers;
using namespace Logging;

typedef float REAL;

Core::Mutex mutexLines;
Core::Mutex mutexPoints;
std::list<Geometry::Line> lines;
std::list< Vector<2,REAL> > points;

#include "ForwardProjection.h"
#include "BackwardsProjection.h"

/*
class Runner : public Core::Thread {
public:
    void Run() {
        run();
    }
};

class Painter
    : public Core::IListener<Renderers::RenderingEventArg> {
public:
    Painter(){}
    void Handle(RenderingEventArg arg) {
        Vector<3,float> color(1.0,0.0,0.0);
        Vector<3,float> left(-1.0,0.0,0.0);
        Vector<3,float> right(1.0,0.0,0.0);

//         color = Vector<3,float>(0.0,0.0,0.0);
//         left = Vector<3,float>(-1000.0,0.0,0.0);
//         right = Vector<3,float>(1000.0,0.0,0.0);
//         arg.renderer.DrawLine( Line(left,right), color, 1 );

//         color = Vector<3,float>(0.0,0.0,0.0);
//         left = Vector<3,float>(0.0,-1000.0,0.0);
//         right = Vector<3,float>(0.0,1000.0,0.0);
//         arg.renderer.DrawLine( Line(left,right), color, 1 );

//         color = Vector<3,float>(0.0,0.0,1.0);
//         left = Vector<3,float>(0.0,0.0,-1000.0);
//         right = Vector<3,float>(0.0,0.0,1000.0);


        color = Vector<3,float>(1.0,0.0,0.0);
        mutexLines.Lock();
        for (std::list<Geometry::Line>::iterator itr = lines.begin();
             itr != lines.end(); itr++) {
            arg.renderer.DrawLine( *itr, color, 1 );
        }
        mutexLines.Unlock();

//         color = Vector<3,float>(0.0,1.0,0.0);
//         mutexPoints.Lock();
//         for (std::list< Vector<2,REAL> >::iterator itr = points.begin();
//              itr != points.end(); itr++) {
//             Vector<2,REAL> p = *itr;
//             Vector<3,float> point(p[0],p[1],0.0);
//             arg.renderer.DrawPoint( point, color, 2 );
//         }
//         mutexPoints.Unlock();
        
    }
};
*/

#define dim 512*512
const REAL mu = 1;
const REAL lambda = 1;

Vector<dim,REAL> R(Vector<dim,REAL> input) {
    return input;
}

Vector<dim,REAL> RT(Vector<dim,REAL> input) {
    return input;
}

Vector<dim,REAL> transform(Vector<dim,REAL> p) {
    // @todo: use 8x1 -8 kernel
    REAL lpx = 0;
    REAL lpy = 0;
    return RT(R(p)) - lambda * lpx - lambda * lpy;
}

void ConjugateGradient() {
    // @todo: use backwards or centeral difference
    REAL gdbx = 0;
    REAL gdby = 0;

    Vector<dim,REAL> g; // recorded projections

    //Vector<dim,REAL> a = b - A*x0;
    Vector<dim,REAL> b = mu*RT(g) - lambda*gdbx - lambda*gdby;
    Vector<dim,REAL> xk(0.0f); // b0approx = x0 = 0, fra article
    Vector<dim,REAL> rk = b; // r0, a=b when x0=0;
    Vector<dim,REAL> pk = rk;

    REAL eta = 0.0001; // required accuracy
    REAL dotA = b * b;
    
    REAL dotRk = (rk * rk);
    while (dotRk/dotA > eta) {
        Vector<dim,REAL> q = transform(pk);
        REAL alphak = dotRk / (pk*q);
        xk = xk + alphak * pk;

        Vector<dim,REAL> rkp = rk - alphak * q;
        REAL betak = (rkp * rkp)/(rk*rk);

        pk = rkp + betak * pk;

        dotRk = (rk * rk);
    }
}

enum Method {Forward, Backward, Unknown};

int main(int argc, char** argv) {
    // timer to mesure execution time
    Utils::Timer timer;
    timer.Start();

    // create a logger to std out
    //StreamLogger* stdlog = new StreamLogger(&std::cout);
    //Logger::AddLogger(stdlog);
    // Create simple setup
    Display::SDLEnvironment* env = new Display::SDLEnvironment(1150,768);
    Utils::SimpleSetup* setup = new Utils::SimpleSetup("Split Bregman",env);
    setup->GetRenderer().SetBackgroundColor(Vector<4,float>(1.0));
    //setup->GetRenderer().PostProcessEvent().Attach(*(new Painter()));

    // prepare for resource loading
    DirectoryManager::AppendPath("./projects/SplitBregman/data/");
    ResourceManager<ITextureResource>::AddPlugin(new FreeImagePlugin());

    // @todo: set camera position and direction
    setup->GetCamera()->SetPosition(Vector<3,float>(0.0,0.0,500.0));
    //setup->GetCamera()->SetPosition(Vector<3,float>(0.0,0.0,1000.0));
    setup->GetCamera()->LookAt(Vector<3,float>(0.0));

    logger.info << "loading time: " << timer.GetElapsedTime() << logger.end;

    if (argc != 4) {
        logger.error << "command error, "
                     << "use SplitBregman <R|RT> <input file> " 
                     << "<output name>" << logger.end;
        exit(0);
    }
    /*
    class Option {
        unsigned int t;
        string longName;
        char shortName;
        string value;
    }

    class Options {
        std::vector<Option> options;
        std::list<Option> required;
        std::list<Option> optional;

        template <class T>
        void Required(string longName, string shortName) {
            Option option(typeid(T), longName, shortName);
            required.push_back();
        }

        template <class T>
        void Optional(string longName, string shortName, T defaultvalue) {
            Option option(typeid(T),longName, shortName);
            option.value = Utils::Convert::ToString(defaultvalue);
            optional.push_back(option);
        }

        template <class T>
        T Get(string str) {
            int index = -1;
            for (unsigned int i=0; i<options.size(); i++) {
                if (options[i] == str)
                    index = i;
            }
            if (index == -1)
                throw Core::Exception("option not found:" + str);
            return ToType<T>(options[index]);
        }

        template <class T>
        T ToType(string value) {
            if (typeid(T) == typeid(float))
                return atof(value);
            else if (typeid(T) == typeid(int))
                return atoi(value);
            else
                throw Core::Exception("typeid unknown");
        }

        void Parse(int agrc, char** argv) {
            std::string args[argc-1];
            unsigned int size = argc-1;
            for (unsigned int i=0; i<size; i++)
                args[i] = argv[i+1];



            for (unsigned int i=0; i<size; i++) {
                string arg = agrs[i];
                if (.startswith("");


            }

        }
    };

    // setup options
    Options options;
    options.Required<unsigned int>("size", 's');
    options.Optional<std::string>("name", 'n', "defaultname");

    // parse to std::list<std::string> 
    Options::parse(argc, argv);
*/

    std::string methodstr = argv[1];
    Method method = Unknown;
    if (methodstr == "R") {
        logger.info << "invoking Forward Radon transformation"
                    << logger.end;
        method = Forward;
    } else if (methodstr == "RT") {
        logger.info << "invoking Backward (inverse) Radon transformation" 
            << logger.end;
        method = Backward;
    } else {
        logger.error << "method error, "
                     << "use R or RT" << logger.end;
        exit(0);
    }

    std::string inputfile = argv[2];
    logger.info << "input file: "
                << inputfile << logger.end;

    std::string outputname = argv[3];
    logger.info << "output name: "
                << outputname << logger.end;

    const REAL startAngle = -PI/2;
    //const REAL fanAngle = (PI / 2);
    const REAL fanAngleInDegrees = 165.0/2.0; // good for fan
    //const REAL fanAngleInDegrees = 105.0/2.0; good for parallel
    const REAL fanAngle = PI*(fanAngleInDegrees/180.0);
    //const REAL maxAngle = PI;
    const REAL maxAngle = (3.0 * PI)/2.0; // fan beam
    //const REAL maxAngle = 2*PI; // for fanbeam
    const bool useFanBeam = true;

    if (method == Forward) {
        //const unsigned int numPixelsPerProjection = 512;
        //    use:   w>h ? h : w; // @todo is this right?
        //const unsigned int numProjections = 512;
        const unsigned int numProjections = 128;
        const unsigned int numPixelsPerProjection = 4*numProjections;
        //    ues:   w>h ? h : w; // @todo is this right?

        unsigned int w = 256;
        unsigned int h = 256;
        REAL hw = w / 2.0;
        REAL hh = h / 2.0;
        REAL radius = sqrt(hw*hw+hh*hh);
        unsigned int numSamplesPerRay = 2 * radius;
        //unsigned int numSamplesPerRay = 3 * 512;

        FP(inputfile, outputname,
           numProjections,
           numPixelsPerProjection,
           numSamplesPerRay,
           startAngle,
           fanAngle,
           maxAngle, radius, useFanBeam);
    } 
    else if (method == Backward) {
        unsigned int originalWidth = 256;
        unsigned int originalHeight = 256;
        REAL hw = originalWidth / 2.0;
        REAL hh = originalHeight / 2.0;
        REAL radius = sqrt(hw*hw+hh*hh);

        BP(inputfile, outputname, startAngle, fanAngle, maxAngle, radius,
           originalWidth, originalHeight, useFanBeam);
    } 

    /*
    bool useGUI = false;
    if (useGUI) {
        Runner* runner = new Runner();
        runner->Start();
        // Start the engine.
        setup->GetEngine().Start();
        runner->Wait();
    } else {
        run();
    }
    */
    logger.info << "execution time: " << timer.GetElapsedTime() << logger.end;
    return EXIT_SUCCESS;
}
