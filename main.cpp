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

//const unsigned int numPixelsOnLine = 512; //w>h ? h : w; // @todo is this right?
//const unsigned int numProjections = 512;
const unsigned int numProjections = 600;
const unsigned int numPixelsOnLine = 4*numProjections; //w>h ? h : w; // @todo is this right?
const unsigned int numSamplesPerLine = 512 * 3;
const REAL startAngle = -PI/2;
//const REAL fanAngle = (PI / 2);
const REAL fanAngleInDegrees = 105/2;
const REAL fanAngle = PI*(fanAngleInDegrees/180.0);

REAL simpleForwardProjection(unsigned int projection,
                             unsigned int targetPixel,
                             REAL angle,
                             Texture2DPtr(REAL) input) {
    //logger.info << " ------------ " << logger.end;

    unsigned int w = input->GetWidth();
    unsigned int h = input->GetHeight();
    //unsigned int numPixelsOnLine = w>h ? h : w; // @todo is this right?
    REAL hw = w / 2.0;
    REAL hh = h / 2.0;

    REAL radius = sqrt(hw*hw+hh*hh);
    REAL halfFanWidth = (2*radius) * tan(fanAngle/2);
    REAL pixelWidth = (2*halfFanWidth) / numPixelsOnLine;
    REAL pixelOffset = pixelWidth / 2;
    REAL pixelStartPosition = halfFanWidth - pixelOffset;

    // LineSegment
    REAL pixelHeight = pixelStartPosition - targetPixel * pixelWidth;
    Vector<2,REAL> sourcePosition(radius, pixelHeight); // parallel beam
    //Vector<2,REAL> sourcePosition(radius, 0); // fan beam
    Vector<2,REAL> pixelCenterPosition(-radius, pixelHeight);

    /*
    logger.info << "source: " << sourcePosition << logger.end;
    logger.info << "pixelHeight: " << pixelHeight << logger.end;
    logger.info << "pixelCenterPosition: " << pixelCenterPosition << logger.end;
    */

    // from: http://en.wikipedia.org/wiki/Rotation_matrix
    // @todo: rotate source and pixel position 
    //angle = startAngle - angle; // clockwise
    angle += startAngle; // counter clockwise
    Matrix<2,2,REAL> rotation(cos(angle), -sin(angle),
                              sin(angle),  cos(angle));

    // @todo: check that the order is right.
    sourcePosition =  rotation * sourcePosition;
    pixelCenterPosition =  rotation * pixelCenterPosition;

    mutexLines.Lock();
    lines.push_back( Line( Vector<3,float>(sourcePosition[0],
                                          sourcePosition[1], 0.0),
                           Vector<3,float>(pixelCenterPosition[0],
                                          pixelCenterPosition[1],0.0) ) );
    mutexLines.Unlock();

    REAL sum = 0.0;
    unsigned int counter = 0;
    for(unsigned int sampleIndex=0; sampleIndex<numSamplesPerLine;
        sampleIndex++) {
        //logger.info << "sample index: " << sampleIndex << logger.end;

        REAL i = ((REAL)sampleIndex) / numSamplesPerLine;
        Vector<2,REAL> samplePoint =
            (1-i)*sourcePosition + i*pixelCenterPosition;
        
        //logger.info << "sample point: " << samplePoint << logger.end;

        // transform coordinates to image space
        REAL X = samplePoint[0] + hw;
        REAL Y = samplePoint[1] + hh;
        if ( !(X < 0 || X > w || Y < 0 || Y > h) ) {
            /*
            unsigned int Xi = floor(X*w);
            unsigned int Yi = floor(Y*h);
            logger.info << "Xi: " << Xi << logger.end;
            logger.info << "Yi: " << Yi << logger.end;
            sum += (*input)(Xi, Yi);
            */
            REAL value = 
                input->InterpolatedPixel(X/w, Y/h)[0];
            //input->GetPixelValues(X, Y)[0];
            sum += value;

            /*
            logger.info << "X: " << X
                        << " Y: " << Y 
                        << " C: " << counter 
                        << " value: " << value << logger.end;
            */
        mutexPoints.Lock();
        points.push_back( samplePoint );
        mutexPoints.Unlock();

            counter++;
        }
    }
    //sum /= counter;
    //logger.info << "sum: " << sum << logger.end;
    return sum;
}
/*
// precondition:
// angle must be between 0 and PI/2,
// if not symetry and rotation can be used.
REAL exactForwardProjection(unsigned int projection,
                             unsigned int pixelOnLine) {

    Vector<2,REAL> sourcePosition(radius,0);

    // @todo: reuse lower as upper in next iteration
    Vector<2,REAL> upperPixelPosition(-radius, pixelOnLine * pixelWidth);
    Vector<2,REAL> lowerPixelPosition(-radius, (pixelOnLine+1) * pixelWidth);

    Vector<2,REAL> upperDir =
        (sourcePosition-upperPixelPosition).GetNormalize();
    REAL upperSlope = upperDir[0]/upperDir[1]; // can fail if uDir[1]==0

    // @todo: rotate source and pixel position 

    // described as parameters on the line segments.
    REAL upperIntersections[w*2];
    REAL lowerIntersections[w*2];

    REAL sum = 0.0;
    for(unsigned int sampleIndex=0; sampleIndex<numSamplesPerLine;
        sampleIndex++) {
        REAL i = ((REAL)sampleIndex)/numSamplesPerLine;
        Vector<2,REAL> samplePoint =
            (1-i)*sourcePosition + i*pixelCenterPosition;
        sum += lookUp(samplePoint[0],samplePoint[1]);
    }
    return sum;
}
*/
/*
REAL simpleBackwardsProjection(unsigned int x, unsigned int y) {
    REAL sum = 0.0;

    std::vector< Vector<2,REAL> > corners;
    corners.push_back( Vector<2,REAL>(x,y) );
    corners.push_back( Vector<2,REAL>(x+1,y) );
    corners.push_back( Vector<2,REAL>(x,y+1) );
    corners.push_back( Vector<2,REAL>(x+1,y+1) );

    for (unsigned int projection = 0; projection<numProjections; projection++) {

        Vector<2,float> s; // source

        // points defining target plate
        Vector<2,REAL> uP; // upper projection end point
        Vector<2,REAL> lP; // lower projection end point
        Line target = Line(uP,lP);

        // @todo: rotate points

        // project pixel/voxel onto recording plate
        // which gives a two intersection point (a line)
        unsigned int numCorners = 4;
        REAL intersection[numCorners];
        for (unsigned int corner = 0; corner<numCorners;; corner++) {
            Line line = Line(s, corners[corner]);
            intersection[corner] = Intersect(target,line);
        }

        REAL minI = 0;
        REAL maxI = 1;
        for (unsigned int corner = 0; corner<numCorners;; corner++) {
            minI = min(minI, intersection[corner]);
            maxI = max(maxI, intersection[corner]);
        }
        REAL minIndex = minI * numPixelsPerProjection;
        REAL maxIndex = maxI * numPixelsPerProjection;

        // fx 127.8 and 136.9 and round these down to 127 and 136
        REAL minRoundedIndex = ceil(minIndex);
        REAL maxRoundedIndex = floor(maxIndex);

        // back project all lines between 127 and 136 
        //    to find intersection with pixel
        Vector<2,float> intersection_a1;
        Vector<2,float> intersection_a2;
        // @todo: calc from minRoundedIndex

        for (unsigned int index = minRoundedIndex+1; index <= maxRoundedIndex;
             index++) {
            Vector<2,float> intersection_b1;
            Vector<2,float> intersection_b2;
            // @todo: calc from index            

            // calcultate area ration for each area spanned by the lines
            // from: http://www.wikihow.com/Calculate-the-Area-of-a-Polygon
            REAL area;

            // sum for those between 127 and 136 by multiplying 
            //    with projection data (hole numbers)
            sum += area * (*sinogram)(projection,index);

            // reuse last calculated intersection point
            intersection_a1 = intersection_b1;
            intersection_a2 = intersection_b2;
        }

        // @todo: add the two corners that does not make 
        //        a clean cut on the projection plate

        // @todo: something speciale has to be done for the line
        //        crossing the center of the voxel
    }
    return sum;
}
*/

class Runner : public Core::Thread {
public:
void Run() {
    // load the RGBA gray scale input file
    string filename = "shepp256.png";
    ITexture2DPtr tex =
        ResourceManager<ITexture2D>::Create(filename);
    tex->Load();
    // peel out one channel for gray scale from the RGBA texture.
    EmptyTextureResourcePtr ut_tex =
        EmptyTextureResource::CloneChannel(tex,0); //@todo: RGBAToGrayScale()
    // convert to floating point arrays
    Texture2DPtr(REAL) input = Utils::TexUtils::ToFloatTexture<REAL>(ut_tex);

    REAL maxAngle = PI;
    unsigned int w = input->GetWidth();
    unsigned int h = input->GetHeight();
    logger.info << "input: " << filename << " ("
                << w << "x" << h << ")" << logger.end;
    logger.info << "numPixelsOnLine: " << numPixelsOnLine << logger.end;

    Texture2DPtr(REAL) sinogram( new Texture2D<REAL>(numProjections,
                                                     numPixelsOnLine, 1));

    // forward projection
    for (unsigned int projection=0; projection<numProjections; projection++) {
        logger.info << "projection: " << projection << logger.end;
        REAL angle = maxAngle *(projection/(REAL)numProjections);
        logger.info << "angle: " << angle << logger.end;

        for (unsigned int targetPixel=0; targetPixel<numPixelsOnLine;
             targetPixel++) {
            //logger.info << "pixel line: " << pixelOnLine << logger.end;
            (*sinogram)(projection, targetPixel) =
                simpleForwardProjection(projection, targetPixel, angle, input);
        }

        mutexPoints.Lock();
        mutexLines.Lock();
        points.clear();
        lines.clear();
        mutexPoints.Unlock();
        mutexLines.Unlock();
    }

    Utils::TexUtils::Normalize(sinogram,0,1);

    // create a output canvas
    UCharTexture2DPtr sinogramChar = Utils::TexUtils::ToUCharTexture<REAL>(sinogram);
    TextureTool<unsigned char>
        ::DumpTexture(Utils::TexUtils::ToRGBAfromLuminance(sinogramChar),
                      "sinogram.png");
    //@todo: dump as EXR

    TextureTool<REAL>
        ::DumpTexture(Utils::TexUtils::ToRGBAfromLuminance(sinogram),
                      "sinogram.exr");

    /*
REAL output[w][h];

    // backwards projection
    for (unsigned int x=0; x<w; x++) {
        for (unsigned int y=0; y<h; y++) {
            output[x][y] =
                simpleBackwardsProjection(x, y);
        }
    }


    // run initiali heat equation
    FloatTexture2DPtr v0 = J(0.24, u0);
    output = Utils::TexUtils::ToUCharTexture(v0);
    TextureTool<unsigned char>::DumpTexture(Utils::TexUtils::ToRGBAfromLuminance(output), "v0.png");

    FloatTexture2DPtr result = ROFEquation(0.000001, u0, v0);
    string outputFile = "output.png";
    output = Utils::TexUtils::ToUCharTexture(result);
    TextureTool<unsigned char>::DumpTexture(Utils::TexUtils::ToRGBAfromLuminance(output), outputFile);
    */

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

        color = Vector<3,float>(0.0,0.0,0.0);
        left = Vector<3,float>(-1000.0,0.0,0.0);
        right = Vector<3,float>(1000.0,0.0,0.0);
        arg.renderer.DrawLine( Line(left,right), color, 1 );

        color = Vector<3,float>(0.0,0.0,0.0);
        left = Vector<3,float>(0.0,-1000.0,0.0);
        right = Vector<3,float>(0.0,1000.0,0.0);
        arg.renderer.DrawLine( Line(left,right), color, 1 );
        /*
        color = Vector<3,float>(0.0,0.0,1.0);
        left = Vector<3,float>(0.0,0.0,-1000.0);
        right = Vector<3,float>(0.0,0.0,1000.0);
        */

        color = Vector<3,float>(1.0,0.0,0.0);
        mutexLines.Lock();
        for (std::list<Geometry::Line>::iterator itr = lines.begin();
             itr != lines.end(); itr++) {
            arg.renderer.DrawLine( *itr, color, 1 );
        }
        mutexLines.Unlock();

        /*
        color = Vector<3,float>(0.0,1.0,0.0);
        mutexPoints.Lock();
        for (std::list< Vector<2,REAL> >::iterator itr = points.begin();
             itr != points.end(); itr++) {
            Vector<2,REAL> p = *itr;
            Vector<3,float> point(p[0],p[1],0.0);
            arg.renderer.DrawPoint( point, color, 2 );
        }
        mutexPoints.Unlock();
*/
    }
};

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
    setup->GetRenderer().PostProcessEvent().Attach(*(new Painter()));

    // prepare for resource loading
    DirectoryManager::AppendPath("./projects/SplitBregman/data/");
    ResourceManager<ITextureResource>::AddPlugin(new FreeImagePlugin());

    // @todo: set camera position and direction
    setup->GetCamera()->SetPosition(Vector<3,float>(0.0,0.0,1000.0));
    setup->GetCamera()->LookAt(Vector<3,float>(0.0));

    Runner* runner = new Runner();
    runner->Start();

    logger.info << "loading time: " << timer.GetElapsedTime() << logger.end;

    // Start the engine.
    setup->GetEngine().Start();

    runner->Wait();
    return EXIT_SUCCESS;
}
