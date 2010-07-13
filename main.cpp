// includes from OpenEngine base
#include <Logging/Logger.h>
#include <Logging/StreamLogger.h>
#include <Resources/Texture2D.h>
#include <Resources/Tex.h>
#include <Resources/ResourceManager.h>
#include <Resources/DirectoryManager.h>
#include <Utils/Timer.h>
#include <Utils/TexUtils.h>

// includes from OpenEngine extensions
#include <Resources/FreeImage.h>
#include <Utils/TextureTool.h>

// other includes
#include <stdlib.h>

using namespace OpenEngine;
using namespace OpenEngine::Resources;
using namespace Logging;

typedef float REAL;


REAL simpleForwardProjection(unsigned int projection,
                             unsigned int pixelOnLine,
                             REAL angle,
                             Texture2DPtr(REAL) input) {

    unsigned int w = input->GetWidth();
    unsigned int h = input->GetHeight();
    unsigned int numPixelsOnLine = w>h ? h : w; // @todo is this right?
    REAL hw = w / 2.0;
    REAL hh = h / 2.0;

    REAL radius = sqrt(hw*hw+hh*hh);
REAL fanAngle = (PI / 3);
REAL halfFanWidth = radius / tan(fanAngle);
REAL pixelWidth = (2*halfFanWidth) / numPixelsOnLine;
REAL pixelOffset = pixelWidth / 2;
REAL pixelStartPosition = halfFanWidth - pixelOffset;

unsigned int numSamplesPerLine = 512;


    // LineSegment
 Vector<2,REAL> sourcePosition(radius, 0);
    REAL pixelHeight = pixelStartPosition - pixelOnLine * pixelWidth;
    Vector<2,REAL> pixelCenterPosition(-radius, pixelHeight);

    /*
    logger.info << "source: " << sourcePosition << logger.end;
    logger.info << "pixelHeight: " << pixelHeight << logger.end;
    logger.info << "pixelCenterPosition: " << pixelCenterPosition << logger.end;
    */

    // from: http://en.wikipedia.org/wiki/Rotation_matrix
    // @todo: rotate source and pixel position 
    Matrix<2,2,REAL> rotation(cos(angle), -sin(angle),
                              sin(angle),  cos(angle));

    // @todo: check that the order is right.
    sourcePosition =  rotation * sourcePosition;
    pixelCenterPosition =  rotation * pixelCenterPosition;

    REAL sum = 0.0;
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
        //logger.info << "X: " << X << logger.end;
        //logger.info << "Y: " << Y << logger.end;
        if ( !(X < 0 || X > w || Y < 0 || Y > h) ) {
            /*
            unsigned int Xi = floor(X*w);
            unsigned int Yi = floor(Y*h);
            logger.info << "Xi: " << Xi << logger.end;
            logger.info << "Yi: " << Yi << logger.end;
            sum += (*input)(Xi, Yi);
            */
            sum += input->InterpolatedPixel(X, Y)[0];
        }
    }
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

int main(int argc, char** argv) {
    // timer to mesure execution time
    Utils::Timer timer;
    timer.Start();

    // create a logger to std out
    StreamLogger* stdlog = new StreamLogger(&std::cout);
    Logger::AddLogger(stdlog);

    // prepare for resource loading
    DirectoryManager::AppendPath("./projects/SplitBregman/data/");
    ResourceManager<ITextureResource>::AddPlugin(new FreeImagePlugin());

    // load the RGBA gray scale input file
    string filename = "shepp256.png";
    ITexture2DPtr tex =
        ResourceManager<ITexture2D>::Create(filename);
    tex->Load();
    // peel out one channel for gray scale from the RGBA texture.
    EmptyTextureResourcePtr ut_tex =
        EmptyTextureResource::CloneChannel(tex,0); //@todo: RGBAToGrayScale()
    // convert to floating point arrays
    FloatTexture2DPtr input = Utils::TexUtils::ToFloatTexture(ut_tex);

const unsigned int numProjections = 128;
REAL maxAngle = PI;
    unsigned int w = input->GetWidth();
    unsigned int h = input->GetHeight();
    unsigned int numPixelsOnLine = 128; //w>h ? h : w; // @todo is this right?
    logger.info << "input: " << filename << " ("
                << w << "x" << h << ")" << logger.end;
    logger.info << "numPixelsOnLine: " << numPixelsOnLine << logger.end;

    Texture2DPtr(REAL) sinogram( new Texture2D<REAL>(numProjections, numPixelsOnLine, 1));


    // forward projection
    for (unsigned int projection=0; projection<numProjections; projection++) {
        logger.info << "projection: " << projection << logger.end;
        REAL angle = maxAngle *(projection/(REAL)numProjections);
        logger.info << "angle: " << angle << logger.end;

        for (unsigned int pixelOnLine=0; pixelOnLine<numPixelsOnLine;
             pixelOnLine++) {
            //logger.info << "pixel line: " << pixelOnLine << logger.end;
            (*sinogram)(projection, pixelOnLine) =
                simpleForwardProjection(projection, pixelOnLine, angle, input);
        }
    }

    // create a output canvas
    UCharTexture2DPtr sinogramChar = Utils::TexUtils::ToUCharTexture(sinogram);
    TextureTool<unsigned char>
        ::DumpTexture(Utils::TexUtils::ToRGBAfromLuminance(sinogramChar),
                      "sinogram.png");

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

    logger.info << "execution time: " << timer.GetElapsedTime() << logger.end;
    return EXIT_SUCCESS;
}
