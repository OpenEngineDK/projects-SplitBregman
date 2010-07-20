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

//const unsigned int numPixelsPerProjection = 512; //w>h ? h : w; // @todo is this right?
//const unsigned int numProjections = 512;
const unsigned int numProjections = 64;
const unsigned int numPixelsPerProjection = 4*numProjections; //w>h ? h : w; // @todo is this right?
const unsigned int numSamplesPerLine = 512 * 3;
const REAL startAngle = -PI/2;
//const REAL fanAngle = (PI / 2);
const REAL fanAngleInDegrees = 105/2;
const REAL fanAngle = PI*(fanAngleInDegrees/180.0);
const REAL maxAngle = PI;

REAL simpleForwardProjection(unsigned int projection,
                             unsigned int targetPixel,
                             REAL angle,
                             Texture2DPtr(REAL) input) {
    //logger.info << " ------------ " << logger.end;

    unsigned int w = input->GetWidth();
    unsigned int h = input->GetHeight();
    //unsigned int numPixelsPerProjection = w>h ? h : w; // @todo is this right?
    REAL hw = w / 2.0;
    REAL hh = h / 2.0;

    REAL radius = sqrt(hw*hw+hh*hh);
    REAL halfFanWidth = (2*radius) * tan(fanAngle/2);
    REAL pixelWidth = (2*halfFanWidth) / numPixelsPerProjection;
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

class LineSegment {
public:
    Vector<2,REAL> point1;
    Vector<2,REAL> point2;

    LineSegment(Vector<2,REAL> point1, Vector<2,REAL> point2)
        : point1(point1), point2(point2) {}

    bool Intersects(LineSegment line) {
        REAL value;
        return Intersection(line, &value);
    }

    // from: http://local.wasp.uwa.edu.au/~pbourke/geometry/lineline2d/
    bool Intersection(LineSegment line, REAL* value) {
        REAL x1 = point1[0];
        REAL x2 = point2[0];
        REAL x3 = line.point1[0];
        REAL x4 = line.point2[0];
        REAL y1 = point1[1];
        REAL y2 = point2[1];
        REAL y3 = line.point1[1];
        REAL y4 = line.point2[1];

        REAL denum = (y4-y3)*(x2-x1)-(x4-x3)*(y2-y1);
        logger.info << "denum: " << denum << logger.end;
        if (denum == 0) return false;
        //@todo: sammenfaldene eller parallele

        REAL ua =( (x4-x3)*(y1-y3)-(y4-y3)*(x1-x3) ) / denum;
        REAL x = x1 + ua*(x2-x1);
        REAL ub =( (x2-x1)*(y1-y3)-(y2-y1)*(x1-x3) ) / denum;
        REAL y = y1 + ub*(y2-y1);

        REAL paramx = (x-x1)/(x2-x1); //@todo:check this
        REAL paramy = (y-y1)/(y2-y1); //@todo:check this
        logger.info << "paramx: " << paramx << logger.end;
        logger.info << "paramy: " << paramy << logger.end;
        if ( !(0 <= paramx && paramx <= 1) ) return false;
        if ( !(0 <= paramy && paramy <= 1) ) return false;
        *value = paramx;
        return true;
    }

    // from: http://softsurfer.com/Archive/algorithm_0102/algorithm_0102.htm
    bool ProjectPointOntoLineSegment(Vector<2,REAL> p, REAL* value) {
        Vector<2,REAL> p0 = point1;
        Vector<2,REAL> p1 = point2;

        Vector<2,REAL> w = p - p0;

        Vector<2,REAL> vl = p1 - p0;

        REAL b = (w * vl) / (vl*vl);

        if (0<=b && b<=1) {
            *value = b;
            return true;
        } else
            return false;
    }

    // from: http://softsurfer.com/Archive/algorithm_0104/algorithm_0104B.htm
    bool ProjectRayOntoLineSegment(LineSegment line, REAL* value) {
        Vector<2,REAL> q0 = point1;
        Vector<2,REAL> q1 = point2;
        Vector<2,REAL> p0 = line.point1;
        Vector<2,REAL> p1 = line.point2;

        Vector<2,REAL> w = p0 - q0;
        Vector<2,REAL> v = q1 - q0;
        Vector<2,REAL> u = p1 - p0;
        /*
        logger.info << "w: " << w << logger.end;
        logger.info << "v: " << v << logger.end;
        logger.info << "u: " << u << logger.end;
        */
        REAL denom = v[0]*u[1]-v[1]*u[0];
        //logger.info << "denom: " << denom << logger.end;

        REAL si = (v[1]*w[0] - v[0]*w[1]) / denom; //on P0-P1
        //logger.info << "si: " << si << logger.end;
        //REAL ti = u[0]*w[1] - u[1]*w[0] / -denom; //on Q0-Q1

        if (0<=si && si<=1) {
            *value = si;
            return true;
        } else
            return false;
    }

    Vector<2,REAL> GetPointOnLine(REAL parameter) {
        return point1 * (1-parameter) + point2 * parameter;
    }

    LineSegment GetIntersectionLineSeqment(unsigned int x,
                                           unsigned int y) {

        std::vector< Vector<2,REAL> > corners;
        corners.push_back( Vector<2,REAL>(x  ,y) );
        corners.push_back( Vector<2,REAL>(x+1,y) );
        corners.push_back( Vector<2,REAL>(x+1,y+1) );
        corners.push_back( Vector<2,REAL>(x,  y+1) );
        // first point again
        corners.push_back( Vector<2,REAL>(x  ,y) );

        Vector<2,REAL> intersections[2];
        unsigned int counter = 0;
        for (unsigned int i=0; i<4; i++) {
            logger.info << "corner: " << i << logger.end;
            REAL a2 = 0.0;
            LineSegment l(corners[i],corners[i+1]);
            mutexLines.Lock();
            lines.push_back( Line( Vector<3,float>(l.point1[0],l.point1[1],0.0),
                                   Vector<3,float>(l.point2[0],l.point2[1],0.0)) );
            mutexLines.Unlock();
            logger.info << "line: " << l.ToString() << logger.end;            
            bool instsa2 = l.ProjectRayOntoLineSegment(*this,&a2);
            if (instsa2) {
                logger.info << "intersection: " << a2 << logger.end;
                if (counter > 1) {
                    //while(true) {}
                    throw Core::Exception("to many intersections");
                }
                else
                    intersections[counter++] = GetPointOnLine(a2);
            }
        }
        if (counter != 1)
            throw Core::Exception("not two intersection points!");
        return LineSegment(intersections[0], intersections[1]);
    }

    std::string ToString() {
        std::string str = "";
        str += "(p1:" + Utils::Convert::ToString(point1);
        str += ", p2:" + Utils::Convert::ToString(point2);
        str += ")";
        return str;
    }
};

REAL simpleBackwardsProjection(REAL x, REAL y,
                               Texture2DPtr(REAL) sinogram) {
    REAL sum = 0.0;

    Vector<2,REAL> center(x+0.5,y+0.5);

    logger.info << "(x,y): (" << x << ", " << y << ")" << logger.end; 
    for (unsigned int projection = 0; projection<numProjections; projection++) {

        unsigned int w = sinogram->GetWidth();
        unsigned int h = sinogram->GetHeight();
        REAL hw = w / 2.0;
        REAL hh = h / 2.0;
        REAL radius = sqrt(hw*hw+hh*hh);

        // points defining target plate
        REAL halfFanWidth = (2*radius) * tan(fanAngle/2);
        //REAL dist = 10000000000.0; //@todo: calc this
        Vector<2,REAL> uP(-radius, halfFanWidth); // upper projection end point
        Vector<2,REAL> lP(-radius, -halfFanWidth); // lower projection end point
        Vector<2,REAL> sourcePosition(radius, 0.0); // parallel beam

        REAL angle = maxAngle *(projection/(REAL)numProjections); 
        
        //logger.info << "projection: " << projection << logger.end; 
        //logger.info << "angle: " << angle << logger.end; 

        // from: http://en.wikipedia.org/wiki/Rotation_matrix
        // @todo: rotate source and pixel position 
        //angle = startAngle - angle; // clockwise
        angle += startAngle; // counter clockwise
        Matrix<2,2,REAL> rotation(cos(angle), -sin(angle),
                                  sin(angle),  cos(angle));

        // @todo: rotate points
        sourcePosition =  rotation * sourcePosition;
        uP =  rotation * uP;
        lP =  rotation * lP;

        LineSegment target(uP,lP);

        mutexLines.Lock();
        lines.push_back( Line( Vector<3,float>(target.point1[0],target.point1[1],0.0),
                                   Vector<3,float>(target.point2[0],target.point2[1],0.0)) );
            mutexLines.Unlock();
        // project pixel/voxel onto recording plate
        // which gives a two intersection point (a line)
        REAL intersection = 0.0;
        bool intersects = false;

        const bool useFanBeam = false;
        if(useFanBeam) {
            LineSegment ray(sourcePosition, center);
            Line line3d( Vector<3,float>(ray.point1[0],ray.point1[1],0.0),
                         Vector<3,float>(ray.point2[0],ray.point2[1],0.0) );
            mutexLines.Lock();
            lines.push_back( line3d );
            mutexLines.Unlock();
            
            intersects = ray.ProjectRayOntoLineSegment(target, &intersection);
        } else { // parallel beam
            intersects = target.ProjectPointOntoLineSegment(center,
                                                            &intersection);
        }
        if (!intersects) continue; //Core::Exception("no intersection");

        REAL index = intersection * numPixelsPerProjection;

//         logger.info << "intersection: " << intersection << logger.end;
//         logger.info << "index: " <<  index << logger.end;

        sum += (*sinogram)(projection, ceil(index));
        //sum += sinogram->InterpolatedPixel(projection, index)[0];
    }
    logger.info << "sum: " << sum << logger.end;
    return sum;
}

/*
REAL exactBackwardsProjection(REAL x, REAL y,
                               Texture2DPtr(REAL) sinogram) {
    REAL sum = 0.0;

    std::vector< Vector<2,REAL> > corners;
    corners.push_back( Vector<2,REAL>(x,y) );
    corners.push_back( Vector<2,REAL>(x+1,y) );
    corners.push_back( Vector<2,REAL>(x+1,y+1) );
    corners.push_back( Vector<2,REAL>(x,y+1) );

    logger.info << "(x,y): (" << x << ", " << y << ")" << logger.end; 
    for (unsigned int projection = 0; projection<numProjections; projection++) {

        unsigned int w = sinogram->GetWidth();
        unsigned int h = sinogram->GetHeight();
        REAL hw = w / 2.0;
        REAL hh = h / 2.0;
        REAL radius = sqrt(hw*hw+hh*hh);

        // points defining target plate
        REAL halfFanWidth = (2*radius) * tan(fanAngle/2);
        //REAL dist = 10000000000.0; //@todo: calc this
        Vector<2,REAL> uP(-radius, halfFanWidth); // upper projection end point
        Vector<2,REAL> lP(-radius, -halfFanWidth); // lower projection end point
        Vector<2,REAL> sourcePosition(radius, 0.0); // parallel beam

        REAL angle = maxAngle *(projection/(REAL)numProjections); 
        
        //logger.info << "projection: " << projection << logger.end; 
        //logger.info << "angle: " << angle << logger.end; 

        // from: http://en.wikipedia.org/wiki/Rotation_matrix
        // @todo: rotate source and pixel position 
        //angle = startAngle - angle; // clockwise
        angle += startAngle; // counter clockwise
        Matrix<2,2,REAL> rotation(cos(angle), -sin(angle),
                                  sin(angle),  cos(angle));

        // @todo: rotate points
        sourcePosition =  rotation * sourcePosition;
        uP =  rotation * uP;
        lP =  rotation * lP;

        LineSegment target(uP,lP);

        mutexLines.Lock();
        lines.push_back( Line( Vector<3,float>(target.point1[0],target.point1[1],0.0),
                                   Vector<3,float>(target.point2[0],target.point2[1],0.0)) );
            mutexLines.Unlock();
        // project pixel/voxel onto recording plate
        // which gives a two intersection point (a line)
        unsigned int numCorners = 4;
        REAL intersection[numCorners];
        for (unsigned int corner = 0; corner<numCorners; corner++) {
            LineSegment ray(sourcePosition, corners[corner]);
            
            mutexLines.Lock();
            lines.push_back( Line( Vector<3,float>(ray.point1[0],ray.point1[1],0.0),
                                   Vector<3,float>(ray.point2[0],ray.point2[1],0.0)) );
            mutexLines.Unlock();
            
            REAL value = 0.0; // @todo: not good default value
            bool intersects = ray.ProjectRayOntoLineSegment(target, &value);

            //getchar();
            if (!intersects) continue; //throw Core::Exception("no intersection");
            intersection[corner] = value;
        }
        //getchar();
        unsigned int count = 0;
        //while(count>1000000){count = count +1;}

        REAL minI = 1;
        REAL maxI = 0;
        for (unsigned int corner = 0; corner<numCorners; corner++) {
            minI = std::min(minI, intersection[corner]);
            maxI = std::max(maxI, intersection[corner]);
        }
        REAL minIndex = minI * numPixelsPerProjection;
        REAL maxIndex = maxI * numPixelsPerProjection;

        // fx 127.8 and 136.9 and round these down to 127 and 136
        REAL minRoundedIndex = ceil(minIndex);
        REAL maxRoundedIndex = floor(maxIndex);

//         logger.info << "minI: " << minI << logger.end;
//         logger.info << "maxI: " << maxI << logger.end;
//         logger.info << "minIndex: " << minIndex << logger.end;
//         logger.info << "maxIndex: " << maxIndex << logger.end;
//         logger.info << "minRoundedIndex: " << minRoundedIndex << logger.end;
//         logger.info << "maxRoundedIndex: " << maxRoundedIndex << logger.end;



        // @todo: add the two corners that does not make 
        //        a clean cut on the projection plate

        // (from minIndex to minRoundedIndex)
        REAL areaMin = 1.0;

//         LineSegment als(target.GetPointOnLine(minRoundedIndex), 
//                          sourcePosition);
//         logger.info << "als: " << als.ToString() << logger.end;

//         LineSegment a = als.GetIntersectionLineSeqment(x, y);
//         Vector<2,float> intersection_a1 = a.point1;
//         Vector<2,float> intersection_a2 = a.point2;
//         logger.info << "a: " << a.ToString() << logger.end;

        //logger.info << "minRoundedIndex: " << minRoundedIndex << logger.end;
        sum += areaMin * (*sinogram)(projection, minRoundedIndex-1);

        
        // (from maxIndex to maxRoundedIndex)

        if (minRoundedIndex < maxRoundedIndex && false) {

            logger.warning << "MIN < MAX rounded index - NOT TESTED !!!!!!!" << logger.end;
            getchar();
            if(false) {
            

        // back project all lines between 127 and 136 
        //    to find intersection with pixel
        // @todo: calc from minRoundedIndex
        LineSegment als(target.GetPointOnLine(minRoundedIndex), 
                         sourcePosition);

        mutexLines.Lock();
        lines.push_back( Line( Vector<3,float>(als.point1[0],als.point1[1],0.0),
                               Vector<3,float>(als.point2[0],als.point2[1],0.0)) );
        mutexLines.Unlock();
        
        logger.info << "als: " << als.ToString() << logger.end;

        LineSegment a = als.GetIntersectionLineSeqment(x, y);
        Vector<2,float> intersection_a1 = a.point1;
        Vector<2,float> intersection_a2 = a.point2;
        logger.info << "a: " << a.ToString() << logger.end;

        for (unsigned int index = minRoundedIndex+1; index <= maxRoundedIndex;
             index++) {

            // @todo: calc from index            
            LineSegment bls(target.GetPointOnLine(index), 
                            sourcePosition);
            LineSegment b = bls.GetIntersectionLineSeqment(x, y);
            Vector<2,float> intersection_b1 = b.point1;
            Vector<2,float> intersection_b2 = b.point2;


            // calcultate area ration for each area spanned by the lines
            // from: http://www.wikihow.com/Calculate-the-Area-of-a-Polygon
            REAL step2 =
                intersection_a1[0] * intersection_a2[1] +
                intersection_a2[0] * intersection_b2[1] +
                intersection_b2[0] * intersection_b1[1] +
                intersection_b1[0] * intersection_a1[1];

            REAL step3 =
                intersection_a1[1] * intersection_a2[0] +
                intersection_a2[1] * intersection_b2[0] +
                intersection_b2[1] * intersection_b1[0] +
                intersection_b1[1] * intersection_a1[0];

            REAL step4 = step3 - step2;

            REAL area = step4 / 2;
            logger.info << "index: " << index << logger.end; 
            logger.info << "intersection a: < " 
                        << intersection_a1 << " | "
                        << intersection_a2 << " >" << logger.end;

            logger.info << "intersection b: < " 
                        << intersection_b1 << " | "
                        << intersection_b2 << " >" << logger.end;

            logger.info << "area: " << area << logger.end;

            // sum for those between 127 and 136 by multiplying 
            //    with projection data (hole numbers)
            sum += area * (*sinogram)(projection, index);

            // reuse last calculated intersection point
            intersection_a1 = intersection_b1;
            intersection_a2 = intersection_b2;
        }
        }

        }        

        // @todo: something speciale has to be done for the line
        //        crossing the center of the voxel
    }
    logger.info << "sum: " << sum << logger.end;
    return sum;
}
*/

void run() {
    /*
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

    unsigned int w = input->GetWidth();
    unsigned int h = input->GetHeight();
    logger.info << "input: " << filename << " ("
                << w << "x" << h << ")" << logger.end;
    logger.info << "numPixelsPerProjection: " << numPixelsPerProjection << logger.end;

    Texture2DPtr(REAL) sinogram( new Texture2D<REAL>(numProjections,
                                                     numPixelsPerProjection, 1));

    // forward projection
    for (unsigned int projection=0; projection<numProjections; projection++) {
        logger.info << "projection: " << projection << logger.end;
        REAL angle = maxAngle *(projection/(REAL)numProjections);
        logger.info << "angle: " << angle << logger.end;

        for (unsigned int targetPixel=0; targetPixel<numPixelsPerProjection;
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
    UCharTexture2DPtr sinogramChar = 
        Utils::TexUtils::ToUCharTexture<REAL>(sinogram);
    TextureTool<unsigned char>
        ::DumpTexture(Utils::TexUtils::ToRGBAfromLuminance(sinogramChar),
                      "sinogram.png");
    TextureTool<REAL>
        ::DumpTexture(Utils::TexUtils::ToRGBAfromLuminance(sinogram),
                      "sinogram.exr");
    */


    string filename = "sinogram.png";
    ITexture2DPtr sinopng =
        ResourceManager<ITexture2D>::Create(filename);
    sinopng->Load();
    // peel out one channel for gray scale from the RGBA texture.
    EmptyTextureResourcePtr sinoe =
        EmptyTextureResource::CloneChannel(sinopng,0); //@todo: RGBAToGrayScale()
    // convert to floating point arrays
    Texture2DPtr(REAL) sino = Utils::TexUtils::ToFloatTexture<REAL>(sinoe);

    unsigned int ws = 512;
    unsigned int hs = 512;
    REAL hws = ws/2.0;
    REAL hhs = hs/2.0;

    Texture2DPtr(REAL) output( new Texture2D<REAL>(ws, hs, 1));

    // backwards projection
    for (unsigned int x=0; x<ws; x++) {
        for (unsigned int y=0; y<hs; y++) {
            (*output)(x,y) =
                simpleBackwardsProjection(x-hws, y-hhs, sino);

        mutexPoints.Lock();
        mutexLines.Lock();
        points.clear();
        lines.clear();
        mutexPoints.Unlock();
        mutexLines.Unlock();
        }
    }

    Utils::TexUtils::Normalize(output,0,1);

    // create a output canvas
    UCharTexture2DPtr outputChar = 
        Utils::TexUtils::ToUCharTexture<REAL>(output);
    TextureTool<unsigned char>
        ::DumpTexture(Utils::TexUtils::ToRGBAfromLuminance(outputChar),
                      "output.png");
    TextureTool<REAL>
        ::DumpTexture(Utils::TexUtils::ToRGBAfromLuminance(output),
                      "output.exr");
}

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
        /*
        color = Vector<3,float>(0.0,0.0,0.0);
        left = Vector<3,float>(-1000.0,0.0,0.0);
        right = Vector<3,float>(1000.0,0.0,0.0);
        arg.renderer.DrawLine( Line(left,right), color, 1 );

        color = Vector<3,float>(0.0,0.0,0.0);
        left = Vector<3,float>(0.0,-1000.0,0.0);
        right = Vector<3,float>(0.0,1000.0,0.0);
        arg.renderer.DrawLine( Line(left,right), color, 1 );
        */
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
    setup->GetCamera()->SetPosition(Vector<3,float>(0.0,0.0,500.0));
    //setup->GetCamera()->SetPosition(Vector<3,float>(0.0,0.0,1000.0));
    setup->GetCamera()->LookAt(Vector<3,float>(0.0));

    logger.info << "loading time: " << timer.GetElapsedTime() << logger.end;

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

    logger.info << "execution time: " << timer.GetElapsedTime() << logger.end;
    return EXIT_SUCCESS;
}
