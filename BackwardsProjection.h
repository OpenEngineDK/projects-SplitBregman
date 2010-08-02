#include "LineSegment.h"

REAL simpleBackwardsProjection(REAL x, REAL y,
                               Texture2DPtr(REAL) sinogram,
                               REAL startAngle,
                               REAL fanAngle,
                               REAL maxAngle,
                               REAL originalRadius,
                               bool useFanBeam) {
    REAL sum = 0.0;
    Vector<2,REAL> center(x+0.5,y+0.5);
    unsigned int numProjections = sinogram->GetWidth();
    for (unsigned int projection = 0; projection<numProjections; projection++) {
        //if(projection % 8 != 0) continue;

        REAL radius = originalRadius;
        // points defining target plate

        REAL halfFanWidth = (2.0*radius) * tan(fanAngle/2.0);
        Vector<2,REAL> uP(-radius, halfFanWidth); // upper projection end point
        Vector<2,REAL> lP(-radius, -halfFanWidth); // lower projection end point

        REAL angle = maxAngle *(projection/(REAL)numProjections); 
        //angle = startAngle - angle; // clockwise
        angle += startAngle; // counter clockwise

        // from: http://en.wikipedia.org/wiki/Rotation_matrix
        Matrix<2,2,REAL> rotation(cos(angle), -sin(angle),
                                  sin(angle),  cos(angle));
        uP =  rotation * uP;
        lP =  rotation * lP;

        LineSegment target(uP,lP);

#ifdef GUI
        mutexLines.Lock();
        lines.push_back( Line( Vector<3,float>(target.point1[0],target.point1[1],0.0),
                                   Vector<3,float>(target.point2[0],target.point2[1],0.0)) );
            mutexLines.Unlock();
#endif
        // project pixel/voxel onto recording plate
        // which gives a two intersection point (a line)
        REAL intersection = 0.0;
        bool intersects = false;

        if (useFanBeam) { // fan beam
            Vector<2,REAL> sourcePosition(radius, 0.0);
            sourcePosition =  rotation * sourcePosition;
            LineSegment ray(sourcePosition, center);
            intersects = ray.ProjectRayOntoLineSegment(target, &intersection);

#ifdef GUI
            Line line3d( Vector<3,float>(ray.point1[0],ray.point1[1],0.0),
                         Vector<3,float>(ray.point2[0],ray.point2[1],0.0) );

            mutexLines.Lock();
            lines.push_back( line3d );
            mutexLines.Unlock();
#endif            
        } else { // parallel beam
            intersects = target.ProjectPointOntoLineSegment(center,
                                                            &intersection);
        }
        if (!intersects) continue; //Core::Exception("no intersection");


        //REAL index = intersection * numPixelsPerProjection;
        //sum += (*sinogram)(projection, ceil(index));
//         logger.info << "index: " <<  index << logger.end;
//         logger.info << "intersection: " << intersection << logger.end;
        REAL value = 
            sinogram->InterpolatedPixel(projection/(REAL)numProjections, 
                                        intersection)[0];
        //if (value == 0) return 0; // outside volume!
        sum += value;
    }
    //logger.info << "sum: " << sum << logger.end;
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

void BP(std::string inputfilename, std::string outputname,
        REAL startAngle, REAL fanAngle, REAL maxAngle, REAL radius,
        unsigned int originalWidth, unsigned int originalHeight,
        bool useFanBeam) {

    ITexture2DPtr sinopng =
        ResourceManager<ITexture2D>::Create(inputfilename);
    sinopng->Load();
    // peel out one channel for gray scale from the RGBA texture.
    EmptyTextureResourcePtr sinoe =
        EmptyTextureResource::CloneChannel(sinopng,0);
    //@todo: RGBAToGrayScale()
    
    // convert to floating point arrays
    Texture2DPtr(REAL) sino = Utils::TexUtils::ToFloatTexture<REAL>(sinoe);

    Texture2DPtr(REAL) output( new Texture2D<REAL>(originalWidth,
                                                   originalHeight, 1));

    // backwards projection
    REAL hw = originalWidth / 2.0;
    REAL hh = originalHeight / 2.0;
    for (unsigned int x=0; x<originalWidth; x++) {
        //logger.info << "x: " << x << logger.end;
        for (unsigned int y=0; y<originalHeight; y++) {
            (*output)(x,y) =
                simpleBackwardsProjection(x-hw, y-hh, sino,
                                          startAngle, fanAngle, maxAngle,
                                          radius, useFanBeam);
#ifdef GUI
        mutexPoints.Lock();
        mutexLines.Lock();
        points.clear();
        lines.clear();
        mutexPoints.Unlock();
        mutexLines.Unlock();
#endif
        }
    }

    Utils::TexUtils::Normalize(output,0,1);

    // create a output canvas
    UCharTexture2DPtr outputChar = 
        Utils::TexUtils::ToUCharTexture<REAL>(output);
    TextureTool<unsigned char>
        ::DumpTexture(Utils::TexUtils::ToRGBAfromLuminance(outputChar),
                      outputname + ".png");
    TextureTool<REAL>
        ::DumpTexture(Utils::TexUtils::ToRGBAfromLuminance(output),
                      outputname + ".exr");
}
