

REAL simpleForwardProjection(unsigned int projection,
                             unsigned int targetPixel,
                             REAL angle,
                             Texture2DPtr(REAL) input,
                             unsigned int numProjections,
                             unsigned int numPixelsPerProjection,
                             unsigned int numSamplesPerRay,
                             REAL startAngle,
                             REAL fanAngle,
                             REAL radius, bool useFanBeam) {
    //logger.info << " ------------ " << logger.end;

    REAL halfFanWidth = (2.0*radius) * tan(fanAngle/2.0);
    REAL pixelWidth = (2.0*halfFanWidth) / numPixelsPerProjection;
    REAL pixelOffset = pixelWidth / 2.0;
    REAL pixelStartPosition = halfFanWidth - pixelOffset;

    // LineSegment
    REAL yPosition = pixelStartPosition - targetPixel * pixelWidth;

    Vector<2,REAL> sourcePosition;
    if (useFanBeam) { // fan beam
        sourcePosition = Vector<2,REAL>(radius, 0);
    } else { // parallel beam
        sourcePosition = Vector<2,REAL>(radius, yPosition);
    }

    Vector<2,REAL> pixelCenterPosition(-radius, yPosition);

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
#ifdef GUI

    mutexLines.Lock();
    lines.push_back( Line( Vector<3,float>(sourcePosition[0],
                                          sourcePosition[1], 0.0),
                           Vector<3,float>(pixelCenterPosition[0],
                                          pixelCenterPosition[1],0.0) ) );
    mutexLines.Unlock();
#endif

    unsigned int w = input->GetWidth();
    unsigned int h = input->GetHeight();
    REAL hw = w / 2.0;
    REAL hh = h / 2.0;
    REAL sum = 0.0;
    unsigned int counter = 0;
    for(unsigned int sampleIndex=0; sampleIndex<numSamplesPerRay;
        sampleIndex++) {
        //logger.info << "sample index: " << sampleIndex << logger.end;

        REAL i = ((REAL)sampleIndex) / numSamplesPerRay;
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
#ifdef GUI
        mutexPoints.Lock();
        points.push_back( samplePoint );
        mutexPoints.Unlock();
#endif
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

void FP(std::string inputfile, std::string outputname,
        unsigned int numProjections,
        unsigned int numPixelsPerProjection,
        unsigned int numSamplesPerLine,
        REAL startAngle,
        REAL fanAngle, REAL maxAngle, REAL radius, bool useFanBeam) {
    // load the RGBA gray scale input file
    //string filename = "shepp256.png";
    ITexture2DPtr tex =
        ResourceManager<ITexture2D>::Create(inputfile);
    tex->Load();
    // peel out one channel for gray scale from the RGBA texture.
    EmptyTextureResourcePtr ut_tex =
        EmptyTextureResource::CloneChannel(tex,0); //@todo: RGBAToGrayScale()
    // convert to floating point arrays
    Texture2DPtr(REAL) input = Utils::TexUtils::ToFloatTexture<REAL>(ut_tex);

    //unsigned int w = input->GetWidth();
    //unsigned int h = input->GetHeight();
    //logger.info << "input: " << inputfile << " ("
    //            << w << "x" << h << ")" << logger.end;
    //logger.info << "numPixelsPerProjection: " << numPixelsPerProjection << logger.end;

    Texture2DPtr(REAL) sinogram( new Texture2D<REAL>(numProjections,
                                                     numPixelsPerProjection, 1));

    // forward projection
    for (unsigned int projection=0; projection<numProjections; projection++) {
        //logger.info << "projection: " << projection << logger.end;
        REAL angle = maxAngle *(projection/(REAL)numProjections);
        //logger.info << "angle: " << angle << logger.end;

        for (unsigned int targetPixel=0; targetPixel<numPixelsPerProjection;
             targetPixel++) {
            //logger.info << "pixel line: " << pixelOnLine << logger.end;
            (*sinogram)(projection, targetPixel) =
                simpleForwardProjection(projection, targetPixel, angle, input,
                                        numProjections, numPixelsPerProjection,
                                        numSamplesPerLine, startAngle,
                                        fanAngle,radius,useFanBeam);
        }

#ifdef GUI
        mutexPoints.Lock();
        mutexLines.Lock();
        points.clear();
        lines.clear();
        mutexPoints.Unlock();
        mutexLines.Unlock();
#endif
    }

    Utils::TexUtils::Normalize(sinogram,0,1);

    // create a output canvas
    UCharTexture2DPtr sinogramChar = 
        Utils::TexUtils::ToUCharTexture<REAL>(sinogram);
    TextureTool<unsigned char>
        ::DumpTexture(Utils::TexUtils::ToRGBAfromLuminance(sinogramChar),
                      outputname + ".png");
    TextureTool<REAL>
        ::DumpTexture(Utils::TexUtils::ToRGBAfromLuminance(sinogram),
                      outputname + ".exr");
}
