
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
