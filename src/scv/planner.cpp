
#include <stdio.h>
#include <algorithm>
#include "planner.h"

// The straight-line part of this is described here:
// http://www.et.byu.edu/~ered/ME537/Notes/Ch5.pdf

namespace scv {

// Stolen from GSL library
// https://github.com/Starlink/gsl/blob/master/poly/solve_quadratic.c
static int gsl_poly_solve_quadratic(double a, double b, double c, double *x0, double *x1)
{
  if (a == 0) /* Handle linear case */
    {
      if (b == 0)
        {
          return 0;
        }
      else
        {
          *x0 = -c / b;
          return 1;
        };
    }

  {
    double disc = b * b - 4 * a * c;

    if (disc > 0)
      {
        if (b == 0)
          {
            double r = sqrt (-c / a);
            *x0 = -r;
            *x1 =  r;
          }
        else
          {
            double sgnb = (b > 0 ? 1 : -1);
            double temp = -0.5 * (b + sgnb * sqrt (disc));
            double r1 = temp / a ;
            double r2 = c / temp ;

            if (r1 < r2)
              {
                *x0 = r1 ;
                *x1 = r2 ;
              }
            else
              {
                *x0 = r2 ;
                  *x1 = r1 ;
              }
          }
        return 2;
      }
    else if (disc == 0)
      {
        *x0 = -0.5 * b / a ;
        *x1 = -0.5 * b / a ;
        return 2 ;
      }
    else
      {
        return 0;
      }
  }
}



planner::planner()
{
    posLimitLower = vec3_zero;
    posLimitUpper = vec3_zero;
    velLimit = vec3_zero;
    accLimit = vec3_zero;
    jerkLimit = vec3_zero;
    resetTraverse();
}

bool planner::calculateMoves()
{
    bool invalidSettings = false;
    if ( velLimit.anyZero() ) {
        printf("Global velocity limit has zero component!\n");
        invalidSettings = true;
    }
    if ( accLimit.anyZero() ) {
        printf("Global acceleration limit has zero component!\n");
        invalidSettings = true;
    }
    if ( jerkLimit.anyZero() ) {
        printf("Global jerk limit has zero component!\n");
        invalidSettings = true;
    }
    if ( invalidSettings ) {
        printf("Aborting move calculation due to invalid global limits!\n");
        return false;
    }

    for (size_t i = 0; i < moves.size(); i++) {
        scv::move& l = moves[i];

        l.segments.clear();
        calculateMove(l);

        if ( i > 0 && l.blendType != CBT_NONE ) {
            move& l0 = moves[i-1];
            blendCorner( l0, l );
        }
    }

    for (size_t i = 0; i < moves.size(); i++) {
        move& l = moves[i];
        std::vector<segment>& segs = l.segments;
        segs.erase( std::remove_if(std::begin(segs), std::end(segs), [](segment& s) { return s.toDelete || s.duration <= 0; }), segs.end());
    }

    collateSegments();

    return true;
}

void planner::clear()
{
    moves.clear();
    segments.clear();
}

void planner::calculateMove(move& m)
{
    scv::vec3 srcPos = m.src;
    scv::vec3 dstPos = m.dst;
    scv::vec3 ldir = dstPos - srcPos;
    double llen = ldir.Normalize();

    scv::vec3 bv = getBoundedVector(ldir, velLimit);
    scv::vec3 ba = getBoundedVector(ldir, accLimit);
    scv::vec3 bj = getBoundedVector(ldir, jerkLimit);

    double v = min( (double)bv.Length(), m.vel); // target speed
    double a = min( (double)ba.Length(), m.acc);
    double j = min( (double)bj.Length(), m.jerk);
    double halfDistance = 0.5 * llen;     // half of the total distance we want to move

    double T = 2 * a / j;   // duration of both curve sections
    double T1 = 0.5 * T;    // duration of concave section
    double TL = 0;          // duration of linear asection
    double T2 = 0.5 * T;    // duration of convex section


    // Values at the start of each segment. These will be updated as we go to keep track of the overall progress.
    //    ps = starting position
    //    vs = starting velocity
    //    as = starting acceleration
    // They're all zero for the first segment, so let's set them for the second segment (the end of first the segment).
    double t = T1;
    double ps = (j * t * t * t) / 6.0;
    double vs = (j * t * t) / 2.0;
    double as = (j * t);


    double dvInCurve = (a*a) / (2*j);   // change in velocity caused by a curve segment
    double v1 = 0 + dvInCurve;          // velocity at end of concave curve
    double v2 = v - dvInCurve;          // velocity at start of convex curve

    if ( v1 > v2 ) {
        // Fully performing both curve segments would exceed the velocity set-point.
        // Need to reduce the time spent in each curve segment.
        // Alter T1,T2 such that the concave and convex segments meet with a tangential transition.
        T = sqrt(4*v*j) / j;
        T1 = (j * T) / (2 * j);
        T2 = T - T1;

        // recalculate values for end of first segment
        t = T1;
        ps = (j * t * t * t) / 6.0;
        vs = (j * t * t) / 2.0;
        as = (j * t);
    }
    else if ( v2 > v1 ) {
        // The velocity change due to the curve segments is not enough to reach the set-point velocity.
        // Add a linear segment in between them where velocity will change with a constant acceleration.
        double vr = v2 - v1; // remaining velocity to make up
        TL = vr / as;        // duration of linear segment

        // The inserted linear segment must not cause the required distance to be exceeded.
        // First, find out the total distance of all three segments:
        //     led = ss + vs * TL + as * TL * TL / 2.0;                                         distance at end of linear phase
        //     lev = vs + (as * TL);                                                            velocity at end of linear phase
        //     tot = led + (lev * T2) + ((as * T2 * T2) / 2.0) - ((j * T2 * T2 * T2) / 6.0);    distance at end of convex segment
        // These condense down to:
        //     tot =   0.5 * j * t * TL * TL   + 1.5 * j * t * t * TL   + j * t * t * t;
        // which is a quadratic equation for TL.

        double totalDistance =  0.5 * j * t * TL * TL   + 1.5 * j * t * t * TL   + j * t * t * t;

        if ( totalDistance > halfDistance ) {
            double qa = 0.5 * j * t;
            double qb = 1.5 * j * t * t;
            double qc = j * t * t * t       - halfDistance;
            double x0 = -99999;
            double x1 = -99999;
            gsl_poly_solve_quadratic( qa, qb, qc, &x0, &x1 );
            double bestSolution = max(x0, x1);
            if ( bestSolution >= 0 )
                TL = bestSolution;
        }
    }


    double bothCurvesDistance = (j * t * t * t);    // distance traveled during both concave and convex segments, if no linear phase involved
    if ( bothCurvesDistance > halfDistance ) {
        // Distance required is too short to allow fully performing both curves.
        // Reduce the time allowed in each curve. No linear phases will be used anywhere.

        T = std::cbrt( halfDistance / j );     // this is the bothCurvesDistance equation above, solved for t
        T1 = T2 = T;    // same duration for both concave and convex curves
        TL = 0;

        // recalculate values for end of first segment
        t = T1;
        ps = (j * t * t * t) / 6.0;
        vs = (j * t * t) / 2.0;
        as = (j * t);
    }

    scv::vec3 origin = srcPos;

    // segment 1, concave rising
    segment c1;
    c1.pos = origin;
    c1.vel = scv::vec3_zero;
    c1.acc = scv::vec3_zero;
    c1.jerk = j * ldir;
    c1.duration = T1;
    m.segments.push_back(c1);

    // ss,vs,as are already calculated above, no need to change them for this one

    // segment 2, rising linear phase (maybe)
    if ( TL > 0 ) {
        segment c2;
        c2.pos = origin + ps * ldir;
        c2.vel = vs * ldir;
        c2.acc = as * ldir;
        c2.jerk = scv::vec3_zero;
        c2.duration = TL;
        m.segments.push_back(c2);

        t = TL;
        ps += (vs * t) + (as * t * t) / 2.0;
        vs += (as * t);
    }

    // segment 3, convex rising
    segment c3;
    c3.pos = origin + ps * ldir;
    c3.vel = vs * ldir;
    c3.acc = as * ldir;
    c3.jerk = -j * ldir;
    c3.duration = T2;
    m.segments.push_back(c3);

    t = T2;
    ps += (vs * t) + ((as * t * t) / 2.0) - ((j * t * t * t) / 6.0);
    vs += ((j * t * t) / 2.0);
    as = 0;

    // segment 4, constant velocity linear phase (maybe)
    double totalRiseDistance = 2 * ps;
    double remainingDistance = llen - totalRiseDistance;
    if ( remainingDistance > 0.000001 ) {

        segment c4;
        c4.pos = origin + ps * ldir;
        c4.vel = vs * ldir;
        c4.acc = scv::vec3_zero;
        c4.jerk = scv::vec3_zero;
        c4.duration = remainingDistance / v;
        m.segments.push_back(c4);

        ps += vs * c4.duration; // a nice simple calculation for a change
    }

    // segment 5, convex falling
    segment c5;
    c5.pos = origin + ps * ldir;
    c5.vel = vs * ldir;
    c5.acc = as * ldir;
    c5.jerk = -j * ldir;
    c5.duration = T2;
    m.segments.push_back(c5);

    t = T2;
    ps += (vs * t) + ((as * t * t) / 2.0) + (-j * t * t * t) / 6.0;
    vs += (-j * t * t) / 2.0;
    as += (-j * t);

    // segment 6, falling linear phase (maybe)
    if ( TL > 0 ) {
        segment c6;
        c6.pos = origin + ps * ldir;
        c6.vel = vs * ldir;
        c6.acc = as * ldir;
        c6.jerk = scv::vec3_zero;
        c6.duration = TL;
        m.segments.push_back(c6);

        t = TL;
        ps += (vs * t) + (as * t * t) / 2.0;
        vs += (as * t);
    }

    // segment 7, falling convex
    segment c7;
    c7.pos = origin + ps * ldir;
    c7.vel = vs * ldir;
    c7.acc = as * ldir;
    c7.jerk = j * ldir;
    c7.duration = T1;
    m.segments.push_back(c7);
}

void planner::getSegmentState(segment& s, double t, vec3* pos, vec3* vel, vec3* acc, vec3* jerk )
{
    *pos = s.pos + (t * s.vel) + ((t * t) / 2.0) * s.acc + ((t * t * t) / 6.0) * s.jerk;
    *vel = s.vel + (t * s.acc) + ((t * t) / 2.0) * s.jerk;
    *acc = s.acc + t * s.jerk;
    *jerk = s.jerk;
}

void planner::getSegmentPosition(segment& s, double t, scv::vec3* pos )
{
    *pos = s.pos + (t * s.vel) + ((t * t) / 2.0) * s.acc + ((t * t * t) / 6.0) * s.jerk;
}

bool planner::getTrajectoryState(double t, int *segmentIndex, vec3 *pos, vec3 *vel, vec3 *acc, vec3 *jerk)
{
    // no segments, return zero vectors
    if ( segments.empty() ) {
        *segmentIndex = -1;
        *pos = vec3_zero;
        *vel = vec3_zero;
        *acc = vec3_zero;
        *jerk = vec3_zero;
        return false;
    }

    // time is negative, return starting point
    if ( t <= 0 ) {
        *pos = segments[0].pos;
        *vel = segments[0].vel;
        *acc = segments[0].acc;
        *jerk = segments[0].jerk;
        return t == 0;
    }

    double totalT = 0;
    size_t segmentInd = 0;
    while (segmentInd < segments.size()) {
        *segmentIndex = segmentInd;
        segment& s = segments[segmentInd];
        double endT = totalT + s.duration;
        if ( t >= totalT && t < endT ) {
            getSegmentState(s, t - totalT, pos, vel, acc, jerk);
            return true;
        }
        segmentInd++;
        totalT = endT;
    }

    // time exceeds total time of trajectory, return end point
    scv::segment& lastSegment = segments[segments.size()-1];
    getSegmentState(lastSegment, lastSegment.duration, pos, vel, acc, jerk);
    return false;
}

scv_float planner::getTotalTime()
{
    scv_float t = 0;
    size_t segmentInd = 0;
    while (segmentInd < segments.size()) {
        segment& s = segments[segmentInd];
        t += s.duration;
        segmentInd++;
    }
    return t;
}

void planner::resetTraverse()
{
    traversal_segmentIndex = 0;
    traversal_segmentTime = 0;
}

bool planner::advanceTraverse(double dt, vec3 *p)
{
    if ( segments.size() < 1 ) {
        *p = vec3_zero;
        return false;
    }

    traversal_segmentTime += dt;
    segment& seg = segments[traversal_segmentIndex];

    // Use 'while' here to consume zero-duration (or otherwise very short) segments immediately!
    // It's pretty important to make sure that dt is actually within the next segment instead of
    // just assuming it is, otherwise we might return a location beyond the end of the next
    // segment, and then in the following iteration a location near the start of the following
    // segment, which could potentially reverse the direction of travel!
    while ( traversal_segmentTime > seg.duration ) {
        // exceeded current segment
        if ( traversal_segmentIndex < ((int)segments.size()-1) ) {
            // more segments remain
            traversal_segmentIndex++;
            traversal_segmentTime -= seg.duration;
            seg = segments[traversal_segmentIndex];
        }
        else {
            // already on final segment
            getSegmentPosition(seg, seg.duration, p);
            return false;
        }
    }

    getSegmentPosition(seg, traversal_segmentTime, p);
    return true;
}

scv::vec3 getClosestPointOnInfiniteLine(scv::vec3 line_start, scv::vec3 line_dir, scv::vec3 point, double* d)
{
    *d = scv::dot( point - line_start, line_dir);
    return line_start + *d * line_dir;
}

// This assumes the jerk and acceleration are in the same direction
scv_float calculateDurationFromJerkAndAcceleration(scv::vec3 j, scv::vec3 a)
{
    if ( j.x != 0 )
        return sqrt( a.x / j.x );
    else if ( j.y != 0 )
        return sqrt( a.y / j.y );
    else if ( j.z != 0 )
        return sqrt( a.z / j.z );
    // jerk is zero, so duration would be infinite!
    return 0;
}

void markSkippedSegments(move& l, int whichEnd)
{
    int numSegments = l.segments.size();
    if ( whichEnd == 0 ) { // remove the latter end
        if ( numSegments == 5 ) {
            l.segments[3].toDelete = true;
            l.segments[4].toDelete = true;
        }
        else {
            l.segments[4].toDelete = true;
            l.segments[5].toDelete = true;
            l.segments[6].toDelete = true;
        }
    }
    else { // remove the beginning
        if ( numSegments == 5 ) {
            l.segments[0].toDelete = true;
            l.segments[1].toDelete = true;
        }
        else {
            l.segments[0].toDelete = true;
            l.segments[1].toDelete = true;
            l.segments[2].toDelete = true;
        }
    }
}

void planner::blendCorner(move& l0, move& l1)
{

    int numPrevSegments = l0.segments.size();
    int numNextSegments = l1.segments.size();

    if ( ! (numPrevSegments == 5 || numPrevSegments == 7) ||
         ! (numNextSegments == 5 || numNextSegments == 7))
        return;

    scv::vec3 l0srcPos = l0.src;
    scv::vec3 l0dstPos = l0.dst;

    scv::vec3 l1srcPos = l1.src;
    scv::vec3 l1dstPos = l1.dst;

    scv::vec3 l0dir = l0dstPos - l0srcPos;
    scv::vec3 l1dir = l1dstPos - l1srcPos;
    l0dir.Normalize();
    l1dir.Normalize();

    // find the constant speed sections in the middle of each move
    segment& seg0 = numPrevSegments == 5 ? l0.segments[2] : l0.segments[3];
    segment& seg1 = numNextSegments == 5 ? l1.segments[2] : l1.segments[3];
    segment& seg2 = numNextSegments == 5 ? l1.segments[3] : l1.segments[4]; // segment after the outgoing linear phase


    scv::vec3 v0 = seg0.vel;
    scv::vec3 v1 = seg1.vel;

    scv::vec3 dv = v1 - v0;
    scv::vec3 jerkDir = dv;
    jerkDir.Normalize();

    scv::vec3 a = jerkDir;
    scv::vec3 j = jerkDir;

    // in each axis, make these vectors too long, then trim them down
    a *= 1.5 * scv::max(accLimit.x, accLimit.y);
    j *= 1.5 * scv::max(jerkLimit.x, jerkLimit.y);

    if ( fabs(a.x) > accLimit.x )
        a *= accLimit.x / fabs(a.x);
    if ( fabs(a.y) > accLimit.y )
        a *= accLimit.y / fabs(a.y);
    if ( fabs(a.z) > accLimit.z )
        a *= accLimit.z / fabs(a.z);

    if ( fabs(j.x) > jerkLimit.x )
        j *= jerkLimit.x / fabs(j.x);
    if ( fabs(j.y) > jerkLimit.y )
        j *= jerkLimit.y / fabs(j.y);
    if ( fabs(j.z) > jerkLimit.z )
        j *= jerkLimit.z / fabs(j.z);

    double amag = a.Length();
    double jmag = j.Length();
    if ( l0.acc < amag )
        a *= l0.acc / amag;
    if ( l0.jerk < jmag )
        j *= l0.jerk / jmag;

    double maxJerkLim = 1; // max allowable jerk for smooth velocity transition

    // at least one component of dv should be non-zero, so use the highest one for this part
    if ( fabs(dv.x) > 0 ) {
        double mjx = (a.x*a.x) / dv.x;
        maxJerkLim = scv::min(maxJerkLim, (double)fabsf(mjx / j.x));
    }
    if ( fabs(dv.y) > 0 ) {
        double mjy = (a.y*a.y) / dv.y;
        maxJerkLim = scv::min(maxJerkLim, (double)fabsf(mjy / j.y));
    }
    if ( fabs(dv.z) > 0 ) {
        double mjz = (a.z*a.z) / dv.z;
        maxJerkLim = scv::min(maxJerkLim, (double)fabsf(mjz / j.z));
    }

    if ( maxJerkLim < 1 ) { // only use this to reduce jerk
        j *= maxJerkLim;
    }


    scv::vec3 earliestStart;
    scv::vec3 latestStart;
    scv::vec3 earliestEnd;
    scv::vec3 latestEnd;

    double maxJerkLength = 0;


    double T = 0;

    scv::vec3 startPoint = 0.5 * (l0srcPos + l0dstPos);
    scv::vec3 endPoint =   0.5 * (l1srcPos + l1dstPos);

    if ( dv.Length() < 0.00001 ) {
        double distance = (endPoint - startPoint).Length();
        T = 0.5 * distance / v0.Length();
    }
    else {
        T = calculateDurationFromJerkAndAcceleration(j, v1-v0);
    }

    bool doubleBack = false;

    double dot = scv::dot(l1dir, l0dir);
    dot = scv::min( 1.0, scv::max(-1.0, dot) );
    float angleToTurn = acos( dot );
    if ( angleToTurn < 0.00001 ) {

        // easy case where movement doesn't turn

        double t = T;

        scv::vec3 maxJerkEndPoint = 2 * t * v0    +    (t * t * t) * j;
        maxJerkLength = maxJerkEndPoint.Length();

        segment& seg0After =  numPrevSegments == 5 ? l0.segments[3] : l0.segments[4];

        earliestStart = startPoint;
        latestEnd = endPoint;
        latestStart = seg0After.pos;
        earliestEnd = seg1.pos;

    }
    else if ( angleToTurn > 3.14159 ) {

        // A special annoying case of movement going back in the exact direction it came from.

        double curveSpan = 0; // the furthest point the deceleration curve will reach, measured from the start (or finish) point, whichever is furthest

        double qa = j.Length() / 2.0;
        double qb = 0;
        double qc = -v0.Length();
        double x0 = -99999;
        double x1 = -99999;
        gsl_poly_solve_quadratic( qa, qb, qc, &x0, &x1 );
        double t = scv::max(x0, x1);

        scv::vec3 p0 =      (t * v0) + (( t * t * t) / 6.0) * j; // furthest point reached in first half of reversal
        curveSpan = scv::max(curveSpan, p0.Length());

        qa = j.Length() / 2.0;
        qb = 0;
        qc = -v1.Length();
        x0 = 0;
        x1 = 0;
        gsl_poly_solve_quadratic( qa, qb, qc, &x0, &x1 );
        t = scv::max(x0, x1);

        scv::vec3 p1 = /*sh +*/ (t * v1) /*+ ((t * t) / 2.0) * ah*/ + ((t * t * t) / 6.0) * -j; // furthest point reached in second half of reversal
        curveSpan = scv::max(curveSpan, p1.Length());

        t = T;
        scv::vec3 maxJerkDelta = 2 * t * v0    +    (t * t * t) * j;

        double longestAllowableLength = (startPoint - l0dstPos).Length();
        longestAllowableLength = scv::min(longestAllowableLength, (endPoint - l0dstPos).Length());
        if ( longestAllowableLength == 0 )
            return; // impossible

        double ratio = (curveSpan + maxJerkDelta.Length()) / longestAllowableLength;
        if ( ratio > 1 )
            return; // not enough room

        if ( l1.blendType == CBT_MIN_JERK ) {
            j *= ratio*ratio;

            T = calculateDurationFromJerkAndAcceleration(j, v1-v0);

            curveSpan /= ratio;
            maxJerkDelta *= 1 / ratio;
        }

        scv::vec3 v0dir = v0;
        v0dir.Normalize();
        startPoint = l0dstPos + -curveSpan * v0dir;
        endPoint = startPoint; // will change below

        if ( scv::dot(maxJerkDelta, v0dir) < 0 )
            maxJerkDelta *= -1;
        if ( v0.LengthSquared() > v1.LengthSquared() )
            startPoint += -maxJerkDelta;
        else
            endPoint += -maxJerkDelta;

        doubleBack = true;
    }
    else {

        double t = T;

        scv::vec3 finalVelocity = v0 + t * t * j;
        finalVelocity.Normalize();


        scv::vec3 curveEndPoint = 2 * t * v0    +    (t * t * t) * j;


        // find the usable start and end of each constant speed section
        scv::vec3 seg0Start, seg0End, seg1Start, seg1End;

        seg0Start = 0.5 * (l0srcPos + l0dstPos); // earliest start is at middle of preceding move
        seg0End = l0dstPos; // latest start is at end of preceding move

        seg1Start = l0dstPos;// seg0End; // earliest end is a beginning of following move
        seg1End = 0.5 * (l1srcPos + l1dstPos); // latest end is at middle of following segment


        scv::vec3 projBase = l0dstPos;

        scv::vec3 curveEndPointNormalized = curveEndPoint;
        curveEndPointNormalized.Normalize();
        scv_float dummy;
        scv::vec3 cpoSpan = getClosestPointOnInfiniteLine( l0srcPos, curveEndPointNormalized, projBase, &dummy);

        scv::vec3 dirForProjection = projBase - cpoSpan;

        dirForProjection.Normalize();

        double A0, A1, B0, B1;

        getClosestPointOnInfiniteLine( projBase, dirForProjection, seg0Start, &A0);
        getClosestPointOnInfiniteLine( projBase, dirForProjection, seg0End, &A1 );
        getClosestPointOnInfiniteLine( projBase, dirForProjection, seg1Start, &B0 );
        getClosestPointOnInfiniteLine( projBase, dirForProjection, seg1End, &B1 );

        double D0 = A1;
        double D1 = B0;

        D0 = A0;
        D1 = B1;

        // ensure the 0 is less than the 1
        if ( A0 > A1 )
            std::swap(A0, A1);
        if ( B0 > B1 )
            std::swap(B0, B1);

        if (( A0 > B0 && A0 > B1) ||
            ( A1 < B0 && A1 < B1)) {
            // no overlap between constant velocity sections
            return;
        }

        double ds[4];
        ds[0] = A0;
        ds[1] = A1;
        ds[2] = B0;
        ds[3] = B1;
        std::sort(std::begin(ds), std::end(ds));

        double inner = ds[1];
        double outer = ds[2];
        if ( fabs(inner) > fabs(outer) )
            std::swap(inner, outer);

        earliestStart = projBase + (outer / D0) * (seg0Start - projBase);
        latestStart =   projBase + (inner / D0) * (seg0Start - projBase);
        earliestEnd =   projBase + (inner / D1) * (seg1End - projBase);
        latestEnd =     projBase + (outer / D1) * (seg1End - projBase);

        maxJerkLength = curveEndPoint.Length();
    }

    double shortestAllowableLength = (latestStart - earliestEnd).Length();  // higher jerk
    double longestAllowableLength = (earliestStart - latestEnd).Length(); // lower jerk

    if ( longestAllowableLength != 0 ) {
        if ( maxJerkLength > (longestAllowableLength + 0.0000001) ) {
            // jerk limit does not allow turning as tight as required
            return;
        }
    }


    if ( doubleBack ) {
        // startPoint and endPoint already determined
    }
    else if ( l1.blendType == CBT_MAX_JERK ) {
        if ( maxJerkLength <= shortestAllowableLength ) {
            // lower jerk to fit shortest allowable curve
            double ratio = maxJerkLength / shortestAllowableLength;
            j *= ratio*ratio;
            T = calculateDurationFromJerkAndAcceleration(j, v1-v0);
            startPoint = latestStart;
            endPoint = earliestEnd;
        }
        else {
            // jerk is already between the limits, just need to position the start correctly
            double f = fabs((maxJerkLength - shortestAllowableLength) / (longestAllowableLength - shortestAllowableLength));
            scv::vec3 midStart = latestStart + f * (earliestStart - latestStart);
            scv::vec3 midEnd = earliestEnd + f * (latestEnd - earliestEnd);
            startPoint = midStart;
            endPoint = midEnd;
        }
    }
    else {
        // lower jerk to match longest corner curve        
        if ( j.LengthSquared() == 0 ) { // a straight-line case where T was already decided, don't change it

        }
        else if ( longestAllowableLength != 0 ) { // a move that doubles back can have a zero length
            double ratio = maxJerkLength / longestAllowableLength;
            j *= ratio*ratio;
            T = calculateDurationFromJerkAndAcceleration(j, v1-v0);
        }

        startPoint = earliestStart;
        endPoint = latestEnd;
    }

    // remove latter part of linear segment of original first line
    double linear0Len = (startPoint - seg0.pos).Length();
    seg0.duration = linear0Len / seg0.vel.Length();

    // remove first part of linear segment of original second line
    double linear1Len = (seg2.pos - endPoint).Length();
    seg1.duration = linear1Len / seg1.vel.Length();
    seg1.pos = endPoint;

    markSkippedSegments(l0, 0);
    markSkippedSegments(l1, 1);

    // update midpoint values
    double t = T;
    scv::vec3 sh =      t * v0 + (( t * t * t) / 6.0) * j;
    scv::vec3 vh = v0 + ((t * t) / 2.0) * j;
    scv::vec3 ah =      t * j;

    segment c0;
    c0.pos = startPoint;
    c0.vel = v0;
    c0.acc = scv::vec3_zero;
    c0.jerk = j;
    c0.duration = T;
    l0.segments.push_back(c0);

    segment c1;
    c1.pos = sh + startPoint;
    c1.vel = vh;
    c1.acc = ah;
    c1.jerk = -j;
    c1.duration = T;
    l0.segments.push_back(c1);
}

void planner::appendMove(move &m)
{
    if ( m.vel == 0 ) {
        printf("Ignoring move with zero velocity\n");
        return;
    }
    if ( m.acc == 0 ) {
        printf("Ignoring move with zero acceleration\n");
        return;
    }
    if ( m.jerk == 0 ) {
        printf("Ignoring move with zero jerk\n");
        return;
    }

    if ( ! moves.empty() ) {
        move& lastMove = moves[moves.size()-1];
        m.src = lastMove.dst;
        if ( m.src == m.dst ) {
            printf("Ignoring move with no actual dp\n");
            return;
        }
    }

    moves.push_back(m);
}

void planner::collateSegments()
{
    segments.clear();

    for (size_t i = 0; i < moves.size(); i++) {
        move& l = moves[i];
        for (size_t k = 0; k < l.segments.size(); k++) {
            segments.push_back(l.segments[k]);
        }
    }
}

void planner::setPositionLimits(double lx, double ly, double lz, double ux, double uy, double uz)
{
    posLimitLower = vec3(lx,ly,lz);
    posLimitUpper = vec3(ux,uy,uz);
}

void planner::setVelocityLimits(double x, double y, double z)
{
    velLimit = vec3(x,y,z);
}

void planner::setAccelerationLimits(double x, double y, double z)
{
    accLimit = vec3(x,y,z);
}

void planner::setJerkLimits(double x, double y, double z)
{
    jerkLimit = vec3(x,y,z);
}

void planner::printConstraints()
{
    printf("Planner global constraints:\n");
    printf("  Min pos:  %f, %f, %f\n", posLimitLower.x, posLimitLower.y, posLimitLower.z);
    printf("  Max pos:  %f, %f, %f\n", posLimitUpper.x, posLimitUpper.y, posLimitUpper.z);
    printf("  Max vel:  %f, %f, %f\n", velLimit.x, velLimit.y, velLimit.z);
    printf("  Max acc:  %f, %f, %f\n", accLimit.x, accLimit.y, accLimit.z);
    printf("  Max jerk: %f, %f, %f\n", jerkLimit.x, jerkLimit.y, jerkLimit.z);
}

void planner::printMoves()
{
    for (size_t i = 0; i < moves.size(); i++) {
        move& l = moves[i];
        printf("  Move %d:\n", (int)i);
        printf("    src: %f, %f, %f\n", l.src.x, l.src.y, l.src.z);
        printf("    dst: %f, %f, %f\n", l.dst.x, l.dst.y, l.dst.z);
        printf("    Vel: %f\n", l.vel);
        printf("    Acc: %f\n", l.acc);
        printf("    Jerk: %f\n", l.jerk);
        printf("    Blend: %s\n", l.blendType==CBT_MAX_JERK ? "max jerk":l.blendType==CBT_MIN_JERK?"min jerk":"none");
    }
}

void planner::printSegments()
{
    for (size_t i = 0; i < segments.size(); i++) {
        segment& s = segments[i];
        printf("  Segment %d\n", (int)i);
        printf("    pos : %f, %f, %f\n", s.pos.x, s.pos.y, s.pos.z);
        printf("    vel : %f, %f, %f\n", s.vel.x, s.vel.y, s.vel.z);
        printf("    acc : %f, %f, %f\n", s.acc.x, s.acc.y, s.acc.z);
        printf("    jerk: %f, %f, %f\n", s.jerk.x, s.jerk.y, s.jerk.z);
        printf("    duration: %f\n", s.duration);
    }
}

std::vector<segment> &planner::getSegments()
{
    return segments;
}


} // namespace
