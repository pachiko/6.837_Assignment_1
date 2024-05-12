#include "curve.h"
#include "extra.h"
#ifdef WIN32
#include <windows.h>
#endif
#include <GL/gl.h>
using namespace std;

namespace
{
    // Approximately equal to.  We don't want to use == because of
    // precision issues with floating point.
    inline bool approx( const Vector3f& lhs, const Vector3f& rhs )
    {
        const float eps = 1e-8f;
        return ( lhs - rhs ).absSquared() < eps;
    }
}
    

Curve evalBezier( const vector< Vector3f >& P, unsigned steps )
{
    // Check
    if( P.size() < 4 || P.size() % 3 != 1 )
    {
        cerr << "evalBezier must be called with 3n+1 control points." << endl;
        exit( 0 );
    }

    // TODO:
    // You should implement this function so that it returns a Curve
    // (e.g., a vector< CurvePoint >).  The variable "steps" tells you
    // the number of points to generate on each piece of the spline.
    // At least, that's how the sample solution is implemented and how
    // the SWP files are written.  But you are free to interpret this
    // variable however you want, so long as you can control the
    // "resolution" of the discretized spline curve with it.

    // Make sure that this function computes all the appropriate
    // Vector3fs for each CurvePoint: V,T,N,B.
    // [NBT] should be unit and orthogonal.

    // Also note that you may assume that all Bezier curves that you
    // receive have G1 continuity.  Otherwise, the TNB will not be
    // be defined at points where this does not hold.

    unsigned num_pieces = P.size()/3;
    float dt = 1.f/steps; // time range of a piece is 0 -> 1

    std::vector<CurvePoint> res;

    Vector3f b_prev(0.f, 0.f, 1.f); // Initial bi-normal. INITIAL TO A SPLINE

    Matrix4f b_bezier( // Bezier basis matrix
        Vector4f(1, -3, 3, -1),
        Vector4f(0, 3, -6, 3),
        Vector4f(0, 0, 3, -3),
        Vector4f(0, 0, 0, 1), false);

    Matrix4f t_bezier( // Bezier tangent matrix
        Vector4f(-3, 6, -3, 0),
        Vector4f(3, -12, 9, 0),
        Vector4f(0, 6, -9, 0),
        Vector4f(0, 0, 3, 0), false);

    for (unsigned i = 0; i < num_pieces; i++) { // 7 control-points = 2 pieces. last CP is shared
        unsigned idx = 3*i;
        Vector3f p1 = P[idx];
        Vector3f p2 = P[idx + 1];
        Vector3f p3 = P[idx + 2];
        Vector3f p4 = P[idx + 3];

        // Control points (Geometry Matrix)
        Matrix4f g_bezier(Vector4f(p1, 1.f), Vector4f(p2, 1.f), Vector4f(p3, 1.f), Vector4f(p4, 1.f));

        for (unsigned j = 0; j <= steps; j++) {
            float t = dt*j;
            Vector4f time(1.f, t, t*t, t*t*t); // Power basis

            CurvePoint cp;
            cp.V = (g_bezier * b_bezier * time).xyz(); // Position
            cp.T = (g_bezier * t_bezier * time).xyz(); // Tangent
            cp.N = b_prev.cross(b_prev, cp.T).normalized(); // Normal
            b_prev = cp.T.cross(cp.T, cp.N).normalized();
            cp.B = b_prev; // Bi-normal

            res.push_back(cp);
        }
    }

    return res;

    //cerr << "\t>>> evalBezier has been called with the following input:" << endl;

    //cerr << "\t>>> Control points (type vector< Vector3f >): "<< endl;
    //for( unsigned i = 0; i < P.size(); ++i )
    //{
    //    cerr << "\t>>> " << P[i] << endl;
    //}

    //cerr << "\t>>> Steps (type steps): " << steps << endl;
    //cerr << "\t>>> Returning empty curve." << endl;

    // Right now this will just return this empty curve.
    //return Curve();
}

Curve evalBspline( const vector< Vector3f >& P, unsigned steps )
{
    // Check
    if( P.size() < 4 )
    {
        cerr << "evalBspline must be called with 4 or more control points." << endl;
        exit( 0 );
    }

    // Bezier basis (inverted)
    Matrix4f b_bezier(Vector4f(1, -3, 3, -1),
                        Vector4f(0, 3, -6, 3),
                        Vector4f(0, 0, 3, -3),
                        Vector4f(0, 0, 0, 1), false);
    b_bezier = b_bezier.inverse();

    // BSpline basis
    Matrix4f b_bSpline(Vector4f(1, -3, 3, -1),
                        Vector4f(4, 0, -6, 3),
                        Vector4f(1, 3, 3, -3),
                        Vector4f(0, 0, 0, 1), false);
    b_bSpline /= 6.f;

    // Basis transformation
    Matrix4f B = b_bSpline * b_bezier;

    unsigned num_pieces = P.size() - 3;
    std::vector<Vector3f> p_bezier;

    for (unsigned i = 0; i < num_pieces; i++) {
        // Conversions must use 4 control-points
        Matrix4f g(Vector4f(P[i], 1.f),
                    Vector4f(P[i + 1], 1.f),
                    Vector4f(P[i + 2], 1.f),
                    Vector4f(P[i + 3], 1.f));
        Matrix4f res = g*B;

        if (i == 0) { // Only reuse first control-point. this is now in Bezier basis
            p_bezier.push_back(res.getCol(0).homogenized().xyz());
        }
        p_bezier.push_back(res.getCol(1).homogenized().xyz());
        p_bezier.push_back(res.getCol(2).homogenized().xyz());
        p_bezier.push_back(res.getCol(3).homogenized().xyz());
    }

    return evalBezier(p_bezier, steps);

    //cerr << "\t>>> evalBSpline has been called with the following input:" << endl;

    //cerr << "\t>>> Control points (type vector< Vector3f >): "<< endl;
    //for( unsigned i = 0; i < P.size(); ++i )
    //{
    //    cerr << "\t>>> " << P[i] << endl;
    //}

    //cerr << "\t>>> Steps (type steps): " << steps << endl;
    //cerr << "\t>>> Returning empty curve." << endl;

    //// Return an empty curve right now.
    //return Curve();
}

/*Curve evalBspline(const vector< Vector3f >& P, unsigned steps)
{
    // Check
    if (P.size() < 4)
    {
        cerr << "evalBspline must be called with 4 or more control points." << endl;
        exit(0);
    }

    // TODO:
    // It is suggested that you implement this function by changing
    // basis from B-spline to Bezier.  That way, you can just call
    // your evalBezier function.

    cerr << "\t>>> evalBSpline has been called with the following input:" << endl;

    cerr << "\t>>> Control points (type vector< Vector3f >): " << endl;
    for (unsigned i = 0; i < P.size(); ++i)
    {
        cerr << "\t>>> " << P[i] << endl;
    }

    cerr << "\t>>> Steps (type steps): " << steps << endl;
    //cerr << "\t>>> Returning empty curve." << endl;

    int seg;			//Determine number of segments if 4 or 
                //more control points
    if (P.size() == 4) {
        seg = 1;
    }
    else {
        seg = (P.size() - 3);
    }

    Curve R((seg) * (steps)+1);

    Vector3f Binit;		//Initial Binormal (arbitrary only at beginning)
    Vector3f Blast;		//Most recent Binormal

    bool beginning = true;

    for (unsigned i = 0; i < P.size() - 3; ++i)
    {
        if (beginning) {
            Binit = Vector3f(0.f, 0.f, 1.f);
        }
        else {
            Binit = Blast;
        }

        for (unsigned delta = 0; delta <= steps; ++delta)
        {
            float t = float(delta) / steps;

            //Point Matrix
            Matrix4f relativePoints(P[i + 0][0], P[i + 1][0], P[i + 2][0], P[i + 3][0],
                P[i + 0][1], P[i + 1][1], P[i + 2][1], P[i + 3][1],
                P[i + 0][2], P[i + 1][2], P[i + 2][2], P[i + 3][2],
                0.f, 0.f, 0.f, 0.f);
            //V Matrix
            Matrix4f MatV(1.f / 6, -3.f / 6, 3.f / 6, -1.f / 6,
                4.f / 6, 0.f, -6.f / 6, 3.f / 6,
                1.f / 6, 3.f / 6, 3.f / 6, -3.f / 6,
                0.f, 0.f, 0.f, 1.f / 6);
            //T Matrix
            Matrix4f MatT(-3.f / 6, 6.f / 6, -3.f / 6, 0.f / 6,
                0.f / 6, -12.f / 6, 9.f / 6, 0.f / 6,
                3.f / 6, 6.f / 6, -9.f / 6, 0.f / 6,
                0.f, 0.f, 0.f, 3.f / 6);

            //polynomial (t) Matrix
            Vector4f timeT(1, t, t * t, t * t * t);

            //Calculate V Vector
            R[(i * steps) + delta].V = Vector3f((relativePoints * MatV * timeT)[0],
                (relativePoints * MatV * timeT)[1],
                (relativePoints * MatV * timeT)[2]);
            //Calculate Tangent
            R[(i * steps) + delta].T = Vector3f((relativePoints * MatT * timeT)[0],
                (relativePoints * MatT * timeT)[1],
                (relativePoints * MatT * timeT)[2]).normalized();
            //Calculate Normal
            R[(i * steps) + delta].N = Vector3f::cross(Binit,
                R[(i * steps) + delta].T).normalized();
            //Calulate Binormal
            R[(i * steps) + delta].B = Vector3f::cross(R[(i * steps) + delta].T,
                R[(i * steps) + delta].N).normalized();
            //Keep track of current Binormal
            Binit = R[(i * steps) + delta].B;

            beginning = false;
            Blast = Binit;
        }
    }

    // Return an empty curve right now.
    //return Curve();
    return R;
}*/

Curve evalCircle( float radius, unsigned steps )
{
    // This is a sample function on how to properly initialize a Curve
    // (which is a vector< CurvePoint >).
    
    // Preallocate a curve with steps+1 CurvePoints
    Curve R( steps+1 );

    // Fill it in counterclockwise
    for( unsigned i = 0; i <= steps; ++i )
    {
        // step from 0 to 2pi
        float t = 2.0f * M_PI * float( i ) / steps;

        // Initialize position
        // We're pivoting counterclockwise around the y-axis
        R[i].V = radius * Vector3f( cos(t), sin(t), 0 );
        
        // Tangent vector is first derivative
        R[i].T = Vector3f( -sin(t), cos(t), 0 );
        
        // Normal vector is second derivative
        R[i].N = Vector3f( -cos(t), -sin(t), 0 );

        // Finally, binormal is facing up.
        R[i].B = Vector3f( 0, 0, 1 );
    }

    return R;
}

void drawCurve( const Curve& curve, float framesize )
{
    // Save current state of OpenGL
    glPushAttrib( GL_ALL_ATTRIB_BITS );

    // Setup for line drawing
    glDisable( GL_LIGHTING ); 
    glColor4f( 1, 1, 1, 1 );
    glLineWidth( 1 );
    
    // Draw curve
    glBegin( GL_LINE_STRIP );
    for( unsigned i = 0; i < curve.size(); ++i )
    {
        glVertex( curve[ i ].V );
    }
    glEnd();

    glLineWidth( 1 );

    // Draw coordinate frames if framesize nonzero
    if( framesize != 0.0f )
    {
        Matrix4f M;

        for( unsigned i = 0; i < curve.size(); ++i )
        {
            M.setCol( 0, Vector4f( curve[i].N, 0 ) );
            M.setCol( 1, Vector4f( curve[i].B, 0 ) );
            M.setCol( 2, Vector4f( curve[i].T, 0 ) );
            M.setCol( 3, Vector4f( curve[i].V, 1 ) );

            glPushMatrix();
            glMultMatrixf( M );
            glScaled( framesize, framesize, framesize );
            glBegin( GL_LINES );
            glColor3f( 1, 0, 0 ); glVertex3d( 0, 0, 0 ); glVertex3d( 1, 0, 0 );
            glColor3f( 0, 1, 0 ); glVertex3d( 0, 0, 0 ); glVertex3d( 0, 1, 0 );
            glColor3f( 0, 0, 1 ); glVertex3d( 0, 0, 0 ); glVertex3d( 0, 0, 1 );
            glEnd();
            glPopMatrix();
        }
    }
    
    // Pop state
    glPopAttrib();
}

