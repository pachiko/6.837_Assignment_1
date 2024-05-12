#include "surf.h"
#include "extra.h"
using namespace std;

namespace
{
    
    // We're only implenting swept surfaces where the profile curve is
    // flat on the xy-plane.  This is a check function.
    static bool checkFlat(const Curve &profile)
    {
        for (unsigned i=0; i<profile.size(); i++)
            if (profile[i].V[2] != 0.0 ||
                profile[i].T[2] != 0.0 ||
                profile[i].N[2] != 0.0)
                return false;
    
        return true;
    }
}

void formTriangles(Surface& surface, size_t sweep_size, size_t profile_size) {
    // Make triangles. VERTICES must be formed COUNTER-CLOCKWISE. Check your curve direction
    // If current and previous, you are traversing the curves clockwise
    // If current and next, you are traversing the curves counter-clockwise (since rotation is CCW)

    for (unsigned j = 0; j < sweep_size; j++) { // each slice
        unsigned c1 = j * profile_size; // current curve
        //unsigned c2 = (j == 0)? (steps - 1) * profile_size : (j - 1) * profile_size; // previous curve
        unsigned c2 = (j == sweep_size - 1) ? 0 : (j + 1) * profile_size; // next curve

        for (unsigned k = 0; k < profile_size - 1; k++) { // each face
            // Vertices for current and previous curves
            //surface.VF.push_back(Tup3u(c1 + k, c2 + k, c2 + k + 1));
            //surface.VF.push_back(Tup3u(c2 + k + 1, c1 + k + 1, c1 + k));

            // Vertices for current and next curves
            surface.VF.push_back(Tup3u(c1 + k, c2 + k + 1, c2 + k));
            surface.VF.push_back(Tup3u(c2 + k + 1, c1 + k, c1 + k + 1));
        }
    }
}

Surface makeSurfRev(const Curve &profile, unsigned steps)
{
    Surface surface;
    
    if (!checkFlat(profile))
    {
        cerr << "surfRev profile curve must be flat on xy plane." << endl;
        exit(0);
    }

    float dTheta = 2.f * M_PI / steps; // Angle increment in radians
    size_t l = profile.size() - 1; // number of triangles connecting current and previous curve (on ONE side)

    for (unsigned i = 0; i < steps; i++) {
        for (auto p : profile) {
            Matrix4f rot;
            rot = rot.rotateY(i * dTheta); // returns a rotation matrix along y-axis with angle dTheta in radians

            // Normals of curves are pointed inwards towards center. Surfaces need outward-pointing normals
            // Transpose-Inverse of Rotation Matrix is itself
            surface.VN.push_back(-(rot.getSubmatrix3x3(0, 0) * p.N).normalized()); // Don't translate normals (they are vectors)
            surface.VV.push_back((rot * Vector4f(p.V, 1.f)).xyz());
        }
    }

    formTriangles(surface, steps, profile.size());

    // TODO: Here you should build the surface.  See surf.h for details.
    //cerr << "\t>>> makeSurfRev called (but not implemented).\n\t>>> Returning empty surface." << endl;
 
    return surface;
}

Surface makeGenCyl(const Curve &profile, const Curve &sweep )
{
    Surface surface;

    if (!checkFlat(profile))
    {
        cerr << "genCyl profile curve must be flat on xy plane." << endl;
        exit(0);
    }

    for (auto c : sweep) {
        Matrix4f mat(Vector4f(c.N, 0.f), Vector4f(c.B, 0.f), Vector4f(c.T, 0.f), Vector4f(c.V, 1.f), true);
        Matrix3f matN = mat.getSubmatrix3x3(0, 0).inverse().transposed();

        for (auto p : profile) {
            surface.VN.push_back(-(matN * p.N).normalized()); // Don't translate normals (they are vectors)
            surface.VV.push_back((mat * Vector4f(p.V, 1.f)).homogenized().xyz());
        }
    }

    formTriangles(surface, sweep.size(), profile.size());

    // TODO: Here you should build the surface.  See surf.h for details.

    //cerr << "\t>>> makeGenCyl called (but not implemented).\n\t>>> Returning empty surface." <<endl;

    return surface;
}

void drawSurface(const Surface &surface, bool shaded)
{
    // Save current state of OpenGL
    glPushAttrib(GL_ALL_ATTRIB_BITS);

    if (shaded)
    {
        // This will use the current material color and light
        // positions.  Just set these in drawScene();
        glEnable(GL_LIGHTING);
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

        // This tells openGL to *not* draw backwards-facing triangles.
        // This is more efficient, and in addition it will help you
        // make sure that your triangles are drawn in the right order.
        glEnable(GL_CULL_FACE);
        glCullFace(GL_BACK);
    }
    else
    {        
        glDisable(GL_LIGHTING);
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
        
        glColor4f(0.4f,0.4f,0.4f,1.f);
        glLineWidth(1);
    }

    glBegin(GL_TRIANGLES);
    for (unsigned i=0; i<surface.VF.size(); i++)
    {
        glNormal(surface.VN[surface.VF[i][0]]);
        glVertex(surface.VV[surface.VF[i][0]]);
        glNormal(surface.VN[surface.VF[i][1]]);
        glVertex(surface.VV[surface.VF[i][1]]);
        glNormal(surface.VN[surface.VF[i][2]]);
        glVertex(surface.VV[surface.VF[i][2]]);
    }
    glEnd();

    glPopAttrib();
}

void drawNormals(const Surface &surface, float len)
{
    // Save current state of OpenGL
    glPushAttrib(GL_ALL_ATTRIB_BITS);

    glDisable(GL_LIGHTING);
    glColor4f(0,1,1,1);
    glLineWidth(1);

    glBegin(GL_LINES);
    for (unsigned i=0; i<surface.VV.size(); i++)
    {
        glVertex(surface.VV[i]);
        glVertex(surface.VV[i] + surface.VN[i] * len);
    }
    glEnd();

    glPopAttrib();
}

void outputObjFile(ostream &out, const Surface &surface)
{
    
    for (unsigned i=0; i<surface.VV.size(); i++)
        out << "v  "
            << surface.VV[i][0] << " "
            << surface.VV[i][1] << " "
            << surface.VV[i][2] << endl;

    for (unsigned i=0; i<surface.VN.size(); i++)
        out << "vn "
            << surface.VN[i][0] << " "
            << surface.VN[i][1] << " "
            << surface.VN[i][2] << endl;

    out << "vt  0 0 0" << endl;
    
    for (unsigned i=0; i<surface.VF.size(); i++)
    {
        out << "f  ";
        for (unsigned j=0; j<3; j++)
        {
            unsigned a = surface.VF[i][j]+1;
            out << a << "/" << "1" << "/" << a << " ";
        }
        out << endl;
    }
}
