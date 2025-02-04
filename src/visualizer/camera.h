#ifndef CAMERA_H
#define CAMERA_H

#include "vec3.h"
#include "matrix.h"

// A basic camera for FPS game style movement

#define DEGTORAD ((scv_float)0.0174532925f)

// https://registry.khronos.org/OpenGL-Refpages/gl2.1/xhtml/gluLookAt.xml
void my_gluLookAt(GLfloat eyeX, GLfloat eyeY, GLfloat eyeZ, GLfloat lookAtX, GLfloat lookAtY, GLfloat lookAtZ, GLfloat upX, GLfloat upY, GLfloat upZ) {

    scv::vec3 F = scv::vec3(lookAtX - eyeX, lookAtY - eyeY, lookAtZ - eyeZ);
    scv::vec3 UP = scv::vec3(upX, upY, upZ);
    F.Normalize();
    UP.Normalize();

    scv::vec3 s = cross(F, UP);
    scv::vec3 u = cross(s, F);

    float m[16] = {
         s.x,  u.x, -F.x, 0,
         s.y,  u.y, -F.y, 0,
         s.z,  u.z, -F.z, 0,
         0, 0, 0,  1
    };

    glMultMatrixf(m);
    glTranslatef(-eyeX, -eyeY, -eyeZ);
}

class Camera {
public:
    float yaw, pitch;

    scv::vec3 right;
    scv::vec3 up;
    scv::vec3 forward;

    scv::vec3 location;
    scv::vec3 focuspoint;

    Camera()
    {
        setLocation(10, 10, 10);
        setDirection(45, -45);
    }

    void setLocation(float x, float y, float z)
    {
        location = scv::vec3(x,y,z);
    }

    void setDirection(float _yaw, float _pitch)
    {
        yaw = _yaw;
        pitch = _pitch;
        if ( pitch > 80 )
            pitch = 80;
        else if ( pitch < -80 )
            pitch = -80;

        right =   scv::vec3(1, 0, 0);
        forward = scv::vec3(0, 1, 0);
        up =      scv::vec3(0, 0, 1);
        Matrix t = Matrix::zrotation(-yaw*DEGTORAD) * Matrix::xrotation(pitch*DEGTORAD);
        t.transform(forward);
        t.transform(up);
        t.transform(right);
    }

    void translate(float _right, float _forward, float _up)
    {
        location += _right * right;
        location += _forward * forward;
        location += _up * up;
    }

    void gluLookAt()
    {
        focuspoint = location + forward;
        my_gluLookAt(
            location.x, location.y, location.z,
            focuspoint.x, focuspoint.y, focuspoint.z,
            0, 0, 1
        );
        
    }
};


#endif
