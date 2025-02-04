
#ifndef SCV_VEC3_H
#define SCV_VEC3_H

#include <math.h>

#define scv_float       float

namespace scv {

    struct vec3
    {
        scv_float x, y, z;

        vec3() {}

        vec3(scv_float x, scv_float y, scv_float z) : x(x), y(y), z(z) {}

        void SetZero() { x = 0.0; y = 0.0; z = 0.0; }

        void Set(scv_float x_, scv_float y_, scv_float z_) { x = x_; y = y_; z = z_; }

        scv_float& operator[] (size_t i)
        {
            switch (i) {
            case 0: return x;
            case 1: return y;
            case 2: return z;
            default: return x; // ??
            }
        }

        vec3 operator -() const { vec3 v; v.Set(-x, -y, -z); return v; }

        void operator += (const vec3& v)
        {
            x += v.x; y += v.y; z += v.z;
        }

        void operator -= (const vec3& v)
        {
            x -= v.x; y -= v.y; z -= v.z;
        }

        bool operator == (const vec3& b)
        {
            return x == b.x && y == b.y && z == b.z;
        }

        void operator *= (scv_float a)
        {
            x *= a; y *= a; z *= a;
        }

        scv_float Length() const
        {
            return (scv_float)sqrt(x * x + y * y + z * z);
        }

        scv_float LengthSquared() const
        {
            return x * x + y * y + z * z;
        }

        bool anyZero() const
        {
            return x == 0 || y == 0 || z == 0;
        }

        scv_float Normalize()
        {
            scv_float length = Length();
            if (length < 0.00000001)
                return 0.0;
            scv_float invLength = 1.0f / length;
            x *= invLength;
            y *= invLength;
            z *= invLength;

            return length;
        }
    };

    extern const vec3 vec3_zero;

    inline scv_float dot(const vec3& a, const vec3& b)
    {
        return a.x * b.x + a.y * b.y + a.z * b.z;
    }

    inline vec3 cross(vec3 v1, vec3 v2)
    {
        return vec3(
            v1.y * v2.z - v1.z * v2.y,
            v1.z * v2.x - v1.x * v2.z,
            v1.x * v2.y - v1.y * v2.x
        );
    }

    inline vec3 operator + (const vec3& a, const vec3& b)
    {
        return vec3(a.x + b.x, a.y + b.y, a.z + b.z);
    }

    inline vec3 operator * (const vec3& a, const vec3& b)
    {
        return vec3(a.x * b.x, a.y * b.y, a.z * b.z);
    }

    inline vec3 operator - (const vec3& a, const vec3& b)
    {
        return vec3(a.x - b.x, a.y - b.y, a.z - b.z);
    }

    inline vec3 operator * (scv_float s, const vec3& a)
    {
        return vec3(s * a.x, s * a.y, s * a.z);
    }

    inline vec3 abs(const vec3& v) {
        return vec3( (scv_float)fabs(v.x), (scv_float)fabs(v.y), (scv_float)fabs(v.z));
    }

    template <typename T>
    inline T min(T a, T b)
    {
        return a < b ? a : b;
    }

    inline vec3 min(const vec3& a, const vec3& b)
    {
        return vec3(min(a.x, b.x), min(a.y, b.y), min(a.z, b.z));
    }

    template <typename T>
    inline T max(T a, T b)
    {
        return a > b ? a : b;
    }

    inline vec3 max(const vec3& a, const vec3& b)
    {
        return vec3(max(a.x, b.x), max(a.y, b.y), max(a.z, b.z));
    }

    vec3 getBoundedVector(vec3 dir, vec3 lim);

}


#endif
