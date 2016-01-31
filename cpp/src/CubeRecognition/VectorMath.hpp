#pragma once

#include <cmath>
#include <iostream>
#include <limits>

//*
#define REAL_DOUBLE
typedef double Real;
/*/
typedef float REAL;
//*/

template <class T> inline T Clamp(T value, T a, T b)
{
    if (value < a)
        return a;
    if (value > b)
        return b;
    return value;
}

/*
#define FastSqrt sqrt
/*/
inline float FastSqrt(float square)
{
    float retval;

    __asm__(
        "sub    $0x3F800000, %0\n\t"
        "sar    $1, %0\n\t"
        "add    $0x3F800000, %0\n\t"
        : "=r" (retval)
        : "0" (square)
    );
    return retval;
}
//*/

template <class T> struct Vec2
{
public:
    union
    {
        struct
        {
            T x,y;
        };
        T e[2];
    };
    Vec2<T>() { }
    Vec2<T>(const Vec2<T> &a): x(a.x), y(a.y) { }
    Vec2<T>(T x, T y) : x(x), y(y) { }
    Vec2<T> operator + (const Vec2<T> &a) const
    {
        return Vec2<T>(x+a.x, y+a.y);
    };
    Vec2<T> operator - (const Vec2<T> &a) const
    {
        return Vec2<T>(x-a.x, y-a.y);
    };
    Vec2<T> operator - () const
    {
        return Vec2<T>(-x, -y);
    };

    T operator * (const Vec2<T> &a) const // dot product
    {
        return x*a.x + y*a.y;
    };
    T Dot (const Vec2<T> &a) const // dot product
    {
        return x*a.x + y*a.y;
    };

    Vec2<T> ElemMult(const Vec2<T> &a) const // Element by element multiplication
    {
        return Vec2<T>(x*a.x, y*a.y);
    };

    friend Vec2<T> operator * (const Vec2<T> &a, const T b)
    {
        return Vec2<T>(b*a.x, b*a.y);
    };
    friend Vec2<T> operator * (const T a, const Vec2<T> &b)
    {
        return Vec2<T>(a*b.x, a*b.y);
    };
    friend Vec2<T> operator / (const Vec2<T> &a, T b)
    {
        //*
        b=1/b;
        return Vec2<T>(a.x*b, a.y*b);
        /*/
        return Vec2<T>(a.x/b, a.y/b);
        //*/
    }
    void operator  = (const Vec2<T> &a)
    {
        x =a.x; y= a.y;
    }
    void operator += (const Vec2<T> &a)
    {
        x+=a.x; y+=a.y;
    }
    void operator -= (const Vec2<T> &a)
    {
        x-=a.x; y-=a.y;
    }
/*
    void operator *= (const Vec2<T> a)
    {
        x *= a.x;
        y *= a.y;
    }
*/
    void operator *= (const T a)
    {
        x*=a;
        y*=a;
    }
    void operator /= (T a)
    {
        a=1/a;
        x*=a;
        y*=a;
    }
    bool operator == (const Vec2<T> a) const
    {
        if (x==a.x && y==a.y)
            return true;
        return false;
    }
    bool operator != (const Vec2<T> a) const
    {
        if (x!=a.x || y!=a.y)
            return true;
        return false;
    }
    /*
    bool operator == (const REAL a) const
    {
        if (x==a && y==a)
            return true;
        return false;
    }
    */
    T Length() const
    {
        return sqrt(x*x + y*y);
    }
    T FastLength() const
    {
        return FastSqrt(x*x + y*y);
    }
    T SqrLength() const
    {
        return x*x + y*y;
    }
    void SetToZero() {x=y=0;}

    void Normalize()
    {
        T length = Length();
        if (length == 0)
            return;
        *this /= length;
    }

    void FastNormalize()
    {
        T length = FastLength();
        if (length == 0)
            return;
        *this /= length;
    }

    friend Vec2<T>& Normalize(Vec2<T> a)
    {
        a.Normalize();
        return a;
    }

    friend Vec2<T>& FastNormalize(Vec2<T> a)
    {
        a.FastNormalize();
        return a;
    }

    friend std::ostream& operator << (std::ostream &os, const Vec2<T> &a)
    {
        return os << a.x << " " << a.y;
    }
    friend std::istream& operator >> (std::istream &is, Vec2<T> &a)
    {
        return is >> a.x >> a.y;
    }
};

template <class T> struct Vec3
{
public:
    union
    {
        struct
        {
            T x,y,z;
        };
        T e[3];
    };
    Vec3<T>() { }
    Vec3<T>(const Vec3<T> &a): x(a.x), y(a.y), z(a.z) { }
    Vec3<T>(T x, T y, T z) : x(x), y(y), z(z) { }
    Vec3<T> operator + (const Vec3<T> &a) const
    {
        return Vec3<T>(x+a.x, y+a.y, z+a.z);
    }
    Vec3<T> operator - (const Vec3<T> &a) const
    {
        return Vec3<T>(x-a.x, y-a.y, z-a.z);
    }
    Vec3<T> operator - () const
    {
        return Vec3<T>(-x, -y, -z);
    }
    Vec3<T> operator % (const Vec3<T> &a) const // cross product
    {
        return Vec3<T>(y*a.z - z*a.y, z*a.x - x*a.z, x*a.y - y*a.x);
    }
    Vec3<T> Cross (const Vec3<T> &a) const // cross product
    {
        return Vec3<T>(y*a.z - z*a.y, z*a.x - x*a.z, x*a.y - y*a.x);
    }
    Vec3<T> UnitCross (const Vec3<T> &a) const // cross product
    {
        Vec3<T> tmp(y*a.z - z*a.y, z*a.x - x*a.z, x*a.y - y*a.x);
        return tmp / tmp.Length();
    }

    T operator * (const Vec3<T> &a) const // dot product
    {
        return x*a.x + y*a.y + z*a.z;
    }
    T Dot (const Vec3<T> &a) const // dot product
    {
        return x*a.x + y*a.y + z*a.z;
    }

    Vec3<T> ElemMult(const Vec3<T> &a) const // Element by element multiplication
    {
        return Vec3<T>(x*a.x, y*a.y, z*a.z);
    }

    friend Vec3<T> operator * (const Vec3<T> &a, const T b)
    {
        return Vec3<T>(b*a.x, b*a.y, b*a.z);
    }
    friend Vec3<T> operator * (const T a, const Vec3<T> &b)
    {
        return Vec3<T>(a*b.x, a*b.y, a*b.z);
    }
    friend Vec3<T> operator / (const Vec3<T> &a, T b)
    {
        //*
        b=1/b;
        return Vec3<T>(a.x*b, a.y*b, a.z*b);
        /*/
        return Vec3<T>(a.x/b, a.y/b, a.z/b);
        //*/
    }
    void operator  = (const Vec3<T> &a)
    {
        x =a.x; y= a.y; z =a.z;
    }
    void operator += (const Vec3<T> &a)
    {
        x+=a.x; y+=a.y; z+=a.z;
    }
    void operator -= (const Vec3<T> &a)
    {
        x-=a.x; y-=a.y; z-=a.z;
    }
/*
    void operator *= (const Vec3<T> a)
    {
        x *= a.x;
        y *= a.y;
        z *= a.z;
    }
*/
    void operator *= (const T a)
    {
        x*=a;
        y*=a;
        z*=a;
    }
    void operator /= (T a)
    {
        a=1/a;
        x*=a;
        y*=a;
        z*=a;
    }
    bool operator == (const Vec3<T> a) const
    {
        if (x==a.x && y==a.y && z==a.z)
            return true;
        return false;
    }
    bool operator != (const Vec3<T> a) const
    {
        if (x!=a.x || y!=a.y || z!=a.z)
            return true;
        return false;
    }
    /*
    bool operator == (const REAL a) const
    {
        if (x==a && y==a && z==a)
            return true;
        return false;
    }
    */
    T Length() const
    {
        return sqrt(x*x + y*y + z*z);
    }
    T FastLength() const
    {
        return FastSqrt(x*x + y*y + z*z);
    }
    T SqrLength() const
    {
        return x*x + y*y + z*z;
    }
    void SetToZero() {x=y=z=0;};

    void Normalize()
    {
        T length = Length();
        if (length == 0)
            return;
        *this /= length;
    }

    void FastNormalize()
    {
        T length = FastLength();
        if (length == 0)
            return;
        *this /= length;
    }

    friend Vec3<T>& Normalize(Vec3<T> a)
    {
        a.Normalize();
        return a;
    }

    friend Vec3<T>& FastNormalize(Vec3<T> a)
    {
        a.FastNormalize();
        return a;
    }

    friend std::ostream& operator << (std::ostream &os, const Vec3<T> &a)
    {
        return os << a.x << " " << a.y << " " << a.z;
    }
    friend std::istream& operator >> (std::istream &is, Vec3<T> &a)
    {
        return is >> a.x >> a.y >> a.z;
    }
};

template <class T> struct Quat4
{
public:
//    union
//    {
        struct
        {
            T w;//,x,y,z;
            Vec3<T> vec;
        };
//        T e[4];
//    };

    Quat4<T>() { }
//    Quat4<T>(const Quat4<T> &a): w(a.w), x(a.x), y(a.y), z(a.z) { }
    Quat4<T>(const Quat4<T> &a): w(a.w), vec(a.vec) { }
//    Quat4<T>(const T p_w, const T p_x, const T p_y, const T p_z) : w(p_w), x(p_x), y(p_y), z(p_z) { }
    Quat4<T>(const T p_w, const T p_x, const T p_y, const T p_z) : w(p_w), vec(p_x, p_y, p_z) { }
    // from angle and vector
    Quat4<T>(const T angle, const Vec3<T> &p_vec)
    {
        const T a = angle * 0.5f;
        const T s = (T) sin(a);
        const T c = (T) cos(a);
        w = c;
        vec = p_vec * s;
    }

    friend Quat4<T> operator * (const Quat4<T> &a, const Quat4<T> &b)
    {
        return Quat4<T>( a.w*b.w - a.vec.x*b.vec.x - a.vec.y*b.vec.y - a.vec.z*b.vec.z,
                         a.w*b.vec.x + a.vec.x*b.w + a.vec.y*b.vec.z - a.vec.z*b.vec.y,
                         a.w*b.vec.y - a.vec.x*b.vec.z + a.vec.y*b.w + a.vec.z*b.vec.x,
                         a.w*b.vec.z + a.vec.x*b.vec.y - a.vec.y*b.vec.x + a.vec.z*b.w );
    }

    friend Vec3<T> operator * (const Quat4<T> &q, const Vec3<T> &v)
    {
        Quat4<T> q_v, q_i;

        q_v.w = 0.0;
        q_v.vec = v;

        q_i.w = q.w;
        q_i.vec = -q.vec;

        return (q * q_v * q_i).vec;
    }

    friend Quat4<T> operator * (const T s, const Quat4<T> &p_quat)
    {
        Quat4<T> quat(p_quat);
        quat.w *= s;
        quat.vec *= s;
        return quat;
    }

    friend Quat4<T> operator * (const Quat4<T> &p_quat, const T s)
    {
        Quat4<T> quat(p_quat);
        quat.w *= s;
        quat.vec *= s;
        return quat;
    }

    Quat4<T>& operator = (const Quat4<T> &quat)
    {
        w = quat.w;
        vec = quat.vec;
        return *this;
    }

    Quat4<T>& operator += (const Quat4<T> &quat)
    {
        w += quat.w;
        vec += quat.vec;
        return *this;
    }

    void SetToZero()
    {
        w = 1;
        vec.SetToZero();
    }

    void Normalize()
    {
        T invLength = 1.0/sqrt(w*w + vec.SqrLength());
        if ( invLength == 0 )
            return;
        w *= invLength;
        vec *= invLength;
    }

    // NOTE: this does not handle gimbal lock
    friend Vec3<T> Euler(const Quat4<T> &a)
    {
        double sqx = a.vec.x*a.vec.x;
        double sqy = a.vec.y*a.vec.y;
        double sqz = a.vec.z*a.vec.z;
#define w a.w
#define x a.vec.x
#define y a.vec.y
#define z a.vec.z
        return Vec3<T>( atan2(2*(w*x+y*z), 1-2*(sqx+sqy)),
                        asin(2*(w*y-z*x)),
                        atan2(2*(w*z+x*y), 1-2*(sqy+sqz)) );
#undef w
#undef x
#undef y
#undef z
    }

    friend Quat4<T> Normalize(Quat4<T> a)
    {
        a.Normalize();
        return a;
    }

    friend Quat4<T> Conjugate(const Quat4<T> &a)
    {
        return Quat4<T>(a.w, -a.vec);
    }

    friend std::ostream& operator << (std::ostream &os, const Quat4<T> &a)
    {
        return os << a.w << " " << a.vec;
    }
    friend std::istream& operator >> (std::istream &is, Quat4<T> &a)
    {
        return is >> a.w >> a.vec;
    }
};


// Column major, same as opengl
template <class T> struct Mat3x3
{
public:
//    union
//    {
        struct
        {
            Vec3<T> vec1; // x-axis = col1
            Vec3<T> vec2; // y-axis = col2
            Vec3<T> vec3; // z-axis = col3
        };
//        T e[9];
//    };

    Mat3x3<T>() { }
    Mat3x3<T>(const Mat3x3<T> &a) : vec1(a.vec1), vec2(a.vec2), vec3(a.vec3) { }
    Mat3x3<T>(const T vec1, const T vec2, const T vec3) : vec1(vec1), vec2(vec2), vec3(vec3) { }
    Mat3x3<T>(T a1, T a2, T a3, T b1, T b2, T b3, T c1, T c2, T c3) : vec1(a1, a2, a3), vec2(b1, b2, b3), vec3(c1, c2, c3) { }
    // from quaternion
    explicit Mat3x3<T>(const Quat4<T> &quat)
    {
        Vec3<T> fT  = 2.0f*quat.vec;
        Vec3<T> fTw = fT*quat.w;
        Vec3<T> fTx = fT*quat.vec.x;
        Vec3<T> fTy = fT*quat.vec.y;
        Vec3<T> fTz = fT*quat.vec.z;
//*
        vec1 = Vec3<T>(1.0f-(fTy.y+fTz.z), fTx.y+fTw.z, fTx.z-fTw.y);
        vec2 = Vec3<T>(fTx.y-fTw.z, 1.0f-(fTx.x+fTz.z), fTy.z+fTw.x);
        vec3 = Vec3<T>(fTx.z+fTw.y, fTy.z-fTw.x, 1.0f-(fTx.x+fTy.y));
/*/
        vec1 = Vec3<T>(1.0f-(fTy.y+fTz.z), fTx.y-fTw.z, fTx.z+fTw.y);
        vec2 = Vec3<T>(fTx.y+fTw.z, 1.0f-(fTx.x+fTz.z), fTy.z-fTw.x);
        vec3 = Vec3<T>(fTx.z-fTw.y, fTy.z+fTw.x, 1.0f-(fTx.x+fTy.y));
//*/
    }

    Vec3<T> operator * (const Vec3<T> &vec) const
    {
        return Vec3<T>(vec1.x*vec.x + vec2.x*vec.y + vec3.x*vec.z,
                       vec1.y*vec.x + vec2.y*vec.y + vec3.y*vec.z,
                       vec1.z*vec.x + vec2.z*vec.y + vec3.z*vec.z);
    }

    friend Vec3<T> operator * (const Vec3<T> &vec, const Mat3x3<T> &mat)
    {
        return Vec3<T>(vec * mat.vec1, vec * mat.vec2, vec * mat.vec3);
    }

    friend Mat3x3<T> operator * (const Mat3x3<T> &mat1, const Mat3x3<T> &mat2)
    {
        return Mat3x3<T>(
            mat1.vec1 * Vec3<T>(mat2.vec1.x, mat2.vec2.x, mat2.vec3.x),
            mat1.vec1 * Vec3<T>(mat2.vec1.y, mat2.vec2.y, mat2.vec3.y),
            mat1.vec1 * Vec3<T>(mat2.vec1.z, mat2.vec2.z, mat2.vec3.z),
            mat1.vec2 * Vec3<T>(mat2.vec1.x, mat2.vec2.x, mat2.vec3.x),
            mat1.vec2 * Vec3<T>(mat2.vec1.y, mat2.vec2.y, mat2.vec3.y),
            mat1.vec2 * Vec3<T>(mat2.vec1.z, mat2.vec2.z, mat2.vec3.z),
            mat1.vec3 * Vec3<T>(mat2.vec1.x, mat2.vec2.x, mat2.vec3.x),
            mat1.vec3 * Vec3<T>(mat2.vec1.y, mat2.vec2.y, mat2.vec3.y),
            mat1.vec3 * Vec3<T>(mat2.vec1.z, mat2.vec2.z, mat2.vec3.z)
            );
    }

    void operator = (const Mat3x3<T> mat)
    {
        vec1 = mat.vec1;
        vec2 = mat.vec2;
        vec3 = mat.vec3;
    }
};

typedef Vec2<Real> Vec2r;
typedef Vec3<Real> Vec3r;
typedef Quat4<Real> Quat4r;
typedef Mat3x3<Real> Mat3x3r;
