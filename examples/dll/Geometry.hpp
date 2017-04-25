// -*- coding: utf-8 -*-
// $Id: geometry.hpp 4 2008-03-12 10:30:55Z Naoyuki.Hirayama $

#ifndef GEOMETRY_HPP
#define GEOMETRY_HPP

#include <cmath>
#include <cstdlib>
#include <limits>

////////////////////////////////////////////////////////////////
// 超適当な vector
struct Vector {
    float x;
    float y;
    float z;

    Vector(){}
    Vector(float ax, float ay, float az)
        : x(ax), y(ay), z(az) {}

    void assign(float ax, float ay, float az) {
        x = ax; y = ay; z = az;
    }

};

inline float length_sq(const Vector& v) {
    return v.x * v.x + v.y * v.y + v.z * v.z;
}

inline float length(const Vector& v) {
    return sqrtf(length_sq(v));
}

inline Vector normalize(const Vector& v) {
    float ilen = 1.0f / length(v);
    Vector r;
    r.x = v.x * ilen;
    r.y = v.y * ilen;
    r.z = v.z * ilen;
    return r;
}

inline void normalize_f(Vector& v) {
    v = normalize(v);
}

inline Vector operator-(const Vector& v) {
    Vector r;
    r.x = -v.x;
    r.y = -v.y;
    r.z = -v.z;
    return r;
}

inline Vector operator-(const Vector& lhs, const Vector& rhs) {
    Vector v;
    v.x = lhs.x - rhs.x;
    v.y = lhs.y - rhs.y;
    v.z = lhs.z - rhs.z;
    return v;
}

inline Vector operator+(const Vector& lhs, const Vector& rhs) {
    Vector v;
    v.x = lhs.x + rhs.x;
    v.y = lhs.y + rhs.y;
    v.z = lhs.z + rhs.z;
    return v;
}

inline Vector operator*(const Vector& lhs, float rhs) {
    Vector v;
    v.x = lhs.x * rhs;
    v.y = lhs.y * rhs;
    v.z = lhs.z * rhs;
    return v;
}

inline Vector operator/(const Vector& lhs, float rhs) {
    Vector v;
    v.x = lhs.x / rhs;
    v.y = lhs.y / rhs;
    v.z = lhs.z / rhs;
    return v;
}

inline Vector& operator-=(Vector& lhs, const Vector& rhs) {
    lhs.x -= rhs.x;
    lhs.y -= rhs.y;
    lhs.z -= rhs.z;
    return lhs;
}

inline Vector& operator+=(Vector& lhs, const Vector& rhs) {
    lhs.x += rhs.x;
    lhs.y += rhs.y;
    lhs.z += rhs.z;
    return lhs;
}

inline Vector operator*=(Vector& lhs, float rhs) {
    lhs.x *= rhs;
    lhs.y *= rhs;
    lhs.z *= rhs;
    return lhs;
}

inline Vector operator/=(Vector& lhs, float rhs) {
    lhs.x /= rhs;
    lhs.y /= rhs;
    lhs.z /= rhs;
    return lhs;
}

inline bool operator==(const Vector& lhs, const Vector& rhs) {
    return lhs.x == rhs.x && lhs.y == rhs.y && lhs.z == rhs.z;
}

inline bool operator!=(const Vector& lhs, const Vector& rhs) {
    return !(lhs == rhs);
}

inline Vector
cross(const Vector& u, const Vector& v) {
    Vector r;
    r.x =  u.y * v.z - u.z * v.y;
    r.y = -u.x * v.z + u.z * v.x;
    r.z =  u.x * v.y - u.y * v.x;
    return r;
}

inline float
dot(const Vector& u, const Vector& v) {
    return u.x * v.x + u.y * v.y + u.z * v.z;
}

inline Vector max_vector() {
    float fmax = (std::numeric_limits<float>::max)();
    return Vector(fmax, fmax, fmax);
}

inline Vector min_vector() {
    return -max_vector();
}

inline void update_bb(Vector& mn, Vector& mx, const Vector& pn) {
    if (pn.x < mn.x) { mn.x = pn.x; }
    if (pn.y < mn.y) { mn.y = pn.y; }
    if (pn.z < mn.z) { mn.z = pn.z; }
    if (mx.x < pn.x) { mx.x = pn.x; }
    if (mx.y < pn.y) { mx.y = pn.y; }
    if (mx.z < pn.z) { mx.z = pn.z; }
}

////////////////////////////////////////////////////////////////
// 適当な matrix
struct Matrix {
    union {
        struct {
            float m00; float m01; float m02; float m03;
            float m10; float m11; float m12; float m13;
            float m20; float m21; float m22; float m23;
            float m30; float m31; float m32; float m33;
        };
        float m[16];
    };

    Matrix(){}
    Matrix(
        float e00, float e01, float e02, float e03,
        float e10, float e11, float e12, float e13,
        float e20, float e21, float e22, float e23,
        float e30, float e31, float e32, float e33) {
        m00 = e00; m01 = e01; m02 = e02; m03 = e03; 
        m10 = e10; m11 = e11; m12 = e12; m13 = e13; 
        m20 = e20; m21 = e21; m22 = e22; m23 = e23; 
        m30 = e30; m31 = e31; m32 = e32; m33 = e33;
    }
    Matrix(const Vector& x, const Vector& y, const Vector& z) {
        m00 = x.x; m01 = x.y; m02 = x.z; m03 = 0; 
        m10 = y.x; m11 = y.y; m12 = y.z; m13 = 0; 
        m20 = z.x; m21 = z.y; m22 = z.z; m23 = 0; 
        m30 = 0;   m31 = 0;   m32 = 0;   m33 = 1;
    }

    Vector& xaxis() const { return *((Vector*)(&m00)); }
    Vector& yaxis() const { return *((Vector*)(&m10)); }
    Vector& zaxis() const { return *((Vector*)(&m20)); }

    float& operator[](int n) {
        return m[n];
    }

    const float& operator[](int n) const {
        return m[n];
    }

    void invert();
    void transpose();

    static const Matrix& identity() {
        static Matrix t{
            1.0, 0.0, 0.0, 0.0,
            0.0, 1.0, 0.0, 0.0,
            0.0, 0.0, 1.0, 0.0,
            0.0, 0.0, 0.0, 1.0,
        };
        return t;
    }

    static Matrix translate(float x, float y, float z) {
        return Matrix { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, x, y, z, 1 };
    }

    static Matrix rotate(float angle, float x, float y, float z) {
        float s = sin(angle);
        float c = cos(angle);

        Matrix r {
            x * x *(1 - c)+ c, y * x *(1 - c)+ z * s, x * z *(1 - c)- y * s, 0,
            x * y *(1 - c)- z * s, y * y *(1 - c)+ c, y * z *(1 - c)+ x * s, 0,
            x * z *(1 - c)+ y * s, y * z *(1 - c)- x * s, z * z *(1 - c)+ c, 0,
            0, 0, 0, 1
        };

        return r;
    }

    static Matrix scale(float x, float y, float z) {
        return Matrix { x, 0, 0, 0, 0, y, 0, 0, 0, 0, z, 0, 0, 0, 0, 1 };
    }

};

inline Vector operator*(const Vector& lhs, const Matrix& rhs)
{
    Vector r;
    r.x = lhs.x * rhs.m00 + lhs.y * rhs.m10 + lhs.z * rhs.m20 + 1 * rhs.m30;
    r.y = lhs.x * rhs.m01 + lhs.y * rhs.m11 + lhs.z * rhs.m21 + 1 * rhs.m31;
    r.z = lhs.x * rhs.m02 + lhs.y * rhs.m12 + lhs.z * rhs.m22 + 1 * rhs.m32;
    return r;
}

inline Matrix operator*(const Matrix& lhs, float rhs)
{
    Matrix m;
    m.m00 = lhs.m00 * rhs;
    m.m01 = lhs.m01 * rhs;
    m.m02 = lhs.m02 * rhs;
    m.m03 = lhs.m03 * rhs; 
    m.m10 = lhs.m10 * rhs;
    m.m11 = lhs.m11 * rhs;
    m.m12 = lhs.m12 * rhs;
    m.m13 = lhs.m13 * rhs; 
    m.m20 = lhs.m20 * rhs;
    m.m21 = lhs.m21 * rhs;
    m.m22 = lhs.m22 * rhs;
    m.m23 = lhs.m23 * rhs; 
    m.m30 = lhs.m30 * rhs;
    m.m31 = lhs.m31 * rhs;
    m.m32 = lhs.m32 * rhs;
    m.m33 = lhs.m33 * rhs;
    return m;
}

inline Matrix operator*(const Matrix& m, const Matrix& n)
{
    Matrix tmp;
    const float *row, *column;
    div_t d;
    int i, j;

    for (i = 0; i < 16; i++) {
        tmp[i] = 0;
        d = div(i, 4);
        row = &(n[0]) + d.quot * 4;
        column = &m[0] + d.rem;
        for (j = 0; j < 4; j++)
            tmp[i] += row[j] * column[j * 4];
    }
    return tmp;
}

template <class T>
inline Matrix& operator*=(Matrix& lhs, const T& rhs)
{
    lhs = lhs * rhs;
    return lhs;
}

inline Matrix
look_at_lh(const Vector& camera_position,
            const Vector& camera_target,
            const Vector& camera_up)
{
    Vector zaxis = normalize(camera_target - camera_position);
    Vector xaxis = normalize(cross(camera_up, zaxis));
    Vector yaxis = cross(zaxis, xaxis);

    Matrix m;
    m.m00 = xaxis.x;  m.m01 = yaxis.x;  m.m02 = zaxis.x;  m.m03 = 0;
    m.m10 = xaxis.y;  m.m11 = yaxis.y;  m.m12 = zaxis.y;  m.m13 = 0;
    m.m20 = xaxis.z;  m.m21 = yaxis.z;  m.m22 = zaxis.z;  m.m23 = 0;
    m.m30 = -dot(xaxis, camera_position);
    m.m31 = -dot(yaxis, camera_position);
    m.m32 = -dot(zaxis, camera_position);
    m.m33 = 1;

    return m;
}

inline Matrix
look_at_rh(const Vector& camera_position,
            const Vector& camera_target,
            const Vector& camera_up)
{
    Vector zaxis = normalize(camera_position - camera_target);
    Vector xaxis = normalize(cross(camera_up, zaxis));
    Vector yaxis = cross(zaxis, xaxis);

    Matrix m;
    m.m00 = xaxis.x;  m.m01 = yaxis.x;  m.m02 = zaxis.x;  m.m03 = 0;
    m.m10 = xaxis.y;  m.m11 = yaxis.y;  m.m12 = zaxis.y;  m.m13 = 0;
    m.m20 = xaxis.z;  m.m21 = yaxis.z;  m.m22 = zaxis.z;  m.m23 = 0;
    m.m30 = -dot(xaxis, camera_position);
    m.m31 = -dot(yaxis, camera_position);
    m.m32 = -dot(zaxis, camera_position);
    m.m33 = 1;

    return m;
}

inline Matrix
perspective_fov_lh(
    float fovy,
    float aspect,
    float zn,
    float zf)
{
    float yscale = 1/tanf(fovy*0.5f);
    float xscale = yscale / aspect;

    return Matrix {
        xscale, 0,      0,              0,
        0,      yscale, 0,              0,
        0,      0,      zf/(zf-zn),     1,
        0,      0,      -zn*zf/(zf-zn), 0
    };
}

inline Matrix
perspective_fov_rh(
    float fovy,
    float aspect,
    float zn,
    float zf)
{


    float yscale = 1/tanf(fovy*0.5f);
    float xscale = yscale / aspect;

    return Matrix {
        xscale, 0,      0,              0,
        0,      yscale, 0,              0,
        0,      0,      zf/(zn-zf),     -1,
        0,      0,      zn*zf/(zn-zf),  0
    };
}

inline float
determinant(const Matrix& m)
{
    return
        (m.m00*m.m11 - m.m01*m.m10)*(m.m22*m.m33 - m.m23*m.m32)
        -(m.m00*m.m12 - m.m02*m.m10)*(m.m21*m.m33 - m.m23*m.m31)
        +(m.m00*m.m13 - m.m03*m.m10)*(m.m21*m.m32 - m.m22*m.m31)
        +(m.m01*m.m12 - m.m02*m.m11)*(m.m20*m.m33 - m.m23*m.m30)
        -(m.m01*m.m13 - m.m03*m.m11)*(m.m20*m.m32 - m.m22*m.m30)
        +(m.m02*m.m13 - m.m03*m.m12)*(m.m20*m.m31 - m.m21*m.m30);
}

inline void
invert(Matrix& sm)
{
    float s = determinant(sm);
    if (s == 0.0)
        return ;
    s = 1/s;

    Matrix m(
        sm.m11*(sm.m22*sm.m33 - sm.m23*sm.m32) + sm.m12*(sm.m23*sm.m31 - sm.m21*sm.m33) + sm.m13*(sm.m21*sm.m32 - sm.m22*sm.m31),
        sm.m21*(sm.m02*sm.m33 - sm.m03*sm.m32) + sm.m22*(sm.m03*sm.m31 - sm.m01*sm.m33) + sm.m23*(sm.m01*sm.m32 - sm.m02*sm.m31),
        sm.m31*(sm.m02*sm.m13 - sm.m03*sm.m12) + sm.m32*(sm.m03*sm.m11 - sm.m01*sm.m13) + sm.m33*(sm.m01*sm.m12 - sm.m02*sm.m11),
        sm.m01*(sm.m13*sm.m22 - sm.m12*sm.m23) + sm.m02*(sm.m11*sm.m23 - sm.m13*sm.m21) + sm.m03*(sm.m12*sm.m21 - sm.m11*sm.m22),

        sm.m12*(sm.m20*sm.m33 - sm.m23*sm.m30) + sm.m13*(sm.m22*sm.m30 - sm.m20*sm.m32) + sm.m10*(sm.m23*sm.m32 - sm.m22*sm.m33),
        sm.m22*(sm.m00*sm.m33 - sm.m03*sm.m30) + sm.m23*(sm.m02*sm.m30 - sm.m00*sm.m32) + sm.m20*(sm.m03*sm.m32 - sm.m02*sm.m33),
        sm.m32*(sm.m00*sm.m13 - sm.m03*sm.m10) + sm.m33*(sm.m02*sm.m10 - sm.m00*sm.m12) + sm.m30*(sm.m03*sm.m12 - sm.m02*sm.m13),
        sm.m02*(sm.m13*sm.m20 - sm.m10*sm.m23) + sm.m03*(sm.m10*sm.m22 - sm.m12*sm.m20) + sm.m00*(sm.m12*sm.m23 - sm.m13*sm.m22),

        sm.m13*(sm.m20*sm.m31 - sm.m21*sm.m30) + sm.m10*(sm.m21*sm.m33 - sm.m23*sm.m31) + sm.m11*(sm.m23*sm.m30 - sm.m20*sm.m33),
        sm.m23*(sm.m00*sm.m31 - sm.m01*sm.m30) + sm.m20*(sm.m01*sm.m33 - sm.m03*sm.m31) + sm.m21*(sm.m03*sm.m30 - sm.m00*sm.m33),
        sm.m33*(sm.m00*sm.m11 - sm.m01*sm.m10) + sm.m30*(sm.m01*sm.m13 - sm.m03*sm.m11) + sm.m31*(sm.m03*sm.m10 - sm.m00*sm.m13),
        sm.m03*(sm.m11*sm.m20 - sm.m10*sm.m21) + sm.m00*(sm.m13*sm.m21 - sm.m11*sm.m23) + sm.m01*(sm.m10*sm.m23 - sm.m13*sm.m20),

        sm.m10*(sm.m22*sm.m31 - sm.m21*sm.m32) + sm.m11*(sm.m20*sm.m32 - sm.m22*sm.m30) + sm.m12*(sm.m21*sm.m30 - sm.m20*sm.m31),
        sm.m20*(sm.m02*sm.m31 - sm.m01*sm.m32) + sm.m21*(sm.m00*sm.m32 - sm.m02*sm.m30) + sm.m22*(sm.m01*sm.m30 - sm.m00*sm.m31),
        sm.m30*(sm.m02*sm.m11 - sm.m01*sm.m12) + sm.m31*(sm.m00*sm.m12 - sm.m02*sm.m10) + sm.m32*(sm.m01*sm.m10 - sm.m00*sm.m11),
        sm.m00*(sm.m11*sm.m22 - sm.m12*sm.m21) + sm.m01*(sm.m12*sm.m20 - sm.m10*sm.m22) + sm.m02*(sm.m10*sm.m21 - sm.m11*sm.m20)
       );

    m *= s;
    sm = m;
}

inline void Matrix::invert() { ::invert(*this); }

inline void
transpose(Matrix& m) {
    m = Matrix {
        m[0], m[4], m[8], m[12],
        m[1], m[5], m[9], m[13],
        m[2], m[6], m[10], m[14],
        m[3], m[7], m[11], m[15]
    };
}

inline void Matrix::transpose() { ::transpose(*this); }

inline void normalize_f(Matrix& m) {
    normalize_f(m.xaxis());
    normalize_f(m.yaxis());
    normalize_f(m.zaxis());
    m.m30 = m.m31 = m.m32 = 0;
    m.m03 = m.m13 = m.m23 = 0;
    m.m33 = 1.0f;
}

const float PARTIX_PI = 3.1415926535897931f;

inline float
DEG2RAD(float theta) {
    return theta / 180.0f * PARTIX_PI;
}

inline float
RAD2DEG(float radian) {
    return radian * 180.0f / PARTIX_PI;
}

inline Vector 
unproject(
    const Matrix& iview,
    const Matrix& proj,
    float w,
    float h,
    const Vector& p) {
    Vector v;
    v.x =(p.x * 2.0f / w - 1)/ proj.m00;
    v.y = -(p.y * 2.0f / h - 1)/ proj.m11;
    v.z = 1;
    v = normalize(v);

    Vector dir;
    dir.x = v.x * iview.m00 + v.y * iview.m10 + v.z * iview.m20;
    dir.y = v.x * iview.m01 + v.y * iview.m11 + v.z * iview.m21;
    dir.z = v.x * iview.m02 + v.y * iview.m12 + v.z * iview.m22;

    return dir;
}

inline bool
test_ray_sphere(
    const Vector& org,
    const Vector& dir,
    const Vector& center,
    float radius,
    float& t,
    Vector& q) {
    Vector m = org - center;
    float b = dot(m, dir);
    float c = dot(m, m)- radius * radius;
    if (0.0f <c && 0.0f <b) { return false; }
    float discr = b*b - c;
    if (discr <0.0f) { return false; }
    t = -b - sqrtf(discr);
    if (t <0.0f)t = 0.0f;
    q = org + dir * t;
    return true;
}

template <class OS>
OS& operator<<(OS& os, const Matrix& m) {
    os << "{ ";
    for (int i = 0 ; i < 16 ; i++) {
        os << m[i] << ", ";
    }
    os << " }";
    return os;
}

////////////////////////////////////////////////////////////////
// 超適当なクォータ二オン
struct Quaternion {
    float    u;
    float    x;
    float    y;
    float    z;

    Quaternion(){}
    Quaternion(float uu, float xx, float yy, float zz)
        : u(uu), x(xx), y(yy), z(zz) {}
    Quaternion(float uu, const Vector& vv)
        : u(uu), x(vv.x), y(vv.y), z(vv.z) {}

    Quaternion operator*(const Quaternion& q) const {
        return Quaternion(
            u * q.u - x * q.x - y * q.y - z * q.y,
            u * q.x + x * q.u + y * q.z - z * q.y,
            u * q.y + y * q.u + z * q.x - x * q.z,
            u * q.z + z * q.u + x * q.y - y * q.x);
    }

    Quaternion& operator *= (const Quaternion& q) {
        *this = *this * q;
        return *this;
    }

    Quaternion normalize() const {
        float m = float(1.0)/ module();
        return Quaternion(u * m, x * m, y * m, z * m);
    }

    Quaternion conjugate() const {
        return Quaternion(u, -x, -y, -z);
    }

    Quaternion inverse() const {
        float d = u * u + x * x + y * y + z * z;
        return Quaternion(u/d, -x/d, -y/d, -z/d);
    }

    float module() const
    {
        return float(sqrt(u * u + x * x + y * y + z * z));
    }

};

inline
bool rotation_arc(Quaternion& q, const Vector& av0, const Vector& av1)
{
    Vector v0 = normalize(av0);
    Vector v1 = normalize(av1);
    Vector c = cross(v0, v1);
    float d = dot(v0, v1);
    if(d < -1.0+1.0e-6) { return false; }
    float s = sqrtf((1 + d) * 2.0f);
    q.x = c.x / s;
    q.y = c.y / s;
    q.z = c.z / s;
    q.u = s / 2.0f;
    return true;
}

inline 
Quaternion slerp(const Quaternion& a, const Quaternion& b, float t) {
    Quaternion r;
    float t_ = 1 - t;
    float Wa, Wb;
    float theta = acos(a.x*b.x + a.y*b.y + a.z*b.z + a.u*b.u);
    float sn = sin(theta);
    Wa = sin(t_*theta) / sn;
    Wb = sin(t*theta) / sn;
    r.x = Wa*a.x + Wb*b.x;
    r.y = Wa*a.y + Wb*b.y;
    r.z = Wa*a.z + Wb*b.z;
    r.u = Wa*a.u + Wb*b.u;
    r.normalize();
    return r;
}

inline 
float clamp(float n, float mn, float mx) {
    if (n < mn) { return mn; }
    if (mx < n) { return mx; }
    return n;
}

#endif // GEOMETRY_HPP

