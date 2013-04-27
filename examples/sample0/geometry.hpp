// $Id: geometry.hpp 32 2008-10-25 12:19:56Z Naoyuki.Hirayama $

#ifndef GEOMETRY_HPP
#define GEOMETRY_HPP

#include <cmath>

////////////////////////////////////////////////////////////////
// ’´“K“–‚È vector
struct Vector {
    float x;
    float y;
    float z;
};

inline float length_sq( const Vector& v )
{
    return v.x * v.x + v.y * v.y + v.z * v.z;
}

inline float length( const Vector& v )
{
    return sqrtf( length_sq( v ) );
}

inline Vector normalize( const Vector& v )
{
    float ilen = 1.0f / length( v );
    Vector r;
    r.x = v.x * ilen;
    r.y = v.y * ilen;
    r.z = v.z * ilen;
    return r;
}

inline Vector operator-( const Vector& v )
{
    Vector r;
    r.x = -v.x;
    r.y = -v.y;
    r.z = -v.z;
    return r;
}

inline Vector operator-( const Vector& lhs, const Vector& rhs )
{
    Vector v;
    v.x = lhs.x - rhs.x;
    v.y = lhs.y - rhs.y;
    v.z = lhs.z - rhs.z;
    return v;
}

inline Vector operator+( const Vector& lhs, const Vector& rhs )
{
    Vector v;
    v.x = lhs.x + rhs.x;
    v.y = lhs.y + rhs.y;
    v.z = lhs.z + rhs.z;
    return v;
}

inline Vector operator*( const Vector& lhs, float rhs )
{
    Vector v;
    v.x = lhs.x * rhs;
    v.y = lhs.y * rhs;
    v.z = lhs.z * rhs;
    return v;
}

inline Vector& operator-=( Vector& lhs, const Vector& rhs )
{
    lhs.x -= rhs.x;
    lhs.y -= rhs.y;
    lhs.z -= rhs.z;
    return lhs;
}

inline Vector& operator+=( Vector& lhs, const Vector& rhs )
{
    lhs.x += rhs.x;
    lhs.y += rhs.y;
    lhs.z += rhs.z;
    return lhs;
}

inline Vector operator*=( Vector& lhs, float rhs )
{
    lhs.x *= rhs;
    lhs.y *= rhs;
    lhs.z *= rhs;
    return lhs;
}

inline Vector
cross( const Vector& u, const Vector& v )
{
    Vector r;
    r.x =  u.y * v.z - u.z * v.y;
    r.y = -u.x * v.z + u.z * v.x;
    r.z =  u.x * v.y - u.y * v.x;
    return r;
}

inline float
dot( const Vector& u, const Vector& v )
{
    return u.x * v.x + u.y * v.y + u.z * v.z;
}
        

////////////////////////////////////////////////////////////////
// “K“–‚È matrix
struct Matrix {
    float m00;  float m01;  float m02;  float m03;
    float m10;  float m11;  float m12;  float m13;
    float m20;  float m21;  float m22;  float m23;
    float m30;  float m31;  float m32;  float m33;

    Matrix(){}
    Matrix(
        const float& e00,const float& e01,const float& e02,const float& e03,
        const float& e10,const float& e11,const float& e12,const float& e13,
        const float& e20,const float& e21,const float& e22,const float& e23,
        const float& e30,const float& e31,const float& e32,const float& e33 )
    {
        m00 = e00;  m01 = e01;  m02 = e02;  m03 = e03; 
        m10 = e10;  m11 = e11;  m12 = e12;  m13 = e13; 
        m20 = e20;  m21 = e21;  m22 = e22;  m23 = e23; 
        m30 = e30;  m31 = e31;  m32 = e32;  m33 = e33;
    }
};

inline Vector operator*( const Vector& left, const Matrix& right )
{
    Vector r;
    r.x =
        left.x * right.m00 +
        left.y * right.m10 +
        left.z * right.m20 +
        1 * right.m30;
    r.y =
        left.x * right.m01 +
        left.y * right.m11 +
        left.z * right.m21 +
        1 * right.m31;
    r.z =
        left.x * right.m02 +
        left.y * right.m12 +
        left.z * right.m22 +
        1 * right.m32;
    return r;
}

inline Matrix
look_at_rh( const Vector& camera_position,
            const Vector& camera_target,
            const Vector& camera_up )
{
    Vector zaxis = normalize( camera_position - camera_target );
    Vector xaxis = normalize( cross( camera_up, zaxis ) );
    Vector yaxis = cross( zaxis, xaxis );

    Matrix m;
    m.m00 = xaxis.x;  m.m01 = yaxis.x;  m.m02 = zaxis.x;  m.m03 = 0;
    m.m10 = xaxis.y;  m.m11 = yaxis.y;  m.m12 = zaxis.y;  m.m13 = 0;
    m.m20 = xaxis.z;  m.m21 = yaxis.z;  m.m22 = zaxis.z;  m.m23 = 0;
    m.m30 = -dot( xaxis, camera_position );
    m.m31 = -dot( yaxis, camera_position );
    m.m32 = -dot( zaxis, camera_position );
    m.m33 = 1;

    return m;
}

#endif // GEOMETRY_HPP
