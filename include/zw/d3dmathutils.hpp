#ifndef D3DMATHUTILS_HPP
#define D3DMATHUTILS_HPP

template < class T >
inline
T square( T x ) { return x * x; }

template < class T >
inline
T signed_square( T x )
{
        T flag = 1.0f;
        if( x < 0 ) { flag = -1.0f; }
        return flag * square( x );
}

template < class T >
inline
T clamp( T x ) { if( x < 0 ) { return 0; } if( T(1) < x ) { return T(1); } return x; }

template < class T >
inline
T limit( T x, T mn = T(-1), T mx = T(1) ) { if( x < mn ) { return mn; } if( mx < x ) { return mx; } return x; }

////////////////////////////////////////////////////////////////
// 2D
inline
float length( const D3DXVECTOR2& x )
{
        return D3DXVec2Length( &x );
}

inline
float dot( const D3DXVECTOR2& a, const D3DXVECTOR2& b )
{
        return D3DXVec2Dot( &a, &b );
}

inline
float cross( const D3DXVECTOR2& a, const D3DXVECTOR2& b )
{
        return a.x * b.y - a.y * b.x;
}

inline
bool line_cross_point( D3DXVECTOR2& result,
                       const D3DXVECTOR2& u0, const D3DXVECTOR2& u1,
                       const D3DXVECTOR2& v0, const D3DXVECTOR2& v1 )
{
        const float epsilon = std::numeric_limits< float >::epsilon();

        D3DXVECTOR2 v0u0 = v0 - u0;
        D3DXVECTOR2 v1v0 = v1 - v0;
        D3DXVECTOR2 u1u0 = u1 - u0;
        D3DXVECTOR2 v1v0t( v1v0.y, -v1v0.x );
        D3DXVECTOR2 u1u0t( u1u0.y, -u1u0.x );

        float r = u1u0.x*v1v0t.x + u1u0.y*v1v0t.y;
        if( fabsf( r ) < epsilon ) {
                //dprintf( "can't calculate junktions\n" );
                return false;
        }

        float s = (v0u0.x*v1v0t.x + v0u0.y*v1v0t.y) / r;
        float t = (v0u0.x*u1u0t.x + v0u0.y*u1u0t.y) / r;
 
        result = D3DXVECTOR2( u0.x + s * u1u0.x, u0.y + s * u1u0.y );
        return true;
}

inline
D3DXVECTOR2 rotate_2d( const D3DXVECTOR2& p, const D3DXVECTOR2& c, float angle )
{
        D3DXVECTOR2 pt = p - c;

        float cos_theta = cosf( angle );
        float sin_theta = sinf( angle );
        return D3DXVECTOR2(
                pt.x * cos_theta - pt.y * sin_theta,
                pt.x * sin_theta + pt.y * cos_theta ) + c;
}

inline
D3DXVECTOR2 project_to_plane( const D3DXVECTOR3& x_axis, const D3DXVECTOR3& y_axis, const D3DXVECTOR3& p )
{
        return D3DXVECTOR2(
                D3DXVec3Dot( &p, &x_axis ) / D3DXVec3Dot( &x_axis, &x_axis ),
                D3DXVec3Dot( &p, &y_axis ) / D3DXVec3Dot( &y_axis, &y_axis ) );
}

////////////////////////////////////////////////////////////////
// 3D
inline
void normalize_f( D3DXVECTOR3& x )
{
        D3DXVec3Normalize( &x, &x );
}

inline
D3DXVECTOR3 normalize( const D3DXVECTOR3& x )
{
        D3DXVECTOR3 y;
        D3DXVec3Normalize( &y, &x );
        return y;
}

inline
float length( const D3DXVECTOR3& x )
{
        return D3DXVec3Length( &x );
}

inline
float length_sq( const D3DXVECTOR3& x )
{
        return D3DXVec3LengthSq( &x );
}

inline
float distance( const D3DXVECTOR3& x, const D3DXVECTOR3& y )
{
        return D3DXVec3Length( &( x - y ) );
}

inline
float distance_sq( const D3DXVECTOR3& x, const D3DXVECTOR3& y )
{
        return D3DXVec3LengthSq( &( x - y ) );
}

inline
D3DXVECTOR3 cross( const D3DXVECTOR3& x, const D3DXVECTOR3& y )
{
        D3DXVECTOR3 t;
        D3DXVec3Cross( &t, &x, &y );
        return t;
}

inline
float dot( const D3DXVECTOR3& x, const D3DXVECTOR3& y )
{
        return D3DXVec3Dot( &x, &y );
}

inline
D3DXVECTOR3 transform( const D3DXVECTOR3& v, const D3DXMATRIX& m )
{
        D3DXVECTOR3 q;
        D3DXVec3TransformCoord( &q, &v, &m );
        return q;
}

inline
float triangle_area( const D3DXVECTOR3& v0, const D3DXVECTOR3& v1, const D3DXVECTOR3& v2 )
{
        D3DXVECTOR3 e0 = v1 - v0;
        D3DXVECTOR3 e1 = v2 - v0;
        D3DXVECTOR3 e2 = v2 - v1;
        
        D3DXVECTOR3 c = cross( e0, e1 );
        return length( c ) * 0.5f;
}

inline
float dihedral_angle( const D3DXVECTOR3& v, const D3DXVECTOR3& u, const D3DXVECTOR3& s, const D3DXVECTOR3& t )
{
        D3DXVECTOR3 n0 = normalize( cross( t - v, u - v ) );
        D3DXVECTOR3 n1 = normalize( cross( s - u, v - u ) );
        
        float d = dot( n0, n1 );
        if( dot( n0, s - t ) <= 0 ) {
                // 山折
                return ( d - 1.0f ) * -0.5f; // 0~1
        } else {
                // 谷折
                return ( d - 1.0f ) * 0.5f; // 0~-1
        }
}

inline
float triangle_compactness( const D3DXVECTOR3& v0, const D3DXVECTOR3& v1, const D3DXVECTOR3& v2 )
{
        // 正三角形 1.0f
        // 平行な点 0.0f

        D3DXVECTOR3 e0 = v1 - v0;
        D3DXVECTOR3 e1 = v2 - v0;
        D3DXVECTOR3 e2 = v2 - v1;
        
        D3DXVECTOR3 c = cross( e0, e1 );
        float a = length( c ) * 0.5f;
        
        return 4.0f * sqrtf( 3.0f ) * a / ( length_sq( e0 ) + length_sq( e1 ) + length_sq( e2 ) );
}

inline void update_minmax( float& mn, float& mx, float v )
{
        if( v < mn ) { mn = v; }
        if( mx < v ) { mx = v; }
}

inline void initialize_bb( D3DXVECTOR3& bbmin, D3DXVECTOR3& bbmax )
{
        const float fmax = std::numeric_limits< float >::max();
        bbmin = D3DXVECTOR3(  fmax,  fmax,  fmax );
        bbmax = D3DXVECTOR3( -fmax, -fmax, -fmax );
}

inline void update_bb( D3DXVECTOR3& bbmin, D3DXVECTOR3& bbmax, const D3DXVECTOR3& p )
{
        update_minmax( bbmin.x, bbmax.x, p.x );
        update_minmax( bbmin.y, bbmax.y, p.y );
        update_minmax( bbmin.z, bbmax.z, p.z );
}

inline void minmax4( float a, float b, float c, float d, float& minv, float& maxv )
{
        float minab, maxab, mincd, maxcd;
        minab = b; maxab = a; if( a < b ) { minab = a; maxab = b; }
        mincd = d; maxcd = c; if( c < d ) { mincd = c; maxcd = d; }
        minv = mincd; if( minab < mincd ) { minv = minab; }
        maxv = maxab; if( maxab < maxcd ) { maxv = maxcd; }
}

inline void minmax3( float a, float b, float c, float& minv, float& maxv )
{
        float minab, maxab;
        minab = b; maxab = a;   if( a < b ) { minab = a; maxab = b; }
        minv = c;               if( minab < c ) { minv = minab; }
        maxv = maxab;           if( maxab < c ) { maxv = c; }
}

inline void minmax2( float a, float b, float& minv, float& maxv )
{
        if( a < b ) { minv = a; maxv = b; return; }
        minv = b; maxv = a;
}

inline
bool test_segment_triangle( const D3DXVECTOR3& r0,
                            const D3DXVECTOR3& r1,
                            const D3DXVECTOR3& v0,
                            const D3DXVECTOR3& v1,
                            const D3DXVECTOR3& v2,
                            D3DXVECTOR3& uvt )
{
        float epsilon = 0.000001f;

        D3DXVECTOR3 dir = r1 - r0;

        /* find vectors for two edges sharing vert0 */
        D3DXVECTOR3 e1 = v1 - v0;
        D3DXVECTOR3 e2 = v2 - v0;

        /* begin calculating determinant - also used to calculate U parameter */
        D3DXVECTOR3 pvec = cross( dir, e2 );

        /* if determinant is near zero, ray lies in plane of triangle */
        float det = dot( e1, pvec );

        if ( det < epsilon )
                return false;

        /* calculate distance from vert0 to ray origin */
        D3DXVECTOR3 tvec = r0 - v0;

        /* calculate U parameter and test bounds */
        uvt.x = dot( tvec, pvec );
        if ( uvt.x < 0 || uvt.x > det )
                return false;

        /* prepare to test V parameter */
        D3DXVECTOR3 qvec = cross( tvec, e1 );

        /* calculate V parameter and test bounds */
        uvt.y = dot( dir, qvec );
        if ( uvt.y < 0 || uvt.x + uvt.y > det)
                return false;

        /* calculate t, scale parameters, ray intersects triangle */
        float z = dot( e2, qvec );
        if( z < 0 || det < z ) 
                return false;

        uvt.z = z;
        uvt /= det;

        return true;
}

inline
bool test_plane_segment( const D3DXVECTOR3& plane_position,
                         const D3DXVECTOR3& plane_normal,
                         const D3DXVECTOR3& s0,
                         const D3DXVECTOR3& s1,
                         float& t)
{
        D3DXVECTOR3 ab = s1 - s0;
        t =
                ( dot( plane_position, plane_normal ) - dot( plane_normal, s0 ) ) /
                dot( plane_normal, ab );

        return float( 0 ) <= t && t <= float( 1 );
}

inline
void point_to_triangle_distance( 
        D3DXVECTOR3& q,
        D3DXVECTOR3& uvw, 
        const D3DXVECTOR3& a,
        const D3DXVECTOR3& b,
        const D3DXVECTOR3& c,
        const D3DXVECTOR3& p )
{
        D3DXVECTOR3 ab = b - a;
        D3DXVECTOR3 ac = c - a;
        D3DXVECTOR3 ap = p - a;

        // PがAの外側の頂点領域にあるかどうかチェック
        float d1 = dot( ab, ap );
        float d2 = dot( ac, ap );
        if( d1 <= 0.0f && d2 <= 0.0f ) { q = a; uvw = D3DXVECTOR3( 1, 0, 0 ); return; } // 重心座標(1,0,0)
                
        // PがBの外側の頂点領域の中にあるかどうかチェック
        D3DXVECTOR3 bp = p - b;
        float d3 = dot( ab, bp );
        float d4 = dot( ac, bp );
        if( d3 >= 0.0f && d4 <= d3 ) { q = b; uvw = D3DXVECTOR3( 0, 1, 0 ); return; } // 重心座標(0,1,0)

        // PがABの辺領域の中にあるかどうかチェックし、あればPのAB上に対する射影を返す
        float vc = d1 * d4 - d3 * d2;
        if( vc <= 0.0f && d1 >= 0.0f && d3 <= 0.0f ) {
                float v = d1 / ( d1 - d3 );
                q = a + v * ab; // 重心座標(1-v,v,0)
                uvw = D3DXVECTOR3( 1-v, v, 0 ); 
                return;
        }

        // PがCの外側の頂点領域の中にあるかどうかチェック
        D3DXVECTOR3 cp = p - c;
        float d5 = dot( ab, cp );
        float d6 = dot( ac, cp );
        if( d6 >= 0.0f && d5 <= d6 ) { q = c; uvw = D3DXVECTOR3( 0, 0, 1 ); return; } // 重心座標(0,0,1)

        // PがACの辺領域の中にあるかどうかチェックし、あればPのAB上に対する射影を返す
        float vb = d5 * d2 - d1 * d6;
        if( vb <= 0.0f && d2 >= 0.0f && d6 <= 0.0f ) {
                float w = d2 / ( d2 - d6 );
                q = a + w * ac;
                uvw = D3DXVECTOR3( 1-w, 0, w );  // 重心座標(1-w,0,w)
                return;
        }

        // PがBCの外側の頂点領域の中にあるかどうかチェック
        float va = d3 * d6 - d5 * d4;
        if( va <= 0.0f && ( d4 - d3 ) >= 0.0f && ( d5 - d6 ) >= 0.0f ) {
                float w = ( d4 - d3 ) / ( ( d4 - d3 ) + ( d5 - d6 ) );
                q = b + w * ( c - b );
                uvw = D3DXVECTOR3( 0, 1-w, w );  // 重心座標(0,1-w,w)
                return;
        }

        // Pは面領域の中にある Qをその重心座標(u,v,w)を用いて計算
        float denom = 1.0f / ( va + vb + vc );
        float v = vb * denom;
        float w = vc * denom;
        q = a + ab * v + ac * w; // = u * a + v * b + w * c, u = va * denom = 1.0f - v - w
        uvw = D3DXVECTOR3( 1.0f - v - w, v, w );
        return;
}

inline 
float segment_to_point_distance( const D3DXVECTOR3& a, const D3DXVECTOR3& b, const D3DXVECTOR3& c )
{
        const float EPS = 0.00001f;
        if ( dot( b-a, c-a ) < EPS ) return distance( c, a );
        if ( dot( a-b, c-b ) < EPS ) return distance( c, b );
        return length( cross( b-a, c-a ) ) / length( b-a );
}

inline
D3DXVECTOR4 get_screen_coord( LPDIRECT3DDEVICE9 device, const D3DXVECTOR3& src )
{
        D3DXMATRIX world; device->GetTransform( D3DTS_WORLD,  &world );
        D3DXMATRIX view;  device->GetTransform( D3DTS_VIEW,  &view );
        D3DXMATRIX proj;  device->GetTransform( D3DTS_PROJECTION,  &proj );
        D3DVIEWPORT9 viewport; device->GetViewport( &viewport );

        D3DXVECTOR4 out;
        D3DXVec3Transform( &out, &src, &world );
        D3DXVec3Transform( &out, (D3DXVECTOR3*)(&out), &view );
        D3DXVec3Transform( &out, (D3DXVECTOR3*)(&out), &proj );

        out.x /= out.w;
        out.y /= out.w;
        out.z /= out.w;
        out.x = viewport.Width / 2 + out.x * viewport.Width / 2;
        out.y = viewport.Height / 2 + out.y * viewport.Height / 2;
        return out;
}

inline
void get_world_ray( LPDIRECT3DDEVICE9 device, D3DXVECTOR3& org, D3DXVECTOR3& dir, int mouse_x, int mouse_y )
{
        D3DXMATRIX proj;
        device->GetTransform( D3DTS_PROJECTION,  &proj );

        D3DVIEWPORT9 viewport;
        device->GetViewport( &viewport );

        D3DXVECTOR3 v;
        v.x =  ( mouse_x * 2.0f / viewport.Width - 1 ) / proj._11;
        v.y = -( mouse_y * 2.0f / viewport.Height - 1 ) / proj._22;
        v.z = 1;
        D3DXVec3Normalize( &v, &v );

        D3DXMATRIX view;
        device->GetTransform( D3DTS_VIEW, &view );
        D3DXMATRIX iview;
        D3DXMatrixInverse( &iview, NULL, &view );

        dir.x = v.x * iview._11 + v.y * iview._21 + v.z * iview._31;
        dir.y = v.x * iview._12 + v.y * iview._22 + v.z * iview._32;
        dir.z = v.x * iview._13 + v.y * iview._23 + v.z * iview._33;

        org.x = iview._41;
        org.y = iview._42;
        org.z = iview._43;
}

inline
void get_triangle_bb(  D3DXVECTOR3& bbmin,
                       D3DXVECTOR3& bbmax,
                       const D3DXVECTOR3& v0,
                       const D3DXVECTOR3& v1,
                       const D3DXVECTOR3& v2 )
{
        minmax3( v0.x, v1.x, v2.x, bbmin.x, bbmax.x );
        minmax3( v0.y, v1.y, v2.y, bbmin.y, bbmax.y );
        minmax3( v0.z, v1.z, v2.z, bbmin.z, bbmax.z );
}

inline
bool rotation_arc( D3DXQUATERNION& q, D3DXVECTOR3 v0, D3DXVECTOR3 v1 )
{
        normalize_f( v0 );
        normalize_f( v1 );
        D3DXVECTOR3 c = cross( v0, v1 );
        float d = dot( v0, v1 );
        if( d < -0.99999f ) { return false; }
        float s = sqrtf( ( 1 + d ) * 2.0f );
        q.x = c.x / s;
        q.y = c.y / s;
        q.z = c.z / s;
        q.w = s / 2.0f;
        return true;
}

inline
void compute_plane( D3DXVECTOR3& n, float& d,
                    const D3DXVECTOR3& a, const D3DXVECTOR3& b, const D3DXVECTOR3& c )
{
        n = normalize( cross( b-a, c-a ) );
        d = dot( n, a );
}

inline
D3DXVECTOR3 difference_from_point_to_plane( const D3DXVECTOR3& p, const D3DXVECTOR3& n, float d )
{
        float t = ( dot( n, p ) - d ) / dot( n, n );
        return -t * n;
}

inline
D3DXVECTOR3 difference_from_point_to_line( const D3DXVECTOR3& p, const D3DXVECTOR3& a, const D3DXVECTOR3& b )
{
        D3DXVECTOR3 ab = a - b;
        float t = dot( p-a, ab ) / dot( ab, ab );
        D3DXVECTOR3 d = a + t * ab;
        return d - p;
}

inline
D3DXVECTOR3 get_safe_rotation_axis( const D3DXVECTOR3& p0, const D3DXVECTOR3& p1, const D3DXVECTOR3 p2 )
{
        D3DXVECTOR3 e0 = p0 - p1;
        D3DXVECTOR3 e1 = p2 - p1;
        D3DXVECTOR3 ne0 = normalize( e0 );
        D3DXVECTOR3 ne1 = normalize( e1 );
        float d = dot( ne0, ne1 );

        D3DXVECTOR3 axis;
        if( fabsf( d )  < 0.99999f ) {
                axis = normalize( cross( ne0, ne1 ) );
        } else {
                D3DXVECTOR3 up( 0, 1, 0 );
                if( fabsf( dot( e0, up ) ) < 0.9f ) {
                        axis = normalize( cross( e0, up ) );
                } else {
                        axis = normalize( cross( e0, D3DXVECTOR3( 0, 0, 1 ) ) ) ;
                }
        }
        return axis;
}

#endif // D3DMATHUTILS_HPP
