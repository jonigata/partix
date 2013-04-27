/*!
  @file     partix_point.hpp
  @brief    <概要>

  <説明>
  $Id: partix_point.hpp 152 2007-04-26 10:12:36Z hirayama $
*/
#ifndef PARTIX_POINT_HPP
#define PARTIX_POINT_HPP

#include "partix_forward.hpp"

namespace partix {

// Point
template < class Traits >
class Point {
public:
    typedef typename Traits::real_type              real_type;
    typedef typename Traits::vector_type            vector_type;
    typedef typename Traits::point_load_type        load_type;

    vector_type     old_position;           // 古い位置
    vector_type     new_position;           // 新しい位置
    vector_type     velocity;               // 現在の速度
    vector_type     normal;                 // 頂点法線
    vector_type     source_position;        // 初期位置
    vector_type     ideal_offset;           // 重心からのオフセット(変形済み)
    vector_type     pushout0;               // 制約による移動
    vector_type     pushout1;               // 制約による移動
    vector_type     forces;                 // 力
    real_type       friction;               // 摩擦
    real_type       mass;                   // 質量
    real_type       invmass;                // 質量の逆数
    load_type       load;                   // 積荷(ユーザー定義)

    // 以下は内部処理用
    bool            surface;                // 表面パーティクル
    bool            process_flag;           // 汎用処理フラグ
    bool            collided;               // spatial hash用
    bool            propagating;            // penetration propagation用
    unsigned char   raytest;                // raytest用実行ステート
    real_type       energy;                 // 運動エネルギー
    real_type       penetration_depth_numerator;
    // penetration depthの分子
    vector_type     penetration_direction_numerator;
    // penetration dirの分子
    real_type       penetration_denominator; // penetrationの分母
    vector_type     penetration_vector;     // penetration vector
    real_type       penetration_error;      // penetration depthの誤差
    vector_type     tmp_velocity;           // 中間速度
    real_type       contact;
    // 1+Σ^n_j=1.hji(barycentric座標のsum, cの分母)
    int             cloth_flags;            // cloth用bitflags
    void*           ray_slot;               // rayprocess用データ

    void check()
    {
#if 0
        if( isnan( friction ) ) {
            char buffer[256];
            sprintf( buffer, "friction: %f\n", friction );
            OutputDebugStringA( buffer );
            DebugBreak();
        }
        if( isnan( new_position.x ) ||
            isnan( new_position.y ) ||
            isnan( new_position.z ) ) {
            char buffer[256];
            sprintf( buffer, "new_position: %f, %f, %f\n", new_position.x, new_position.y, new_position.z );
            OutputDebugStringA( buffer );
            DebugBreak();
        }
        if( isnan( old_position.x ) ||
            isnan( old_position.y ) ||
            isnan( old_position.z ) ) {
            char buffer[256];
            sprintf( buffer, "old_position: %f, %f, %f\n", old_position.x, old_position.y, old_position.z );
            OutputDebugStringA( buffer );
            DebugBreak();
        }
        if( isnan( pushout0.x ) ||
            isnan( pushout0.y ) ||
            isnan( pushout0.z ) ) {
            char buffer[256];
            sprintf( buffer, "pushout0: %f, %f, %f\n", pushout0.x, pushout0.y, pushout0.z );
            OutputDebugStringA( buffer );
            DebugBreak();
        }
        if( isnan( pushout1.x ) ||
            isnan( pushout1.y ) ||
            isnan( pushout1.z ) ) {
            char buffer[256];
            sprintf( buffer, "pushout1: %f, %f, %f\n", pushout1.x, pushout1.y, pushout1.z );
            OutputDebugStringA( buffer );
            DebugBreak();
        }
        if( isnan( mass ) || isnan( invmass ) ) {
            char buffer[256];
            sprintf( buffer, "mass: %f, %f\n", mass, invmass );
            OutputDebugStringA( buffer );
            DebugBreak();
        }
#endif
    }

};

} // namespace

#endif // PARTIX_POINT_HPP
