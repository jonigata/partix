/*!
  @file     partix_point.hpp
  @brief    <�T�v>

  <����>
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

    vector_type     old_position;           // �Â��ʒu
    vector_type     new_position;           // �V�����ʒu
    vector_type     velocity;               // ���݂̑��x
    vector_type     normal;                 // ���_�@��
    vector_type     source_position;        // �����ʒu
    vector_type     ideal_offset;           // �d�S����̃I�t�Z�b�g(�ό`�ς�)
    vector_type     pushout0;               // ����ɂ��ړ�
    vector_type     pushout1;               // ����ɂ��ړ�
    vector_type     forces;                 // ��
    real_type       friction;               // ���C
    real_type       mass;                   // ����
    real_type       invmass;                // ���ʂ̋t��
    load_type       load;                   // �ω�(���[�U�[��`)

    // �ȉ��͓��������p
    bool            surface;                // �\�ʃp�[�e�B�N��
    bool            process_flag;           // �ėp�����t���O
    bool            collided;               // spatial hash�p
    bool            propagating;            // penetration propagation�p
    unsigned char   raytest;                // raytest�p���s�X�e�[�g
    real_type       energy;                 // �^���G�l���M�[
    real_type       penetration_depth_numerator;
    // penetration depth�̕��q
    vector_type     penetration_direction_numerator;
    // penetration dir�̕��q
    real_type       penetration_denominator; // penetration�̕���
    vector_type     penetration_vector;     // penetration vector
    real_type       penetration_error;      // penetration depth�̌덷
    vector_type     tmp_velocity;           // ���ԑ��x
    real_type       contact;
    // 1+��^n_j=1.hji(barycentric���W��sum, c�̕���)
    int             cloth_flags;            // cloth�pbitflags
    void*           ray_slot;               // rayprocess�p�f�[�^

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
