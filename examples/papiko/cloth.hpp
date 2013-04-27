#ifndef CLOTH_HPP
#define CLOTH_HPP

#include "partix_common.hpp"
#include "partix/partix_cloth.hpp"

class Cloth {
public:
    Cloth();
    ~Cloth();

    void load_coarse( px::world_type* world, const char* filename );

    void render( LPDIRECT3DDEVICE9 device, const D3DXVECTOR3& eye );

    D3DXVECTOR3 get_current_center() { return cloth_->get_current_center(); }
    px::cloth_type* get_coarse() { return cloth_.get(); }
    void fix();

    void select( const D3DXVECTOR3& );
    void move_to( const D3DXVECTOR3& );

private:        
    boost::scoped_ptr< px::cloud_type >     cloud_;
    boost::scoped_ptr< px::cloth_type >     cloth_;
    D3DXVECTOR3                             select_origin_;
        
};

#endif // CLOTH_HPP
