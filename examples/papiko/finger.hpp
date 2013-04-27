#ifndef FINGER_HPP
#define FINGER_HPP

#include "partix_common.hpp"

class Finger {
public:
    Finger();
    ~Finger();

    void load_coarse( px::world_type* world, const char* filename );

    void render( LPDIRECT3DDEVICE9 device, const D3DXVECTOR4& eye );

    void attack( const D3DXVECTOR3& pos );
    void wait( const D3DXVECTOR3& pos );

    D3DXVECTOR3 get_current_center() { return coarse_->get_current_center(); }
    px::softvolume_type* get_coarse() { return coarse_.get(); }

private:        
    bool waiting_;
    boost::scoped_ptr< px::tetrahedralmesh_type > mesh_;
    boost::scoped_ptr< px::softvolume_type > coarse_;
        
};

#endif // FINGER_HPP
