#ifndef PAPISHAPE_HPP
#define PAPISHAPE_HPP

#include "partix_common.hpp"

struct dense_handle_tag {};
typedef dense_handle_tag* dense_handle;

class PapiShape {
public:
    PapiShape();
    ~PapiShape();

    void set_data_directory( const std::string& dir );
    dense_handle load_dense(
        LPDIRECT3DDEVICE9 device, const std::string& objfilename );
    void load_coarse(
        px::world_type*     world,
        const std::string&  filename,
        float               mass,
        float               stiffness );
    void subdivide( int level );
    void tie_dense_to_coarse( dense_handle );
    void attach_brdf(
        LPDIRECT3DDEVICE9,
        dense_handle,
        const char*         material,
        const std::string&  brdf_filename );
    void attach_smap(
        LPDIRECT3DDEVICE9,
        dense_handle,
        const char*         material,
        const std::string&  smap_filename );

    void on_lost_device( LPDIRECT3DDEVICE9 device );
    void on_reset_device( LPDIRECT3DDEVICE9 device );

    void update( float tick );
    void render( LPDIRECT3DDEVICE9 device, const D3DXVECTOR3& eye );
    void fix();
    void reload_textures( LPDIRECT3DDEVICE9 );
    void reload_models( LPDIRECT3DDEVICE9 );

    std::vector< boost::shared_ptr< px::softvolume_type > >& get_coarses();

private:
    boost::scoped_ptr< class PapiShapeImp > pimpl;

};

#endif // PAPISHAPE_HPP
