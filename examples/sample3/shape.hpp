/*!
  @file     shape.hpp
  @brief    <ŠT—v>

  <à–¾>
  $Id: shape.hpp 32 2008-10-25 12:19:56Z Naoyuki.Hirayama $
*/
#ifndef SHAPE_HPP
#define SHAPE_HPP

#include "zw/d3d.hpp"
#include "mqoreader.hpp"
#include "texture_cache.hpp"

class Shape {
public:
    Shape(LPDIRECT3DDEVICE9 device);
    ~Shape();

    void clear();
    bool empty();
    void build_from_mqo(
        mqo_reader::document_type&, float scale, DWORD color,
        TextureCache& tc );
    void update( float elapsed );
    void render( LPDIRECT3DDEVICE9 );

    void on_lost_device( LPDIRECT3DDEVICE9 device );
    void on_reset_device( LPDIRECT3DDEVICE9 device );

private:
    boost::scoped_ptr<class ShapeImp>       pimpl;

};

#endif // SHAPE_HPP
