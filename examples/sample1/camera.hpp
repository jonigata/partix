/*!
  @file     camera.hpp
  @brief    <ŠT—v>

  <à–¾>
  $Id: camera.hpp 32 2008-10-25 12:19:56Z Naoyuki.Hirayama $
*/
#ifndef CAMERA_HPP
#define CAMERA_HPP

#include <windows.h>
#include "mouse_acceptor.hpp"

class Camera : public IMouseAcceptor {
public:
    Camera();
    ~Camera();

    void reset();
    void make_view_point( D3DXVECTOR3& view_point ) ; 
    void make_focal_point( D3DXVECTOR3& focal_point ) ; 
    void move_view_point( const D3DXVECTOR3& view_point );
    void move_focal_point( const D3DXVECTOR3& focal_point );

private:
    zw::win::offset_type    get_offset();
        
    MouseAcceptInfo accept_mouse(
        const MouseAcceptInfo::ancestors_type&  ancestors,
        const zw::win::offset_type&             position,
        MouseButtonType                         button );

    void            on_hover(  const zw::win::offset_type& );

private:                
    boost::scoped_ptr<class CameraRotator>  rotator_;
    boost::scoped_ptr<class CameraZoomer>   zoomer_;
    boost::scoped_ptr<class CameraShifter>  shifter_;

};

#endif // CAMERA_HPP
