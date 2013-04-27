// $Id: camera.cpp 32 2008-10-25 12:19:56Z Naoyuki.Hirayama $

#include "camera.hpp"

/*===========================================================================*/
/*!
 * @class CameraRotater
 * @brief 
 *
 * 
 */
/*==========================================================================*/

class CameraRotator : public IMouseReceiver {
public:
    CameraRotator()
    {
        reset();
    }
    ~CameraRotator(){}

    void reset()
    {
        pan_            = D3DX_PI;
        pan_temporary_  = pan_;
        tilt_           = D3DX_PI/10;
        tilt_temporary_ = tilt_;
    }

    void on_down( const zw::win::offset_type& pos )
    {
        start_ = pos;
    }
        
    void on_drag( const zw::win::offset_type& pos )
    {
        zw::win::offset_type d = pos - start_;
                                        
        const float coefficient_x = 0.01f;
        const float coefficient_y = 0.01f;
                    
        pan_temporary_  = fmodf( pan_ + d.dx() * coefficient_x,
                                 D3DX_PI*2 );
        tilt_temporary_ = float( tilt_ + d.dy() * coefficient_y );

        if( tilt_temporary_ < D3DX_PI * -0.4999f ) {
            tilt_temporary_ = D3DX_PI * -0.4999f;
        }
        if( D3DX_PI * 0.4999f < tilt_temporary_ ){
            tilt_temporary_ = D3DX_PI * 0.4999f;
        }
    }
        
    void on_up( const zw::win::offset_type& )
    {
        pan_  = pan_temporary_;
        tilt_ = tilt_temporary_;
    }

    float get_pan() { return pan_temporary_; }
    float get_tilt() { return tilt_temporary_; }

    void set_pan( float pan ) { pan_ = pan_temporary_ = pan; }
    void set_tilt( float tilt ) { tilt_ = tilt_temporary_ = tilt; }

private:
    HWND                    window_;
    float                   tilt_;
    float                   tilt_temporary_;
    float                   pan_;
    float                   pan_temporary_;

    zw::win::offset_type    start_;
        
};

/*===========================================================================*/
/*!
 * @class CameraZoomer
 * @brief 
 *
 * 
 */
/*==========================================================================*/

class CameraZoomer : public IMouseReceiver {
public:
    CameraZoomer()
    {
        reset();
    }
    ~CameraZoomer(){}

    void reset()
    {
        distance_       = 25.0f;
    }

    void on_down( const zw::win::offset_type& pos )
    {
        previous_ = pos;
    }
        
    void on_drag( const zw::win::offset_type& pos )
    {
        int zdelta = pos.dy() - previous_.dy();

        distance_ += zdelta * 0.05f;
        previous_ = pos;
    }
        
    void on_up( const zw::win::offset_type& )
    {
    }

    float get_distance() { return distance_; }

    void set_distance( float distance ) { distance_ = distance; }

private:
    float                   distance_;
    zw::win::offset_type    previous_;
        
};

/*===========================================================================*/
/*!
 * @class CameraShifter
 * @brief 
 *
 * 
 */
/*==========================================================================*/

class CameraShifter : public IMouseReceiver {
public:
    CameraShifter()
    {
        reset();
    }
    ~CameraShifter(){}

    void reset()
    {
        //rising_       = 0.0f;
        offset_ = D3DXVECTOR3( 0, 0, 0 );
    }

    void set_angle( float pan, float tilt )
    {
        pan_ = pan;
        tilt_ = tilt;
    }

    void on_down( const zw::win::offset_type& pos )
    {
        previous_ = pos;
    }
        
    void on_drag( const zw::win::offset_type& pos )
    {
        int xdelta = pos.dx() - previous_.dx();
        int ydelta = pos.dy() - previous_.dy();

        offset_ += foo( xdelta * 0.05f, ydelta * 0.05f );

        //rising_ += ydelta * 0.05f;
        previous_ = pos;
    }
        
    void on_up( const zw::win::offset_type& )
    {
    }

    //float get_rising() { return rising_; }
    D3DXVECTOR3 get_offset()
    {
        return offset_;
    }

    void set_offset( const D3DXVECTOR3& v )
    {
        offset_ = v;
    }
        
private:
    D3DXVECTOR3 foo( float dx, float dy )
    {
        float pan  = -pan_;
        float tilt = D3DX_PI/2 - tilt_;

        D3DXVECTOR3 eye;
        eye.x = sinf( tilt ) * cosf( pan );
        eye.y = cosf( tilt );
        eye.z = sinf( tilt ) * sinf( pan );
        eye *= -1;

        D3DXVECTOR3 c( 0.0f, 1.0f, 0.0f );
        D3DXVECTOR3 xaxis;
        D3DXVec3Cross( &xaxis, &eye, &c );
        D3DXVECTOR3 yaxis;
        D3DXVec3Cross( &yaxis, &xaxis, &eye );

        D3DXVECTOR3 v;
        *((D3DXVECTOR3*)&v) = xaxis * dx + yaxis * dy;
        return v;
    }

private:
    float                   pan_;
    float                   tilt_;
    //float                   rising_;
    D3DXVECTOR3             offset_;
    zw::win::offset_type    previous_;
        
};

/*============================================================================
 *
 * class Camera 
 *
 * 
 *
 *==========================================================================*/
//<<<<<<<<<< Camera

//****************************************************************
// constructor
Camera::Camera()
{
    rotator_.reset( new CameraRotator );
    zoomer_.reset( new CameraZoomer );
    shifter_.reset( new CameraShifter );
}

//****************************************************************
// reset
void Camera::reset()
{
    rotator_->reset();
    zoomer_->reset();
    shifter_->reset();
}

//****************************************************************
// destructor
Camera::~Camera()
{
}

//****************************************************************
// make_view_point
void Camera::make_view_point( D3DXVECTOR3& view_point )
{
    D3DXMATRIX mat;

    float pan  = -rotator_->get_pan();
    float tilt = D3DX_PI/2 - rotator_->get_tilt();
    float distance = zoomer_->get_distance();

    D3DXVECTOR3 eye;
    eye.x = sinf( tilt ) * cosf( pan );
    eye.y = cosf( tilt );
    eye.z = sinf( tilt ) * sinf( pan );
    eye *= distance;
        
    view_point = eye + shifter_->get_offset();
}

//****************************************************************
// make_focal_point
void Camera::make_focal_point( D3DXVECTOR3& focal_point )
{
    focal_point =  shifter_->get_offset();
}

//****************************************************************
// move_view_point
void Camera::move_view_point( const D3DXVECTOR3& view_point )
{
    D3DXVECTOR3 focal_point = shifter_->get_offset();
    D3DXVECTOR3 v = focal_point - view_point;
    float distance = D3DXVec3Length( &v );
    float pan = atan2( v.z, v.x );
    float tilt = -atan2( v.y, sqrtf( v.x * v.x + v.z * v.z ) );
    rotator_->set_pan( pan );
    rotator_->set_tilt( tilt );
    zoomer_->set_distance( distance );
}

//****************************************************************
// move_focal_point
void Camera::move_focal_point( const D3DXVECTOR3& focal_point )
{
    shifter_->set_offset( focal_point );
}

//----------------------------------------------------------------
// get_offset
zw::win::offset_type    Camera::get_offset()
{
    return zw::win::offset_type( 0, 0 );
}

//----------------------------------------------------------------
// accept_mouse
MouseAcceptInfo Camera::accept_mouse(
    const MouseAcceptInfo::ancestors_type&  ancestors,
    const zw::win::offset_type&             ,
    MouseButtonType                         button )
{
    MouseAcceptInfo info( ancestors, this, NULL );

    if( button == button_left  ) { info.receiver = &*zoomer_; }
    if( button == button_right ) { info.receiver = &*rotator_; }
    if( button == button_middle ) {
        shifter_->set_angle(
            rotator_->get_pan(),
            rotator_->get_tilt() );
        info.receiver = &*shifter_;
    }

    return info;
}

//----------------------------------------------------------------
// on_hover
void Camera::on_hover(  const zw::win::offset_type& )
{
}

//>>>>>>>>>> Camera

