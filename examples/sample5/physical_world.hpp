// 2008/11/05 Naoyuki Hirayama

/*!
	@file	  physical_world.hpp
	@brief	  <ŠT—v>

	<à–¾>
*/

#ifndef PHYSICAL_WORLD_HPP_
#define PHYSICAL_WORLD_HPP_

struct Physie;

class PhysicalWorld {
public:
	virtual ~PhysicalWorld(){}
	
	virtual void update( const D3DXVECTOR3& eye ) = 0;
	virtual void render( LPDIRECT3DDEVICE9, const D3DXVECTOR3& eye ) = 0;
	virtual void on_lost_device( LPDIRECT3DDEVICE9 ) = 0;
	virtual void on_reset_device( LPDIRECT3DDEVICE9 ) = 0;

	virtual void clear() = 0;
	virtual Physie* prepare_softshell(
		LPDIRECT3DDEVICE9 device, const char* filename ) = 0;
	virtual Physie* prepare_softvolume(
		LPDIRECT3DDEVICE9 device, const char* filename ) = 0;
	virtual Physie* create_instance( Physie*, const D3DXVECTOR3& pos ) = 0;
	virtual void set_global_accel( Physie*, const D3DXVECTOR3& ) = 0;
};

PhysicalWorld* CreatePhysicalWorld();

#endif // PHYSICAL_WORLD_HPP_
