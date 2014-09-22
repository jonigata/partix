// 2008/11/05 Naoyuki Hirayama

#include "zw/d3dmathutils.hpp"
#include "partix/partix.hpp"
#include "physical_world.hpp"
#include "shape.hpp"
#include "partix_utils.hpp"
#include "draw_physie.hpp"

/*===========================================================================*/
/*!
 * 定数・グローバル変数
 *
 *
 * 
 */
/*==========================================================================*/

const int ENTITY_COUNT = 20;
const float STANDARD_MASS = 0.1f;

/*===========================================================================*/
/*!
 * D3DXVectorTraits
 *
 *  vector traits
 */
/*==========================================================================*/

struct D3DXVectorTraits {
    typedef float           real_type;
    typedef D3DXVECTOR3     vector_type;

    static real_type epsilon(){ return real_type( 0.000001f ); }
    static real_type x( const vector_type& v ) { return v.x; }
    static real_type y( const vector_type& v ) { return v.y; }
    static real_type z( const vector_type& v ) { return v.z; }
    static void x( vector_type& v, real_type x ) { v.x = x; }
    static void y( vector_type& v, real_type y ) { v.y = y; }
    static void z( vector_type& v, real_type z ) { v.z = z; }
    static vector_type make_vector( real_type x, real_type y, real_type z )
    {
        return D3DXVECTOR3( x, y, z );
    }
    static real_type length_sq( const vector_type& v )
    {
        return D3DXVec3LengthSq( &v );
    }
    static real_type length( const vector_type& v )
    {
        return D3DXVec3Length( &v );
    }
};


/*===========================================================================*/
/*!
 * D3DXPartixTraits
 *
 *  partix用のtraitsクラス
 *  vector等はD3DXのものを使用
 */
/*==========================================================================*/

struct D3DXPartixTraits {
    typedef D3DXVectorTraits                vector_traits;
    typedef D3DXVectorTraits::real_type     real_type;
    typedef D3DXVectorTraits::vector_type   vector_type;
    typedef D3DXMATRIX                      matrix_type;
    typedef int                             index_type;

    struct body_load_type {};
    struct block_load_type {};
    struct cloud_load_type {};
    struct point_load_type {};

    static float speed_drag_coefficient() { return  0.0001f; }
    //static float kinetic_friction() { return 200.0f; }
	static float kinetic_friction() { return 0.8f; }

    static float freeze_threshold_energy() { return 2.0f; }
    static float freeze_duration() { return 0.5f; }

    static float tick() { return 0.02f; }
    static void make_matrix(
        matrix_type& d,
        const real_type* s,
        const vector_type& t )
    {
        d._11 = s[0]; d._12 = s[3]; d._13 = s[6]; d._14 = 0;
        d._21 = s[1]; d._22 = s[4]; d._23 = s[7]; d._24 = 0;
        d._31 = s[2]; d._32 = s[5]; d._33 = s[8]; d._34 = 0;
        d._41 = t.x;  d._42 = t.y;  d._43 = t.z;  d._44 = 1;
    }
    static vector_type transform_vector(
        const matrix_type& m,
        const vector_type& v )
    {
        D3DXVECTOR4 t;
        D3DXVec3Transform( &t, &v, &m );
        return (D3DXVECTOR3&)t;
    }
};

struct PartixUser {
    typedef D3DXPartixTraits::vector_type           vector_type;
    typedef partix::World< D3DXPartixTraits >       world_type;
    typedef partix::Point< D3DXPartixTraits >       point_type;
    typedef partix::Cloud< D3DXPartixTraits >       cloud_type;
    typedef partix::Block< D3DXPartixTraits >       block_type;
    typedef partix::Body< D3DXPartixTraits >        body_type;
    typedef partix::SoftShell< D3DXPartixTraits >   softshell_type;
    typedef partix::SoftVolume< D3DXPartixTraits >  softvolume_type;
    typedef partix::TetrahedralMesh< D3DXPartixTraits > tetra_type;
    typedef partix::Face< D3DXPartixTraits >        face_type;

    typedef boost::shared_ptr< body_type >          body_ptr;
    typedef boost::shared_ptr< cloud_type >         cloud_ptr;
    typedef boost::shared_ptr< block_type >         block_ptr;
	typedef boost::shared_ptr< tetra_type >			tetra_ptr;
	typedef boost::shared_ptr< softvolume_type >	softvolume_ptr;
	typedef boost::shared_ptr< softshell_type >		softshell_ptr;
};

/*===========================================================================*/
/*!
 * @class Physie
 * @brief 
 *
 * 
 */
/*==========================================================================*/

struct Physie {
	typedef PartixUser					pu;
	typedef boost::shared_ptr< Shape >	shape_ptr;

	shape_ptr		shape;
	pu::body_ptr	body;

    std::vector< pu::cloud_ptr >	clouds;
    std::vector< pu::block_ptr >    blocks;
    std::vector< pu::tetra_ptr >    tetras;
};

/*===========================================================================*/
/*!
 * @class ConcretePhysicalWorld
 * @brief 
 *
 * 
 */
/*==========================================================================*/
class ConcretePhysicalWorld : public PhysicalWorld {
public:
	typedef PartixUser					pu;
	typedef Physie::shape_ptr			shape_ptr;
	typedef boost::shared_ptr< Physie >	physie_ptr;
	
public:
	ConcretePhysicalWorld()
	{
        prev_eye_ = D3DXVECTOR3( 0, 0, 0 );

        // ...world
        world_.reset( new pu::world_type );

        world_->restart();
	}
	~ConcretePhysicalWorld(){}

	void update( const D3DXVECTOR3& eye )
	{
#if 0
        // 重力方向
        if( eye != prev_eye_ ) {
            D3DXVECTOR3 up( 0, 1.0f, 0 );
            D3DXVECTOR3 cross0 = cross( (D3DXVECTOR3&)eye, up );
            D3DXVECTOR3 cross1 = cross( cross0, (D3DXVECTOR3&)eye );
            normalize_f( cross1 );
            prev_eye_ = eye;

            for( size_t i = 0 ; i < instances_.size() ; i++ ) {
				pu::body_ptr body = instances_[i]->body;
                body->set_frozen( false );
                body->set_global_force( cross1 * -9.8f /* * 2.0f */ );
            }
        }
#endif

		// 落ちすぎたものを削除
		std::vector< physie_ptr >::iterator i = instances_.begin();
		std::vector< physie_ptr >::iterator j = i;
		while( j != instances_.end() ) {
			physie_ptr p = *j;
			if( p->body->get_current_center().y < -100.0f ) {
				world_->remove_body( p->body.get() );
				++j;
				dprintf_real( "erase\n" );
			} else {
				*i = *j; ++i; ++j;
			}
		}
		instances_.erase( i, instances_.end() );

		world_->update( D3DXPartixTraits::tick() );
	}

	void render( LPDIRECT3DDEVICE9 device, const D3DXVECTOR3& eye )
	{
        for( size_t i = 0 ; i < instances_.size() ; i++ ) {
			pu::softvolume_type* volume =
                dynamic_cast< pu::softvolume_type* >(
                    instances_[i]->body.get() );
            if( volume ) {
				D3DXMATRIX m = volume->get_deformed_matrix();
				device->SetTransform(
					D3DTS_WORLD,
					&volume->get_deformed_matrix() );
				//instances_[i]->shape->render( device );
			} else {
				D3DXMATRIX mat_world;
				D3DXMatrixIdentity( &mat_world );
				device->SetTransform( D3DTS_WORLD, &mat_world );
				instances_[i]->shape->render( device );
			}

			if( volume ) {
				draw_physie( device, volume );
			}
        }
	}

    void on_lost_device( LPDIRECT3DDEVICE9 device )
    {
		for( size_t i = 0 ; i < klasses_.size() ; i++ ) {
			klasses_[i]->shape->on_lost_device( device );
		}
    }

    void on_reset_device( LPDIRECT3DDEVICE9 device )
    {
		for( size_t i = 0 ; i < klasses_.size() ; i++ ) {
			klasses_[i]->shape->on_reset_device( device );
		}
    }

	void clear()
	{
		for( std::vector< physie_ptr >::iterator i = instances_.begin();
			 i != instances_.end() ;
			 ++i ) {
			physie_ptr p = *i;
			world_->remove_body( p->body.get() );
		}

		instances_.clear();
	}

	Physie* prepare_softshell(
		LPDIRECT3DDEVICE9 device, const char* filename )
	{
		physie_ptr physie = load_softshell( device, filename );
		klasses_.push_back( physie );
		return physie.get();
	}
	
	Physie* prepare_softvolume(
		LPDIRECT3DDEVICE9 device, const char* filename )
	{
		physie_ptr physie = load_softvolume( device, filename );
		klasses_.push_back( physie );
		return physie.get();
	}

	Physie* create_instance( Physie* physie, const D3DXVECTOR3& pos )
	{
		physie_ptr new_physie( new Physie );
		new_physie->shape = physie->shape;

		pu::softvolume_type* ssv = dynamic_cast< pu::softvolume_type* >(
			physie->body.get() );

		if( ssv ) {
			pu::tetra_type* stt = ssv->get_mesh();
			pu::tetra_ptr dtt( new pu::tetra_type );

			dtt->get_points() = stt->get_points();
			dtt->get_faces() = stt->get_faces();
			dtt->get_tetrahedra() = stt->get_tetrahedra();
			dtt->setup();
			new_physie->tetras.push_back( dtt );
			
			pu::softvolume_ptr dsv( new pu::softvolume_type );
			dsv->set_mesh( dtt.get() );
			//dsv->set_restore_factor( 1.0f );
			//dsv->set_stretch_factor( 0.0f );
			dsv->set_restore_factor( 0.02f );
			dsv->set_stretch_factor( 0.7f );
			dsv->teleport( pos );

			dsv->set_frozen( false );
			//dsv->set_global_force( D3DXVECTOR3( 0, -9.8f /* * 2.0f */, 0 ) );

			new_physie->body = dsv;
		} else {
			pu::softshell_type* sss =
				dynamic_cast< pu::softshell_type* >(
					physie->body.get() );
			assert( sss );
			
			std::map< pu::cloud_type*, pu::cloud_type* > dic;

			pu::softshell_ptr dss( new pu::softshell_type );
			dss->set_features( false, false, true );

			for( size_t i = 0 ; i < sss->get_clouds().size() ; i++ ) {
				pu::cloud_type* sc = sss->get_clouds()[i];
				pu::cloud_ptr dc( new pu::cloud_type );
				dc->get_points() = sc->get_points();
				dss->add_cloud( dc.get() );
				new_physie->clouds.push_back( dc );
				dic[sc] = dc.get();
			}

			for( size_t i = 0 ; i < sss->get_blocks().size() ; i++ ) {
				pu::block_type* sb = sss->get_blocks()[i];
				pu::block_ptr db( new pu::block_type );
				db->set_body( dss.get() );
				db->get_faces() = sb->get_faces();
				db->set_cloud( dic[sb->get_cloud()] );
				db->setup();
				dss->add_block( db.get() );
				new_physie->blocks.push_back( db );
			}

			new_physie->body = dss;
		}

		new_physie->body->regularize();
		world_->add_body( new_physie->body.get() );

		instances_.push_back( new_physie );

		return new_physie.get();
	}

	void set_global_accel( Physie* physie, const D3DXVECTOR3& accel )
	{
		physie->body->set_global_force( accel );
	}

private:
	physie_ptr load_softvolume(
		LPDIRECT3DDEVICE9 device , const char* filename )
	{
		std::string fn( filename );

		std::ifstream ifs( ( fn + ".mqo" ).c_str() );
		mqo_reader::document_type doc;
		mqo_reader::read_mqo( ifs, doc );

		// ...... shape
		shape_ptr shape( new Shape( device ) );
		shape->build_from_mqo( doc, 3.0f, D3DCOLOR_XRGB( 255, 0, 0 ) );
                
		// 作成
		pu::softvolume_ptr sv(
			make_volume_body( ( fn + ".tcf" ).c_str(), shape.get(), 3.0f ) );
		sv->set_auto_freezing( false );

		// 硬さ、摩擦
		//v->set_restore_factor( 1.0f );
		//v->set_stretch_factor( 0.0f );

		pu::tetra_type::points_type& points = sv->get_mesh()->get_points();
		int n = int( points.size() );
		for( int j = 0 ; j < n ; j++ ) {
			pu::point_type& p = points[j];
			//p.friction = 40.0f;
		}

		// 登録
		physie_ptr physie( new Physie );
		physie->shape = shape;
		physie->body = sv;

		return physie;
	}

	physie_ptr load_softshell(
		LPDIRECT3DDEVICE9 device , const char* filename )
	{
		std::ifstream ifs( filename );
		mqo_reader::document_type doc;
		mqo_reader::read_mqo( ifs, doc );

		// ...... shape
		shape_ptr shape( new Shape( device ) );
		shape->build_from_mqo( doc, 1.0f, D3DCOLOR_XRGB( 0, 255, 0 ) );

		// ...... body
		pu::softshell_ptr ss( make_shell_body( doc, "field", 32, 1.0f ) );
		ss->set_features( false, false, true );

		physie_ptr physie( new Physie );
		physie->shape = shape;
		physie->body = ss;

		return physie;
	}

	pu::softvolume_type* make_volume_body(
		const char* filename, Shape* shape, float scale )
    {
		pu::vector_type v0( 0, 0, 0 );

		pu::tetra_ptr t( new pu::tetra_type );

		std::ifstream ifs( filename );
		if( !ifs ) {
			assert(0);
		}

        // .node(頂点座標)読み込み
        {
            int node_count, dummy;
            ifs >> node_count >> dummy >> dummy >> dummy;
            for( int i = 0 ; i < node_count ; i++ ) {
                D3DXVECTOR3 v;
                ifs >> dummy >> v.x >> v.y >> v.z;
                v.x *= scale;
                v.y *= scale;
                v.z *= -scale;
                t->add_point( v, STANDARD_MASS );
            }
        }

        // .ele(tetrahedron)読み込み
        {
            int node_count, dummy;
            ifs >> node_count >> dummy >> dummy;
            for( int i = 0 ; i < node_count ; i++ ) {
                int i0, i1, i2, i3;
                ifs >> dummy >> i0 >> i1 >> i2 >> i3;
                t->add_tetrahedron( i0, i1, i2, i3 );
            }
        }

        // .face(外接面)読み込み
        {
            int node_count, dummy;
            ifs >> node_count >> dummy;
            for( int i = 0 ; i < node_count ; i++ ) {
                int i0, i1, i2;
                ifs >> dummy >> i0 >> i1 >> i2 >> dummy;
                t->add_face( i0, i1, i2 ); // 反転
            }
        }

        t->setup();
		tetras_.push_back( t );

		pu::softvolume_type* v = new pu::softvolume_type;
        v->set_mesh( t.get() );
        v->regularize();

        return v;
    }

	pu::softshell_type* make_shell_body(
        mqo_reader::document_type& doc,
        const char*                objectname,
        int                        threshold,
        float                      scale )
    {
        // MQO version
        mqo_reader::object_type& obj = doc.objects[ objectname ];

		pu::softshell_type* ss = new partix::SoftShell< D3DXPartixTraits >;

        // vertex
		pu::cloud_type* c = new pu::cloud_type;
        {
            for( mqo_reader::vertices_type::iterator i = obj.vertices.begin();
                 i != obj.vertices.end();
                 ++i ) {
                mqo_reader::vertex_type& v = *i;
                c->add_point( D3DXVECTOR3( v.x, v.y, -v.z ) * scale, 0.1f );
            }
        }
        ss->add_cloud( c );
        clouds_.push_back( pu::cloud_ptr( c ) );

        // index
		pu::block_type* b = new partix::Block< D3DXPartixTraits >;
        {
            for( mqo_reader::faces_type::iterator i = obj.faces.begin();
                 i != obj.faces.end();
                 ++i ) {
                mqo_reader::face_type& f = *i;
                int* q = f.vertex_indices;
                if( f.vertex_count == 3 ) {
                    b->add_face( q[0], q[1], q[2] );
                } else if( f.vertex_count == 4 ) {
                    b->add_face( q[0], q[1], q[3] );
                    b->add_face( q[1], q[2], q[3] );
                } else {
                    assert( 0 );
                }
            }
        }

        // 分割
        divide_block( threshold, ss, c, b );

        return ss;
    }

    struct face_compare {
    public:
        face_compare( int a, const pu::cloud_type::points_type& p )
            : axis( a ), points( p ) {}
        bool operator()( const pu::face_type& f0,
                         const pu::face_type& f1 )
        {
            const D3DXVECTOR3& v00 = points[ f0.i0 ].new_position;
            const D3DXVECTOR3& v01 = points[ f0.i1 ].new_position;
            const D3DXVECTOR3& v02 = points[ f0.i2 ].new_position;
            D3DXVECTOR3 c0 = ( v00 + v01 + v02 ) / 3.0f;

            const D3DXVECTOR3& v10 = points[ f1.i0 ].new_position;
            const D3DXVECTOR3& v11 = points[ f1.i1 ].new_position;
            const D3DXVECTOR3& v12 = points[ f1.i2 ].new_position;
            D3DXVECTOR3 c1 = ( v10 + v11 + v12 ) / 3.0f;

            switch( axis ) {
            case 0: return c0.x < c1.x; 
            case 1: return c0.y < c1.y; 
            case 2: return c0.z < c1.z; 
            default: assert(0); return false;
            }
        }

        int									axis;
        const pu::cloud_type::points_type&  points;
    };

    void divide_block(
        int					threshold,
        pu::softshell_type* body,
        pu::cloud_type*		c,
        pu::block_type*		b )
    {
        partix::Block< D3DXPartixTraits >::faces_type& faces =
            b->get_faces();

        int n = int( faces.size() );

        if( n < threshold ) {
            if( b->get_faces().empty() ) {
                delete b;
            } else {
                b->set_cloud( c );
                b->set_body( body ); 
                b->setup();
                body->add_block( b );

				pu::block_ptr bp( b );
                blocks_.push_back( bp );
            }
            return;
        }

		pu::cloud_type::points_type& points = c->get_points();

        // bounding box
        D3DXVECTOR3 bbmin(  FLT_MAX,  FLT_MAX,  FLT_MAX );
        D3DXVECTOR3 bbmax( -FLT_MAX, -FLT_MAX, -FLT_MAX );
        for( int i = 0 ; i < n ; i++ ) {
			pu::face_type& face = faces[i];
            const D3DXVECTOR3& v0 = points[ face.i0 ].new_position;
            const D3DXVECTOR3& v1 = points[ face.i1 ].new_position;
            const D3DXVECTOR3& v2 = points[ face.i2 ].new_position;
            D3DXVECTOR3 center = ( v0 + v1 + v2 ) / 3.0f;
            update_bb( bbmin, bbmax, center );
        }

        // longest axis
        int axis;
        D3DXVECTOR3 bbw = bbmax - bbmin;
        if( bbw.y <= bbw.x && bbw.z <= bbw.x ) {
            axis = 0;
        } else if( bbw.z <= bbw. y ) {
            axis = 1;
        } else {
            axis = 2;
        }
          
        // sort along axis
        std::sort(
			faces.begin(),
			faces.end(),
			face_compare( axis, c->get_points() ) );

        // divide
		pu::block_type* b1 = new pu::block_type;

        for( int i = n/2 ; i < n ; i++ ) {
            b1->add_face( faces[i].i0, faces[i].i1, faces[i].i2 ); 
        }
        faces.erase( faces.begin() + n/2, faces.end() );

        divide_block( threshold, body, c, b );
        divide_block( threshold, body, c, b1 );
    }

    void update_bb(
		D3DXVECTOR3& bbmin,
		D3DXVECTOR3& bbmax,
		const D3DXVECTOR3& v )
    {
        if( v.x < bbmin.x ) { bbmin.x = v.x; }
        if( v.y < bbmin.y ) { bbmin.y = v.y; }
        if( v.z < bbmin.z ) { bbmin.z = v.z; }
        if( bbmax.x < v.x ) { bbmax.x = v.x; }
        if( bbmax.y < v.y ) { bbmax.y = v.y; }
        if( bbmax.z < v.z ) { bbmax.z = v.z; }
    }

private:
    boost::scoped_ptr< pu::world_type > world_;
    std::vector< pu::cloud_ptr >        clouds_;
    std::vector< pu::block_ptr >        blocks_;
    std::vector< pu::tetra_ptr >        tetras_;
    std::vector< boost::shared_ptr< Physie > > klasses_;
    std::vector< boost::shared_ptr< Physie > > instances_;

    D3DXVECTOR3     prev_eye_;

};

PhysicalWorld* CreatePhysicalWorld()
{
	return new ConcretePhysicalWorld;
}

