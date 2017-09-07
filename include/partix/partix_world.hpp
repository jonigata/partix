/*!
  @file     partix_world.hpp
  @brief    <概要>

  <説明>
  $Id: partix_world.hpp 254 2007-06-28 06:47:08Z naoyuki $
  120カラム
*/
#ifndef PARTIX_WORLD_HPP
#define PARTIX_WORLD_HPP

#include "partix_forward.hpp"
#include "partix_spatial_hash.hpp"
#include "partix_rayprocessor.hpp"
#include "partix_collidable.hpp"
#include "partix_constraint.hpp"
#include "partix_contact.hpp"
#include "partix_softshell.hpp"
#include "partix_softvolume.hpp"
#include "partix_cloth.hpp"
#include "partix_plane.hpp"
#include "partix_utilities.hpp"
#include "partix_debug.hpp"
#include "performance_counter.hpp"
#include <algorithm>

namespace partix {

//const int SPATIAL_HASH_TABLE_SIZE = 4999;
const int SPATIAL_HASH_TABLE_SIZE = 9997;
const float SPATIAL_HASH_GRID_SIZE = 0.5f;

template < class Traits >
class World {
public:
    typedef typename Traits::vector_traits          vector_traits;
    typedef typename Traits::real_type              real_type;
    typedef typename Traits::vector_type            vector_type;
    typedef typename Traits::matrix_type            matrix_type;
    typedef typename Traits::index_type             index_type;
    typedef Edge< Traits >                          edge_type;
    typedef Face< Traits >                          face_type;
    typedef World< Traits >                         world_type;
    typedef Point< Traits >                         point_type;
    typedef Cloud< Traits >                         cloud_type;
    typedef Block< Traits >                         block_type;
    typedef Body< Traits >                          body_type;
    typedef Shell< Traits >                         shell_type;
    typedef SoftShell< Traits >                     softshell_type;
    typedef SoftVolume< Traits >                    softvolume_type;
    typedef BoundingPlane< Traits >                 plane_type;
    typedef Cloth< Traits >                         cloth_type;
    typedef Collidable< Traits >                    collidable_type;
    typedef Constraint< Traits >                    constraint_type;
    typedef Contact< Traits >                       contact_type;
	typedef TetrahedralMesh< Traits >				tetra_type;
    typedef typename shell_type::clouds_type        clouds_type;
    typedef typename cloud_type::points_type        points_type;
    typedef typename points_type::iterator          points_iterator;
        
    typedef std::vector< body_type* >               bodies_type;
    typedef std::vector< collidable_type* >         collidables_type;
    typedef std::vector< constraint_type >          constraints_type;
    typedef std::vector< contact_type* >            contacts_type;
    typedef aabb_tree< Traits, collidable_type* >   aabb_tree_type;

    typedef RayProcessor< Traits >                  ray_processor_type;
    typedef typename ray_processor_type::triangle_slot triangle_slot;
    typedef typename ray_processor_type::ray_slot      ray_slot;

    typedef typename Traits::vector_traits vt;

    typedef void ( World< Traits >::*collision_resolver_type )(
        collidable_type*, collidable_type* );

    struct nofilter {
    public:
        bool operator()( body_type* ) const { return true; }
    };

public:
    World()
        :
        page_provider_("world contact"),
        contact_pool_( page_provider_, "contact" ),
        ray_processor_( 
            SPATIAL_HASH_GRID_SIZE,
            SPATIAL_HASH_TABLE_SIZE ),
        cloth_point_tetrahedron_spatial_hash_(
            SPATIAL_HASH_GRID_SIZE,
            SPATIAL_HASH_TABLE_SIZE ),
        cloth_spike_face_spatial_hash_(
            SPATIAL_HASH_GRID_SIZE,
            SPATIAL_HASH_TABLE_SIZE ),
        cloth_edge_face_spatial_hash_(
            SPATIAL_HASH_GRID_SIZE,
            SPATIAL_HASH_TABLE_SIZE ),
        point_tetrahedron_spatial_hash_(
            SPATIAL_HASH_GRID_SIZE,
            SPATIAL_HASH_TABLE_SIZE ),
        edge_face_spatial_hash_(
            SPATIAL_HASH_GRID_SIZE,
            SPATIAL_HASH_TABLE_SIZE ),
        penetration_face_spatial_hash_(
            SPATIAL_HASH_GRID_SIZE,
            SPATIAL_HASH_TABLE_SIZE ) {
        body_id_seed_ = 0;
        init();
        dprintf("<color=yellow>world start</color>");
    }
    ~World() {}

    void add_body( body_type* p )
    {
        add_body_internal( p );
    }
    void remove_body( body_type* p )
    {
        remove_body_internal( p );
    }

    void restart()
    {
        restart_internal();
    }

    void update( real_type elapsed )
    {
        update_internal( elapsed );
    }

    void set_global_force( const vector_type& g )
    {
        set_global_force_internal( g );
    }

    body_type* pick( const vector_type& s0, const vector_type& s1 )
    {
        return pick_internal( s0, s1, nofilter(), NULL );
    }
    body_type* pick( const vector_type& s0, const vector_type& s1,
                     real_type& distance )
    {
        return pick_internal( s0, s1, nofilter(), &distance );
    }
    template < class Filter >
    body_type* pick_filter( const vector_type& s0, const vector_type& s1,
                            Filter f )
    {
        return pick_internal( s0, s1, f, NULL );
    }
    template < class Filter >
    body_type* pick_filter( const vector_type& s0, const vector_type& s1,
                            Filter f, real_type& distance )
    {
        return pick_internal( s0, s1, f, &distance );
    }
        
    real_type get_time() { return time_; } 

    void dump()
    {
        for( typename bodies_type::const_iterator i = bodies_.begin() ;
             i != bodies_.end() ;
             ++i ) {
            (*i)->dump();
        }
    }

private:
    void init()
    {
        make_collision_resolver_table();
        time_ = 0;
		previous_idt_ = real_type( 1 ) / Traits::tick();
    }

    void add_body_internal( body_type* p )
    {
        p->set_id( body_id_seed_++ );
        bodies_.push_back( p ); 
    }                

    void remove_body_internal( body_type* p )
    {
        bodies_.erase(
            std::find(
                bodies_.begin(),
                bodies_.end(),
                p ) );
    }                

    void restart_internal()
    { 
        for( typename bodies_type::const_iterator i = bodies_.begin() ;
             i != bodies_.end() ;
             ++i ) {
            (*i)->regularize();
        }
    }

    void update_internal( real_type elapsed )
    {
        assert( epsilon() <= elapsed );
        real_type dt = elapsed;
        real_type idt = real_type(1) / dt;

        time_ += elapsed;

        // WARNING:
        // 以下の処理の順序を決定するにあたって、
        // 以下の条件を守る必要がある
        // ・update_velocity時に、pointが可能な限り
        //   他のオブジェクトに侵入していないこと。
        // 従って、update_velocityの直前にrestore_shapeを
        // もってきたりするのはよくない。

        PerformanceCounter pc( false );
        pc.print( "update0" );

        begin_frame();
        debug_check();
        pc.print( "update1" );

        // 速度・力の計算
		compute_motion( dt * previous_idt_, dt, idt );
        debug_check();
        pc.print( "update2" );
                
        // 形状復元
        match_shape();
        debug_check();
        pc.print( "update4" );

        restore_shape( dt, idt, 0 );
        debug_check();
        pc.print( "update5" );

        update_display_matrix();
        debug_check();
        pc.print( "update6" );

        // 制約の収集
        clear_constraints();
        debug_check();
        pc.print( "update7" );

        collect_constraints();
        debug_check();
        pc.print( "update8" );
                
        // 制約の適用
        apply_constraints();
        debug_check();
        pc.print( "update9" );

        apply_contacts( dt );
        debug_check();
        pc.print( "update10" );

        // フリーズ状態のアップデート
        update_frozen( dt, idt );
        debug_check();
        pc.print( "update11" );
                
        // 実干渉グラフの作成
        make_actual_contact_lists();
        debug_check();
        pc.print( "update12" );

        end_frame();
        debug_check();

		previous_idt_ = idt;
    }

    void set_global_force_internal( const vector_type& g )
    {
        for( typename bodies_type::const_iterator i = bodies_.begin() ;
             i != bodies_.end() ;
             ++i ) {
            body_type* p = (*i);
            p->set_global_force( g );
        }
    }

    template < class InnerTraits >
    class picker {
    public:
        picker( std::vector< Collidable< InnerTraits >* >& c ) : c_( c ) {} 
        void operator()( Collidable< InnerTraits >* y ) const { c_.push_back( y ); }
    private:
        std::vector< Collidable< InnerTraits >* >& c_;
    };

    template < class Filter >
    body_type* pick_internal( const vector_type& s0, const vector_type& s1,
                              Filter filter, real_type* distance )
    {
        collidables_type S;
        for( typename bodies_type::const_iterator i = bodies_.begin() ;
             i != bodies_.end() ;
             ++i ) {
            (*i)->list_collision_units( S );
        }

        collidable_type* found = NULL;

        real_type min_dist = math< Traits >::real_max();
        for( typename collidables_type::const_iterator i = S.begin() ;
             i != S.end() ;
             ++i ) {
            collidable_type* b = *i;
            if( !filter( b->get_body() ) ) { continue; }

            real_type dist;
            if( math< Traits >::test_aabb_segment(
                    b->get_bbmin(), b->get_bbmax(), s0, s1 )  &&
                b->pick( s0, s1, dist ) ) {
                if( dist < min_dist ) {
                    min_dist = dist;
                    found = b;
                }
            }
        }

        if( found ) {
            if( distance ) { *distance = min_dist; }
            return found->get_body();
        }
        return NULL;
    }

private:
    void make_collision_resolver_table()
    {
        int n = BODY_ID_MAX;
        collision_resolver_table_.resize( n * n );
		
		typedef World< Traits > w;
		std::vector< collision_resolver_type >& t = collision_resolver_table_;

        t[0*n+0] = &w::collision_resolver_shell_shell;
        t[0*n+1] = &w::collision_resolver_shell_shell;
        t[0*n+2] = &w::collision_resolver_shell_volume;
        t[0*n+3] = &w::collision_resolver_shell_plane;
        t[0*n+4] = &w::collision_resolver_shell_cloth;
        t[1*n+0] = &w::collision_resolver_shell_shell;
        t[1*n+1] = &w::collision_resolver_shell_shell;
        t[1*n+2] = &w::collision_resolver_shell_volume;
        t[1*n+3] = &w::collision_resolver_shell_plane;
        t[1*n+4] = &w::collision_resolver_shell_cloth;
        t[2*n+0] = &w::collision_resolver_volume_shell;
        t[2*n+1] = &w::collision_resolver_volume_shell;
        t[2*n+2] = &w::collision_resolver_volume_volume;
        t[2*n+3] = &w::collision_resolver_volume_plane;
        t[2*n+4] = &w::collision_resolver_volume_cloth;
        t[3*n+0] = &w::collision_resolver_plane_shell;
        t[3*n+1] = &w::collision_resolver_plane_shell;
        t[3*n+2] = &w::collision_resolver_plane_volume;
        t[3*n+3] = &w::collision_resolver_plane_plane;
        t[3*n+4] = &w::collision_resolver_plane_cloth;
        t[4*n+0] = &w::collision_resolver_cloth_shell;
        t[4*n+1] = &w::collision_resolver_cloth_shell;
        t[4*n+2] = &w::collision_resolver_cloth_volume;
        t[4*n+3] = &w::collision_resolver_cloth_plane;
        t[4*n+4] = &w::collision_resolver_cloth_cloth;
    }

    void begin_frame()
    {
        for( typename bodies_type::const_iterator i = bodies_.begin() ;
             i != bodies_.end() ;
             ++i ) {
            body_type* p = (*i);
            p->begin_frame();
        }
    }
                
    void compute_motion( real_type pdt, real_type dt, real_type idt )
    {
        for( typename bodies_type::const_iterator i = bodies_.begin() ;
             i != bodies_.end() ;
             ++i ) {
            body_type* p = (*i);
            p->compute_motion( pdt, dt, idt );
        }
    }

    void match_shape()
    {
        for( typename bodies_type::const_iterator i = bodies_.begin() ;
             i != bodies_.end() ;
             ++i ) {
            body_type* p = (*i);
            p->match_shape();
        }
    }
                
    void restore_shape( real_type dt, real_type idt, int kmax )
    {
        for( typename bodies_type::const_iterator i = bodies_.begin() ;
             i != bodies_.end() ;
             ++i ) {
            body_type* p = (*i);
            p->restore_shape( dt, idt, kmax );
        }
    }
                
    void update_display_matrix()
    {
        for( typename bodies_type::const_iterator i = bodies_.begin() ;
             i != bodies_.end() ;
             ++i ) {
            body_type* p = (*i);
            p->update_display_matrix();
        }
    }
                
    void clear_constraints()
    {
        constraints_.clear();
        contacts_.clear();
		contact_pool_.clear();
    }

    void collect_constraints()
    {
        for( typename bodies_type::const_iterator i = bodies_.begin() ;
             i != bodies_.end() ;
             ++i ) {
            body_type* p = (*i);
            p->update_boundingbox();
        }

        broad_collision_phase();
    }

    template < class PTraits >
    class broad_collision_collector {
    public:
        broad_collision_collector(
            World< PTraits >* w, Collidable< PTraits >* x )
            : w_( w ), x_( x ) {}
        
        void operator()( Collidable< PTraits >* y ) const
        {
            w_->resolve_collision( x_, y );
        }

    private:
        World< PTraits >*        w_;
        Collidable< PTraits >*   x_;

    };

    void broad_collision_phase()
    {
        PerformanceCounter pc( false );
                
        pc.print( "broad0" );
        // すべてのcollidableをsrc集合(S)に入れる
        // 重複はないと仮定
        collidables_type S;
        for( typename bodies_type::const_iterator i = bodies_.begin() ;
             i != bodies_.end() ;
             ++i ) {
            (*i)->list_collision_units( S );
        }
        pc.print( "broad1" );

        // 規則正しく並んでいるのは一般によくない
        std::random_shuffle( S.begin(), S.end() ); 
        pc.print( "broad2-0" );


        struct bb_getter {
            void operator()(
                collidable_type* c,
                vector_type& bbmin,
                vector_type& bbmax ) const
            {
                bbmin = c->get_bbmin();
                bbmax = c->get_bbmax();
            }
        };

#if 1
        //// AABB version

        // AABB tree作成 および collision graph初期化
        aabbt_.clear();
        pc.print( "broad2-1" );

        for( typename collidables_type::const_iterator i = S.begin() ;
             i != S.end() ;
             ++i ) {
            collidable_type* b = *i;
            aabbt_.insert( b->get_bbmin(), b->get_bbmax(), b );
        }                        
        pc.print( "broad2-2" );
        for( typename collidables_type::const_iterator i = S.begin() ;
             i != S.end() ;
             ++i ) {
            collidable_type* b = *i;
            b->clear_neighbors();
            b->unmark();
        }                        
        pc.print( "broad2-3" );
                
        // AABBでの衝突検出
        //   collision graphを作成する
        for( typename collidables_type::const_iterator i = S.begin() ;
             i != S.end() ;
             ++i ) {
            collidable_type* c = *i;
            body_type* b = c->get_body();
                        
            if( b->get_positive() &&
                ( !b->get_frozen() || b->get_defrosting() ) ) {
                aabbt_.detect(
                    c->get_bbmin(),
                    c->get_bbmax(),
                    broad_collision_collector< Traits >(
                        this, c ) );
            }
            // この時点で b->collision_ は裏表含み
            // inactive vs inactiveはありえないが、
            // それ以外のものはありうる
        }
        pc.print( "broad3" );
#else
        // spatial hash version
        broad_spatial_hash_.clear();
        for( typename collidables_type::const_iterator i = S.begin() ;
             i != S.end() ;
             ++i ) {
            collidable_type* b = *i;
            broad_spatial_hash_.add_collidable( b );
            b->clear_neighbors();
            b->unmark();
        }                        
        pc.print( "broad2" );

        for( typename collidables_type::const_iterator i = S.begin() ;
             i != S.end() ;
             ++i ) {
            collidable_type* b = *i;
            broad_spatial_hash_.apply(
                b, broad_collision_collector< Traits >(
                    this, b ) );
        }                        
        pc.print( "broad3" );
                
#endif

        // enunique
        for( typename collidables_type::const_iterator i = S.begin() ;
             i != S.end() ;
             ++i ) {
            collidable_type* b = *i;

            collidables_type& neighbors = b->get_neighbors();

            std::sort( neighbors.begin(), neighbors.end() );
            neighbors.erase(
                std::unique(
                    neighbors.begin(),
                    neighbors.end() ),
                neighbors.end() );
        }
        pc.print( "broad4" );

        // グラフをpartitioningしてnarrow collision phaseを行う
        collidables_type T;
        collidables_type D;
        for( typename collidables_type::const_iterator i = S.begin() ;
             i != S.end() ;
             ++i ) {
            collidable_type* collidable = *i;
            if( collidable->marked() ) { continue; }

            body_type* body = collidable->get_body();
            if( !body->get_positive() ||
                ( body->get_frozen() &&
                  !body->get_defrosting() ) ) {
                continue;
            }

            T.push_back( collidable );
            collidable->mark();

            while( !T.empty() ) {
                collidable_type* n = T.back();
                T.pop_back();
                D.push_back( n );

                collidables_type& neighbors =
                    n->get_neighbors();
                for( typename collidables_type::const_iterator
                         j = neighbors.begin() ;
                     j != neighbors.end() ;
                     ++j ) {
                    if( (*j)->marked() ) { continue; }

                    T.push_back( *j );
                    (*j)->mark();
                }
            }

            if( 1 < D.size() ) {
                narrow_collision_phase( D );
            }
            D.clear();
        }
        pc.print( "broad5" );

    }

    void narrow_collision_phase( const collidables_type& D )
    {
        PerformanceCounter pc( false );
        ray_processor_.apply( rayprocessor_responder( this ), D );
        pc.print( "narrow0" );
        narrow_collision_phase_volume_cloth( D );
        pc.print( "narrow1" );
        narrow_collision_phase_volume_volume( D );
        pc.print( "narrow2" );
    }

    template < class InnerTraits >
    class cloth_point_tetrahedron_spatial_hash_replier {
    public:
        cloth_point_tetrahedron_spatial_hash_replier( World< InnerTraits >* w )
            : world_( w ) {}

        void operator()( Cloth< InnerTraits >*               vp, index_type vi,
                         int offset, 
                         TetrahedralMesh< InnerTraits >*     tp, index_type ti,
                         const vector_type& bcc ) const
        {
            point_type& p = vp->get_cloud()->get_points()[vi];
            if( offset < 0 ) {
                p.cloth_flags |= 1;
            } else if( 0 < offset ) {
                p.cloth_flags |= 2;
            } else {
                p.collided = true;
            }
        }

    private:
        World< InnerTraits >* world_;
    };

    template < class InnerTraits >
    class cloth_spike_face_spatial_hash_replier {
    public:
        cloth_spike_face_spatial_hash_replier( World< InnerTraits >* w )
            : world_( w ) {}

        void operator()( Cloth< InnerTraits >* ep,           index_type ei,
                         TetrahedralMesh< InnerTraits >* fp, index_type fi,
                         vector_type& uvt ) const
        {
            points_type& epoints    = ep->get_cloud()->get_points();
            point_type& p           = epoints[ei];

            const points_type& fpoints    = fp->get_points();
            const face_type& f = fp->get_faces()[fi];

            const vector_type& v0 = fpoints[f.i0].new_position;
            const vector_type& v1 = fpoints[f.i1].new_position;
            const vector_type& v2 = fpoints[f.i2].new_position;
                        
            vector_type plane_normal;
            plane_normal = math< InnerTraits >::cross( v1 - v0, v2 - v0 );
            math< InnerTraits >::normalize_f( plane_normal );

            world_->add_constraint(
                ep,
                fp->get_volume(),
                &p,
                plane_normal,
                v0 );
        }

    private:
        World< InnerTraits >* world_;
    };

    template < class InnerTraits >
    class cloth_edge_face_spatial_hash_replier {
    public:
        cloth_edge_face_spatial_hash_replier( World< InnerTraits >* w )
            : world_( w ) {}

        void operator()( Cloth< InnerTraits >* sp,           index_type si,
                         TetrahedralMesh< InnerTraits >* fp, index_type fi,
                         vector_type& uvt ) const
        {
            points_type& points = sp->get_cloud()->get_points();
            const typename cloth_type::spring_type& s = sp->get_springs()[si];

            int index = 0;
            if( points[s.indices.i0].collided ) { index = s.indices.i0; }
            if( points[s.indices.i1].collided ) { index = s.indices.i1; }

            const points_type& fpoints    = fp->get_points();
            const face_type& f = fp->get_faces()[fi];

            const vector_type& v0 = fpoints[f.i0].new_position;
            const vector_type& v1 = fpoints[f.i1].new_position;
            const vector_type& v2 = fpoints[f.i2].new_position;
                        
            vector_type plane_normal;
            plane_normal = math< InnerTraits >::cross( v1 - v0, v2 - v0 );
            math< InnerTraits >::normalize_f( plane_normal );

            world_->add_constraint(
                sp,
                fp->get_volume(),
                &points[index],
                plane_normal,
                v0 );
        }

    private:
        World< InnerTraits >* world_;
    };

    void narrow_collision_phase_volume_cloth( const collidables_type& D )
    {
        real_type edge_ave = 0;

        // 0th stage: volumeとclothを分離
        std::vector< softvolume_type* > volumes;
        std::vector< cloth_type* >      clothes;

        bool have_attacker = false;

        size_t n = D.size(); 
        for( size_t i = 0 ; i < n ; i++ ) {
            softvolume_type* volume = cast_to_softvolume(D[i]);
            if( volume ) {
                if( volume->get_positive() ) {
                    have_attacker = true;
                } else {
                    // positiveなclothがneighborにいなければ受理しない
                    bool accept = false;
                    collidables_type& neighbors = volume->get_neighbors();
                    for( typename collidables_type::const_iterator j =
                             neighbors.begin() ;
                         j != neighbors.end() ;
                         ++j ) {
                        if( cast_to_cloth(*j) &&
                            (*j)->get_body()->get_positive() ) {
                            accept = true;
                        }
                    }                                        
                    if( !accept ) { continue; }
                }
                volumes.push_back( volume );
                edge_ave += volume->get_mesh()->get_average_edge_length();
            }
            cloth_type* cloth = cast_to_cloth( D[i] );
            if( cloth ) {
                if( cloth->get_positive() ) {
                    have_attacker = true;
                } else {
                    // positiveなsoftvolumeがneighborにいなければ受理しない
                    bool accept = false;
                    collidables_type& neighbors = cloth->get_neighbors();
                    for( typename collidables_type::const_iterator j =
                             neighbors.begin() ;
                         j != neighbors.end() ;
                         ++j ) {
                        if( cast_to_softvolume(*j) &&
                            (*j)->get_body()->get_positive() ) {
                            accept = true;
                        }
                    }                                        
                    if( !accept ) { continue; }
                }
                cloth->clear_penetration_vector();
                clothes.push_back( cloth );
                edge_ave += cloth->get_average_edge_length();
            }
        }

        if( !have_attacker ) { return; }
        if( clothes.empty() || volumes.empty() ) { return; }

        // ...spatial hash 初期化
        edge_ave /= real_type( clothes.size() + volumes.size() );
        cloth_point_tetrahedron_spatial_hash_.clear( edge_ave );
        cloth_spike_face_spatial_hash_.clear( edge_ave );
        cloth_edge_face_spatial_hash_.clear( edge_ave );

        // 1st stage: point - tetrahedron intersection

        // ..activeの挿入
        n = clothes.size();
        for( size_t i = 0 ; i < n ; i++ ) {
            cloth_type* cloth = clothes[i];
            cloth_point_tetrahedron_spatial_hash_.add_cloth( cloth );
        } 

        // ..passiveの挿入
        n = volumes.size();
        for( size_t i = 0 ; i < n ; i++ ) {
            softvolume_type* volume = volumes[i];
            cloth_point_tetrahedron_spatial_hash_.add_mesh( volume->get_mesh() );
        }

        // ..match実行
        cloth_point_tetrahedron_spatial_hash_.apply(
            cloth_point_tetrahedron_spatial_hash_replier< Traits >( this ) );

        // 2nd stage: spike - face intersection
        // ..activeの挿入
        n = clothes.size();
        for( size_t i = 0 ; i < n ; i++ ) {
            cloth_type* cloth = clothes[i];
            cloth->mark_spikes( cloth_spike_face_spatial_hash_ );
        } 

        // ..passiveの挿入
        n = volumes.size();
        for( size_t i = 0 ; i < n ; i++ ) {
            tetra_type* mesh = volumes[i]->get_mesh();
                        
            int m = int( mesh->get_faces().size() );
            for( int j = 0 ; j < m ; j++ ) {
                cloth_spike_face_spatial_hash_.add_face( mesh, j );
            }
        }

        // ..match実行
        cloth_spike_face_spatial_hash_.apply(
            cloth_spike_face_spatial_hash_replier< Traits >( this ) );

        // 3rd stage: edge - face intersection
        n = clothes.size();
        for( size_t i = 0 ; i < n ; i++ ) {
            cloth_type* cloth = clothes[i];
            cloth->mark_border_edges( cloth_edge_face_spatial_hash_ );
        } 

        // ..passiveの挿入
        n = volumes.size();
        for( size_t i = 0 ; i < n ; i++ ) {
            tetra_type* mesh = volumes[i]->get_mesh();
                        
            int m = int( mesh->get_faces().size() );
            for( int j = 0 ; j < m ; j++ ) {
                cloth_edge_face_spatial_hash_.add_face( mesh, j );
            }
        }

        // ..match実行
        cloth_edge_face_spatial_hash_.apply(
            cloth_edge_face_spatial_hash_replier< Traits >( this ) );
    }

    class rayprocessor_responder {
    public:
        rayprocessor_responder( World< Traits >* world )
            : world_( world ) {}

        void operator()( typename World< Traits >::ray_slot* rs ) const
        {
            world_->resolve_raytest( rs );
        }

    private:
        World< Traits >* world_;

    };

    template < class PTraits >
    class point_tetrahedron_spatial_hash_replier {
    public:
        typedef TetrahedralMesh< PTraits > mesh_type;
                
    public:
        point_tetrahedron_spatial_hash_replier( World< PTraits >* w )
            : world_( w ) {}

        void operator()( mesh_type* vp, index_type vi,
                         mesh_type* tp, index_type ti ) const
        {
            vp->get_points()[vi].collided = true;
        }

    private:
        World< PTraits >* world_;
    };

    template < class PTraits >
    class edge_face_spatial_hash_replier {
    public:
        typedef TetrahedralMesh< PTraits > mesh_type;
                
    public:
        edge_face_spatial_hash_replier( World< PTraits >* w )
            : world_( w ) {}

        void operator()( mesh_type* ep, index_type ei,
                         mesh_type* fp, index_type fi,
                         vector_type& uvt ) const
        {
            points_type& epoints = ep->get_points();
            typename mesh_type::edge_type& e = ep->get_edges()[ei];

            real_type t = real_type( 1.0 ) - vt::z(uvt);
            if( e.t < t ) { return; }

            e.u = vt::x(uvt);
            e.v = vt::y(uvt);
            e.w = real_type( 1.0 ) - e.u - e.v;
            e.t = t;

            points_type& fpoints = fp->get_points();
            face_type& f = fp->get_faces()[fi];
#if 1
			// smoothed collision normal
            e.collision_normal = math< PTraits >::normalize(
                fpoints[f.i0].normal * e.w +
                fpoints[f.i1].normal * e.u +
                fpoints[f.i2].normal * e.v );
#else
			// raw collision normal
			e.collision_normal = math< PTraits >::normalize(
				math< PTraits >::cross(
					fpoints[f.i1].new_position - fpoints[f.i0].new_position,
					fpoints[f.i2].new_position - fpoints[f.i0].new_position )
				);
#endif
        }

    private:
        World< PTraits >* world_;
    };


    template < class PTraits >
    class penetration_face_spatial_hash_replier {
    public:
        typedef TetrahedralMesh< PTraits > mesh_type;
                
    public:
        penetration_face_spatial_hash_replier( World< PTraits >* w )
            : world_( w ) {}

        void operator()( mesh_type* pp, index_type pi,
                         mesh_type* fp, index_type fi,
                         vector_type& uvt ) const
        {
            points_type& ppoints = pp->get_points();
            point_type& p = ppoints[pi];
            points_type& fpoints = fp->get_points();
            face_type& f = fp->get_faces()[fi];

            world_->add_contact(
                pp->get_volume(),
                fp->get_volume(),
                &p,
                &fpoints[f.i0],
                &fpoints[f.i1],
                &fpoints[f.i2],
                vt::x(uvt),
                vt::y(uvt),
                real_type( 1.0 ) - vt::x(uvt) - vt::y(uvt),
                vt::z(uvt) );
        }

    private:
        World< PTraits >* world_;

    };

    void narrow_collision_phase_volume_volume( const collidables_type& D2 )
    {
        PerformanceCounter pc( false );

        pc.print( "narrow0" );

        // 0th stage: volume以外を取り除く
        //            ついでにエッジの長さの平均も得る
        bool have_attacker = false;

        std::vector< softvolume_type* > D;
        int n = int( D2.size() ); 
        real_type edge_ave = 0;
        for( int i = 0 ; i < n ; i++ ) {
            softvolume_type* volume = cast_to_softvolume( D2[i] );
            if( !volume ) { continue; }
            
            if( volume->get_positive() ) {
                have_attacker = true;
            } else {
                // positiveなsoftvolumeがneighborにいなければ受理しない
                bool accept = false;
                collidables_type& neighbors = volume->get_neighbors();
                for( typename collidables_type::const_iterator j =
                         neighbors.begin() ;
                     j != neighbors.end() ;
                     ++j ) {
                    if( cast_to_softvolume(*j) &&
                        (*j)->get_body()->get_positive() ) {
                        accept = true;
                    }
                }                                        
                if( !accept ) { continue; }
            }
            D.push_back( volume );
            edge_ave += volume->get_mesh()
                ->get_average_edge_length();
        }
        if( !have_attacker ) { return; }

        edge_ave /= real_type( D.size() );
        pc.print( "narrow1" );

        n = int( D.size() );
        if( n < 2 ) { return; }

        point_tetrahedron_spatial_hash_.clear( edge_ave );
        edge_face_spatial_hash_.clear( edge_ave );
        penetration_face_spatial_hash_.clear( edge_ave );
        pc.print( "narrow2" );

        // 1st stage: point - tetrahedron intersection
        // ..activeの挿入
        for( int i = 0 ; i < n ; i++ ) {
            softvolume_type* volume = D[i];
            point_tetrahedron_spatial_hash_.add_active_mesh(
                volume->get_mesh() );
        } 
        pc.print( "narrow3" );

        // ..passiveの挿入
        for( int i = 0 ; i < n ; i++ ) {
            softvolume_type* volume = D[i];
            point_tetrahedron_spatial_hash_.add_passive_mesh(
                volume->get_mesh() );
        }
        pc.print( "narrow4" );

        // ..match実行
        point_tetrahedron_spatial_hash_.apply(
            point_tetrahedron_spatial_hash_replier< Traits >(
                this ) );
        pc.print( "narrow5" );

        // 2nd stage: edge - face intersection
        // ..activeの挿入
        for( int i = 0 ; i < n ; i++ ) {
            softvolume_type* volume = D[i];
            volume->mark_border_edges( edge_face_spatial_hash_ );
        } 
        pc.print( "narrow6" );

        // ..passiveの挿入
        for( int i = 0 ; i < n ; i++ ) {
            softvolume_type* volume = D[i];

            typename softvolume_type::mesh_type::faces_type& faces
                = volume->get_mesh()->get_faces();

            int m = int( faces.size() );
            for( int j = 0 ; j < m ; j++ ) {
                edge_face_spatial_hash_.add_face(
                    volume->get_mesh(), j );
            }
        } 
        pc.print( "narrow7" );

        // ..match実行
        edge_face_spatial_hash_.apply(
            edge_face_spatial_hash_replier< Traits >( this ) );
        pc.print( "narrow8" );

        // 3rd stage: calculate penetration vector
        for( int i = 0 ; i < n ; i++ ) {
            softvolume_type* volume = D[i];
            volume->calculate_penetration_direction();
        } 
        pc.print( "narrow9" );

        // 4th stage: propagation
        for( int i = 0 ; i < n ; i++ ) {
            softvolume_type* volume = D[i];
            volume->propagate_penetration();
        } 
        pc.print( "narrow10" );

        // additional stage: penetration vector / face テストを行い、
        //   Contactを作成

        // ..active挿入
        for( int i = 0 ; i < n ; i++ ) {
            softvolume_type* volume = D[i];

            typename tetra_type::points_type& points
                = volume->get_mesh()->get_points();

            // penetration vectorの登録
            int m = int( points.size() );
            for( int j = 0 ; j < m ; j++ ) {
                typename tetra_type::point_type& p = points[j];
                if( !p.collided ) { continue; }
                if( p.penetration_denominator < epsilon() ) { continue; }

                penetration_face_spatial_hash_.add_penetration(
                    volume->get_mesh(), j );
            }

			//dprintf_real("\n" );
        }
        pc.print( "narrow11" );

        // ...passive挿入
        for( int i = 0 ; i < n ; i++ ) {
            softvolume_type* volume = D[i];

            // faceの登録
            typename softvolume_type::mesh_type::faces_type& faces
                = volume->get_mesh()->get_faces();
            int m = int( faces.size() );
            for( int j = 0 ; j < m ; j++ ) {
                penetration_face_spatial_hash_.add_face(
                    volume->get_mesh(), j );
            }
        }
        pc.print( "narrow12" );

        // ..spatial hash 適用
        penetration_face_spatial_hash_.apply(
            penetration_face_spatial_hash_replier< Traits >(
                this ) );
        pc.print( "narrow13" );

#if 0
        {
            char buffer[256];
            sprintf( buffer, "constraint: %d\n",
                     constraints_.size() );
            OutputDebugStringA( buffer );
        }
#endif
    }

    void apply_constraints()
    {
        const real_type c1 = real_type( 1.0 );
        const vector_type& v0 = math< Traits >::vector_zero();

		// 初期化
        for( typename constraints_type::iterator i =
                 constraints_.begin() ;
             i != constraints_.end() ;
             ++i ) {
            constraint_type& c = *i;
			c.point->tmp_velocity =
				c.point->new_position -
				c.point->old_position;
			c.point->friction_vector = v0;
		}

		// 摩擦力の計算
        for( typename constraints_type::iterator i =
                 constraints_.begin() ;
             i != constraints_.end() ;
             ++i ) {
            constraint_type& c = *i;

			// 摩擦
			const vector_type& n = c.plane_normal; // normalized
            real_type npdotn = dot(
				c.point->new_position - c.plane_position, n );
            if( 0 <= npdotn ) { continue; }
            vector_type u = -c.point->tmp_velocity;
			real_type udotn = dot( u, n );
			vector_type un = n * udotn;	// normal velocity
			vector_type ut = u - un;	// tangencial velocity
			real_type utlen = length( ut );
			if( utlen < epsilon() ) { continue; }
			real_type unlen = std::abs( udotn );

			real_type mu = c.point->friction;
			real_type friction = unlen * mu;
			if( utlen < friction ) {
				friction = utlen;
			}

			c.point->friction_vector += ut * ( friction / utlen );
		}		

        // 制約
        for( typename constraints_type::iterator i =
                 constraints_.begin() ;
             i != constraints_.end() ;
             ++i ) {
            constraint_type& c = *i;

			c.point->new_position += c.point->friction_vector;

			// ペネトレーション
			const vector_type& n = c.plane_normal;
            real_type npdotn = dot(
				c.point->new_position - c.plane_position, n );
            if( 0 <= npdotn ) { continue; }

            real_type nlen = -npdotn;
            vector_type penetration = n * nlen;

			c.point->new_position += penetration;
			c.point->constraint_pushout += penetration;

            c.point->check();
        }
    }

	struct contact_remover {
		bool operator()( contact_type* c ) const
		{
			// 最近だけ残す
			return c->A_point->penetration_magnifier < c->t; 
		}
	};

    void apply_contacts( real_type dt )
    {
        const real_type c1 = real_type( 1.0 );
        const vector_type& zero = math< Traits >::vector_zero();

		// A_pointを共有するcontactを削除する(一つにする)
		contacts_.erase( 
			std::remove_if(
				contacts_.begin(),
				contacts_.end(),
				contact_remover() ),
			contacts_.end() );

        // 接触
        // ..初期化
        for( typename contacts_type::iterator i = contacts_.begin() ;
             i != contacts_.end() ;
             ++i ) {
            contact_type& c = **i;
            c.check();

            // active point
            c.A_point->contact = c1;
            c.A_point->process_flag = false;
			c.A_point->friction_vector = zero;
			c.A_point->view_vector1 = c.A_point->new_position;

            // passive points
            c.B_point0->contact = c1;
            c.B_point0->process_flag = false;
			c.B_point0->friction_vector = zero;

            c.B_point1->contact = c1;
            c.B_point1->process_flag = false;
			c.B_point1->friction_vector = zero;

            c.B_point2->contact = c1;
            c.B_point2->process_flag = false;
			c.B_point2->friction_vector = zero;

#if 0
			c.A_point->tmp_velocity =
				c.A_point->new_position -
				c.A_point->old_position;
			c.B_point0->tmp_velocity =
				c.B_point0->new_position -
				c.B_point0->old_position;
			c.B_point1->tmp_velocity =
				c.B_point1->new_position -
				c.B_point1->old_position;
			c.B_point2->tmp_velocity =
				c.B_point2->new_position -
				c.B_point2->old_position;
#endif
        }

        // ..c定数(の分母)の収集
        for( typename contacts_type::iterator i = contacts_.begin() ;
             i != contacts_.end() ;
             ++i ) {
            contact_type& c = **i;

            c.B_point0->contact += c.w;
            c.B_point1->contact += c.u;
            c.B_point2->contact += c.v;
        }

        // ..active apply
        for( typename contacts_type::iterator i = contacts_.begin() ;
             i != contacts_.end() ;
             ++i ) {
            contact_type& c = **i;
			
            // 押し出し
            // α_i
            c.alpha =
                c.w * c.B_point0->mass / c.B_point0->contact +
                c.u * c.B_point1->mass / c.B_point1->contact +
                c.v * c.B_point2->mass / c.B_point2->contact;
            c.alpha /= ( c.A_point->mass / c.A_point->contact + c.alpha );
            c.check();
                        
			// Δt^2/miFi = Δt^2/mi * mi/Δt^2 * pv * αなので
			// pv * α
            vector_type pushout =
				( c.A_point->penetration_vector ) *
				c.alpha;

			c.A_point->active_contact_pushout = pushout;
			c.A_point->check();
        }

#if 1
        // ..passive apply
        for( typename contacts_type::iterator i = contacts_.begin() ;
             i != contacts_.end() ;
             ++i ) {
            contact_type& c = **i;

            vector_type fi_dt =
				c.A_point->active_contact_pushout *
				c.A_point->mass;

			vector_type pushout0 = fi_dt * ( -c.B_point0->invmass * c.w );
			vector_type pushout1 = fi_dt * ( -c.B_point1->invmass * c.u );
			vector_type pushout2 = fi_dt * ( -c.B_point2->invmass * c.v );
			c.B_point0->passive_contact_pushout += pushout0;
			c.B_point1->passive_contact_pushout += pushout1;
			c.B_point2->passive_contact_pushout += pushout2;
        }
#endif

		// pushout適用
        for( typename contacts_type::iterator i = contacts_.begin() ;
             i != contacts_.end() ;
             ++i ) {
            contact_type& c = **i;

			vector_type v = c.A_point->active_contact_pushout;
			if( length_sq( c.A_point->active_contact_pushout ) <
				length_sq( c.A_point->passive_contact_pushout ) ) {
				v = c.A_point->passive_contact_pushout;
				c.A_point->active_contact_pushout = zero;
			} else {
				c.A_point->passive_contact_pushout = zero;
			}

			c.A_point->new_position += v;
			c.A_point->tmp_velocity =
				c.A_point->new_position -
				c.A_point->old_position;
			c.A_point->process_flag = true;
		}

#if 1
        for( typename contacts_type::iterator i = contacts_.begin() ;
             i != contacts_.end() ;
             ++i ) {
            contact_type& c = **i;
			if( !c.B_point0->process_flag ) {
				c.B_point0->new_position +=
					c.B_point0->passive_contact_pushout;
				c.B_point0->process_flag = true;
			}
			if( !c.B_point1->process_flag ) {
				c.B_point1->new_position +=
					c.B_point1->passive_contact_pushout;
				c.B_point1->process_flag = true;
			}
			if( !c.B_point2->process_flag ) {
				c.B_point2->new_position +=
					c.B_point2->passive_contact_pushout;
				c.B_point2->process_flag = true;
			}
		}
#endif

#if 0
        // 誤差修正
        for( typename contacts_type::iterator i = contacts_.begin() ;
             i != contacts_.end() ;
             ++i ) {
            contact_type& c = **i;

			c.A_point->process_flag = false;
			c.A_point->penetration_error =
				c.A_point->penetration_vector * real_type( 3 )+
				( c.B_point0->penetration_vector +
				  c.B_point1->penetration_vector +
				  c.B_point2->penetration_vector );
			c.A_point->check();
        }

        for( typename contacts_type::iterator i = contacts_.begin() ;
             i != contacts_.end() ;
             ++i ) {
            contact_type& c = **i;
			if( c.A_point->process_flag ) { continue; }
			c.A_point->process_flag = true;

			c.A_point->penetration_error /= 3.0f;
			c.A_point->new_position -= c.A_point->penetration_error;
			c.A_point->active_contact_pushout -= c.A_point->penetration_error;
		}
#endif
		
#if 1
        // 摩擦計算
        for( typename contacts_type::iterator i = contacts_.begin() ;
             i != contacts_.end() ;
             ++i ) {
            contact_type& c = **i;
			c.A_point->process_flag = false;

#if 0
			if( length( c.A_point->active_contact_pushout ) <
				length( c.A_point->passive_contact_pushout ) ) {
				dprintf_real( "p" );
			} else {
				dprintf_real( "a" );
			}
#endif

			vector_type nsrc = c.A_point->active_contact_pushout;
			// passive_contact_pushoutは無視してよい
			real_type nlen = length( nsrc );
			if( nlen < epsilon() ) { continue; }

			vector_type n = nsrc * ( real_type(1) / nlen );
			
			vector_type va = c.A_point->tmp_velocity;

            vector_type vb =
				c.B_point0->tmp_velocity * c.w +
				c.B_point1->tmp_velocity * c.u +
				c.B_point2->tmp_velocity * c.v;

			real_type mu = c.A_point->friction;

			vector_type u = ( vb - va ) * c.alpha;
			vector_type un = n * dot( n, u );
			vector_type ut = u - un; // tangential velocity

			real_type friction = nlen * mu;
			real_type max_friction = length( ut );
			if( max_friction < friction ) { friction = max_friction; }

			c.A_point->friction_vector =
				ut * friction * ( real_type(1) / max_friction );
            c.A_point->check();
        }
#endif
		//dprintf_real( "\n" );

		// 摩擦適用
		vector_type total_friction = zero;
        for( typename contacts_type::iterator i = contacts_.begin() ;
             i != contacts_.end() ;
             ++i ) {
            contact_type& c = **i;
			if( c.A_point->process_flag ) { continue; }
			c.A_point->process_flag = true;

			c.A_point->new_position += c.A_point->friction_vector;
			total_friction += c.A_point->friction_vector;
		}

#if 0
		// 対応チェック
		std::map< int, std::vector< int > > m;
        for( typename contacts_type::iterator i = contacts_.begin() ;
             i != contacts_.end() ;
             ++i ) {
            contact_type& c = **i;
            c.check();
			m[c.A_point->id].push_back( c.B_point0->id );
			m[c.A_point->id].push_back( c.B_point1->id );
			m[c.A_point->id].push_back( c.B_point2->id );
		}

		for( std::map< int, std::vector< int > >::const_iterator i =
				 m.begin() ;
			 i != m.end() ;
			 ++i ) {

			dprintf_real( "m: %d: ", (*i).first );
			const std::vector< int >& v = (*i).second;
			for( std::vector< int >::const_iterator j = v.begin() ;
				 j != v.end() ;
				 ++j ) {
				dprintf_real( "%d, ", (*j) );
			}
			dprintf_real( "\n" );
		}
		dprintf_real( "\n" );
#endif

    }

    void update_frozen( real_type dt, real_type idt )
    {
        for( typename bodies_type::const_iterator i = bodies_.begin() ;
             i != bodies_.end() ;
             ++i ) {
            body_type* p = (*i);
            p->update_frozen( dt, idt );
        }
    }

    void make_actual_contact_lists()
    {
        for( typename bodies_type::const_iterator i = bodies_.begin() ;
             i != bodies_.end() ;
             ++i ) {
            body_type* p = (*i);
            p->get_actual_contact_list().clear();
        }

        for( typename constraints_type::iterator i =
                 constraints_.begin() ;
             i != constraints_.end() ;
             ++i ) {
            constraint_type& c = *i;

            c.A_body->get_actual_contact_list().push_back( c.B_body );
            c.B_body->get_actual_contact_list().push_back( c.A_body );
            if( c.B_body->get_alive() &&
                !c.B_body->get_frozen() ) {
                c.A_body->set_defrosting( true );
            }
            if( c.A_body->get_alive() &&
                !c.A_body->get_frozen() ) {
                c.B_body->set_defrosting( true );
            }
        }                

        for( typename contacts_type::iterator i = contacts_.begin() ;
             i != contacts_.end() ;
             ++i ) {
            contact_type& c = **i;

            c.A_body->get_actual_contact_list().push_back( c.B_body );
            c.B_body->get_actual_contact_list().push_back( c.A_body );
            if( c.B_body->get_alive() &&
                !c.B_body->get_frozen() ) {
                c.A_body->set_defrosting( true );
            }
            if( c.A_body->get_alive() &&
                !c.A_body->get_frozen() ) {
                c.B_body->set_defrosting( true );
            }
        }                

        for( typename bodies_type::const_iterator i = bodies_.begin() ;
             i != bodies_.end() ;
             ++i ) {
            body_type* p = (*i);
            std::sort( 
                p->get_actual_contact_list().begin(),
                p->get_actual_contact_list().end() );
            p->get_actual_contact_list().erase(
                std::unique(
                    p->get_actual_contact_list().begin(),
                    p->get_actual_contact_list().end() ),
                p->get_actual_contact_list().end() );
        }
    }

    void end_frame()
    {
        for( typename bodies_type::const_iterator i = bodies_.begin() ;
             i != bodies_.end() ;
             ++i ) {
            body_type* p = (*i);
            p->update_boundingbox();
            p->end_frame();
        }
    }

    void debug_check()
    {
#if 0
        for( typename bodies_type::const_iterator i = bodies_.begin() ;
             i != bodies_.end() ;
             ++i ) {
            body_type* p = (*i);
            p->debug_check();
        }
#endif
    }

private:
    //// callbacks

    // collision resolvers
    void collision_resolver_shell_shell(
        collidable_type* x,
        collidable_type* y )
    {
        x->add_neighbor( y );
        y->add_neighbor( x );
    }

    void collision_resolver_shell_volume(
        collidable_type* x,
        collidable_type* y )
    {
        x->add_neighbor( y );
        y->add_neighbor( x );
    }

    void collision_resolver_shell_plane(
        collidable_type* xx,
        collidable_type* yy )
    {
        // constraintを作成する
        block_type* x = static_cast< block_type* >( xx );
        plane_type* y = static_cast< plane_type* >( yy );

        if( !math< Traits >::test_aabb_plane( x->get_bbmin(),
                                              x->get_bbmax(),
                                              y->get_position(),
                                              y->get_normal() ) ) {
            return;
        }

        real_type ndotn = dot( y->normal_, y->normal_ );
        real_type rndotn = real_type( 1.0 ) / ndotn;

        // TODO: cloudが共有されているときに複数回行うことになる！！！
        cloud_type* cloud = x->get_cloud();
                        
        // foreach cloud.point
        for( points_iterator j = cloud->get_points().begin() ;
             j != cloud->get_points().end() ;
             ++j ) {
            point_type& point = *j;
            const vector_type& np = point.new_position;

            real_type pdotn = dot( np - y->position_, y->normal_ );

            if( 0 < pdotn ) { continue; }
                                
            Constraint< Traits > c;
            c.A_body        = x->get_body();
            c.B_body        = y;
            c.point         = &point;
            c.plane_normal  = y->normal_;
            c.plane_position = y->position_;
                        
            constraints_.push_back( c );
        }
    }

    void collision_resolver_shell_cloth(
        collidable_type* x,
        collidable_type* y )
    {
    }

    void collision_resolver_volume_shell(
        collidable_type* x,
        collidable_type* y )
    {
        x->add_neighbor( y );
        y->add_neighbor( x );
    }

    void collision_resolver_volume_volume(
        collidable_type* x,
        collidable_type* y )
    {
        x->add_neighbor( y );
        y->add_neighbor( x );
    }

    void collision_resolver_volume_plane(
        collidable_type* xx,
        collidable_type* yy )
    {
        // constraintを作成する
        softvolume_type* x = static_cast< softvolume_type* >( xx );
        plane_type* y = static_cast< plane_type* >( yy );

        if( !math< Traits >::test_aabb_plane( x->get_bbmin(),
                                              x->get_bbmax(),
                                              y->get_position(),
                                              y->get_normal() ) ) {
            return;
        }

        real_type ndotn = dot( y->normal_, y->normal_ );
        real_type rndotn = real_type( 1.0 ) / ndotn;

        typedef typename softvolume_type::points_type points_type;

        points_type& points = x->get_mesh()->get_points();
                
        // foreach cloud.point
        for( typename points_type::iterator j = points.begin() ;
             j !=points.end() ;
             ++j ) {
            point_type& p = *j;

            real_type pdotn = dot(
                p.new_position - y->position_, y->normal_ );
            if( 0 < pdotn ) { continue; }
                                
            Constraint< Traits > c;
            c.A_body         = x->get_body();
            c.B_body         = y;
            c.point          = &p;
            c.plane_normal   = y->normal_;
            c.plane_position = y->position_;

            constraints_.push_back( c );
        }
    }

    void collision_resolver_volume_cloth(
        collidable_type* x,
        collidable_type* y )
    {
        x->add_neighbor( y );
        y->add_neighbor( x );
    }

    void collision_resolver_plane_shell(
        collidable_type* x,
        collidable_type* y )
    {
        // 何もしない
    }

    void collision_resolver_plane_volume(
        collidable_type* x,
        collidable_type* y )
    {
        // 何もしない
    }

    void collision_resolver_plane_plane(
        collidable_type* x,
        collidable_type* y )
    {
        // 何もしない
    }

    void collision_resolver_plane_cloth(
        collidable_type* x,
        collidable_type* y )
    {
    }

    void collision_resolver_cloth_shell(
        collidable_type* x,
        collidable_type* y )
    {
        x->add_neighbor( y );
        y->add_neighbor( x );
    }

    void collision_resolver_cloth_volume(
        collidable_type* x,
        collidable_type* y )
    {
        x->add_neighbor( y );
        y->add_neighbor( x );
    }

    void collision_resolver_cloth_plane(
        collidable_type* x,
        collidable_type* y )
    {
    }

    void collision_resolver_cloth_cloth(
        collidable_type* x,
        collidable_type* y )
    {
    }

private:
    // utility
	real_type epsilon()
	{
		return math< Traits >::epsilon();
	}

	real_type length( const vector_type& v )
	{
		return math< Traits >::length( v );
	}

	real_type length_sq( const vector_type& v )
	{
		return math< Traits >::length_sq( v );
	}

    real_type dot( const vector_type& a, const vector_type& b )
    {
        return math< Traits >::dot( a, b );
    }

    vector_type cross( const vector_type& a, const vector_type& b )
    {
        return math< Traits >::cross( a, b );
    }

private:
    // broad_collision_collectorから呼ばれるヘルパー関数
    void resolve_collision( collidable_type* x, collidable_type* y )
    {
        if( x == y ) { return; }

        body_type* xbody = x->get_body();
        body_type* ybody = y->get_body();
                
        if( xbody == ybody ) { return; }

        if( !xbody->get_positive() && !ybody->get_positive() ) { 
            // 両方非攻撃性オブジェクトなら何もしない
            return;
        }

        int xn = xbody->classid();
        int yn = ybody->classid();
                
        ( this->*collision_resolver_table_[xn*BODY_ID_MAX+yn] )(
            x, y );
    }
    template < class T > friend class broad_collision_collector;

    void resolve_raytest( typename RayProcessor< Traits >::ray_slot* rs )
    {
        collidable_type* a_collidable = rs->collidable;
        collidable_type* b_collidable = rs->nearest.collidable;

        body_type* a_body = a_collidable->get_body();
        body_type* b_body = b_collidable->get_body();

        assert(!(a_body->as_volume()&& b_body->as_volume()));

        if( !a_body->get_positive() && !b_body->get_positive() ) { 
            // 両方非攻撃性オブジェクトなら何もしない
            return;
        }

        if( !a_body->get_alive() ) {
            // 攻撃側がaliveでなければ何もしない
            return;
        }

        if( !b_body->get_influential() ) {
            // 防御側がinfluentialでなければ何もしない
            return;
        }

        //vector_type plane_normal = math< Traits >::cross( e1, e2 );
        //math< Traits >::normalize_f( plane_normal );
        const vector_type& plane_normal = rs->nearest.n;

        if( a_body->get_alive() && a_body->get_influential() &&
            b_body->get_alive() && b_body->get_influential() ) {

			if( vt::z(rs->uvt) < rs->target->penetration_magnifier ) {
				rs->target->penetration_magnifier = vt::z(rs->uvt);

				vector_type vv = ( rs->target->new_position - rs->source );
				vector_type v = vv * ( real_type( 1.0 ) - vt::z(rs->uvt) );
				vector_type penetration =
					plane_normal * dot( -v, plane_normal );

				contact_type* c = (contact_type*)contact_pool_.allocate();
				c->A_body = a_body;
				c->B_body = b_body;
				c->A_point = rs->target;
				c->B_point0 = rs->nearest.p0;
				c->B_point1 = rs->nearest.p1;
				c->B_point2 = rs->nearest.p2;
				c->A_point->penetration_vector = penetration;
				c->u = vt::x(rs->uvt);
				c->v = vt::y(rs->uvt);
				c->w = real_type( 1.0 ) - c->u - c->v;
				c->t = vt::z(rs->uvt);
				c->alpha = 0;
				c->check();

				contacts_.push_back( c );
			}
        } else {
            constraint_type c;
            c.A_body          = a_body;
            c.B_body          = b_body;
            c.point           = rs->target;
            c.plane_normal    = plane_normal;
            c.plane_position  = rs->nearest.p0->new_position;
            constraints_.push_back( c );
        }
    }


    // penetration_face_spatial_hash_replierから呼ばれるヘルパー関数
    void add_contact(
        body_type* A_body,
        body_type* B_body,
        point_type* A_point,
        point_type* B_point0,
        point_type* B_point1,
        point_type* B_point2,
        real_type u,
        real_type v,
        real_type w,
		real_type t )
    {
        if( !A_body->get_positive() && !B_body->get_positive() ) { 
            // 両方非攻撃性オブジェクトなら何もしない
            return;
        }

        if( !A_body->get_alive() ) {
            // 攻撃側がaliveでなければ何もしない
            return;
        }

        if( !B_body->get_influential() ) {
            // 防御側がinfluentialでなければ何もしない
            return;
        }

        if( A_body->get_alive() && A_body->get_influential() &&
            B_body->get_alive() && A_body->get_influential() ) {

			if( t < A_point->penetration_magnifier ) {
				A_point->penetration_magnifier = t;

				contact_type* c = (contact_type*)contact_pool_.allocate();
				c->A_body = A_body;
				c->B_body = B_body;
				c->A_point = A_point;
				c->B_point0 = B_point0;
				c->B_point1 = B_point1;
				c->B_point2 = B_point2;
				c->u = u;
				c->v = v;
				c->w = w;
				c->t = t;
				c->alpha = 0;
				c->check();

				contacts_.push_back( c );
			}
        } else {
            const vector_type& v0 = B_point0->new_position;
            const vector_type& v1 = B_point1->new_position;
            const vector_type& v2 = B_point2->new_position;
                        
            vector_type plane_normal = cross( v1 - v0, v2 - v0 );
            math< Traits >::normalize_f( plane_normal );

            constraint_type c;
            c.A_body        = A_body;
            c.B_body        = B_body;
            c.point         = A_point;
            c.plane_normal  = plane_normal;
            c.plane_position = v0;
            constraints_.push_back( c );
        }
    }

    // cloth_penetration_face_spatial_hash_replierから呼ばれるヘルパー関数
    void add_constraint(
        body_type* A_body,
        body_type* B_body,
        point_type* point,
        const vector_type& plane_normal,
        const vector_type& plane_position )
    {
        if( !A_body->get_positive() && !B_body->get_positive() ) { 
            // 両方非攻撃性オブジェクトなら何もしない
            return;
        }

        if( !A_body->get_alive() ) {
            // 攻撃側がaliveでなければ何もしない
            return;
        }

        if( !B_body->get_influential() ) {
            // 防御側がinfluentialでなければ何もしない
            return;
        }

        constraint_type c;
        c.A_body          = A_body;
        c.B_body          = B_body;
        c.point           = point;
        c.plane_normal    = plane_normal;
        c.plane_position  = plane_position;
        constraints_.push_back( c );
    }

    softvolume_type* cast_to_softvolume(collidable_type* p) {
        body_type* body = p->get_body();
        if (body->classid() == BODY_ID_SOFTVOLUME) {
            return static_cast<softvolume_type*>(body);
        }
        return nullptr;
    }
        
    cloth_type* cast_to_cloth(collidable_type* p) {
        body_type* body = p->get_body();
        if (body->classid() == BODY_ID_CLOTH) {
            return static_cast<cloth_type*>(body);
        }
        return nullptr;
    }
        
    template < class T >
    friend class penetration_face_spatial_hash_replier;

private:
    int                                    body_id_seed_;
    real_type                              time_;
	real_type							   previous_idt_;
    bodies_type                            bodies_;
    std::vector< collision_resolver_type > collision_resolver_table_;
    constraints_type                       constraints_;
    contacts_type                          contacts_;
    default_page_provider					page_provider_;
    fixed_pool< sizeof( contact_type ), default_page_provider > contact_pool_;
    //BroadSpatialHash< Traits >      broad_spatial_hash_;
    aabb_tree_type                         aabbt_;
    ray_processor_type                     ray_processor_;
    PointTetrahedronSpatialHash< Traits >  point_tetrahedron_spatial_hash_;
    EdgeFaceSpatialHash< Traits >          edge_face_spatial_hash_;
    PenetrationFaceSpatialHash< Traits >   penetration_face_spatial_hash_;
    ClothPointTetrahedronSpatialHash< Traits > cloth_point_tetrahedron_spatial_hash_;
    ClothSpikeFaceSpatialHash< Traits >        cloth_spike_face_spatial_hash_;
    ClothEdgeFaceSpatialHash< Traits >      cloth_edge_face_spatial_hash_;

};

} // namespace partix

#endif // PARTIX_WORLD_HPP
