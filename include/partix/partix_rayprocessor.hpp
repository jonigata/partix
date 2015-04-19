/*!
  @file     partix_raytest.hpp
  @brief    <概要>

  <説明>
  $Id: partix_rayprocessor.hpp 152 2007-04-26 10:12:36Z hirayama $
*/
#ifndef PARTIX_RAYPROCESSOR_HPP
#define PARTIX_RAYPROCESSOR_HPP

#include "partix_collidable.hpp"

namespace partix {

template < class Traits >
class RayProcessor {
public:
    typedef typename Traits::vector_traits          vector_traits;
    typedef typename Traits::real_type              real_type;
    typedef typename Traits::vector_type            vector_type;
    typedef typename Traits::matrix_type            matrix_type;
    typedef typename Traits::index_type             index_type;
    typedef std::vector< index_type >               indices_type;
    typedef Face< Traits >                          face_type;
    typedef std::vector< face_type >                faces_type;
    typedef Point< Traits >                         point_type;
    typedef Cloud< Traits >                         cloud_type;
    typedef Collidable< Traits >                    collidable_type;
    typedef Body< Traits >                          body_type;
    typedef typename std::vector< cloud_type* >     clouds_type;
    typedef typename std::vector< collidable_type* > collidables_type;
    typedef typename cloud_type::points_type        points_type;
        
    typedef std::set< cloud_type* >                 cloud_set;
    typedef std::set< collidable_type* >            collidable_set;

    struct triangle_slot {
        int                     body_id;
        collidable_type*        collidable;
        point_type*             p0;
        point_type*             p1;
        point_type*             p2;
        vector_type             n;
    };

    struct ray_slot {
        int                     body_id;
        collidable_type*        collidable;
        vector_type             source;
        point_type*             target;
        int                     index;
        vector_type             uvt;
        triangle_slot           nearest; // ポインタにしてはだめ
                                         // (書き換えてしまうから)
    };

    struct ray_triangle_spatial_hash_applier {
        void operator()(
            ray_slot* r,
            const triangle_slot& t,
            const vector_type& uvt ) const
        {
            real_type z = vector_traits::z( uvt );
            if( z < vector_traits::z( r->uvt ) ) {
                r->uvt = uvt;
                r->nearest = t;
            }
        }
    };

public:
    RayProcessor( real_type gridsize, int hashsize )
        : pool_( page_provider_, "rayproc" ), rtsh_( gridsize, hashsize ) {}
    ~RayProcessor(){}

    template < class F >
    void apply( F reflect, const collidables_type& B2 )
    {
        PerformanceCounter pc( false );

        pool_.clear();

        typedef typename collidables_type::const_iterator collidables_iterator;

        pc.print( "rp0" );
        
        // volumeとしか接触していないvolumeを取り除く
        collidables_type B;
        for( collidables_iterator i = B2.begin() ;
             i != B2.end() ;
             ++i ) {
            collidable_type * collidable = *i;
            if (!collidable->get_body()->as_volume()) {
                B.push_back( collidable );
                continue;
            }
                        
            bool f = false;
                        
            collidables_type& neighbors =
                collidable->get_neighbors();

            for( collidables_iterator i = neighbors.begin() ;
                 i != neighbors.end() ;
                 ++i ) {
                if (!(*i)->get_body()->as_volume()) {
                    f = true;
                    break;
                }
            }

            if( f ) {
                B.push_back( collidable );
            }
        }

        pc.print( "rp1" );
        // Ray集合、Triangle集合の作成
        float grid_size = 0;
        collidable_set R;
        collidable_set T;
        for( collidables_iterator i = B.begin() ;
             i != B.end() ;
             ++i ) {
            collidable_type* c = *i;

            grid_size += c->get_recommended_grid_size();

            // 非攻撃性オブジェクトはRayとして使用しない
            //  ( frozenなのは別にいい )
            if( !c->get_body()->get_positive() ) { continue; }

            R.insert( c );

            // 攻撃性オブジェクトから参照されているオブジェクトのみ
            // Triangleとして使用する
            T.insert( c->get_neighbors().begin(),
                      c->get_neighbors().end() );
        }

        pc.print( "rp2" );
        // Cloud集合の作成
        cloud_set C;
        for( typename collidable_set::const_iterator i = R.begin() ;
             i != R.end() ;
             ++i ) {
            C.insert( (*i)->get_cloud() );
        }

        for( typename cloud_set::const_iterator i = C.begin() ;
             i != C.end() ;
             ++i ) {
            cloud_type* c = *i;

            points_type& points = c->get_points();
            for( typename points_type::iterator j = points.begin() ;
                 j != points.end() ;
                 ++j ) {
                point_type& p = *j;
                p.raytest     = 0;
            }
        }

        pc.print( "rp3" );
        // spatial hash
        rtsh_.clear( grid_size );

        pc.print( "rp4" );
        // rayをspatial hashに追加
        for( typename collidable_set::const_iterator j = R.begin() ;
             j != R.end() ;
             ++j ) {
            collidable_type* collidable = *j;
            cloud_type*      cloud      = collidable->get_cloud();
            body_type*       body       = collidable->get_body();

            int         body_id = body->get_id();
            vector_type center  = collidable->get_center();

            indices_type&   indices   = collidable->get_indices(); 
            points_type&    points    = cloud->get_points();

            for( typename indices_type::const_iterator k =
                     indices.begin() ;
                 k != indices.end() ;
                 ++k ) {
                index_type  i0    = *k;
                point_type& point = points[i0];
                if( point.raytest ) {
                    // 同じrayを複数回行わないように
                    continue;
                }
                                
                point.raytest = 1;

                ray_slot* ri = (ray_slot*)pool_.allocate();
                ri->body_id     = body_id;
                ri->collidable  = collidable;
                ri->source      = collidable->get_center();
                ri->target      = &point;
                ri->index       = int( i0 );
                ri->uvt         = math< Traits >::vector_max();
                rtsh_.add_ray( ri->source, ri->target->new_position, ri );
                point.ray_slot = ri;
            }
        }

        // Ray側がVolumeだけだったら
        // Triangle側のVolumeは削除していい
        bool remove_volume = true;
        for( typename collidable_set::const_iterator i = R.begin() ;
             i != R.end() ;
             ++i ) {
            if (!(*i)->get_body()->as_volume()) {
                remove_volume = false;
                break;
            }
        }

        pc.print( "rp5" );
        // triangleをspatial hashに追加
        for( typename collidable_set::const_iterator i = T.begin() ;
             i != T.end() ;
             ++i ) {
            if (remove_volume && (*i)->get_body()->as_volume()) {
                continue;
            }

            collidable_type* collidable = *i;
            cloud_type*      cloud      = collidable->get_cloud();
            body_type*       body       = collidable->get_body();
                        
            int body_id = body->get_id();

            faces_type&     faces     = collidable->get_faces(); 
            points_type&    points    = cloud->get_points();

            for( typename faces_type::const_iterator k = faces.begin() ;
                 k != faces.end() ;
                 ++k ) {
                const face_type& face = *k;

                point_type& p0 = points[face.i0];
                point_type& p1 = points[face.i1];
                point_type& p2 = points[face.i2];

                vector_type& v0 = p0.new_position;
                vector_type& v1 = p1.new_position;
                vector_type& v2 = p2.new_position;

                vector_type n = math< Traits >::normalize(
                    math< Traits >::cross( v1 - v0, v2 - v0 ) );

                triangle_slot ti;
                ti.body_id    = body_id;
                ti.collidable = collidable;
                ti.p0         = &p0;
                ti.p1         = &p1;
                ti.p2         = &p2;
                ti.n          = n;
                rtsh_.add_triangle( v0, v1, v2, n, ti );
            }
        }
                
        pc.print( "rp6" );
        rtsh_.apply( ray_triangle_spatial_hash_applier() );

        pc.print( "rp7" );
        for( typename collidable_set::const_iterator j = R.begin() ;
             j != R.end() ;
             ++j ) {
            collidable_type* collidable = *j;
            cloud_type*      cloud      = collidable->get_cloud();
            body_type*       body       = collidable->get_body();

            int         body_id = body->get_id();
            vector_type center  = collidable->get_center();

            indices_type&   indices   = collidable->get_indices(); 
            points_type&    points    = cloud->get_points();

            for( typename indices_type::const_iterator k =
                     indices.begin() ;
                 k != indices.end() ;
                 ++k ) {
                index_type  i0    = *k;
                point_type& point = points[i0];
                if( point.raytest == 2 ) {
                    // 同じrayを複数回行わないように
                    continue;
                }
                                
                point.raytest = 2;

                ray_slot* rs = (ray_slot*)point.ray_slot;
                if( vector_traits::z( rs->uvt ) < 1.0f ) {
                    reflect( rs );
                }
            }
        }
    }

private:
    default_page_provider                                   page_provider_;
    fixed_pool< sizeof( ray_slot ), default_page_provider > pool_;
    RayTriangleSpatialHash< Traits, ray_slot*, triangle_slot > rtsh_;

};

}

#endif // PARTIX_RAYTEST_HPP
