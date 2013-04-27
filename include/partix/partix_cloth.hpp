#ifndef PARTIX_CLOTH_HPP
#define PARTIX_CLOTH_HPP

#include "partix_forward.hpp"
#include "partix_id.hpp"
#include "partix_math.hpp"
#include "partix_body.hpp"

namespace partix {

template < class Traits >
class ClothSnapShot : public BodySnapShot< Traits > {
public:
    typedef typename Traits::vector_type    vector_type;
    typedef typename Traits::matrix_type    matrix_type;

    BodySnapShot< Traits >* clone()
    {
        return new ClothSnapShot< Traits >( *this );
    }

    std::vector< Cloud< Traits > >  clouds;
    vector_type                     initial_center;
    vector_type                     global_force;
};

template < class Traits >
class Cloth : public Body< Traits >, public Collidable< Traits > {
public:
    typedef typename Traits::vector_traits      vector_traits;
    typedef typename Traits::real_type          real_type;
    typedef typename Traits::index_type         index_type;
    typedef typename Traits::vector_type        vector_type;
    typedef typename Traits::matrix_type        matrix_type;
    typedef TetrahedralMesh< Traits >           mesh_type;
    typedef Face< Traits >                      face_type;
    typedef Body< Traits >                      body_type;
    typedef Cloud< Traits >                     cloud_type;
    typedef Collidable< Traits >                collidable_type;
    typedef std::vector< collidable_type* >     collidables_type;
    typedef typename cloud_type::point_type     point_type;
    typedef typename cloud_type::points_type    points_type;
    typedef std::vector< face_type >            faces_type;
    typedef std::vector< index_type >           indices_type;

public:
    struct spring_type {
        Edge< Traits >  indices;
        bool            border;
        real_type       t; // ("not collided"-"collided")の係数
        real_type       u;
        real_type       v;
        real_type       w;
        vector_type     collision_normal;
        real_type       natural_length;
    };
    typedef std::vector< spring_type > springs_type;
        
public:
    Cloth()
    {
        touch_level_    = 2;
        current_center_ = initial_center_ = vector_type( 0, 0, 0 );
        average_edge_length_ = 0;
    }
    ~Cloth() {}

    // implements Body
    int classid() { return BODY_ID_CLOTH; }

    ClothSnapShot< Traits >* make_snapshot()
    {
        return make_snapshot_internal();
    }
    void apply_snapshot( const BodySnapShot< Traits >* ss )
    {
        apply_snapshot_internal( ss );
    }

    void regularize() { regularize_internal(); }
    void list_collision_units( collidables_type& s ) { s.push_back( this ); } 

    void move( const vector_type& v ) { move_internal( v ); }
    void teleport( const vector_type& v ) { teleport_internal( v ); }

    void begin_frame() { begin_frame_internal(); }
    void update_velocity( real_type dt, real_type idt )
    {
        update_velocity_internal( dt, idt );
    }
    void apply_forces( real_type dt, real_type idt )
    {
        apply_forces_internal( dt, idt );
    }
    void match_shape() {}
    void restore_shape( real_type dt, real_type idt, int kmax )
    {
        restore_shape_internal( dt, idt, kmax );
    }
    void end_frame() { end_frame_internal(); }
    
    void update_display_matrix() {}
    void update_boundingbox() { update_boundingbox_internal(); }
    vector_type get_initial_center() { return initial_center_; }
    vector_type get_current_center() { return current_center_; }

    // implements Collidable
    body_type*              get_body() { return this; }
    cloud_type*             get_cloud() { return cloud_; }
    indices_type&           get_indices() { return indices_; }
    faces_type&             get_faces() { return faces_; }
    vector_type             get_center() { return current_center_; }
    vector_type             get_bbmin() { return bbmin_; }
    vector_type             get_bbmax() { return bbmax_; }
    real_type               get_recommended_grid_size()
    {
        return get_average_edge_length(); 
    }

    void clear_neighbors() { clear_neighbors_internal(); }
    void add_neighbor( collidable_type* p ) { neighbors_.push_back( p ); }
    collidables_type&   get_neighbors() { return neighbors_; }
    bool pick( const vector_type& s0, const vector_type& s1, real_type& dist )
    {
        return pick_internal( s0, s1, dist );
    }

    void mark() { marked_ = true; }
    void unmark() { marked_ = false; }
    bool marked() { return marked_; }

    // original
    void    set_cloud( cloud_type* cloud ) { cloud_ = cloud; }
    void    add_spring( int i0, int i1 )
    {
        spring_type s; s.indices.i0 = i0; s.indices.i1 = i1;
        springs_.push_back( s );
    }
    springs_type& get_springs() { return springs_; }

    void    add_face( int i0, int i1, int i2 )
    {
        face_type f; f.i0 = i0; f.i1 = i1; f.i2 = i2;
        faces_.push_back( f );
    }
        
    void set_thickness( real_type t ) { thickness_ = t; }
    real_type get_thickness() { return thickness_; }

    real_type get_average_edge_length()
    {
        return average_edge_length_;
    }

    void mark_spikes( ClothSpikeFaceSpatialHash< Traits >& hash )
    {
        mark_spikes_internal( hash );
    }
    void mark_border_edges( ClothEdgeFaceSpatialHash< Traits >& hash )
    {
        mark_border_edges_internal( hash );
    }

private:
    Cloth( const Cloth& ){}
    void operator=( const Cloth& ){}

private:
    ClothSnapShot< Traits >* make_snapshot_internal()
    {
        // 所有権はSnapShotに移動する

        ClothSnapShot< Traits >* ds = new ClothSnapShot< Traits >;

        ds->clouds.push_back( *cloud_ );
        ds->initial_center = initial_center_;
        ds->global_force   = this->get_global_force();

        return ds;
    }

    void apply_snapshot_internal( const BodySnapShot< Traits >* ss )
    {
        const ClothSnapShot< Traits >* ds =
            dynamic_cast< const ClothSnapShot< Traits >* >( ss );
        assert( ds );

        int jj = 0;
        *cloud_ = ds->clouds.front();
        initial_center_ = ds->initial_center;
        set_global_force( ds->global_force );
    }

    void restore_shape_internal( real_type dt, real_type idt, int kmax )
    {
        return;

        points_type& points = cloud_->get_points();

        // 基点を発見
        //  一定以上の大きさの不自然な力が働いている部分を処理キューに入れる
        size_t n = points.size();
        for( size_t i = 0 ; i < n ; i++ ) {
            point_type& p = points[i];
            p.process_flag = false;
            if( p.cloth_flags ) {
                p.process_flag = true;
            }
        }

        // 基点からシードフィルペイント

        int processed_count;
        //OutputDebugStringA( "start ik\n" );

        n = springs_.size();
        do {
            processed_count = 0;
            for( size_t i = 0 ; i < n ; i++ ) {
                spring_type& s = springs_[i];
                point_type* p0 = &points[s.indices.i0];
                point_type* p1 = &points[s.indices.i1];
                if( p0->process_flag == p1->process_flag ) { continue; }
                if( !p0->process_flag ) { std::swap( p0, p1 ); }

                vector_type v = p1->new_position - p0->new_position;
                real_type l = vector_traits::length( v );
                if( l < math< Traits >::epsilon() ) {
                    l = math < Traits >::epsilon();
                }
                if( s.natural_length * 1.05f < l ) {
                    v *= ( s.natural_length * 1.05f ) / l;
                    p1->new_position = p0->new_position + v;
                }
                p1->process_flag = true;
                processed_count++;
            }
                        
        } while( processed_count );
    }

    void begin_frame_internal()
    {
        if( touch_level_ == 2 ) {
            regularize_internal();
        }
    }

    void update_velocity_internal( real_type dt, real_type idt )
    {
        if( touch_level_ == 0 ) {
            if( !this->get_alive() ) { return; }
            if( this->get_frozen() && !this->get_defrosting() ) { return; }
        }
        cloud_->update_velocity( dt, idt );

#if 0
        // spring
        points_type& points = cloud_->get_points();
        for( springs_type::const_iterator i = springs_.begin() ;
             i != springs_.end() ;
             ++i ) {
            point_type& p0 = points[(*i).indices.i0];
            point_type& p1 = points[(*i).indices.i1];

            vector_type tension_direction = p1.new_position - p0.new_position;
            real_type length = Traits::vector_length( tension_direction );
            real_type extention = length - (*i).natural_length;

            real_type tension = (*i).tension * extention / (*i).natural_length;

            math< Traits >::normalize_f( tension_direction );

            p0.forces += tension_direction * tension;
            p1.forces += tension_direction * -tension;
        }                
#endif
    }

    void apply_forces_internal( real_type dt, real_type idt )
    {
        if( touch_level_ == 0 ) {
            if( !this->get_alive() ) { return ; }
            if( this->get_frozen() && !this->get_defrosting() ) { return; }
        }

        const real_type drag_coefficient =
            real_type( 1.0 ) -
            pow(  real_type( 1.0 ) - this->get_drag_factor(), dt );

        cloud_->apply_forces(
            dt,
            idt,
            math< Traits >::vector_zero(),
            this->get_global_force(),
            drag_coefficient,
            1 );

        points_type& points = cloud_->get_points();
        for( typename springs_type::const_iterator i = springs_.begin() ;
             i != springs_.end() ;
             ++i ) {
            real_type dmax = real_type( 1.05 );

            point_type& p0 = points[(*i).indices.i0];
            point_type& p1 = points[(*i).indices.i1];

            vector_type v = p1.new_position - p0.new_position;
            real_type l = vector_traits::length( v );
            vector_type u =
                v * ( 1.0f / l ) * ( l - dmax * (*i).natural_length );

            real_type m0 = p0.mass / ( p0.mass + p1.mass );
            real_type m1 = p1.mass / ( p0.mass + p1.mass );

            p0.new_position += u * m1;
            p1.new_position -= u * m0;
        }                
    }


    void update_frozen( real_type dt, real_type idt )
    {
        if( this->get_auto_freezing() ) {
            real_type energy = 0;
            real_type e = 0;
            points_type& points = cloud_->get_points();
            for( typename points_type::iterator i = points.begin() ;
                 i != points.end() ;
                 ++i ) {
                point_type& p = *i;
                math< Traits >::mount( energy, e, p.energy );
            }

            if( energy <= Traits::freeze_threshold_energy() ) {
                freezing_duration_ += dt;
                if( Traits::freeze_duration() <= freezing_duration_ ) {
                    this->set_frozen( true );
                    freezing_duration_ = 0;
#if 0
                    for( points_type::iterator i = points.begin() ;
                         i != points.end() ;
                         ++i ) {
                        point_type& p = *i;
                        p.old_position = p.new_position;
                    }
#endif

#if 0
                    OutputDebugStringA( "frozen\n" );
#endif
                }
            } else {
#if 0
                {
                    char buffer[256];
                    sprintf( buffer, "energy: %f\n", energy );
                    OutputDebugStringA( buffer );
                }
#endif
                this->set_frozen( false );
            }
        }
        this->set_defrosting( false );
    }

    void update_boundingbox_internal( )
    {
        if( touch_level_ == 0 ) {
            if( !this->get_alive() ) { return; }
            if( this->get_frozen() ) { return; }
        }

        bbmin_ = math< Traits >::vector_max();
        bbmax_ = math< Traits >::vector_min();
                
        cloud_->update_bb( bbmin_, bbmax_ );

        bbmin_.x -= thickness_;
        bbmin_.y -= thickness_;
        bbmin_.z -= thickness_;
        bbmax_.x += thickness_;
        bbmax_.y += thickness_;
        bbmax_.z += thickness_;
    }

    void end_frame_internal()
    {
        touch_level_ = 0;
    }

    void move_internal( const vector_type& v )
    {
        cloud_->move( v );
        set_touch_level( 1 );
    }

    void teleport_internal( const vector_type& v )
    {
        cloud_->teleport( v );
        set_touch_level( 1 );
    }

    void set_orientation_internal( const matrix_type& m )
    {
        cloud_->set_orientation( m );
        set_touch_level( 1 );
    }

    void rotate_internal( real_type w, real_type x, real_type y, real_type z )
    {
        quaternion< real_type > q( w, x, y, z );
        quaternion< real_type > rq( w, -x, -y, -z );
        cloud_->rotate( q, rq, current_center_ );
        set_touch_level( 1 );
    }

private:
    // world friend
    void clear_penetration_vector()
    {
        points_type& points = cloud_->get_points();
        for( typename points_type::iterator j = points.begin() ;
             j != points.end() ;
             ++j ) {
            point_type& p = *j;
            p.penetration_vector = math< Traits >::vector_zero();
        }
    }

private:
    void set_touch_level( int n )
    {
        if( touch_level_ < n ) { touch_level_ = n; }
    }

    void regularize_internal()
    {
        vector_type v0 = math< Traits >::vector_zero();

        calculate_initial_center();

        // init
        points_type& points = cloud_->get_points();
        for( typename points_type::iterator j = points.begin() ;
             j != points.end() ;
             ++j ) {
            point_type& p = *j;
            p.ideal_offset    = p.source_position - initial_center_;
        }

        real_type total_edge_length = 0;
        for( typename springs_type::iterator i = springs_.begin();
             i != springs_.end();
             ++i ) {
            vector_type v =
                points[(*i).indices.i1].source_position -
                points[(*i).indices.i0].source_position;
            (*i).natural_length = vector_traits::length( v );
            total_edge_length += (*i).natural_length;
        }
        average_edge_length_ = total_edge_length / springs_.size();                
    }

    void calculate_initial_center()
    {
        vector_type total_center = math< Traits >::vector_zero();
        vector_type e = math< Traits >::vector_zero();
        real_type total_mass = 0;
        real_type mass_e = 0;

        points_type& points = cloud_->get_points();
        for( typename points_type::const_iterator i = points.begin() ;
             i != points.end() ;
             ++i ) {
            const point_type& p = *i;
            math< Traits >::mount(
                total_center, e, p.source_position * p.mass );
            math< Traits >::mount( total_mass, mass_e, p.mass );
        }

        total_center *= real_type( 1.0 ) / total_mass;
        initial_center_ = total_center;
    }

    void calculate_center()
    {
        vector_type total_center = math< Traits >::vector_zero();
        vector_type e = math< Traits >::vector_zero();
        real_type total_mass = 0;
        real_type mass_e = 0;

        points_type& points = cloud_->get_points();
        for( typename points_type::const_iterator i = points.begin() ;
             i != points.end() ;
             ++i ) {
            const point_type& p = *i;
            if( !p.surface ) { continue; }

            math< Traits >::mount( total_center, e, p.new_position * p.mass );
            math< Traits >::mount( total_mass, mass_e, p.mass );
        }

        total_center /= total_mass;
        current_center_ = total_center;
    }

    void clear_neighbors_internal()
    {
        neighbors_.clear();
        points_type& points = cloud_->get_points();
        for( typename points_type::iterator i = points.begin() ;
             i != points.end() ;
             ++i ) {
            (*i).cloth_flags = 0;
            (*i).collided = false;
        }
        for( typename springs_type::iterator i = springs_.begin() ;
             i != springs_.end() ;
             ++i ) {
            spring_type& e = *i;
            e.border = false;
            e.t = math< Traits >::real_max();
        }
        make_normals();
    }

    bool pick_internal( const vector_type& s0, const vector_type& s1, real_type& dist )
    {
        if( math< Traits >::test_aabb_segment( bbmin_, bbmax_, s0, s1 ) ) {
            const points_type& points = get_cloud()->get_points();

            SegmentTriangleSpatialHash< Traits > hash(
                average_edge_length_, 4999 );
            hash.add_segment( s0, s1 );
            for( typename faces_type::const_iterator i = faces_.begin() ;
                 i != faces_.end() ;
                 ++i ) {
                const face_type& f = *i;
                hash.add_triangle(
                    points[f.i0].new_position,
                    points[f.i1].new_position,
                    points[f.i2].new_position );
                hash.add_triangle(
                    points[f.i0].new_position,
                    points[f.i2].new_position,
                    points[f.i1].new_position );
            }
            return hash.apply( dist );
        }
        return false;
    }

    void mark_spikes_internal( ClothSpikeFaceSpatialHash< Traits >& hash )
    {
        points_type& points = get_cloud()->get_points();

        int ii = 0;
        for( typename points_type::iterator i = points.begin() ;
             i != points.end() ;
             ++i, ++ii ) {
            point_type& p = *i;
            if( p.cloth_flags == 1 || p.cloth_flags == 2 ) {
                // 3, 0のときはスキップ
                hash.add_spike( this, ii );
            }
        }
    }

    void mark_border_edges_internal( ClothEdgeFaceSpatialHash< Traits >& hash )
    {
        points_type& points = get_cloud()->get_points();

        int ii = 0;
        for( typename springs_type::iterator i = springs_.begin() ;
             i != springs_.end() ;
             ++i, ++ii ) {
            const spring_type& s = *i;

            point_type& p0 = points[s.indices.i0];
            point_type& p1 = points[s.indices.i1];

            // f0/f1 == 埋もれてるので引っ張り出すフラグ
            bool f0 = p0.collided && ( p0.cloth_flags == 0 ||
                                       p0.cloth_flags == 3 );
            bool f1 = p1.collided && ( p1.cloth_flags == 0 ||
                                       p1.cloth_flags == 3 );
                        
            if( ( f0 && !p1.collided ) || ( !p0.collided && f1 ) ) {
                hash.add_edge( this, points, ii, s.indices.i0, s.indices.i1 );
            }
        }
    }

    void make_normals()
    {
        vector_type v0 = math< Traits >::vector_zero();
                
        points_type& points = get_cloud()->get_points();
        for( typename points_type::iterator i = points.begin() ;
             i != points.end() ;
             ++i ) {
            (*i).normal = v0;
        }

        for( typename faces_type::const_iterator i = faces_.begin() ;
             i != faces_.end() ;
             ++i ) {
            const face_type& f = *i;
            vector_type e1 =
                points[f.i1].new_position - points[f.i0].new_position;
            vector_type e2 =
                points[f.i2].new_position - points[f.i0].new_position;
            vector_type n = math< Traits >::cross( e1, e2 );
            //math< Traits >::normalize_f( n );
            points[f.i0].normal += n;
            points[f.i1].normal += n;
            points[f.i2].normal += n;
        }

        for( typename points_type::iterator i = points.begin() ;
             i != points.end() ;
             ++i ) {
            math< Traits >::normalize_f( (*i).normal );
        }
    }

//private:
public:
    // touch_level_
    //   1 ... 位置
    //   2 ... 位置 + 質量
    int             touch_level_;
    bool            mark_;
        
    vector_type     initial_center_;
    vector_type     current_center_;
    vector_type     bbmin_;
    vector_type     bbmax_;
    real_type       freezing_duration_;
    real_type       average_edge_length_;

    cloud_type*     cloud_;
    indices_type    indices_;
    springs_type    springs_;
    faces_type      faces_;
    real_type       thickness_;

    std::vector< Collidable< Traits >* >    neighbors_;
    bool                                    marked_;

    template < class T > friend class World;
};

}

#endif // PARTIX_CLOTH_HPP
