#ifndef EASE_IN_OUT_HPP
#define EASE_IN_OUT_HPP

template < class T >
class ease_in_out_interpolation {
public:
    void reset( const T& from, const T& to, float time )
    {
        assert( 0 <= time );
        value_ = from;
        target_ = to;
        speed_ = from - from;
        acceleration_ = ( to - from ) / ( time * time / 4 );
        remaining_time_ = total_time_ = time;
    }

    operator bool()
    {
        return 0 <= remaining_time_;
    }

    T operator()( float delta_time )
    {
        remaining_time_ -= delta_time;
        if( remaining_time_ < total_time_ / 2 ) {
            speed_ -= acceleration_ * delta_time;
        } else {
            speed_ += acceleration_ * delta_time;
        }
        value_ += speed_ * delta_time;
        return value_;
    }
        
private:
    T       value_;
    T       target_;
    T       acceleration_;
    T       speed_;
    float   remaining_time_;
    float   total_time_;
        
};

#endif // EASE_IN_OUT_HPP
