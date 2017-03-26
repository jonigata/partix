/*!
  @file		voxel_traverser.hpp
  @brief	<ŠT—v>

  <à–¾>
  $Id: voxel_traverser.hpp 251 2007-06-22 15:15:48Z naoyuki $
*/
#ifndef VOXEL_TRAVERSER_HPP
#define VOXEL_TRAVERSER_HPP

#include <limits>
#include <math.h>
#include <iostream>

template < class Traits >
class voxel_traverser {
public:
    typedef typename Traits::vector_traits  vector_traits;
    typedef typename Traits::vector_type    vector_type;
    typedef typename Traits::real_type      real_type;

    typedef vector_traits vt;

public:
    voxel_traverser(
        const vector_type& v0,
        const vector_type& v1,
        real_type gridsize)
        : v0_(v0),
          v1_(v1),
          gridsize_(gridsize),
          rgridsize_(real_type(1.0) / gridsize) {
        vector_type v = v1 - v0;

        x_ = coord(vt::x(v0_));
        y_ = coord(vt::y(v0_));
        z_ = coord(vt::z(v0_));

        endx_ = coord(vt::x(v1_));
        endy_ = coord(vt::y(v1_));
        endz_ = coord(vt::z(v1_));

        stepx_ = sgn(vt::x(v));
        stepy_ = sgn(vt::y(v));
        stepz_ = sgn(vt::z(v));

        set_tmax(tmaxx_, vt::x(v), vt::x(v0), stepx_);
        set_tmax(tmaxy_, vt::y(v), vt::y(v0), stepy_);
        set_tmax(tmaxz_, vt::z(v), vt::z(v0), stepz_);

        tdeltax_ = std::abs(gridsize_ / vt::x(v));
        tdeltay_ = std::abs(gridsize_ / vt::y(v));
        tdeltaz_ = std::abs(gridsize_ / vt::z(v));
        if (x_ == endx_) {
            tdeltax_ = 0;
            tmaxx_ =(std::numeric_limits<real_type>::max)();
        }
        if (y_ == endy_) {
            tdeltay_ = 0;
            tmaxy_ =(std::numeric_limits<real_type>::max)();
        }
        if (z_ == endz_) {
            tdeltaz_ = 0;
            tmaxz_ =(std::numeric_limits<real_type>::max)();
        }

        ok_ = true;
    }
    ~voxel_traverser() {}

    bool operator()(int& x, int& y, int& z) {
        if (!ok_) { return false; }

        x = x_;
        y = y_;
        z = z_;
        if (reach(x_, stepx_, endx_)&&
            reach(y_, stepy_, endy_)&&
            reach(z_, stepz_, endz_)) {
            ok_ = false;
        }

        if (tmaxx_ <tmaxy_) {
            if (tmaxx_ <tmaxz_) {
                proceed(x_, stepx_, tmaxx_, tdeltax_);
            } else {
                proceed(z_, stepz_, tmaxz_, tdeltaz_);
            }
        } else {
            if (tmaxy_ <tmaxz_) {
                proceed(y_, stepy_, tmaxy_, tdeltay_);
            } else {
                proceed(z_, stepz_, tmaxz_, tdeltaz_);
            }
        }
        return true;
    }

private:
    int coord(real_type a) { return int(floor(a * rgridsize_)); }

    int sgn(real_type x) {
        if (x <0) { return -1; } else if (0 <x) { return 1; } else { return 0; }
    }

/*
  bool reach(int a, int step, int e) {
  return
  step == 0 ||
  (step <0 && a <= e)||
  (0 <step && e <= a);
  }
*/
    bool reach(int a, int step, int e) { return e*step <= a*step; }

    real_type nearest_bound(int step, real_type a) {
        if (step <0) {
            return floor(a * rgridsize_) * gridsize_;
        } else {
            return ceil(a * rgridsize_) * gridsize_;
        }
    }
    void set_tmax(real_type& tmax, real_type v, real_type v0, int step) {
        if (v == 0) {
            tmax =(std::numeric_limits<real_type>::max)();
        } else {
            tmax = std::abs((nearest_bound(step, v0)- v0)/ v);
        }
    }

    void proceed(int& a, int step, real_type& tmax, real_type tdelta) {
        a += step;
        tmax += tdelta;
    }

    vector_type v0_, v1_;
    real_type gridsize_, rgridsize_;
    int x_, y_, z_, endx_, endy_, endz_, stepx_, stepy_, stepz_;
    real_type tmaxx_, tmaxy_, tmaxz_;
    real_type tdeltax_, tdeltay_, tdeltaz_;
    bool ok_;

};

#endif // VOXEL_TRAVERSER_HPP
