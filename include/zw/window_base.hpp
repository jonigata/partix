/*!
  @file     window_base.hpp
  @brief    <ŠT—v>

  <à–¾>
  $Id: window_base.hpp 16 2008-03-28 14:58:11Z Naoyuki.Hirayama $
*/
#ifndef WINDOW_BASE_HPP
#define WINDOW_BASE_HPP

#include <string>
#include <stdexcept>
#include <boost/cstdint.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/scoped_ptr.hpp>
#include <boost/noncopyable.hpp>
#include "visitor.hpp"
#include "vector.hpp"

namespace zw {

namespace win {

typedef basic_offset< basic_vector2< int > > offset_type ; 
typedef basic_extent< basic_vector2< int > > extent_type ; 
typedef basic_bounds< basic_vector2< int > > bounds_type ; 

typedef void* dc_type;
typedef void* window_handle_type;
typedef void* menu_handle_type;
typedef void* accel_handle_type;

typedef boost::uint8_t uint8_t;
typedef boost::uint16_t uint16_t;

struct window_exception : public std::runtime_error {
        window_exception( const std::string& s ) : std::runtime_error( s ) { }
};

} // namespace win

} // namespace zw

#endif // WINDOW_BASE_HPP
