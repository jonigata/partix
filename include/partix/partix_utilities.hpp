/*!
  @file     partix_utilities.hpp
  @brief    <ŠT—v>

  <à–¾>
  $Id: partix_utilities.hpp 109 2007-04-12 08:52:16Z hirayama $
*/
#ifndef PARTIX_UTILITIES_HPP
#define PARTIX_UTILITIES_HPP

namespace partix {

template < class Real >
inline bool isnan( Real f ) { return f != f; }

template < class Real >
inline Real square( Real f ) { return f * f; }

};

#endif // PARTIX_UTILITIES_HPP
