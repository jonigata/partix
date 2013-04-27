/*!
  @file		partix_shell.hpp
  @brief	<ŠT—v>

  <à–¾>
  $Id: partix_shell.hpp 252 2007-06-23 08:32:33Z naoyuki $
*/
#ifndef PARTIX_SHELL_HPP
#define PARTIX_SHELL_HPP

#include "partix_body.hpp"
#include "partix_cloud.hpp"
#include "partix_block.hpp"

namespace partix {

template < class Traits >
class Shell : public Body< Traits > {
public:
	typedef Cloud< Traits >					cloud_type;
	typedef Block< Traits >					block_type;
	typedef std::vector< cloud_type* >		clouds_type;
	typedef std::vector< block_type* >		blocks_type;
	typedef typename cloud_type::point_type	 point_type;
	typedef typename cloud_type::points_type  points_type;
	typedef typename block_type::indices_type indices_type;

	void add_cloud( cloud_type* p ) { clouds_.push_back( p ); }
	void add_block( block_type* p ) { blocks_.push_back( p ); }

	const clouds_type&		get_clouds() { return clouds_; }
	const blocks_type&		get_blocks() { return blocks_; }

protected:
	clouds_type		clouds_;
	blocks_type		blocks_;

};

} // namespace partix

#endif // PARTIX_SHELL_HPP
