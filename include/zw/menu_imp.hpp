/*!
  @file     menu_imp.hpp
  @brief    <ŠT—v>

  <à–¾>
  $Id: menu_imp.hpp 16 2008-03-28 14:58:11Z Naoyuki.Hirayama $
*/
#ifndef MENU_IMP_HPP
#define MENU_IMP_HPP

#include <string>
#include <vector>
#include <boost/shared_ptr.hpp>
#include <boost/function.hpp>
#include "window_base.hpp"

namespace zw {

namespace win {

namespace detail {

typedef boost::shared_ptr<class abstract_menudata> menudata_ptr;
typedef boost::function1<void,bool> menu_enabler_type;
typedef boost::function1<void,bool> menu_checker_type;

class abstract_menudata {
public:
        virtual ~abstract_menudata(){}

        virtual void set_enabler( menu_enabler_type ) = 0;
        virtual void set_checker( menu_checker_type ) = 0;
        virtual void invoke()=0;
        virtual void add_item(menudata_ptr) = 0;
        virtual std::string get_text() = 0;
        virtual bool get_children( 
                std::vector< menudata_ptr >::const_iterator& b, 
                std::vector< menudata_ptr >::const_iterator& e ) = 0;
        virtual void enable( bool f ) = 0;
        virtual void check( bool f ) = 0;
        virtual bool enabled() = 0;
        virtual bool checked() = 0;
        virtual void set_accelerator( 
                bool ctrl, bool alt, bool shift, uint16_t key ) = 0;
        virtual void get_accelerator( 
                bool& ctrl, bool& alt, bool& shift, uint16_t& key ) = 0;
        
};

template <class F>
class concrete_menudata : public abstract_menudata {
public:
        concrete_menudata( const std::string& s, const F& f )
                : s_( s ),
                  f_( f ),
                  accel_key_( 0 ),
                  enabled_( true ),
                  checked_( false ) { }

        void set_enabler( menu_enabler_type e ) { e_ = e; }
        void set_checker( menu_checker_type c ) { c_ = c; }
        void invoke() { f_() ; }
        void add_item( menudata_ptr ) { }
        std::string get_text() { return s_ ; }
        bool get_children( 
                std::vector< menudata_ptr >::const_iterator&, 
                std::vector< menudata_ptr >::const_iterator& )
        {
                return false;
        }
        
        void enable( bool f )
        {
                if( f != enabled_ ) { enabled_ = f ; e_( f ) ; }
        }
        void check( bool f ) { checked_ = f ; c_( f ) ; }
        bool enabled() { return enabled_ ; }
        bool checked() { return checked_ ; }
        void set_accelerator( bool ctrl, bool alt, bool shift, uint16_t key )
        { 
                accel_ctrl_ = ctrl ; 
                accel_alt_ = alt ; 
                accel_shift_ = shift ; 
                accel_key_ = key ; 
        }
        void get_accelerator(
                bool& ctrl, bool& alt, bool& shift, uint16_t& key )
        {
                ctrl = accel_ctrl_;
                alt = accel_alt_;
                shift = accel_shift_;
                key = accel_key_;
        }

private:
        menu_enabler_type       e_;
        menu_checker_type       c_;
        std::string             s_;
        F                       f_;
        bool                    accel_ctrl_;
        bool                    accel_alt_;
        bool                    accel_shift_;
        uint16_t                accel_key_;
        bool                    enabled_;
        bool                    checked_;

};

class menulist_imp : public abstract_menudata {
public:
        menulist_imp() : enabled_(true){}
        menulist_imp( const std::string& s ) : s_( s ), enabled_( true ) { }
        menulist_imp( const menulist_imp& x )
                : s_( x.s_ ), items_( x.items_ ), enabled_( true ) { }
        ~menulist_imp() { }

        void set_enabler( menu_enabler_type e ) { e_ = e; }
        void set_checker( menu_checker_type ) {}
        void invoke(){}
        void add_item( menudata_ptr m ) { items_.push_back( m ) ; }
        std::string get_text() { return s_ ; }
        bool get_children(
                std::vector< menudata_ptr >::const_iterator& b, 
                std::vector< menudata_ptr >::const_iterator& e )
        {
                b = items_.begin();
                e = items_.end();
                return true;
        }        
        void enable( bool f ) { if( f != enabled_ ) { enabled_ = f; e_(f); } }
        void check( bool ) {}
        bool enabled() { return enabled_; }
        bool checked() { return true; }
        void set_accelerator( bool ctrl, bool alt, bool shift, uint16_t key ){}
        void get_accelerator(
                bool& ctrl, bool& alt, bool& shift, uint16_t& key ) { }

private:
        menu_enabler_type               e_;
        std::string                     s_;
        std::vector<menudata_ptr>       items_;
        bool                            enabled_;

};

class separator_menudata : public abstract_menudata {
public:
        separator_menudata() { }

        void set_enabler( menu_enabler_type e ) {}
        void set_checker( menu_checker_type ) {}
        void invoke() { }
        void add_item( menudata_ptr m ) { }
        std::string get_text() { return "-" ; }
        bool get_children( 
                std::vector< menudata_ptr >::const_iterator&, 
                std::vector< menudata_ptr >::const_iterator& )
        {
                return false;
        }
        void enable( bool ) { }
        void check( bool ) { }
        bool enabled() { return true; }
        bool checked() { return true; }
        void set_accelerator( bool ctrl, bool alt, bool shift, uint16_t key ){}
        void get_accelerator(
                bool& ctrl, bool& alt, bool& shift, uint16_t& key ){}

};

} // namespace detail

} // namespace win

} // namespace zw

#endif // MENU_IMP_HPP
