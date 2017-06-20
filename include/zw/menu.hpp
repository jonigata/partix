#ifndef MENU_HPP
#define MENU_HPP

#include "menu_imp.hpp"

namespace zw {

namespace win {

class menuitem {
public:
        menuitem() { init( new detail::separator_menudata ); }
        menuitem( const menuitem& x ) : imp( x.imp ) { }

        template <class F>
        menuitem( const std::string& s, F f )
        {
                init( new detail::concrete_menudata< F >( s, f ) );
        }
        ~menuitem(){}

        menuitem& operator=( const menuitem& x )
        {
                imp = x.imp;
                return *this;
        }

        void enable( bool f ) { imp->enable( f ); }
        void check( bool f ) { imp->check( f ); }
        bool enabled() { return imp->enabled(); }
        bool checked() { return imp->checked(); }

        void set_accelerator( bool ctrl, bool alt, bool shift, uint16_t key )
        {
                imp->set_accelerator( ctrl, alt, shift, key );
        }

private:
        void init( detail::abstract_menudata* data ) { imp.reset( data ); }


private:
        friend class window_manager;
        friend class menulist;
        detail::abstract_menudata* get() { return imp.get(); }

private:
        boost::shared_ptr<detail::abstract_menudata> imp;

};

class menuseparator {
public:
        menuseparator() : imp( new detail::separator_menudata ) { }
        menuseparator(const menuseparator& x) : imp( x.imp ) { }
        ~menuseparator() { }

        menuseparator& operator=( const menuseparator& x ) { imp = x.imp; }

private:
        void init(detail::abstract_menudata*);

private:
        friend class window_manager;
        friend class menulist;
        detail::abstract_menudata* get() { return imp.get(); }

private:
        boost::shared_ptr<detail::abstract_menudata> imp;

};

class menulist {
public:
        menulist() : imp( new detail::menulist_imp ) { }
        menulist( const std::string& s )
                : imp( new detail::menulist_imp( s ) ) { }
        menulist( const menulist& x ) : imp( x.imp ) { }

        menulist& operator=( const menulist& x )
        {
                imp = x.imp;
                return *this;
        }

        menulist& operator<<( const menuitem& x )
        {
                if( !imp.unique() ) {
                        imp.reset( new detail::menulist_imp( *imp ) );
                }
                imp->add_item( x.imp );
                return *this;
        }
        
        menulist& operator<<( const menuseparator& x )
        {
                if( !imp.unique() ) {
                        imp.reset( new detail::menulist_imp( *imp ) );
                }
                imp->add_item( x.imp );
                return *this;
        }
        
        menulist& operator<<( const menulist& x )
        {
                if( !imp.unique() ) {
                        imp.reset( new detail::menulist_imp( *imp ) );
                }
                imp->add_item( x.imp );
                return *this;
        }

        void enable( bool f )
        {
                imp->enable( f );
        }

private:
        friend class window_manager;
        detail::abstract_menudata* get()
        {
                return imp.get();
        }

private:
        boost::shared_ptr<detail::menulist_imp> imp;
};

}

}

#endif // MENU_HPP
