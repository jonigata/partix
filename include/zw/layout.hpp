/*!
  @file     layout.hpp
  @brief    <概要>

  <説明>
*/
#ifndef LAYOUT_HPP
#define LAYOUT_HPP

#include <iostream>
#include <stdexcept>
#include <vector>
#include <boost/shared_ptr.hpp>
#include <boost/cast.hpp>
#include "vector.hpp"

namespace zw {

namespace layout {

typedef basic_offset<basic_vector2<int> > offset_type;
typedef basic_extent<basic_vector2<int> > extent_type;
typedef basic_bounds<basic_vector2<int> > bounds_type;

inline int infinity()   {return std::numeric_limits<int>::max()-1;}
inline int infinityx()  {return std::numeric_limits<int>::max();}
inline int fit()        {return std::numeric_limits<int>::min();}

class layout_exception : public std::runtime_error {
public:
        layout_exception(const std::string& s) : std::runtime_error(s){}
};

namespace detail {

class abstract_visitor_holder {
public:
        virtual ~abstract_visitor_holder(){}
};

template <class V>
class concrete_visitor_holder : public abstract_visitor_holder {
public:
        concrete_visitor_holder(V& v) : v_(v){}
        V& v_;
};

template <class V>
class accept_helper {
public:
        template <class T,class AH>
        accept_helper( T& t, AH* ah )
        {
                called = false;
                concrete_visitor_holder<V>* cvh=
                        dynamic_cast<concrete_visitor_holder<V>*>(ah);
                if( cvh ) {
                        t.accept(cvh->v_);
                        called = true;
                }
        }

        bool called;

        operator bool(){return called;}
};

template <>
class accept_helper<void> {
public:
        template <class T,class AH>
        accept_helper(T&,AH*){}

        operator bool(){return false;}
};


}

/*============================================================================
 *
 * class panel
 *
 * 
 *
 *==========================================================================*/

class panel {
protected:
        enum align_type {
                align_center,
                align_left,
                align_top,
                align_right,
                align_bottom
        };

        class panel_imp;
        typedef boost::shared_ptr<panel_imp> imp_ptr;

        class panel_imp {
        public:
                virtual ~panel_imp(){}
                virtual panel_imp*      clone()=0;
                virtual void            clear()=0;
                virtual void            add( imp_ptr ) { assert(0); }
        
                align_type  get_align()const            {return align_;}
                void        set_align(align_type a)     {align_=a;}

                extent_type get_logical_extent()const               {return extent_;}
                void        set_logical_extent(const extent_type& e){extent_=e;}

                int get_logical_width ()const{return extent_.w();}
                int get_logical_height()const{return extent_.h();}

                int calc_physical_width(const extent_type& extent)const
                {
                        if(get_logical_width()==fit()){
                                // このboxの幅は中身による

                                // 子の情報を取得する際のboundsを、
                                // 可能な限り決定的にしておく
                                int box_height=get_logical_height();
                                if(infinity()<=box_height){
                                        if(infinity()<=extent.h()){
                                                // 無限大の空間で伸びきろうとした
                                                throw layout_exception(
                                                        "infinite elements appeared in infinite box.");
                                        } else if(extent.h()==fit()){
                                                if(box_height!=infinity()){
                                                        // 最小矩形内で伸びきろうとした
                                                        throw layout_exception(
                                                                "infinite elements appeared in fitting box.");
                                                }
						box_height=0;
                                        } else {
						box_height=extent.h();
					}
                                }

                                return calc_fit_width(extent_type(extent.w(),box_height)); // 仮想関数呼び出し

                        } else if(infinity()<=get_logical_width()){
                                // このboxの幅は無限大

                                if(infinity()<=extent.w()){
                                        // 無限大の空間で伸びきろうとした
                                        throw layout_exception(
                                                "infinite elements appeared in infinite box.");
                                } else if(extent.w()==fit()){
                                        if(get_logical_width()==infinity()){
                                                // 準無限よりfitの方が強い
                                                return 0;
                                        } else {
                                                // 最小矩形内で伸びきろうとした
                                                throw layout_exception(
                                                        "infinite elements appeared in fitting box.");
                                        }
                                }
                                // 決定的
                                return extent.w();
                        } else {
                                // このboxの幅は決定されている
                                return get_logical_width();
                        }
                }
                int calc_physical_height(const extent_type& extent)const
                {
                        if(get_logical_height()==fit()){
                                // このboxの高さは中身による

                                // 子の情報を取得する際のboundsを、
                                // 可能な限り決定的にしておく
                                int box_width=get_logical_width();
                                if(infinity()<=box_width){
                                        if(infinity()<=extent.w()){
                                                // 無限大の空間で伸びきろうとした
                                                throw layout_exception(
                                                        "infinite elements appeared in infinite box.");
                                        } else if(extent.w()==fit()){
                                                if(box_width!=infinity()){
                                                        // 最小矩形内で伸びきろうとした
                                                        throw layout_exception(
                                                                "infinite elements appeared in fitting box.");
                                                }
						box_width=0;
                                        } else {
						box_width=extent.w();
					}
                                }

                                return calc_fit_height(extent_type(box_width,extent.h()));

                        } else if(infinity()<=get_logical_height()){
                                // このboxの高さが無限大
                
                                if(infinity()<=extent.h()){
                                        // 無限大の空間で伸びきろうとした
                                        throw layout_exception(
                                                "infinite elements appeared in infinite box.");
                                } else if(extent.h()==fit()){
                                        if(get_logical_height()==infinity()){
                                                // 準無限よりfitの方が強い
                                                return 0;
                                        } else {
                                                // 最小矩形内で伸びきろうとした
                                                throw layout_exception(
                                                        "infinite elements appeared in fitting box.");
                                        }
                                }
                                // 決定的
                                return extent.h();
                        } else {
                                // このboxの高さは決定されている
                                return get_logical_height();
                        }
                }
    
                virtual bool is_width_infinity()const=0;
                virtual bool is_width_infinityx()const=0;
                virtual bool is_height_infinity()const=0;
                virtual bool is_height_infinityx()const=0;
                virtual int calc_fit_width(const extent_type& extent)const=0;
                virtual int calc_fit_height(const extent_type& extent)const=0;
                virtual void set_physical_bounds(const bounds_type&)=0;
                virtual bool accept( detail::abstract_visitor_holder*, bool reverse_layer )=0;
    
        protected:
                panel_imp(){set_align(align_center);extent_=extent_type(fit(),fit());}
                panel_imp(const panel_imp& x) : align_(x.align_),extent_(x.extent_){}

        private:
                align_type  align_;
                extent_type extent_;
        
        };

public:
        panel(){}
        panel(const panel& x) : imp(x.imp){}

        bool empty() { return !imp; }
        void clear() { if( imp ) { imp->clear(); } }
        void add( const panel& x ) { assert( imp ); imp->add( x.get_entity() ); }

        void layout(const bounds_type& b)
        {
                if( !imp ) { return; }
                imp->set_physical_bounds(b);
        }

        template <class V>
        bool accept( V& v, bool reverse_layer = false )
        {
                if( !imp ) { return false; }

                detail::concrete_visitor_holder<V> vh(v);
                bool called = imp->accept( &vh, reverse_layer );

                return called;
        }

        void set_extent( const extent_type& e ) { get_entity()->set_logical_extent( e ); }

        extent_type get_extent( const extent_type& e )
        {
                return extent_type( get_entity()->calc_physical_width( e ),
                                    get_entity()->calc_physical_height( e ) );
        }

protected:
        panel(panel_imp* x) : imp(x){}
        panel(imp_ptr x) : imp(x){}

        void enunique()
        {
		if(!imp.unique()){
			extent_type	e=imp->get_logical_extent();
			align_type	a=imp->get_align();
			imp.reset(imp->clone());
			imp->set_logical_extent(e);
			imp->set_align(a);
		}
        }

        bool concrete_value(int n)
        {
                return
                        n!=fit() &&
                        n!=infinity() &&
                        n!=infinityx();
        }

        imp_ptr get_entity(){return imp;}
        imp_ptr get_entity()const{return imp;}
        imp_ptr friend_entity(const panel& x){return x.imp;}

private:
        imp_ptr imp;

};

/*============================================================================
 *
 * class frame
 *
 * 
 *
 *==========================================================================*/

template <class V1,class V2=void,class V3=void,class V4=void,class V5=void,class V6=void,class V7=void,class V8=void,class V9=void>
class frame : public panel {
protected:
        template <class T>
        class frame_imp : public panel_imp {
        public:
                frame_imp( T& t ) : t_( t ) { }
                frame_imp( T& t, const extent_type& e ) : t_( t ) { set_logical_extent( e ); }
                ~frame_imp(){}

                panel_imp*      clone(){return new frame_imp<T>(t_);}
                void            clear(){}

                bool is_width_infinity()const{return get_logical_width()==infinity();}
                bool is_width_infinityx()const{return get_logical_width()==infinityx();}
                bool is_height_infinity()const{return get_logical_height()==infinity();}
                bool is_height_infinityx()const{return get_logical_height()==infinityx();}
                int calc_fit_width(const extent_type&)const{return t_.width();}
                int calc_fit_height(const extent_type&)const{return t_.height();}
        
                void set_physical_bounds(const bounds_type& b)
                {
                        t_.bounds(b);
                }
                
                bool accept( detail::abstract_visitor_holder* h, bool reverse_layer )
                {
                        if(detail::accept_helper<V1>(t_,h))return true;
                        if(detail::accept_helper<V2>(t_,h))return true;
                        if(detail::accept_helper<V3>(t_,h))return true;
                        if(detail::accept_helper<V4>(t_,h))return true;
                        if(detail::accept_helper<V5>(t_,h))return true;
                        if(detail::accept_helper<V6>(t_,h))return true;
                        if(detail::accept_helper<V7>(t_,h))return true;
                        if(detail::accept_helper<V8>(t_,h))return true;
                        if(detail::accept_helper<V9>(t_,h))return true;
                        return false;
                }

        private:
                T& t_;

        };

public:
        template < class T >
        frame(T& x) : panel( new frame_imp< T >( x ) ) {}
        template < class T >
        frame(T& x, const extent_type& e ) : panel( new frame_imp<T>( x, e ) ) {}
        frame( const frame& x ) : panel( x ) {}
        ~frame(){}
        
        frame& operator = ( const frame& x ) { panel::operator = ( x ) ; return *this ; }
};

/*============================================================================
 *
 * class layer
 *
 * 
 *
 *==========================================================================*/

class layer : public panel {
public:
        class layer_imp : public panel_imp {
        private:
                typedef std::vector<imp_ptr> elements_type;
        
        public:
                layer_imp(){}
                layer_imp(const extent_type& e){set_logical_extent(e);}
                ~layer_imp(){}

                panel_imp*      clone(){return new layer_imp(*this);}
                void            clear(){elements_.clear();}

                bool is_width_infinity()const
                {
                        if(get_logical_width()==infinity()){ return true; }
                        if(get_logical_width()==fit()) {
                                for(elements_type::const_iterator i=elements_.begin(); i!=elements_.end(); ++i){
                                        if((*i)->is_width_infinity()) { return true; }
                                }
                        }
                        return false;
                }
                bool is_width_infinityx()const
                {
                        if(get_logical_width()==infinityx()){ return true; }
                        if(get_logical_width()==fit()) {
                                for(elements_type::const_iterator i=elements_.begin(); i!=elements_.end(); ++i){
                                        if((*i)->is_width_infinityx()) { return true; }
                                }
                        }
                        return false;
                }
                bool is_height_infinity()const
                {
                        if(get_logical_height()==infinity()){ return true; }
                        if(get_logical_height()==fit()) {
                                for(elements_type::const_iterator i=elements_.begin(); i!=elements_.end(); ++i){
                                        if((*i)->is_height_infinity()) { return true; }
                                }
                        }
                        return false;
                }
                bool is_height_infinityx()const
                {
                        if(get_logical_height()==infinityx()){ return true; }
                        if(get_logical_height()==fit()) {
                                for(elements_type::const_iterator i=elements_.begin(); i!=elements_.end(); ++i){
                                        if((*i)->is_height_infinityx()) { return true; }
                                }
                        }
                        return false;
                }
                int calc_fit_width(const extent_type& e)const
                {
                        assert(get_logical_width()==fit());

                        // 要素情報の取得
                        int box_width=0;
                        for(elements_type::const_iterator i=elements_.begin();
                            i!=elements_.end();
                            ++i){
                                // 子に無限要素がある場合、↓の
                                // calc_physical_width呼び出しで例外
                                box_width=std::max(box_width,(*i)->calc_physical_width(e));
                        }
            
                        return box_width;
                }
                int calc_fit_height(const extent_type& e)const
                {
                        assert(get_logical_height()==fit());

                        // 要素情報の取得
                        int box_height=0;
                        for(elements_type::const_iterator i=elements_.begin();
                            i!=elements_.end();
                            ++i){
                                // 子に無限要素がある場合、↓の
                                // calc_physical_height呼び出しで例外
                                box_height=std::max(box_height,(*i)->calc_physical_height(e));
                        }
            
                        return box_height;
                }

                void set_physical_bounds(const bounds_type& b)
                {
                        extent_type e(b.extent());

                        // まず大きさを測る
                        int cw=get_logical_width();
                        int ch=get_logical_height();
                        if(cw==infinityx() || cw==infinity()){cw=e.w();}
                        if(ch==infinityx() || ch==infinity()){ch=e.h();}

                        for( elements_type::const_iterator i = elements_.begin();
                             i != elements_.end();
                             ++i){
                                imp_ptr p = *i;

                                int iw=p->calc_physical_width(e);
                                int ih=p->calc_physical_height(e);
                                if(iw==infinityx()){iw=e.w();}
                                if(iw==infinity()){
                                        if(cw==fit()){iw=0;}else{iw=e.w();}
                                }
                                if(ih==infinityx()){ih=e.h();}
                                if(ih==infinity()){
                                        if(ch==fit()){ih=0;}else{ih=e.h();}
                                }
                                cw=std::max(cw,iw);
                                ch=std::max(ch,ih);
                        }

                        bounds_type b2(0,0,cw,ch);
                        b2.center(b.center());
                        e.w(cw);
                        e.h(ch);

                        // 配置
                        for( elements_type::const_iterator i = elements_.begin();
                             i != elements_.end();
                             ++i){
                                imp_ptr p = *i;

                                int iw=p->calc_physical_width(e);
                                int ih=p->calc_physical_height(e);
                                if(iw==infinityx()){iw=e.w();}
                                if(iw==infinity()){
                                        if(cw==fit()){iw=0;}else{iw=e.w();}
                                }
                                if(ih==infinityx()){ih=e.h();}
                                if(ih==infinity()){
                                        if(ch==fit()){ih=0;}else{ih=e.h();}
                                }

                                align_type  align   =get_align();
                                int x=b2.hcenter()-(iw>>1);
                                int y=b2.vcenter()-(ih>>1);

                                switch(align){
                                case align_top:
                                        y=b2.y0();
                                        break;
                                case align_bottom:
                                        y=b2.y1()-ih;
                                        break;
                                case align_left:
                                        x=b2.x0();
                                        break;
                                case align_right:
                                        x=b2.x1()-iw;
                                        break;
                                default:
                                        break;
                                };

                                p->set_physical_bounds(bounds_type(x,y,x+iw,y+ih));
                        }
                }

                bool accept( detail::abstract_visitor_holder* vh, bool reverse_layer )
                {
                        bool f = false;
                        if( !reverse_layer ) {
                                for( elements_type::const_iterator i = elements_.begin();
                                     i != elements_.end();
                                     ++i){
                                        if( ( *i )->accept( vh, reverse_layer ) ){
                                                f = true;
                                        }
                                }
                        } else {
                                for( elements_type::const_reverse_iterator i = elements_.rbegin();
                                     i != elements_.rend();
                                     ++i ){
                                        if( ( *i )->accept( vh, reverse_layer ) ){
                                                f = true;
                                        }
                                }
                        }
                        return f;
                }

                void add(imp_ptr x)
                {
                        if( !x ) { throw layout_exception( "no instance" ); }
                        elements_.push_back(x);
                }

        protected:
                layer_imp(const layer_imp& x) : panel_imp(x), elements_(x.elements_){}
        
        protected:
                elements_type   elements_;

        };
    
public:
        layer() : panel(new layer_imp){}
        layer(const extent_type& e) : panel(new layer_imp(e)){}
        layer(int w,int h) : panel(new layer_imp(extent_type(w,h))){}
        ~layer(){}

        layer& operator<<(const panel& x)
        {
                enunique();
                get_entity()->add(friend_entity(x));
                return *this;
        }

protected:
        layer(const layer& x) : panel(x){}
        layer& operator=(const layer& x){panel::operator=(x);return *this;}

};

/*============================================================================
 *
 * class vbox_base
 *
 * 
 *
 *==========================================================================*/

class vbox_base : public panel {
public:
        class vbox_base_imp : public panel_imp {
        protected:
                typedef std::vector<imp_ptr> elements_type;

        public:
                vbox_base_imp(){}
                vbox_base_imp(const extent_type& e){set_logical_extent(e);}
                ~vbox_base_imp(){}

                panel_imp*      clone(){return new vbox_base_imp(*this);}
                void            clear(){elements_.clear();}

                bool is_width_infinity()const
                {
                        if(get_logical_width()==infinity()){ return true; }
                        if(get_logical_width()==fit()) {
                                for(elements_type::const_iterator i=elements_.begin(); i!=elements_.end(); ++i){
                                        if((*i)->is_width_infinity()) { return true; }
                                }
                        }
                        return false;
                }
                bool is_width_infinityx()const
                {
                        if(get_logical_width()==infinityx()){ return true; }
                        if(get_logical_width()==fit()) {
                                for(elements_type::const_iterator i=elements_.begin(); i!=elements_.end(); ++i){
                                        if((*i)->is_width_infinityx()) { return true; }
                                }
                        }
                        return false;
                }
                bool is_height_infinity()const
                {
                        if(get_logical_height()==infinity()){ return true; }
                        if(get_logical_height()==fit()) {
                                for(elements_type::const_iterator i=elements_.begin(); i!=elements_.end(); ++i){
                                        if((*i)->is_height_infinity()) { return true; }
                                }
                        }
                        return false;
                }
                bool is_height_infinityx()const
                {
                        if(get_logical_height()==infinityx()){ return true; }
                        if(get_logical_height()==fit()) {
                                for(elements_type::const_iterator i=elements_.begin(); i!=elements_.end(); ++i){
                                        if((*i)->is_height_infinityx()) { return true; }
                                }
                        }
                        return false;
                }
                int calc_fit_width(const extent_type& e)const
                {
                        assert(get_logical_width()==fit());

                        // 要素情報の取得
                        int box_width=0; // 非infinite要素の高さの最大
                        for( elements_type::const_iterator i = elements_.begin();
                             i != elements_.end();
                             ++i){
                                // 子に無限要素がある場合、↓の
                                // calc_physical_width呼び出しで例外
                                box_width = std::max(box_width,(*i)->calc_physical_width(e));
                        }
            
                        return box_width;
                }
                int calc_fit_height(const extent_type& e)const
                {
                        assert(get_logical_height()==fit());

                        // 要素情報の取得
                        int box_height=0; // 非infinite要素の高さの合計
                        for( elements_type::const_iterator i = elements_.begin();
                             i != elements_.end();
                             ++i){
                                // 子に無限要素がある場合、↓の
                                // calc_physical_height呼び出しで例外
                                box_height += (*i)->calc_physical_height(e);
                        }
            
                        return box_height;
                }

                void set_physical_bounds(const bounds_type& b)
                {
                        // 無限に関する情報の取得
			int ph	=calc_physical_height(b.extent());
                        int rh  =ph;
                        int inf =0;
                        int infx=0;
                        for( elements_type::const_iterator i = elements_.begin();
                             i != elements_.end();
                             ++i){
                                if((*i)->is_height_infinityx()){infx++;}
                                else if((*i)->is_height_infinity()){inf++;}
                                else {
                                        rh-=(*i)->calc_physical_height(extent_type(b.width(),fit()));
                                }
                        }

                        // 無限大の高さのアイテムの実際の高さを算出
                        int y=b.vcenter()-ph/2;
                        int inf_height  =0;
                        int infx_height =0;
                        if(0<infx){
                                infx_height=rh/infx;
                        } else if(0<inf){
                                inf_height=rh/inf;
                        }

                        // 配置
                        for( elements_type::const_iterator i = elements_.begin();
                             i != elements_.end();
                             ++i){
                                int iw=(*i)->calc_physical_width(b.extent());
                                int ih;
                                if((*i)->is_height_infinityx()){ih=infx_height;}
                                else if((*i)->is_height_infinity()){ih=inf_height;}
                                else {
                                        ih=(*i)->calc_physical_height(extent_type(b.width(),fit()));
                                }

                                // アライン
                                int         x;
                                align_type  align   =get_align();
                                switch(align){
                                case align_left:  x = b.x0();           break;
                                case align_right: x = b.x1()-iw;        break;
                                default:          x = b.hcenter()-iw/2; break;
                                };

                                // 配置
                                (*i)->set_physical_bounds(bounds_type(x,y,x+iw,y+ih));

                                // ループ更新処理
                                y+=ih;
                        }

                }

                bool accept( detail::abstract_visitor_holder* vh, bool reverse_layer )
                {
                        bool f = false;
                        for( elements_type::const_iterator i = elements_.begin();
                             i != elements_.end();
                             ++i){
                                if( ( *i )->accept( vh, reverse_layer ) ){
                                        f = true;
                                }
                        }
                        return f;
                }

                void add(imp_ptr x)
                {
                        if( !x ) { throw layout_exception( "no instance" ); }
                        elements_.push_back(x);
                }

        protected:
                elements_type    elements_;

        };
    
public:
        ~vbox_base(){}
    
protected:
        vbox_base() : panel(new vbox_base_imp){}
        vbox_base(panel_imp* x) : panel(x){}
        vbox_base(imp_ptr x) : panel(x){}
        vbox_base(const extent_type& e) : panel(new vbox_base_imp(e)){}
        vbox_base(const vbox_base& x) : panel(x){}
        vbox_base& operator=(const vbox_base& x)
        {
                panel::operator=(x);return *this;
        }
    
};

/*============================================================================
 *
 * class hbox_base
 *
 * 
 *
 *==========================================================================*/

class hbox_base : public panel {
public:
        class hbox_base_imp : public panel_imp {
        protected:
                typedef std::vector<imp_ptr> elements_type;

        public:
                hbox_base_imp(){}
                hbox_base_imp(const extent_type& e){set_logical_extent(e);}
                ~hbox_base_imp(){}

                panel_imp*      clone(){return new hbox_base_imp(*this);}
                void            clear(){elements_.clear();}

                bool is_width_infinity()const
                {
                        if(get_logical_width()==infinity()){ return true; }
                        if(get_logical_width()==fit()) {
                                for(elements_type::const_iterator i=elements_.begin(); i!=elements_.end(); ++i){
                                        if((*i)->is_width_infinity()) { return true; }
                                }
                        }
                        return false;
                }
                bool is_width_infinityx()const
                {
                        if(get_logical_width()==infinityx()){ return true; }
                        if(get_logical_width()==fit()) {
                                for(elements_type::const_iterator i=elements_.begin(); i!=elements_.end(); ++i){
                                        if((*i)->is_width_infinityx()) { return true; }
                                }
                        }
                        return false;
                }
                bool is_height_infinity()const
                {
                        if(get_logical_height()==infinity()){ return true; }
                        if(get_logical_height()==fit()) {
                                for(elements_type::const_iterator i=elements_.begin(); i!=elements_.end(); ++i){
                                        if((*i)->is_height_infinity()) { return true; }
                                }
                        }
                        return false;
                }
                bool is_height_infinityx()const
                {
                        if(get_logical_height()==infinityx()){ return true; }
                        if(get_logical_height()==fit()) {
                                for(elements_type::const_iterator i=elements_.begin(); i!=elements_.end(); ++i){
                                        if((*i)->is_height_infinityx()) { return true; }
                                }
                        }
                        return false;
                }
                int calc_fit_width(const extent_type& e)const
                {
                        assert(get_logical_width()==fit());

                        // 要素情報の取得
                        int box_width=0; // 非infinite要素の高さの合計
                        for( elements_type::const_iterator i = elements_.begin();
                             i != elements_.end();
                             ++i){
                                // 子に無限要素がある場合、↓の
                                // calc_physical_width呼び出しで例外
                                box_width += (*i)->calc_physical_width(e);
                        }
            
                        return box_width;
                }
                int calc_fit_height(const extent_type& e)const
                {
                        assert(get_logical_height()==fit());

                        // 要素情報の取得
                        int box_height=0; // 非infinite要素の高さの合計
                        for( elements_type::const_iterator i = elements_.begin();
                             i != elements_.end();
                             ++i){
                                // 子に無限要素がある場合、↓の
                                // calc_physical_height呼び出しで例外
                                box_height = std::max(box_height,(*i)->calc_physical_height(e));
                        }
            
                        return box_height;
                }

                void set_physical_bounds(const bounds_type& b)
                {
                        // 無限に関する情報の取得
			int pw	=calc_physical_width(b.extent());
                        int rw  =pw;
                        int inf =0;
                        int infx=0;
                        for( elements_type::const_iterator i = elements_.begin();
                             i != elements_.end();
                             ++i){
                                if((*i)->is_width_infinityx()){infx++;}
                                else if((*i)->is_width_infinity()){inf++;}
                                else {
                                        rw-=(*i)->calc_physical_width(extent_type(fit(),b.height()));
                                }
                        }

                        // 無限大の幅のアイテムの実際の幅を算出
                        int x=b.hcenter()-pw/2;
                        int inf_width=0;
                        int infx_width=0;
                        if(0<infx){
                                infx_width=rw/infx;
                        } else if(0<inf){
                                inf_width=rw/inf;
                        }

                        // 配置
                        for( elements_type::const_iterator i = elements_.begin();
                             i != elements_.end();
                             ++i){
                                int iw;
                                int ih=(*i)->calc_physical_height(b.extent());
                                if((*i)->is_width_infinityx()){iw=infx_width;}
                                else if((*i)->is_width_infinity()){iw=inf_width;}
                                else {
                                        iw=(*i)->calc_physical_width(extent_type(fit(),b.height()));
                                }

                                // アライン
                                int         y;
                                align_type  align   =get_align();
                                switch(align){
                                case align_top:         y=b.y0();               break;
                                case align_bottom:      y=b.y1()-ih;            break;
                                default:                y=b.vcenter()-ih/2;     break;
                                };

                                // 配置
                                (*i)->set_physical_bounds(bounds_type(x,y,x+iw,y+ih));

                                // ループ更新処理
                                x+=iw;
                        }

                }

                bool accept( detail::abstract_visitor_holder* vh, bool reverse_layer )
                {
                        bool f = false;
                        for( elements_type::const_iterator i = elements_.begin();
                             i != elements_.end();
                             ++i){
                                if( ( *i )->accept( vh, reverse_layer ) ){
                                        f = true;
                                }
                        }
                        return f;
                }

                void add(imp_ptr x)
                {
                        if( !x ) { throw layout_exception( "no instance" ); }
                        elements_.push_back(x);
                }

        protected:
                hbox_base_imp(const hbox_base_imp& x)
                        : panel_imp(x),elements_(x.elements_){}
        
        protected:
                elements_type elements_;

        };
    
public:
        ~hbox_base(){}
    
protected:
        hbox_base() : panel(new hbox_base_imp){}
        hbox_base(hbox_base_imp* x) : panel(x){}
        hbox_base(imp_ptr x) : panel(x){}
        hbox_base(const extent_type& e) : panel(new hbox_base_imp(e)){}
        hbox_base(const hbox_base& x) : panel(x){}
        hbox_base& operator=(const hbox_base& x)
        {
                panel::operator=(x);return *this;
        }
    
};

/*============================================================================
 *
 * class vbox
 *
 * 
 *
 *==========================================================================*/

class vbox : public vbox_base {
public:
        vbox(){}
        vbox(const extent_type& e) : vbox_base(e){}
        vbox(int w,int h) : vbox_base(extent_type(w,h)){}
        ~vbox(){}

        vbox& operator<<(const panel& x)
        {
                enunique();
                get_entity()->add(friend_entity(x));
                return *this;
        }

};

/*============================================================================
 *
 * class vfillbox
 *
 * 
 *
 *==========================================================================*/

class vfillbox : public vbox_base {
public:
        vfillbox() : vbox_base(extent_type(fit(),infinity())){} 
        ~vfillbox(){}

        vfillbox& operator<<(const panel& x)
        {
                enunique();
                get_entity()->add(friend_entity(x));
                return *this;
        }

};

/*============================================================================
 *
 * class hbox
 *
 * 
 *
 *==========================================================================*/

class hbox : public hbox_base {
public:
        hbox(){}
        hbox(const extent_type& e) : hbox_base(e){}
        hbox(int w,int h) : hbox_base(extent_type(w,h)){}
        ~hbox(){}

        hbox& operator<<(const panel& x)
        {
                enunique();
                get_entity()->add(friend_entity(x));
                return *this;
        }

};

/*============================================================================
 *
 * class hfillbox
 *
 * 
 *
 *==========================================================================*/

class hfillbox : public hbox_base {
public:
        hfillbox() : hbox_base(extent_type(infinity(),fit())){} 
        ~hfillbox(){}

        hfillbox& operator<<(const panel& x)
        {
                enunique();
                get_entity()->add(friend_entity(x));
                return *this;
        }

};

/*============================================================================
 *
 * class vspace
 *
 * 
 *
 *==========================================================================*/

class vspace: public panel {
protected:
        class vspace_imp : public panel_imp {
        public:
                vspace_imp(int b){set_logical_extent(extent_type(0,b));}
                ~vspace_imp(){}

                panel_imp*      clone(){return new vspace_imp(get_logical_height());}
                void            clear(){}

                bool is_width_infinity()const{return false;}
                bool is_width_infinityx()const{return false;}
                bool is_height_infinity()const{return get_logical_height()==infinity();}
                bool is_height_infinityx()const{return get_logical_height()==infinityx();}
                int calc_fit_width(const extent_type&)const{return 0;}
                int calc_fit_height(const extent_type&)const{return 0;}

                void set_physical_bounds(const bounds_type&){}

                bool accept( detail::abstract_visitor_holder*, bool ) { return false ; }

        private:

        };
    
public:
        vspace() : panel(new vspace_imp(0)){}
        explicit vspace(int b) : panel(new vspace_imp(b)){}
        ~vspace(){}

protected:
        vspace(const vspace&){}
        vspace& operator=(const vspace&){return *this;}
    
};

/*============================================================================
 *
 * class hspace
 *
 * 
 *
 *==========================================================================*/

class hspace : public panel  {
protected:
        class hspace_imp : public panel_imp {
        public:
                hspace_imp(int b){set_logical_extent(extent_type(b,0));}
                ~hspace_imp(){}

                panel_imp*      clone(){return new hspace_imp(get_logical_width());}
                void            clear(){}

                bool is_width_infinity()const{return get_logical_width()==infinity();}
                bool is_width_infinityx()const{return get_logical_width()==infinityx();}
                bool is_height_infinity()const{return false;}
                bool is_height_infinityx()const{return false;}
                int calc_fit_width(const extent_type&)const{return 0;}
                int calc_fit_height(const extent_type&)const{return 0;}

                void set_physical_bounds(const bounds_type&){}

                bool accept( detail::abstract_visitor_holder*, bool ){return false;}

        private:

        };
    
public:
        hspace() : panel(new hspace_imp(0)){}
        explicit hspace(int b) : panel(new hspace_imp(b)){}
        ~hspace(){}

protected:
        hspace(const hspace&){}
        hspace& operator=(const hspace&){return *this;}
    
};

/*============================================================================
 *
 * class vfill
 *
 * 
 *
 *==========================================================================*/

class vfill : public vspace  {
public:
        vfill() : vspace(infinity()){}
        ~vfill(){}
protected:
        vfill(const vfill&){}
        vfill& operator=(const vfill&){return *this;}
};

/*============================================================================
 *
 * class hfill
 *
 * 
 *
 *==========================================================================*/

class hfill : public hspace  {
public:
        hfill() : hspace(infinity()){}
        ~hfill(){}
protected:
        hfill(const hfill&){}
        hfill& operator=(const hfill&){return *this;}
};

/*============================================================================
 *
 * class vfillx
 *
 * 
 *
 *==========================================================================*/

class vfillx : public vspace  {
public:
        vfillx() : vspace(infinityx()){}
        ~vfillx(){}
protected:
        vfillx(const vfillx&){}
        vfillx& operator=(const vfillx&){return *this;}
};

/*============================================================================
 *
 * class hfillx
 *
 * 
 *
 *==========================================================================*/

class hfillx : public hspace  {
public:
        hfillx() : hspace(infinityx()){}
        ~hfillx(){}
protected:
        hfillx(const hfillx&){}
        hfillx& operator=(const hfillx&){return *this;}
    
};

#if 0

/*============================================================================
 *
 * class table_row
 *
 * 
 *
 *==========================================================================*/

class table_row : public hbox_base {
protected:
        class table_row_imp : public hbox_base_imp {
        public:
                typedef hbox_base_imp::elements_type elements_type;

        public:
                table_row_imp(){}
                table_row_imp(const extent_type& e) : hbox_base_imp(e){}
                ~table_row_imp(){}
        
                elements_type& elements() {return elements_;}
                
        };
    
public:
        table_row() : hbox_base(new table_row_imp){}
        table_row(const extent_type& e) : hbox_base(new table_row_imp(e)){}
        table_row(int w,int h) : hbox_base(new table_row_imp(extent_type(w,h))){}
        ~table_row(){}

        table_row& operator<<(const table_row& x)
        {
                enunique();
                get_entity()->add(friend_entity(x));
                return *this;
        }
    
protected:
        table_row(const table_row&){}
        table_row& operator=(const table_row&){return *this;}

        friend class table;

};

/*============================================================================
 *
 * class table
 *
 * 
 *
 *==========================================================================*/

class table : public vbox_base {
private:
        typedef table_row::table_row_imp table_row_imp;

public:
        class table_imp : public vbox_base_imp {
        public:
                typedef table::table_row_imp::elements_type row_elements_type;

        public:
                table_imp(){}
                ~table_imp(){}
        
                panel_imp*      clone(){return new table_imp(*this);}

                int calc_fit_width(const extent_type& e)const
                {
                        // column数の取得
                        int cols=int(get_column_count());
            
                        // 要素情報の取得
                        int box_width=0; // 非infinite要素の高さの合計
                        for(int i=0;i<cols;i++){
                                int n= get_column_fit_width( i, e );
                                if(n==infinityx()){
                                        return infinityx();
                                } else if(n==infinity()){
                                        n=0;
                                }
                                box_width+=n;
                        }
            
                        return box_width;
                }

                void set_physical_bounds(const bounds_type& b)
                {
                        // logical_col_widths, logical_row_heights取得
                        size_t cols=get_column_count();
                        size_t rows=elements_.size();
                        std::vector<int> logical_col_widths;
                        std::vector<int> logical_row_heights;
                        for(size_t x=0;x<cols;x++){
                                logical_col_widths.push_back(get_column_logical_width(int(x)));
                        }
                        for(size_t y=0;y<rows;y++){
                                logical_row_heights.push_back(elements_[y]->get_row_logical_height());
                        }

                        // infinite情報取得
                        int cinfx=0;
                        int cinf =0;
                        for(size_t x=0;x<cols;x++){
                                int n=logical_col_widths[x];
                                if(n==infinityx()){
                                        cinfx++;
                                } else if(n==infinity()){
                                        cinf++;
                                }
                        }

                        int rinfx=0;
                        int rinf =0;
                        for(size_t y=0;y<rows;y++){
                                int n=logical_row_heights[x];
                                if(n==infinityx()){
                                        rinfx++;
                                } else if(n==infinity()){
                                        rinf++;
                                }
                        }

                        // 最小サイズの取得
                        int fitw=0;
                        int fith=0;
                        for(size_t x=0;x<cols;x++){
                                fitw+=get_column_fit_width(x);
                        }
                        for(size_t y=0;y<rows;y++){
                                fith+=elements_[y]->calc_fit_height(extent_type(fit(),fit()));
                        }

                        int rw=b.w()-fitw;
                        int rh=b.h()-fith;

                        // 配置
                        std::vector<int> physical_col_widths;
                        std::vector<int> physical_row_heights;
                        for(size_t x=0;x<cols;x++){
                                int n=get_column_logical_width(x);
                                if(n==infinityx()){
                                        n=rw/cinfx;
                                } else if(n==infinity()){
                                        if(cinfx){ n=0; } else { n=rw/cinf; }
                                }
                                physical_col_widths.push_back(n);                                
                        }

                        for(size_t y=0;y<rows;y++){
                                int n=elements_[y]->get_row_logical_height(x);
                                if(n==infinityx()){
                                        n=rh/rinfx;
                                } else if(n==infinity()){
                                        if(rinfx){ n=0; } else { n=rh/rinf; }
                                }
                                physical_row_heights.push_back(n);
                        }

                        // 設定
                        for(size_t y=0;y<elements_.size();y++){
                                for(size_t x=0;x<cols;x++){
                                }
                        }
                }
        
                bool accept( detail::abstract_visitor_holder* vh, bool reverse_layer )
                {
                        bool f = false;
                        for(size_t y=0;y<elements_.size();y++){
                                row_elements_type& row_elements=
                                        boost::polymorphic_downcast<table_row_imp*>(
                                                elements_[y].get())->elements();
                                
                                for(size_t x=0;x<row_elements.size();x++){
                                        if( f || row_elements[ x ]->accept( vh, reverse_layer ) ){
                                                f = true;
                                        }
                                }
                        }
                        return f;
                }

        protected:
                size_t get_column_count()const
                {
                        size_t m=0;
                        for(size_t y=0;y<elements_.size();y++){
                                row_elements_type& row_elements=
                                        boost::polymorphic_downcast<table_row_imp*>(
                                                elements_[y].get())->elements();
                                
                                m=std::max(m,row_elements.size());
                        }
                        return m;
                }

                int get_column_logical_width(int index)const
                {
                        int m=0;
                        for(size_t y=0;y<elements_.size();y++){
                                // row data
                                row_elements_type& row_elements=
                                        boost::polymorphic_downcast<table_row_imp*>(
                                                elements_[y].get())->elements();

                                if(index<row_elements.size()){
                                        m=zfx::max(m,row_elements[index]->get_logical_width());
                                }
                        }
                        return m;
                }

                int get_column_fit_width(int index,const extent_type& e)const
                {
                        int m=0;
                        for(size_t y=0;y<elements_.size();y++){

                                // row data
                                row_elements_type& row_elements=
                                        boost::polymorphic_downcast<table_row_imp*>(
                                                elements_[y].get())->elements();

                                m=zfx::max(m,elements[index]->calc_fit_width(extent_type(fit(),fit())));
                        }
            
                        return m;
                }

        protected:
                table_imp(const table_imp& x) : vbox_base_imp(x){}

        private:
        
        };

public:
        table() : vbox_base(new table_imp){}
        table(const extent_type& e)
                : vbox_base(new table_imp(e)){}
        ~table(){}

        table& operator<<(const table_row& x)
        {
                enunique();
                get_entity()->add(friend_entity(x));
                return *this;
        }
    
protected:
        table(const table&){}
        table& operator=(const table&){return *this;}
    
};

#endif

}

}



#endif // LAYOUT_HPP
