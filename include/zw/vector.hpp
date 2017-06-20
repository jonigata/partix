#if !defined(ZFX_VECT2D_HPP)
#define ZFX_VECT2D_HPP

#include "config.hpp"

namespace zw {

template <class T>
void swap(T& x,T& y)
{
    T z=x;
    x=y;
    y=z;
}

#if defined(ZW_HAVE_VCSTYLE_PACK)
#pragma pack(push,1)
#endif
    
template <class T>
class basic_vector2 {
public:
        typedef T                       coord_type;
	typedef basic_vector2<T>	self_type;
    
public:
        coord_type x;
	coord_type y;

	basic_vector2()                        {}
	basic_vector2(const T& ax,const T& ay) {x=ax;y=ay;}

        self_type&  operator=(const self_type& v){x=v.x;y=v.y;return *this;}

        self_type   operator-()const{return self_type(-x,-y);}
        self_type&  operator+=(const self_type& r){x+=r.x;y+=r.y;return *this;}
        self_type&  operator-=(const self_type& r){x-=r.x;y-=r.y;return *this;}
        self_type&  operator*=(const coord_type& n){x*=n;y*=n;return *this;}
        self_type&  operator/=(const coord_type& n){x/=n;y/=n;return *this;}

        coord_type  length()const{return coord_type(::hypot(x,y));}
};

template <class T>
basic_vector2<T> operator+(const basic_vector2<T>& l,
                           const basic_vector2<T>& r)
{
    return basic_vector2<T>(l.x+r.x,l.y+r.y);
}

template <class T>
basic_vector2<T> operator-(const basic_vector2<T>& l,
                           const basic_vector2<T>& r)
{
    return basic_vector2<T>(l.x-r.x,l.y-r.y);
}

template <class T>
basic_vector2<T> operator+(const basic_vector2<T>& l,
                           const T&                r)
{
    return basic_vector2<T>(l.x+r,l.y+r);
}

template <class T>
basic_vector2<T> operator-(const basic_vector2<T>& l,
                           const T&                r)
{
    return basic_vector2<T>(l.x-r,l.y-r);
}

template <class T>
basic_vector2<T> operator*(const basic_vector2<T>& l,
                           const T&                r)
{
    return basic_vector2<T>(l.x*r,l.y*r);
}

template <class T>
basic_vector2<T> operator/(const basic_vector2<T>& l,
                           const T&                r)
{
    return basic_vector2<T>(l.x/r,l.y/r);
}

template <class T>
bool operator==(const basic_vector2<T>& l,
                const basic_vector2<T>& r)
{
    return l.x==r.x && l.y==r.y;
}

template <class T>
bool operator!=(const basic_vector2<T>& l,
                const basic_vector2<T>& r)
{
    return !operator==(r);
}
    
// offset
template <class V>
class basic_offset {
public:
        typedef V                       vector_type;
        typedef typename V::coord_type  coord_type;
        typedef basic_offset<V>         self_type;

        basic_offset(){}
        basic_offset(const coord_type& x,const coord_type& y) : v(x,y){}
        explicit basic_offset(const vector_type& av) : v(av){}
        basic_offset(const self_type& o) : v(o.v){}
        ~basic_offset(){}

	self_type&  operator=(const vector_type& av){v=av;return *this;}
        self_type&  operator=(const self_type& o){v=o.v;return *this;}

        operator bool()const{return v.x!=0 || v.y!=0;}
        bool operator!()const{return !bool(*this);}

        self_type&  operator+=(const self_type& o){v+=o;return *this;}
        self_type&  operator-=(const self_type& o){v-=o;return *this;}
        self_type&  operator+=(const coord_type& n) {v+=o;return *this;}
        self_type&  operator-=(const coord_type& n) {v-=o;return *this;}
        self_type&  operator*=(const coord_type& n) {v*=n;return *this;}
        self_type&  operator/=(const coord_type& n) {v/=n;return *this;}

        self_type   operator-()const{return self_type(-v);}
        self_type   operator+(const self_type& o)const
        {return self_type(v+o.v);}
        self_type   operator-(const self_type& o)const
        {return self_type(v-o.v);}
        self_type   operator+(const coord_type& n)const
        {return self_type(v+n);}
        self_type   operator-(const coord_type& n)const
        {return self_type(v-n);}
        self_type   operator*(const coord_type& n)const {return self_type(v*n);}
        self_type   operator/(const coord_type& n)const {return self_type(v/n);}

        bool operator==(const self_type& x)const {return v==x.v;}
        bool operator!=(const self_type& x)const {return !operator==(x);}

        operator vector_type()const             {return v;}

        void clear(){v.x=0;v.y=0;}

        coord_type  length()const               {return v.length();}

        coord_type  dx()const                   {return v.x;}
        coord_type  dy()const                   {return v.y;}
        void        dx(const coord_type& ax)    {v.x=ax;}
        void        dy(const coord_type& ay)    {v.y=ay;}

private:
        V   v;
    
};

// extent
template <class V>
class  basic_extent {
public:
        typedef V                       vector_type;
        typedef typename V::coord_type  coord_type;
        typedef basic_extent<V>         self_type;
        typedef basic_offset<V>         offset_type;

        basic_extent(){}
        basic_extent(const coord_type& w,const coord_type& h) : v(w,h){}
        explicit basic_extent(const vector_type& av) : v(av){}
        basic_extent(const self_type& e) : v(e.v){}
        ~basic_extent(){}

        operator bool()const{return v.x!=0 || v.y!=0;}
        bool operator!()const{return !bool(*this);}

        self_type&  operator=(const vector_type& av){v=av;return *this;}
        self_type&  operator=(const self_type& e)   {v=e.v;return *this;}

        self_type&  operator+=(const self_type& e)  {v+=e;return *this;}
        self_type&  operator-=(const self_type& e)  {v-=e;return *this;}
        self_type&  operator*=(const self_type& e)  {v*=e;return *this;}
        self_type&  operator/=(const self_type& e)  {v/=e;return *this;}

        self_type&  operator+=(const coord_type& n) {v+=n;return *this;}
        self_type&  operator-=(const coord_type& n) {v+=n;return *this;}
        self_type&  operator*=(const coord_type& n) {v*=n;return *this;}
        self_type&  operator/=(const coord_type& n) {v/=n;return *this;}

        self_type   operator-()const                    {return self_type(-v);}

        self_type   operator+(const self_type& o)const  {return self_type(v+o.v);}
        self_type   operator-(const self_type& o)const  {return self_type(v-o.v);}
        self_type   operator*(const self_type& o)const
        {return self_type(v.x*o.v.x,v.y*o.v.y);}
        self_type   operator/(const self_type& o)const
        {return self_type(v.x/o.v.x,v.y/o.v.y);}

        self_type   operator+(const coord_type& n)const {return self_type(v+n);}
        self_type   operator-(const coord_type& n)const {return self_type(v-n);}
        self_type   operator*(const coord_type& n)const {return self_type(v*n);}
        self_type   operator/(const coord_type& n)const {return self_type(v/n);}

        bool operator==(const self_type& x)const {return v==x.v;}
        bool operator!=(const self_type& x)const {return !operator==(x);}

        operator vector_type()const{return v;}

        coord_type  width()const                    {return v.x;}
        coord_type  height()const                   {return v.y;}
        void        width(const coord_type& ax)     {v.x=ax;}
        void        height(const coord_type& ay)    {v.y=ay;}

        coord_type  w()const                    {return v.x;}
        coord_type  h()const                    {return v.y;}
        void        w(const coord_type& ax)     {v.x=ax;}
        void        h(const coord_type& ay)     {v.y=ay;}

        void clear(){v.x=0;v.y=0;}
	bool empty()const{return v.x==0 || v.y==0;}

        void        normalize()
        {
                v.x=abs(v.x);
                v.y=abs(v.y);
        }

        self_type   inner_largest/*_similar_extent*/(self_type ie)const
        {
                ie.normalize();
                self_type re;
                self_type oe=*this;oe.normalize();

                re.width(ie.width()*oe.height()/ie.height());
                re.height(oe.height());
                if(oe.width()<re.width()){
                        re.width(oe.width());
                        re.height(ie.height()*oe.width()/ie.width());
                }
                return re;
        }

        self_type   outer_smallest/*_similar_extent*/(self_type oe)const
        {
                oe.normalize();
                self_type re=*this;
                self_type ie=*this;ie.normalize();
        
                re.width(oe.width()*ie.height()/oe.height());
                re.height(ie.height());
                if(re.width()<ie.width()){
                        re.width(ie.width());
                        re.height(oe.height()*ie.width()/oe.width());
                }
                return re;
        }

        offset_type     offset()const               {return offset_type(v);}

private:
        V   v;
    
};

// bounds
template <class V>
class basic_bounds {
public:
        typedef V                       vector_type;
        typedef typename V::coord_type  coord_type;
        typedef basic_offset<V>         offset_type;
        typedef basic_extent<V>         extent_type;
        typedef basic_bounds<V>         self_type;

public:
        basic_bounds(){}
    
        basic_bounds(
                const coord_type& x0,const coord_type& y0,
                const coord_type& x1,const coord_type& y1)
                : v0(x0,y0),v1(x1,y1){}
    
        basic_bounds(
                const vector_type& av0,
                const vector_type& av1)
                : v0(av0),v1(av1){}
    
        basic_bounds(
                const offset_type& o,
                const extent_type& e)
                : v0(o),v1(vector_type(o)+vector_type(e)){}
    
        explicit basic_bounds(const vector_type& v) : v0(0,0),v1(v){}

        explicit basic_bounds(const extent_type& e) : v0(0,0),v1(e){}
    
        basic_bounds(const self_type& r)
                : v0(r.v0),v1(r.v1){}
    
        ~basic_bounds(){}
    
        operator bool()const{return v0.x!=0 || v0.y!=0 || v1.x!=0 || v1.y!=0;}
        bool operator!()const{return !bool(*this);}
    
        self_type&  operator=(const self_type& e)   {v0=e.v0;v1=e.v1;return *this;}
    
        self_type&  operator+=(const offset_type& o)
        {v0+=vector_type(o);v1+=vector_type(o);return *this;}
        self_type&  operator-=(const offset_type& o)
        {v0-=vector_type(o);v1-=vector_type(o);return *this;}
        self_type&  operator+=(const extent_type& e)
        {v1+=vector_type(e);return *this;}
        self_type&  operator-=(const extent_type& e)
        {v1-=vector_type(e);return *this;}
        self_type&  operator*=(const coord_type& n) {v0*=n;v1*=n;return *this;}
        self_type&  operator/=(const coord_type& n) {v0/=n;v1/=n;return *this;}

        self_type   operator-()const
        {return self_type(-v1,-v0);}
        self_type   operator+(const offset_type& o)const
        {return self_type(v0+vector_type(o),v1+vector_type(o));}
        self_type   operator-(const offset_type& o)const
        {return self_type(v0-vector_type(o),v1-vector_type(o));}
        self_type   operator+(const extent_type& e)const
        {return self_type(v0,v1+vector_type(e));}
        self_type   operator-(const extent_type& e)const
        {return self_type(v0,v1-vector_type(e));}
        self_type   operator*(const coord_type& n)const
        {return self_type(v0*n,v1*n);}
        self_type   operator/(const coord_type& n)const
        {return self_type(v0/n,v1/n);}

        bool operator==(const self_type& x)const {return v0==x.v0 && v1==x.v1;}
        bool operator!=(const self_type& x)const {return !operator==(x);}
    
        offset_type  topleft()       const {return offset_type(v0);}
        offset_type  topright()      const {return offset_type(v1.x,v0.y);}
        offset_type  bottomleft()    const {return offset_type(v0.x,v1.y);}
        offset_type  bottomright()   const {return offset_type(v1);}
        void    topleft(const offset_type& p)    {move(p);}
        void    topright(const offset_type& p)   {move(p-horz());}
        void    bottomleft(const offset_type& p) {move(p-vert());}
        void    bottomright(const offset_type& p){move(p-diag());}
    
        coord_type  hcenter() const                 {return (v0.x+v1.x)/2;}
        coord_type  vcenter() const                 {return (v0.y+v1.y)/2;}
        void        hcenter(const coord_type& x)    {hslide(x-hcenter());}
        void        vcenter(const coord_type& y)    {vslide(y-vcenter());}
        void        hslide(const coord_type& dx)    {v0.x+=dx;v1.x+=dx;}
        void        vslide(const coord_type& dy)    {v0.y+=dy;v1.y+=dy;}

        offset_type  center()        const {return offset_type((v0+v1)/2);}
        offset_type  topcenter()     const {return offset_type(hcenter(),v0.y);}
        offset_type  bottomcenter()  const {return offset_type(hcenter(),v1.y);}
        offset_type  leftcenter()    const {return offset_type(v0.x,vcenter());}
        offset_type  rightcenter()   const {return offset_type(v1.x,vcenter());}
        void    center(const offset_type& p)         {move(p-diag()/2);}
        void    topcenter(const offset_type& p)      {move(p-horz()/2);}
        void    bottomcenter(const offset_type& p)   {move(p-horz()/2-vert());}
        void    leftcenter(const offset_type& p)     {move(p-vert()/2);}
        void    rightcenter(const offset_type& p)    {move(p-horz()-vert()/2);}

        extent_type extent()    const {return extent_type(v1-v0);}
        coord_type  width()     const {return (v1.x-v0.x);}
        coord_type  height()    const {return (v1.y-v0.y);}
        void    extent(const extent_type& e){v1=v0+vector_type(e);}
        void    width(const coord_type& w)  {v1.x=v0.x+w;}
        void    height(const coord_type& h) {v1.y=v0.y+h;}

        offset_type diag()  const {return offset_type(width(),height());}
        offset_type horz()  const {return offset_type(width(),0);}
        offset_type vert()  const {return offset_type(0,height());}
        void    diag(const offset_type& o)  {horz(o);vert(o);}
        void    horz(const offset_type& o)  {v1.x=v0.x+o.dx();}
        void    vert(const offset_type& o)  {v1.y=v0.y+o.dy();}

        coord_type  x0()    const {return v0.x;}
        coord_type  y0()    const {return v0.y;}
        void        x0(const coord_type& ax)    {v0.x=ax;}
        void        y0(const coord_type& ay)    {v0.y=ay;}

        coord_type  x1()    const {return v1.x;}
        coord_type  y1()    const {return v1.y;}
        void        x1(const coord_type& ax)    {v1.x=ax;}
        void        y1(const coord_type& ay)    {v1.y=ay;}

        coord_type  w()     const {return width();}
        coord_type  h()     const {return height();}

        coord_type  left()  const {return v0.x;}
        coord_type  top()   const {return v0.y;}
        void        left    (const coord_type& ax)    {v0.x=ax;}
        void        top     (const coord_type& ay)    {v0.y=ay;}

        coord_type  right()     const {return v1.x;}
        coord_type  bottom()    const {return v1.y;}
        void        right   (const coord_type& ax)    {v1.x=ax;}
        void        bottom  (const coord_type& ay)    {v1.y=ay;}

        void        clear(){v0.x=0;v0.y=0;v1.x=0;v1.y=0;}
	bool        empty()const
	{
		return width()==0 || height()==0;
	}

        void        normalize()
        {
                if(v1.x<v0.x){
                        swap(v1.x,v0.x);
                }
                if(v1.y<v0.y){
                        swap(v1.y,v0.y);
                }
        }

        void        stable_normalize()
        {
                if(v1.x<v0.x){
                        swap(v1.x,v0.x);
                        v0.x++;
                        v1.x++;
                }
                if(v1.y<v0.y){
                        swap(v1.y,v0.y);
                        v0.x++;
                        v1.x++;
                }
        }

	bool		normalized()const
	{
		return v0.x<v1.x && v0.y<v1.y;
	}

	self_type	intersect(const self_type& x)const
	{
		self_type y=*this;
		self_type z=x;
		y.normalize();
		z.normalize();

		self_type r(
			y.left()<z.left()	? z.left()	: y.left(),
			y.top()<z.top()		? z.top()	: y.top(),
			y.right()<z.right()	? y.right()	: z.right(),
			y.bottom()<z.bottom()	? y.bottom()	: z.bottom());

		if(r.normalized()){
			return r;
		} else {
			return self_type(0,0,0,0);
		}
	}

	self_type	unite(const self_type& x)const
	{
                if(!(*this)){
                        return x;
                } else if(!x){
                        return *this;
                }

		self_type y=*this;
		self_type z=x;
		y.normalize();
		z.normalize();

		return self_type(
			y.left()<z.left()	? y.left()	: z.left(),
			y.top()<z.top()		? y.top()	: z.top(),
			y.right()<z.right()	? z.right()	: y.right(),
			y.bottom()<z.bottom()	? z.bottom()	: y.bottom());
	}

	bool	contains(const offset_type& p)const
	{
		self_type z=*this;
		z.normalize();
		return 
			z.left()<=p.dx() && p.dx()<z.right() && 
			z.top()<=p.dy() && p.dy()<z.bottom();
	}

	bool	contains(self_type r)const
	{
		self_type z=*this;
		z.normalize();
		r.normalize();
		return 
			z.left()<=r.left() && r.left()<z.right() &&
			z.top()<=r.top() && r.top()<z.bottom() &&
			z.left()<=r.left() && r.right()<=z.right() &&
			z.top()<=r.bottom() && r.bottom()<=z.bottom();

	}

        void    reverse()
        {
                rvect2d_swap(v0,v1);
        }
        void    hreverse()
        {
                rvect2d_swap(v0.x,v1.x);
        }
        void    vreverse()
        {
                rvect2d_swap(v0.y,v1.y);
        }

	offset_type	clip(const offset_type& o)const
	{
		self_type r=*this;
		r.normalized();
		return offset_type(
			o.dx()<r.left()?r.left():(r.right ()<o.dx()?r.right ():o.dx()),
			o.dy()<r.top() ?r.top ():(r.bottom()<o.dy()?r.bottom():o.dy()));
	}

        self_type   inner_largest/*_similar_bounds*/(self_type ir)const
        {
                ir.normalize();
                self_type or=*this;or.normalize();
                self_type r(or.extent().inner_largest(ir.extent()));
                r.center(or.center());
                return r;
        }

        self_type   outer_smallest/*_similar_bounds*/(self_type or)const
        {
                or.normalize();
                self_type ir=*this;ir.normalize();
                self_type r(ir.extent().outer_smallest(or.extent()));
                r.center(ir.center());
                return r;
        }

protected:
        void    move(const offset_type& p)
        {
                offset_type o=diag();
                v0=vector_type(p);
                v1=vector_type(p+o);
        }
    
private:
        V   v0,v1;
    
};
    
#if defined(ZW_HAVE_VCSTYLE_PACK)
#pragma pack(pop)
#endif
    
} // namespace zw

#endif
