/*!
  @file     visitor.hpp
  @brief    <ŠT—v>

  <à–¾>
  $Id: visitor.hpp 16 2008-03-28 14:58:11Z Naoyuki.Hirayama $
*/
#ifndef VISITOR_HPP
#define VISITOR_HPP

namespace zw {

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


class abstract_acceptor {
protected:
        class abstract_acceptor_implement {
        public:
                virtual ~abstract_acceptor_implement(){}

                virtual bool accept(detail::abstract_visitor_holder*)=0;
        };

public:
        ~abstract_acceptor(){delete imp;}

        template <class V>
        bool accept(V& v)
        {
                bool called;

                detail::abstract_visitor_holder* vh =
                        new detail::concrete_visitor_holder<V>(v);
                called = imp->accept(vh);
                delete vh;

                return called;
        }

protected:
        void overwrite(abstract_acceptor_implement* i)
        {
                delete imp;
                imp=i;
        }

        abstract_acceptor(abstract_acceptor_implement* i) : imp(i){}
        abstract_acceptor_implement* imp;

};

template <class V>
class accept_helper {
public:
        template <class T,class AH>
        accept_helper(T& t,AH* ah)
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

} // namespace detail

template <class T,class V1,class V2=void,class V3=void,class V4=void,class V5=void,class V6=void,class V7=void,class V8=void,class V9=void>
class basic_acceptor : public detail::abstract_acceptor {
private:
        class basic_acceptor_implement : public abstract_acceptor_implement {
        public:
                basic_acceptor_implement(T& t) : t_(t){}
                ~basic_acceptor_implement(){}

                bool accept(detail::abstract_visitor_holder* h)
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

                T& t_;
        };

public:
        basic_acceptor(T& t) : detail::abstract_acceptor(new basic_acceptor_implement(t)){}
        ~basic_acceptor(){}
};

} // namespace zw

#endif // VISITOR_HPP
