#if !defined(FIXED_POOL_HPP)
#define FIXED_POOL_HPP

#include <new>
#include <cassert>
#include "zw/dprintf.hpp"

#include "partix_debug.hpp"

#pragma pack(push,1)

// fixed_pool
//	 固定サイズオブジェクト用メモリプール

template < size_t Alignment, class PageProvider >
class fixed_pool { 
private:
    struct page_header { 
        page_header*	next;
    }; 

    struct formatted_tag { 
        formatted_tag*	next;
    }; 

public:
    fixed_pool(PageProvider& pp, const char* name)
        : page_provider_(pp), name_(name) {
        pages_head_ = nullptr;
        unused_pages_ = nullptr;

        unformatted_beginning_ = nullptr;
        unformatted_end_ = nullptr;
        formatted_ = nullptr;

        counter_ = 0;
    }
    ~fixed_pool() {
        page_header* p = pages_head_;
        while (p) {
            page_header* q = p->next;
            page_provider_.deallocate(p);
            p = q;
        }

        p = unused_pages_;
        while (p) {
            page_header* q = p->next;
            page_provider_.deallocate(p);
            p = q;
        }
    }

    void clear() {
        if (pages_head_) {
            append(unused_pages_, pages_head_);
            pages_head_ = nullptr;
        }

        unformatted_beginning_ = nullptr;
        unformatted_end_ = nullptr;
        formatted_ = nullptr;

        counter_ = 0;
    }

    void* allocate() {
        counter_++;
        if (formatted_) {
            // プールされている場合、それを返す
            void* x =(void*)formatted_;
            formatted_ = formatted_->next;
            return x;
        } else {
            // プールされていない場合、
            // 未フォーマット領域を削って返す
            int m = alignment();
            if (unformatted_end_ < unformatted_beginning_ + m) {
                require_new_page();
            }

            char* p = unformatted_beginning_;
            unformatted_beginning_ += m;
            return p;
        }
    }

    void deallocate(void* p) {
        counter_--;
        // 返されたものをプールする
        ((formatted_tag*)p)->next = formatted_;
        formatted_ =(formatted_tag*)p;
    }
		
protected:
    int alignment() const { // 最適化後は定数
        int min_alignment = sizeof(formatted_tag);
        return(Alignment +(min_alignment - 1))/
            min_alignment * min_alignment;
    }
		
    void require_new_page() {
        int m = alignment();
        int size = int(page_provider_.page_size() / m * m);

        page_header* new_page;
        if (unused_pages_) {
            // 1枚はがして持ってくる
            new_page = unused_pages_;
            unused_pages_ = new_page->next;
#if 0
            partix::dprintf("reuse page: %s\n", name_.c_str());
#endif
        } else {
            // 新たに割り当てる
            new_page = (page_header*)page_provider_.allocate(size);

#if 0
            partix::dprintf("new page: %s\n", name_.c_str());
#endif
        }

        // 先頭に前のページへのポインタを埋め込む
        new_page->next = pages_head_; 
        pages_head_ = new_page; 

        unformatted_beginning_ =
            ((char*)new_page)+ sizeof(page_header); 
        unformatted_end_ =((char*)new_page)+ size; 
    }

    void append(page_header*& p, page_header* q) {
        if (!p) { p = q; return; }

        page_header* pp = p;
        for (;;) {
            if (pp->next == nullptr) { pp->next = q; break; }
            pp = pp->next;
        }
    }

    int count_pages(page_header* p) {
        int n = 0;
        while (p != nullptr) {
            p = p->next;
            n++;
        }
        return n;
    }

private:
    PageProvider&	page_provider_;
    page_header*	pages_head_;
    page_header*	unused_pages_;
    char*		unformatted_beginning_;
    char*		unformatted_end_;
    formatted_tag*	formatted_;

    std::string		name_;
    int counter_;
}; 

// variable_pool
//	 fixed_poolを利用した可変長サイズの割り当て
//	 インライン展開後の最適化を期待して、敢えてテーブルなどを使わない。

template <class PageProvider>
class variable_allocator { 
public:
    variable_allocator(PageProvider& pp)
        : page_provider_(pp),
          pool4_(pp), pool8_(pp), pool16_(pp),
          pool32_(pp), pool64_(pp) { }
    ~variable_allocator() {}
		
    void* allocate(size_t n, void*) {
        return operator new(n);
    }
    void deallocate(void* p, size_t) {
        operator delete((char*)p);
    }
		
    void* allocate_fixed(size_t size) {
        // インライン展開後の最適化を期待して、わざと冗長に書いている
        if (size <= 4) {
            return pool4_.allocate();
        } else if (size <= 8) {
            return pool8_.allocate();
        } else if (size <= 16) {
            return pool16_.allocate();
        }  else if (size <= 32) {
            return pool32_.allocate();
        } else if (size <= 64) {
            return pool64_.allocate();
        } else {
            assert(0);
            return nullptr;
        }
    }
    void deallocate_fixed(void* p, size_t size) {
        // インライン展開後の最適化を期待して、わざと冗長に書いている
        if (size <= 4) {
            return pool4_.deallocate(p);
        } else if (size <= 8) {
            return pool8_.deallocate(p);
        } else if (size <= 16) {
            return pool16_.deallocate(p);
        } else if (size <= 32) {
            return pool32_.deallocate(p);
        } else if (size <= 64) {
            return pool64_.deallocate(p);
        } else {
            assert(0);
        }
    }

protected:
    PageProvider&     page_provider_;
    fixed_pool<4, PageProvider> pool4_;
    fixed_pool<8, PageProvider> pool8_;
    fixed_pool<16, PageProvider> pool16_;
    fixed_pool<32, PageProvider> pool32_;
    fixed_pool<64, PageProvider> pool64_;
		
};

class default_page_provider {
public:
    default_page_provider(const char* name) : name_(name) {
#if 0
        counter_ = 0;
#endif
    }
    ~default_page_provider() {}

    size_t page_size() { return 32768; }
    void* allocate(size_t size) {
#if 0
        partix::dprintf(
            "default_page_provider(%s) ++: %d\n", name_.c_str(), ++counter_);
#endif
        return new char[size];
    }
    void deallocate(void* p) {
#if 0
        partix::dprintf(
            "default_page_provider(%s) --: %d\n", name_.c_str(), --counter_);
#endif
        delete []((char*)p);
    }

    std::string name_;
#if 0
    int counter_;
#endif
};


#pragma pack(pop)

#endif
