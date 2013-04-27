#if !defined(FIXED_POOL_HPP)
#define FIXED_POOL_HPP

#include <new>
#include <cassert>

#pragma pack(push,1)

// fixed_pool
//	 �Œ�T�C�Y�I�u�W�F�N�g�p�������v�[��

template < size_t Alignment, class PageProvider >
class fixed_pool { 
private:
	struct page_header { 
		page_header*	next;
	}; 

	struct formated_tag { 
		formated_tag*	next;
	}; 

public:
	fixed_pool( PageProvider& pp ) : page_provider_( pp )
	{ 
		pages_head_ = NULL;
		unused_pages_ = NULL;
				
		unformated_beginning_ = NULL;
		unformated_end_ = NULL;
		formated_ = NULL;
	}
	~fixed_pool()
	{
		page_header* p = pages_head_;
		while( p ) {
			page_header* q = p->next;
			page_provider_.deallocate( p );
			p = q;
		}

		p  = unused_pages_;
		while( p ) {
			page_header* q = p->next;
			page_provider_.deallocate( p );
			p = q;
		}
	}

	void clear()
	{
		if( pages_head_ ) {
			append( unused_pages_, pages_head_ );
			pages_head_ = NULL;
		}

		unformated_beginning_ = NULL;
		unformated_end_ = NULL;
		formated_ = NULL;
	}

	void*	allocate()
	{ 
		if( formated_ ) { 
			// �v�[������Ă���ꍇ�A�����Ԃ�
			void* x = ( void* )formated_;
			formated_ = formated_->next;
			return x;
		} else { 
			// �v�[������Ă��Ȃ��ꍇ�A
			// ���t�H�[�}�b�g�̈������ĕԂ�
			int m = alignment();
			if( unformated_end_ < unformated_beginning_ + m ) {
				require_new_page();
			}

			char* p = unformated_beginning_;
			unformated_beginning_ += m;
			return p;
		}
	}
	void	deallocate( void* p )
	{ 
		// �Ԃ��ꂽ���̂��v�[������
		( ( formated_tag* )p )->next = formated_;
		formated_ = ( formated_tag* )p;
	}
		
protected:
	int alignment() const // �œK����͒萔
	{ 
		int min_alignment = sizeof( formated_tag );
		return ( Alignment + ( min_alignment - 1 ) ) /
			min_alignment * min_alignment;
	}
		
	void require_new_page()
	{
		int m = alignment();
		int size = int( page_provider_.page_size() / m * m );

		page_header* new_page;
		if( unused_pages_ ) {
			// 1���͂����Ď����Ă���
			new_page = unused_pages_;
			unused_pages_ = new_page->next;
		} else {
			// �V���Ɋ��蓖�Ă�
			new_page =
				(page_header*)page_provider_.allocate( size );
		}
				
		// �擪�ɑO�̃y�[�W�ւ̃|�C���^�𖄂ߍ���
		new_page->next = pages_head_; 
		pages_head_ = new_page; 
				
		unformated_beginning_ =
			( ( char* )new_page ) + sizeof( page_header ); 
		unformated_end_ = ( ( char* )new_page ) + size; 
	}

	void append( page_header*& p, page_header* q )
	{
		if( !p ) { p = q; return; }

		for(;;) {
			if( p->next == NULL ) { p->next = q; break; }
			p = p->next;
		}
	}		

private:
	PageProvider&	page_provider_;
	page_header*	pages_head_;
	page_header*	unused_pages_;
	char*			unformated_beginning_;
	char*			unformated_end_;
	formated_tag*	formated_;

}; 

// variable_pool
//	 fixed_pool�𗘗p�����ϒ��T�C�Y�̊��蓖��
//	 �C�����C���W�J��̍œK�������҂��āA�����ăe�[�u���Ȃǂ��g��Ȃ��B

template < class PageProvider >
class variable_allocator { 
public:
	variable_allocator( PageProvider& pp )
		: page_provider_( pp ), 
		  pool4_( pp ), pool8_( pp ), pool16_( pp ),
		  pool32_( pp ), pool64_( pp )
	{ }
	~variable_allocator() { }
		
	void*	allocate( size_t n, void* )
	{
		return operator new ( n );
	}
	void	deallocate( void* p, size_t )
	{
		operator delete ( ( char* )p );
	}
		
	void*	allocate_fixed( size_t size )
	{ 
		// �C�����C���W�J��̍œK�������҂��āA�킴�Ə璷�ɏ����Ă���
		if( size <= 4 ) { return pool4_.allocate(); }
		else if( size <= 8 ) { return pool8_.allocate(); }
		else if( size <= 16 ) { return pool16_.allocate(); }
		else if( size <= 32 ) { return pool32_.allocate(); }
		else if( size <= 64 ) { return pool64_.allocate(); }
		else { 
			assert( 0 );
			return NULL;
		}
	}
	void	deallocate_fixed( void* p, size_t size )
	{ 
		// �C�����C���W�J��̍œK�������҂��āA�킴�Ə璷�ɏ����Ă���
		if( size <= 4 ) { return pool4_.deallocate( p ); }
		else if( size <= 8 ) { return pool8_.deallocate( p ); }
		else if( size <= 16 ) { return pool16_.deallocate( p ); }
		else if( size <= 32 ) { return pool32_.deallocate( p ); }
		else if( size <= 64 ) { return pool64_.deallocate( p ); }
		else { 
			assert( 0 );
		}
	}

protected:
	PageProvider&					page_provider_;
	fixed_pool< 4, PageProvider >	pool4_;
	fixed_pool< 8, PageProvider >	pool8_;
	fixed_pool< 16, PageProvider >	pool16_;
	fixed_pool< 32, PageProvider >	pool32_;
	fixed_pool< 64, PageProvider >	pool64_;
		
};

class default_page_provider {
public:
	default_page_provider()
	{
#if 0
		counter_ = 0;
#endif
	}
	~default_page_provider() {}

	size_t page_size() { return 256; }
	void* allocate( size_t size )
	{
#if 0
		char buffer[256];
		sprintf( buffer, "default_page_provider: %d\n", ++counter_ );
		OutputDebugStringA( buffer );
#endif
		return new char[size];
	}
	void deallocate( void* p )
	{
#if 0
		char buffer[256];
		sprintf( buffer, "default_page_provider: %d\n", --counter_ );
		OutputDebugStringA( buffer );
#endif
		delete [] ( (char*)p );
	}

#if 0
	int counter_;
#endif
};


#pragma pack(pop)

#endif
