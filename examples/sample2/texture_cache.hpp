#ifndef TEXTURE_CACHE_HPP
#define TEXTURE_CACHE_HPP

/*===========================================================================*/
/*!
 * @class TextureHolder, TextureCache
 * @brief 
 *
 * 
 */
/*==========================================================================*/

struct TextureHolder {
    std::string             filename;
    IDirect3DTexture9*      texture;
};

class TextureCache {
public:
    TextureCache( const std::string& data_directory )
        : dir_( data_directory + "/" ) {}
    ~TextureCache()
    {
        for( dictionary_type::const_iterator i = dic_.begin() ;
             i != dic_.end() ;
             ++i ) {
            TextureHolder* p = (*i).second.get();
            if( p->texture ) {
                p->texture->Release();
            }
        }
    }
        
    TextureHolder* get_texture( const std::string& filename )
    {
        dictionary_type::const_iterator i = dic_.find( filename );
        if( i != dic_.end() ) {
            return (*i).second.get();
        }

        boost::shared_ptr< TextureHolder > p( new TextureHolder );
        dic_[filename] = p;

        p->filename = filename;
        p->texture = NULL;

        return p.get();
    }

    bool setup( LPDIRECT3DDEVICE9 device )
    {
        bool result = false; // relayout‚ª•K—v‚©‚Ç‚¤‚©

        for( dictionary_type::const_iterator i = dic_.begin() ;
             i != dic_.end() ;
             ++i ) {
            TextureHolder* p = (*i).second.get();
            if( !p->texture ) {
                D3DXCreateTextureFromFileA(
                    device, ( dir_ + p->filename ).c_str(), &p->texture );
                result = true;
            }
        }

        return result;
    }

    void on_lost_device( LPDIRECT3DDEVICE9 device )
    {
        for( dictionary_type::const_iterator i = dic_.begin() ;
             i != dic_.end() ;
             ++i ) {
            TextureHolder* p = (*i).second.get();
            if( p->texture ) {
                p->texture->Release();
                p->texture = NULL;
            }
        }
    }

    void on_reset_device( LPDIRECT3DDEVICE9 device )
    {
        setup( device );
    }

private:
    typedef std::map< std::string, boost::shared_ptr< TextureHolder > >
            dictionary_type;
    dictionary_type dic_;
    std::string     dir_;
                
};


#endif // TEXTURE_CACHE_HPP
