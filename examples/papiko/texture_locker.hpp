/*!
  @file     texture_locker.hpp
  @brief    <ŠT—v>

  <à–¾>
  $Id: texture_locker.hpp 252 2007-06-23 08:32:33Z naoyuki $
*/
#ifndef TEXTURE_LOCKER_HPP
#define TEXTURE_LOCKER_HPP

class texture_locker {
private:
    IDirect3DTexture9*      tex_;
    D3DLOCKED_RECT          locked_rect_;


public:
    texture_locker(IDirect3DTexture9* tex)
        : tex_(tex)
    {
        tex_->LockRect( 0, &locked_rect_, NULL, 0 );
    }
    ~texture_locker()
    {
        tex_->UnlockRect(0);
    }

    D3DXVECTOR4     get(int x,int y)
    {
        char* data = get_addr( x, y );
                
        D3DXVECTOR4 out;
#if 1
        out.x = ((unsigned char*)data)[2] / 255.0f;
        out.y = ((unsigned char*)data)[1] / 255.0f;
        out.z = ((unsigned char*)data)[0] / 255.0f;
        out.w = ((unsigned char*)data)[3] / 255.0f;
#else
        out.x = ((*((DWORD*)data) & ( 0x3ff << 20 )) >> 20) / 1023.0f;
        out.y = ((*((DWORD*)data) & ( 0x3ff << 10 )) >> 10) / 1023.0f;
        out.z = ((*((DWORD*)data) & ( 0x3ff <<  0 )) >>  0) / 1023.0f;
        out.w = ((*((DWORD*)data) & ( 0x3   << 30 )) >> 30) / 1023.0f;
#endif

        return out;
    }

    void            put(int x,int y,const D3DXVECTOR4& c)
    {
        char* data = get_addr( x, y );

#if 1
        ((unsigned char*)data)[2] = (unsigned char)(c.x * 255.0f);
        ((unsigned char*)data)[1] = (unsigned char)(c.y * 255.0f);
        ((unsigned char*)data)[0] = (unsigned char)(c.z * 255.0f);
        ((unsigned char*)data)[3] = (unsigned char)(c.w * 255.0f);
#else
        *((DWORD*)data) =
            (DWORD(c.x * 1023.0f) << 20) +
            (DWORD(c.y * 1023.0f) << 10) +
            (DWORD(c.z * 1023.0f) <<  0) +
            (DWORD(c.w *    3.0f) << 30);
#endif
    }

    void            put_dw( int x, int y, DWORD c )
    {
        *( (DWORD*)get_addr( x, y ) ) = c;
    }


private:
    char* get_addr(int x,int y)
    {
        char* head = (char*)locked_rect_.pBits + locked_rect_.Pitch * y;
        char* data = (char*)head;

        data += x*4;

        return data;
    }

};

#endif // TEXTURE_LOCKER_HPP
