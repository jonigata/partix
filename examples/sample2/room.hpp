/*!
  @file     room.hpp
  @brief    <ŠT—v>

  <à–¾>
  $Id: room.hpp 32 2008-10-25 12:19:56Z Naoyuki.Hirayama $
*/
#ifndef ROOM_HPP
#define ROOM_HPP

class Room {
private:
#pragma pack( push, 1 )
    struct vertex_type {
        enum { format = (D3DFVF_XYZ | D3DFVF_DIFFUSE) };
        D3DXVECTOR3     pos;
        DWORD           color;
        void operator()(FLOAT xx,FLOAT yy,FLOAT zz,DWORD cc)
        {
            pos.x = xx; pos.y = yy; pos.z = zz; color = cc;
        }
    };
#pragma pack( pop )

public:
    Room();
    ~Room();

    void render(LPDIRECT3DDEVICE9);

private:
    vertex_type     face_vertices_[8];
    WORD            face_indices_[36];
    vertex_type     edge_vertices_[8];
    WORD            edge_indices_[24];

};

#endif // ROOM_HPP
