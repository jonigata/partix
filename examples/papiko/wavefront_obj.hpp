#ifndef WAVEFRONT_OBJ_HPP
#define WAVEFRONT_OBJ_HPP

#include <string>
#include <iostream>
#include <sstream>
#include <vector>
#include <map>
#include "zw/d3d.hpp"

class wavefront_obj_reader {
public:
    struct corner {
        int     vertex_index;
        int     normal_index;
        int     uv_index;
    };

    struct face {
        std::vector< corner > corners;
    };

    struct group {
        std::string             name;
        std::string             material;
        std::vector< face >     faces;                
    };

    std::vector< D3DXVECTOR3 >              v;
    std::vector< D3DXVECTOR3 >              vn;
    std::vector< D3DXVECTOR2 >              vt;
    std::vector< group >                    groups;
    group*                                  curr;
    std::map< std::string, std::string >    mat2tex;

public:
    wavefront_obj_reader( std::istream& ifs )
    {
        load( ifs, "." );
    }

    wavefront_obj_reader( std::istream& ifs, const std::string& directory )
    {
        load( ifs, directory );
    }


private:
    void load( std::istream& ifs, const std::string& directory )
    {
        groups.push_back( group() );
        curr = &groups[0];
        curr->name = "default_group";
                
        std::string matlib_name;
        std::string material_name;

        std::string s;
        while( std::getline( ifs, s ) ) {
            if( s[0] == '#' ) { continue; }
                        
            std::stringstream line( s );
            std::string opcode;
            line >> opcode;
            //std::cout << "opcode: " << opcode << std::endl;

            if( opcode == "mtllib" ) {
                line >> matlib_name;
                load_material( directory, matlib_name );
            }
            if( opcode == "usemtl" ) {
                line >> material_name;
                curr->material = material_name;
            }

            if( opcode == "v" ) {
                D3DXVECTOR3 p;
                line >> p.x >> p.y >> p.z;
                v.push_back( p );                                
            }
            if( opcode == "vn" ) {
                D3DXVECTOR3 p;
                line >> p.x >> p.y >> p.z;
                vn.push_back( p );                                
            }
            if( opcode == "vt" ) {
                D3DXVECTOR2 p;
                line >> p.x >> p.y;
                vt.push_back( p );                                
            }
            if( opcode == "f" ) {
                face f;

                std::string vertex;
                while( line >> vertex ) {
                    //std::cout << "vertex: " << vertex << std::endl;
                    std::stringstream vss( vertex );
                    std::string q;
                    corner c;
                    std::getline( vss, q, '/' );
                    c.vertex_index = atoi( q.c_str() ) - 1;
                    std::getline( vss, q, '/' );
                    c.uv_index = atoi( q.c_str() ) - 1;
                    std::getline( vss, q );
                    c.normal_index = atoi( q.c_str() ) - 1;

                    f.corners.push_back( c );
                }
                curr->faces.push_back( f );
            }
            if( opcode == "g" ) {
                if( !curr->faces.empty() ) {
                    group g;
                    line >> g.name;
                    g.material = material_name;
                    groups.push_back( g );
                    curr = &groups.back();
                }
            }
        }                        
    }        

    void load_material(
        const std::string& directory,
        const std::string& filename )
    {
        std::ifstream ifs( ( directory + "\\" + filename ).c_str() );

        std::string material_name;

        std::string s;
        while( std::getline( ifs, s ) ) {
            if( s[0] == '#' ) { continue; }
                        
            std::stringstream line( s );
            std::string opcode;
            line >> opcode;
            if( opcode == "newmtl" ) {
                line >> material_name;
            }
            if( opcode == "map_Kd" ) {
                std::string texture_name;
                line >> texture_name;
                mat2tex[material_name] = texture_name;
            }
        }
    }

};

#endif // WAVEFRONT_OBJ_HPP
