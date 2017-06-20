// $Id: mqo2smesh.cpp 25 2008-09-18 10:14:03Z Naoyuki.Hirayama $

#include "mqoreader.hpp"
#include <fstream>
#include <cassert>

void to_smesh( std::ostream& ofs, const mqo_reader::object_type& o )
{
        size_t vertex_count = o.vertices.size();

        // Part 1 - node list
        // * First line: <# of points> <dimension (must be 3)> <# of attributes> <# of boundary markers (0 or 1)>
        ofs << vertex_count << " " << 3 << " " << 0 << " " << 0 << std::endl;

        // * Remaining lines list # of points:
        // <point #> <x> <y> <z>[attributes] [boundary marker]
        for( size_t i = 0 ; i < vertex_count ; i++ ) {
                ofs << i << " "
                          << o.vertices[i].x << " "
                          << o.vertices[i].y << " "
                          << o.vertices[i].z << std::endl;
        }

        // Part 2 - facet list
        // * One line: <# of facets> <boundary markers (0 or 1)>
        size_t face_count = o.faces.size();

        ofs << face_count << " " << 1 << std::endl;

        // * Following lines list # of facets:
        // <# of corners> <corner 1> <corner 2> ... <corner #> [boundary marker]
        for( size_t i = 0 ; i < face_count ; i++ ) {
                int n = o.faces[i].vertex_count;

                if( n == 3 ) {
                        ofs << 3;
                        for( int j = 0 ; j < 3 ; j++ ) {
                                ofs << " " << o.faces[i].vertex_indices[j];
                        }
                        ofs << " " << i << std::endl;
                } else {
                        assert( n == 4 );
                        ofs << 3;
                        for( int j = 0 ; j < 3 ; j++ ) {
                                ofs << " " << o.faces[i].vertex_indices[j];
                        }
                        ofs << " " << i << std::endl;

                        ofs << 3;
                        for( int j = 0 ; j < 3 ; j++ ) {
                                const int indirect[] = { 0, 2, 3 };
                                ofs << " " << o.faces[i].vertex_indices[indirect[j]];
                        }
                        ofs << " " << i << std::endl;
                }

        }

        // Part 3 - hole list
        // * One line: <# of holes>
        // * Following lines list # of holes:
        // <hole #> <x> <y> <z>
        ofs << 0 << std::endl;

        // Part 4 - region attributes list
        // * One line: <# of region>
        // * Following lines list # of region attributes:
        // <region #> <x> <y> <z><region number><region attribute>
        ofs << 0 << std::endl;
}

int main( int argc, const char** argv )
{
        if( argc < 2 ) {
                std::cerr << "usage: mqo2smesh <input-filename> [output-filename] [physical-object-name]\n";
                exit(1);
        }

        std::string input_filename = argv[1];

        std::string output_filename;
        if( 2 < argc ) {
                output_filename = argv[2];
        } else {
                char drive[MAX_PATH];
                char dir[MAX_PATH];
                char fname[MAX_PATH];
                char ext[MAX_PATH];
                _splitpath( argv[1], drive, dir, fname, ext );

                char ofname[MAX_PATH];
                _makepath( ofname, drive, dir, fname, "smesh" );
                output_filename = ofname;
        }

        std::string physical_name = "physical";
        if( 3 < argc ) {
                physical_name = argv[3];
        }

        try {
                std::ifstream ifs( input_filename.c_str() );
                if( !ifs ) {
                        throw std::runtime_error(
                                "can't find file: " + input_filename );
                }

                mqo_reader::document_type doc;
                mqo_reader::read_mqo( ifs, doc );

                mqo_reader::objdic_type::const_iterator i =
                        doc.objects.find( physical_name );
                if( i == doc.objects.end() ) {
                        throw std::runtime_error(
                                "can't find the object named '" + physical_name + "'" );
                }

                std::ofstream ofs( output_filename.c_str() );
                if( !ofs ) {
                        throw std::runtime_error(
                                "can't open file: " + output_filename );
                }

                to_smesh( ofs, (*i).second );
        }
        catch( std::exception& e ) {
                std::cerr << e.what() << std::endl;
                exit(1);
        }
}
