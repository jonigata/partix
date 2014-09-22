// 2008/08/29 Naoyuki Hirayama

#include <iostream>
#include <vector>
#include <fstream>
#include <string>
#include "mqoreader.hpp"

struct Vector {
	float x;
	float y;
	float z;

	Vector(){}
	Vector( float ax, float ay, float az ) : x(ax), y(ay), z(az) {}
};

typedef Vector vector_type;

struct Primitive {
	std::vector< vector_type >	vertices;
	std::vector< int >			indices;
};

static void
cat( std::ostream& os, const std::string& filename )
{
	std::ifstream ifs( filename.c_str() );

	std::string line;
	while( std::getline( ifs, line )  ) {
		os << line << std::endl;
	}
}

static void
create_triangle_list(
	std::vector< Primitive >&	primitives,
	const mqo_reader::document_type& doc )
{
	for( mqo_reader::objdic_type::const_iterator i =
			 doc.objects.begin() ;
		 i != doc.objects.end() ;
		 ++i ) {
		const mqo_reader::object_type& obj = (*i).second;

		Primitive p;

		// Ç∆ÇËÇ†Ç¶Ç∏í∏ì_ëSïîï˙ÇËçûÇﬁ
		for( mqo_reader::vertices_type::const_iterator j =
				 obj.vertices.begin() ;
			 j != obj.vertices.end() ;
			 ++j ) {
			const mqo_reader::vertex_type& src = *j;
			p.vertices.push_back( vector_type( src.x, src.y, src.z ) );
		}

		// ñ Ç‡ëSïîï˙ÇËçûÇﬁ
		// ñ ì|Ç»ÇÃÇ≈éläpÉ|ÉäÉSÉìÇÕÇ»Ç¢Ç‡ÇÃÇ∆âºíË
		for( mqo_reader::faces_type::const_iterator j =
				 obj.faces.begin() ;
			 j != obj.faces.end() ;
			 ++j ) {
			const mqo_reader::face_type& face = *j;
			int i0 = face.vertex_indices[0];
			int i1 = face.vertex_indices[1];
			int i2 = face.vertex_indices[2];
			p.indices.push_back( i0 );
			p.indices.push_back( i1 );
			p.indices.push_back( i2 );
		}

		primitives.push_back( p );
	}
}


static void
to_tcf( const std::string&					output_filename,
		const mqo_reader::document_type&	doc,
		const std::string&					physical_name )
{
	std::vector< Primitive > primitives;
	create_triangle_list( primitives, doc );

	////////////////////////////////////////////////////////////////
	// make tetgenio in
	tetgenio in, out;

	in.firstnumber = 0;

	// vertex count
	in.numberofpoints = 0;
	in.numberoffacets = 0;
	for( int i = 0 ; i < int(primitives.size()) ; i++ ) {
		in.numberofpoints += primitives[i].vertices.size();
		in.numberoffacets += primitives[i].indices.size() / 3;
	}

	// store vertices
	in.pointlist = new REAL[in.numberofpoints * 3]; // 3 = vector x, y, z
	REAL* vp = in.pointlist;
	for( int i = 0 ; i < int(primitives.size()) ; i++ ) {
		const Primitive& prim = primitives[i];
		for( int j = 0 ; j < int(prim.vertices.size()) ; j++ ) {
			*vp++ = prim.vertices[j].x;
			*vp++ = prim.vertices[j].y;
			*vp++ = prim.vertices[j].z;
		}
	}

	// store indices
	in.facetlist = new tetgenio::facet[in.numberoffacets];

	int facet_index = 0;
	for( int i = 0 ; i < int(primitives.size()) ; i++ ) {
		const Primitive& prim = primitives[i];
		for( int j = 0 ; j < int(prim.indices.size()) ; j+=3 ) {
			tetgenio::facet* f = &in.facetlist[facet_index++];
			f->numberofpolygons = 1;
			f->polygonlist = new tetgenio::polygon[1];
			f->numberofholes = 0;
			f->holelist = NULL;
			tetgenio::polygon* p = &f->polygonlist[0];
			p->numberofvertices = 3;
			p->vertexlist = new int[3];
			p->vertexlist[0] = prim.indices[j+0];
			p->vertexlist[1] = prim.indices[j+1];
			p->vertexlist[2] = prim.indices[j+2];
		}
	}

	// store facet markers
	in.facetmarkerlist = new int[in.numberoffacets];
	memset(in.facetmarkerlist, 0, sizeof(int)*in.numberoffacets);

	// Tetrahedralize the PLC. Switches are chosen to read a PLC (p),
	//	 do quality mesh generation (q) with a specified quality bound
	//	 (1.414), and apply a maximum volume constraint (a0.1).


	////////////////////////////////////////////////////////////////
	// write
	tetrahedralize("-pqi", &in, &out);

	// Output mesh to files 'barout.node', 'barout.ele' and 'barout.face'.
	out.save_nodes(const_cast<char*>(output_filename.c_str()));
	out.save_elements(const_cast<char*>(output_filename.c_str()));
	out.save_faces(const_cast<char*>(output_filename.c_str()));

	std::string os = output_filename;
	std::ofstream ofs( os.c_str() );
	cat( ofs, os + ".node" );
	cat( ofs, os + ".ele" );
	cat( ofs, os + ".face" );
	_unlink( ( os + ".node" ).c_str() );
	_unlink( ( os + ".ele" ).c_str() );
	_unlink( ( os + ".face" ).c_str() );
}

int main(int argc,char** argv)
{
	if( argc < 2 ) {
		std::cerr << "usage: mqo2tcf <input-filename> [output-filename] [physical-object-name]\n";
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
		_makepath( ofname, drive, dir, fname, "tcf" );
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

		to_tcf( output_filename, doc, physical_name );
	}
	catch( std::exception& e ) {
		std::cerr << e.what() << std::endl;
		exit(1);
	}
}
