#include "mqoreader.hpp"
#include <boost/lexical_cast.hpp>
#include <fstream>
#include <sstream>
#include "windows.h"

namespace mqo_reader {

class substr {
public:
        substr() : buffer_(NULL), i_(0), n_(0) {}

        substr( const std::string& buffer, size_t i, size_t n )
                : buffer_( &buffer ), i_( i ), n_( n ) {}
        ~substr(){}

        substr& operator=( const substr& x )
        {
                buffer_ = x.buffer_;
                i_ = x.i_; n_ = x.n_;
                return *this;
        }

        bool operator==( const char* s )
        {
                const char* ps = buffer_->data() + i_;
                const char* pe = ps + n_;
                const char* p = ps;
                const char* q = s;
                while( p < pe && *q && *p == *q ) {
                        p++;
                        q++;
                }
                return p == pe && *q == 0;
        }
        bool operator!=( const char* s ) { return !( (*this) == s ); }

        char operator[]( size_t x ) { return (*buffer_)[i_+x]; }
        size_t index() { return i_; }
        size_t length() { return n_; }

        std::string str() { return buffer_->substr( i_, n_ ); }
        const std::string& buffer() { return *buffer_; }

private:
        const std::string*      buffer_;
        size_t                  i_;
        size_t                  n_;

};

class tokenizer {
public:
        tokenizer(std::istream& is)
                : is_(is),
                  lineno_(0),
                  curr_( 0 ), prev_( 0 )
        {
        }
        ~tokenizer(){}

        substr get()
        {
                // skip whitespaces
                prev_ = curr_;
                for(;;){
                        while( curr_ < buffer_.size() &&
                               isspace( buffer_[curr_] ) &&
                               buffer_[curr_] != '\n' ) {
                                curr_++;
                        }
                        if( curr_ < buffer_.size() ) {
                                prev_ = curr_;
                                break;
                        }
                        if( std::getline( is_, buffer_ ).eof() ) {
                                return substr( buffer_, 0, 0 );
                        }
                        if( 0 < buffer_.size() &&
                            buffer_[ buffer_.size() - 1 ] == '\r' ) {
                                buffer_[ buffer_.size() - 1 ] = '\n';
                        } else {
                                buffer_ += '\n';
                        }
                        lineno_++;

                        //char buffer[256];
                        //sprintf( buffer, "ln: %d\n", lineno_ );
                        //OutputDebugStringA( buffer );

                        curr_ = 0;
                        prev_ = 0;
                }

                if( buffer_[curr_] == '\n' ) {
                        curr_++;
                        return substr( buffer_, prev_, curr_ - prev_ );
                }

                // word
                {
                        if( isalpha( buffer_[curr_] ) ) {
                                curr_++;
                                while( isalpha( buffer_[curr_] ) ||
                                       isdigit( buffer_[curr_] ) ||
                                       buffer_[curr_] == '_' ) {
                                        curr_++;
                                }
                                return substr( buffer_, prev_, curr_ - prev_ );
                        }
                }

                // digit
                {
                        if( buffer_[curr_] == '-' ||
                            isdigit( buffer_[curr_] ) ) {
                                curr_++;
                                while( isdigit( buffer_[curr_] ) ||
                                       buffer_[curr_]=='.' ) {
                                        curr_++;
                                }
                                return substr( buffer_, prev_, curr_ - prev_ );
                        }
                }
                
                // operators
                {
                        if( strchr( "[{}(),]", buffer_[curr_] ) ) {
                                curr_++;
                                return substr( buffer_, prev_, curr_ - prev_ );
                        }
                }
                
                // string
                if( buffer_[curr_] == '"' ) {
                        curr_++;
                        while( curr_ < buffer_.size() &&
                               buffer_[curr_] != '"' ) {
                                curr_++; 
                        }
                        if( buffer_[curr_] == '"' ) {
                                curr_++;
                                return substr( buffer_, prev_, curr_ - prev_ );
                        } else {
                                throw mqo_reader_error( "no closing \"" );
                        }
                }

                // ƒGƒ‰[
                while( !isspace( buffer_[curr_] ) ) {
                        curr_++;
                }
                throw mqo_reader_error(
                        "bad token: " +
                        buffer_.substr( prev_, curr_ - prev_  ) );
        }

        substr get( substr& s )
        {
                s = get();
                return s;
        }
        
        substr operator()()
        {
                return get();
        }

        substr operator()( substr& s )
        {
                return get( s );
        }

        int lineno(){return lineno_;}

public:
        void expect_linefeed()
        {
                substr token;
                if( get( token ) != "\n" ){
                        throw mqo_reader_error(
                                "unexpected token '" +
                                token.str() + "' for '\\n'");
                }
        }

        void expect_literal(const char* s)
        {
                substr token;
                if( get( token ) != s ){
                        throw mqo_reader_error(
                                "unexpected token '" +
                                token.str() + "' for '" + s + "'" );
                }
        }

        substr expect_string()
        {
                substr token = get();
                if( token[0] != '"' || token[token.length()-1] != '"' ) {
                        throw mqo_reader_error(
                                "unexpected token '" +
                                token.str() + "' for string" );
                }
                return substr(
                        token.buffer(),
                        token.index() + 1,
                        token.length() - 2 );
        }

        substr expect_string(size_t size)
        {
                substr token = expect_string();
                if( size < token.length() ){
                        throw mqo_reader_error(
                                "too long string " + token.str() );
                }
                return token;
        }

        int expect_integer()
        {
                substr token = get();
                try {
                        int n = atoi( token.str().c_str() );
                        return n;
                }
                catch(boost::bad_lexical_cast&){
                        throw mqo_reader_error(
                                "unexpected token '" +
                                token.str() + "for integer" );
                }
        }

        int expect_integer( int mn )
        {
                int n = expect_integer();
                if(n<mn){
                        throw mqo_reader_error(
                                "number out of range: " +
                                boost::lexical_cast<std::string>( n ) );
                }
                return n;
        }

        int expect_integer(int mn,int mx)
        {
                int n = expect_integer();
                if( n< mn || mx < n ) {
                        throw mqo_reader_error(
                                "number out of range: " +
                                boost::lexical_cast<std::string>( n ) );
                }
                return n;
        }

        DWORD expect_dword()
        {
                substr token = get();
                char* p;
                DWORD x = strtoul( token.str().c_str(), &p, 10 );
                if( !p ) {
                        throw mqo_reader_error(
                                "unexpected token: " +
                                token.str() );
                }
                return x;
        }

        float expect_float()
        {
                substr token = get();
                try {
                        float n = float( atof( token.str().c_str() ) );
                        return n;
                }
                catch( boost::bad_lexical_cast& ) {
                        throw mqo_reader_error(
                                "unexpected token '" +
                                token.str() + "for float" );
                }
        }

        float expect_float( float mn )
        {
                float n = expect_float();
                if( n < mn ) {
                        throw mqo_reader_error(
                                "number out of range: " +
                                boost::lexical_cast< std::string >( n ) );
                }
                return n;
        }

        float expect_float(float mn,float mx)
        {
                float n = expect_float();
                if( n < mn || mx < n ){
                        throw mqo_reader_error(
                                "number out of range: " +
                                boost::lexical_cast< std::string >( n ) );
                }
                return n;
        }

        bool expect_bool()
        {
                return expect_integer( 0, 1 ) ? true : false;
        }

private:
        std::istream&   is_;
        std::stringstream ss_;
        int             lineno_;
        std::string     buffer_;
        size_t          curr_;
        size_t          prev_;


};

void skip_to_linefeed(tokenizer& t)
{
        for(;;){
                if( t() == "\n" ) { break; }
        }
}

void skip_chunk(tokenizer& t)
{
        t.expect_literal( "{" );

        // “K“–
        int nest=1;
        for(;;){
                substr token = t();
                if( token == "{" ) { 
                        nest++ ; 
                } else if( token == "}" ) { 
                        nest-- ; 
                        if( nest == 0 ) { 
                                break ; 
                        }
                }
        }
}

void read_header(tokenizer& t,document_type& doc)
{
        t.expect_literal( "Metasequoia" ) ; 
        t.expect_literal( "Document" ) ; 
        t.expect_linefeed() ; 
        t.expect_literal( "Format" ) ; 
        t.expect_literal( "Text" ) ; 
        t.expect_literal( "Ver" );

        substr version = t();
        std::stringstream ss( version.str() );
        char c;
        ss >> doc.major_version
           >> c
           >> doc.minor_version;

        t.expect_linefeed();
}

void read_scene(tokenizer& t,document_type& doc)
{
        //std::cerr << "unsupported data: Scene" << std::endl;
        t.expect_literal( "{" );
        for( ;; ) {
                substr token = t();
                if( token == "}" ) { break; }
                if( token == "amb" ) {
                        doc.scene.ambient.red = t.expect_float(0,1);
                        doc.scene.ambient.green = t.expect_float(0,1);
                        doc.scene.ambient.blue = t.expect_float(0,1);
                        doc.scene.ambient.alpha = 1;
						t.expect_linefeed();
                } else {
                        skip_to_linefeed(t);
                }
        }
}

void read_backimage(tokenizer& t,document_type& doc)
{
        //std::cerr << "unsupported data: BackImage" << std::endl;
        skip_chunk(t);
}

void read_material(tokenizer& t,document_type& doc)
{
        int count=t.expect_integer(1);
        t.expect_literal( "{" );
        t.expect_linefeed();
        for(int i=0;i<count;i++){
                material_type m;
                m.name=t.expect_string(31).str();
                m.shader = shader_phong;
                m.vertex_color = false;
                m.color.red = m.color.green = m.color.blue = m.color.alpha =
                        1.0f;
                m.diffuse = m.ambient = m.emissive = m.specular = m.power =
                        1.0f;
                m.projection = projection_uv;
                m.proj_pos.x = m.proj_pos.y = m.proj_pos.z = 0;
                m.proj_scale.x = m.proj_scale.y = m.proj_scale.z = 0;
                m.proj_angle.heading =
                        m.proj_angle.pitching =
                        m.proj_angle.banking = 0;
                for(;;){
                        substr token = t();
                        if( token == "shader" ) { 
                                t.expect_literal( "(" );
                                m.shader = shader_type(
                                        t.expect_integer( 0, 4 ) );
                                t.expect_literal( ")" );
                        } else if( token == "vcol" ) { 
                                t.expect_literal( "(" );
                                m.vertex_color = t.expect_bool();
                                t.expect_literal( ")" );
                        } else if( token == "col" ) { 
                                t.expect_literal( "(" );
                                m.color.red = t.expect_float( 0, 1.0f );
                                m.color.green = t.expect_float( 0, 1.0f );
                                m.color.blue = t.expect_float( 0, 1.0f );
                                m.color.alpha = t.expect_float( 0, 1.0f );
                                t.expect_literal( ")" );
                        } else if( token == "dif" ) { 
                                t.expect_literal( "(" );
                                m.diffuse = t.expect_float( 0, 1.0f );
                                t.expect_literal( ")" );
                        } else if( token == "amb" ) { 
                                t.expect_literal( "(" );
                                m.ambient = t.expect_float( 0, 1.0f );
                                t.expect_literal( ")" );
                        } else if( token == "emi" ) { 
                                t.expect_literal( "(" );
                                m.emissive = t.expect_float( 0, 1.0f );
                                t.expect_literal( ")" );
                        } else if( token == "spc" ) { 
                                t.expect_literal( "(" );
                                m.specular = t.expect_float( 0, 1.0f );
                                t.expect_literal( ")" );
                        } else if( token == "power" ) { 
                                t.expect_literal( "(" );
                                m.power = t.expect_float( 0, 100.0f );
                                t.expect_literal( ")" );
                        } else if( token == "tex" ) { 
                                t.expect_literal( "(" );
                                m.texture = t.expect_string( 63 ).str();
                                t.expect_literal( ")" );
                        } else if( token == "aplane" ) { 
                                t.expect_literal( "(" );
                                m.aplane = t.expect_string( 63 ).str();
                                t.expect_literal( ")" );
                        } else if( token == "bump" ) { 
                                t.expect_literal( "(" );
                                m.bump = t.expect_string( 63 ).str();
                                t.expect_literal( ")" );
                        } else if( token == "proj_type" ) { 
                                t.expect_literal( "(" );
                                m.projection = projection_type( 
                                        t.expect_integer( 0, 3 ) );
                                t.expect_literal( ")" );
                        } else if( token == "proj_pos" ) { 
                                t.expect_literal( "(" );
                                m.proj_pos.x = t.expect_float();
                                m.proj_pos.y = t.expect_float();
                                m.proj_pos.z = t.expect_float();
                                t.expect_literal( ")" );
                        } else if( token == "proj_scale" ) { 
                                t.expect_literal( "(" );
                                m.proj_scale.x = t.expect_float();
                                m.proj_scale.y = t.expect_float();
                                m.proj_scale.z = t.expect_float();
                                t.expect_literal( ")" );
                        } else if( token == "proj_angle" ) { 
                                t.expect_literal( "(" );
                                m.proj_angle.heading = t.expect_float();
                                m.proj_angle.pitching = t.expect_float();
                                m.proj_angle.banking = t.expect_float();
                                t.expect_literal( ")" );
                        } else if( token == "\n" ) { 
                                break;
                        } else { 
                                throw mqo_reader_error(
                                        "unexpected token: "+token.str() );
                        }
                }
                doc.materials.push_back( m );
        }
        t.expect_literal( "}" );
        t.expect_linefeed();
}

void read_vertices(tokenizer& t,int count,std::vector<vertex_type>& vertices)
{
        t.expect_literal( "{" );
        t.expect_linefeed();
        for( int i = 0 ; i< count ; i++ ) {
                vertex_type v; 
                v.x = t.expect_float();
                v.y = t.expect_float();
                v.z = t.expect_float();
                t.expect_linefeed();
                vertices.push_back( v );
        }
        t.expect_literal( "}" );
        t.expect_linefeed();
 }

void read_faces(tokenizer& t,int count,std::vector<face_type>& faces)
{ 
        t.expect_literal( "{" );
        t.expect_linefeed();
        for( int i = 0 ; i< count ; i++ ) { 
                face_type f;
                f.vertex_count = t.expect_integer( 2, 4 );
                f.material_index = -1;
                for( int j = 0 ; j < f.vertex_count ; j++ ) { 
                        f.colors[ j ].red = f.colors[ j ].green = 
                                f.colors[ j ].blue = f.colors[ j ].alpha = 1;
                }
                for( ; ; ) { 
                        substr token = t();
                        if( token == "V" ) { 
                                t.expect_literal( "(" );
                                for( int j = 0 ; j< f.vertex_count ; j++ ) { 
                                        f.vertex_indices[ j ] =
                                                t.expect_integer( 0 );
                                }
                                t.expect_literal( ")" );
                        } else if( token == "M" ) { 
                                t.expect_literal( "(" );
                                f.material_index = t.expect_integer( -1 );
                                t.expect_literal( ")" );
                        } else if( token == "UV" ) { 
                                t.expect_literal( "(" );
                                for( int j = 0 ; j< f.vertex_count ; j++ ) { 
                                        f.uv[ j ].u = t.expect_float();
                                        f.uv[ j ].v = t.expect_float();
                                }
                                t.expect_literal( ")" );
                        } else if( token == "COL" ) { 
                                t.expect_literal( "(" );
                                for( int j = 0 ; j < f.vertex_count ; j++ ) { 
                                        DWORD c = t.expect_dword();
                                        f.colors[ j ].red =
                                                ( c & 0xff ) / 255.0f;
                                        f.colors[ j ].green =
                                                ( ( c & 0xff00 ) >> 8 ) /
                                                255.0f;
                                        f.colors[ j ].blue =
                                                ( ( c & 0xff0000 ) >> 16 ) /
                                                255.0f;
                                        f.colors[ j ].alpha =
                                                ( ( c & 0xff000000 ) >> 24 ) /
                                                255.0f;
                                }
                                t.expect_literal( ")" );
                        } else if( token == "\n" ) { 
                                break;
                        } else { 
                                throw mqo_reader_error(
                                        "unexpected token: "+token.str() );
                        }
                }
                faces.push_back( f );
        }
        t.expect_literal( "}" );
        t.expect_linefeed();
}

void read_object(tokenizer& t,document_type& doc)
{
        object_type obj;
        obj.name=t.expect_string().str();
        obj.depth = 0;
        obj.folding = 0;
        obj.scale.x = 1.0f;
        obj.scale.y = 1.0f;
        obj.scale.z = 1.0f;
        obj.rotation.x = 0;
        obj.rotation.y = 0;
        obj.rotation.z = 0;
        obj.translation.x = 0;
        obj.translation.y = 0;
        obj.translation.z = 0;
        obj.patch = patch_plane;
        obj.segment = 4;
        obj.visible = true;
        obj.locking = false;
        obj.shading = shading_flat;
        obj.facet = 59.5f;
        obj.color.red = 1.0f;
        obj.color.green = 1.0f;
        obj.color.blue = 1.0f;
        obj.color.alpha = 1.0f;
        obj.color_type = edgecolor_environment;
        obj.mirror = mirror_none;
        obj.mirror_axis = mirroraxis_none;
        obj.mirror_dis = 0;
        obj.lathe = lathe_none;
        obj.lathe_axis = latheaxis_x;
        obj.lathe_seg = 12;

        t.expect_literal( "{" );
        for(;;){
                substr token=t();
                if(token==""){
                        throw mqo_reader_error("unexpected eof");
                } else if( token == "}" ) {
                        break;
                } else if( token == "depth" ) {
                        obj.depth = t.expect_integer( 0 );
                        t.expect_linefeed();
                } else if( token == "folding" ) {
                        obj.folding = t.expect_integer( 0, 1 ) ? true : false;
                        t.expect_linefeed();
                } else if( token == "scale" ) {
                        obj.scale.x = t.expect_float();
                        obj.scale.y = t.expect_float();
                        obj.scale.z = t.expect_float();
                        t.expect_linefeed();
                } else if( token == "rotation" ) {
                        obj.scale.x = t.expect_float();
                        obj.scale.y = t.expect_float();
                        obj.scale.z = t.expect_float();
                        t.expect_linefeed();
                } else if( token == "translation" ) {
                        obj.scale.x = t.expect_float();
                        obj.scale.y = t.expect_float();
                        obj.scale.z = t.expect_float();
                        t.expect_linefeed();
                } else if( token == "patch" ) { 
                        obj.patch = patch_type( t.expect_integer( 0, 3 ) );
                        t.expect_linefeed();
                } else if( token == "segment" ) {
                        obj.segment = t.expect_integer(1,16);
                        t.expect_linefeed();
                } else if( token == "visible" ) {
                        obj.visible = t.expect_integer() ? true : false;
                        t.expect_linefeed();
                } else if( token == "locking" ) {
                        obj.locking = t.expect_bool();
                        t.expect_linefeed();
                } else if( token == "shading" ) {
                        obj.shading = shading_type( t.expect_integer( 0, 1 ) );
                        t.expect_linefeed();
                } else if( token == "facet" ) {
                        obj.facet   =t.expect_float(0,180.0f);
                        t.expect_linefeed();
                } else if( token == "color" ) {
                        obj.color.red = t.expect_float( 0, 1.0f ) ;
                        obj.color.green = t.expect_float( 0, 1.0f );
                        obj.color.blue = t.expect_float( 0, 1.0f );
                        obj.color.alpha = 1.0f;
                        t.expect_linefeed();
                } else if( token == "color_type" ) {
                        obj.color_type =
                                edgecolor_type( t.expect_integer( 0, 1 ) );
                        t.expect_linefeed();
                } else if( token == "mirror" ) {
                        obj.mirror = mirror_type( t.expect_integer( 0, 2 ) );
                        t.expect_linefeed();
                } else if( token == "mirror_axis" ) {
                        obj.mirror_axis =
                                mirroraxis_type( t.expect_integer( 0, 7 ) );
                        t.expect_linefeed();
                } else if( token == "mirror_dis" ) {
                        obj.mirror_dis =t.expect_float(0);
                        t.expect_linefeed();
                } else if( token == "lathe" ) {
                        obj.lathe = lathe_type( t.expect_integer( 0, 3 ) );
                        t.expect_linefeed();
                } else if( token == "lathe_axis" ) { 
                        obj.lathe_axis =
                                latheaxis_type( t.expect_integer( 0, 2 ) );
                        t.expect_linefeed();
                } else if( token == "lathe_seg" ) {
                        obj.lathe_seg =
                                latheaxis_type( t.expect_integer( 3 ) );
                        t.expect_linefeed();
                } else if( token == "vertex" ) {
                        int count = t.expect_integer();
                        read_vertices( t, count, obj.vertices );
                } else if( token == "vertex_attr" ) {
                        //std::cerr << "unsupported data: vertex_attr"
                        //          << std::endl;
                        t.expect_integer();
                        skip_chunk(t);
                } else if( token == "BVertex" ) { 
                        throw mqo_reader_error(
                                "encount 'BVertex': "
                                "older version is not supported" );
                        //std::cerr << "unsupported data: BVertex"
                        //          << std::endl;
                        t.expect_integer();
                        skip_chunk( t );
                } else if( token == "face" ) {
                        int count=t.expect_integer();
                        read_faces( t, count, obj.faces );
                }
        }
        doc.objects[obj.name]=obj;
}

void read_blob(tokenizer& t,document_type& doc)
{
        //std::cerr << "unsupported data: Blob" << std::endl;
        skip_chunk(t);
}

void read_mqo(std::istream& is,document_type& doc)
{ 
        tokenizer t( is );

        try { 
                read_header( t, doc );

                for( ; ; ) { 
                        substr chunk_type = t();
                        if( chunk_type == "" ) { 
                                break;
                        } else if( chunk_type == "\n" ) { 
                                continue;
                        } else if( chunk_type == "TrialNoise" ) { 
                                throw mqo_reader_error(
                                        "encount 'TrialNoise'" );
                        } else if( chunk_type == "Scene" ) { 
                                read_scene( t, doc );
                        } else if( chunk_type == "BackImage" ) { 
                                read_backimage( t, doc );
                        } else if( chunk_type == "Material" ) { 
                                read_material( t, doc );
                        } else if( chunk_type == "Object" ) { 
                                read_object( t, doc );
                        } else if( chunk_type == "Blob" ) { 
                                read_blob( t, doc );
                        } else if( chunk_type == "Eof" ) { 
                                return;
                        } else { 
                                throw mqo_reader_error( 
                                        "unexpected token for chunk type: " +
                                        chunk_type.str() );
                        }
                }
        }
        catch( mqo_reader_error& e ) { 
                throw mqo_reader_error( 
                        "line "+
                        boost::lexical_cast< std::string >( t.lineno() )+": " +
                        e.what() );
        
        }
}

} // namespace mqo_reader
