/*!
  @file     svgout.hpp
  @brief    <ŠT—v>

  <à–¾>
  $Id: svgout.hpp 32 2008-10-25 12:19:56Z Naoyuki.Hirayama $
*/
#ifndef SVGOUT_HPP
#define SVGOUT_HPP

#include <fstream>
#include <iomanip>

class svgout {
public:
    svgout( const char* filename, size_t w, size_t h ) : ofs(filename)
    {
        ofs << std::setprecision(6) << std::fixed;
        ofs
            << "<?xml version=\"1.0\" standalone=\"no\"?>" << std::endl
            << "<!DOCTYPE svg PUBLIC \"-//W3C//DTD SVG 1.1//EN\" " << std::endl
            << "  \"http://www.w3.org/Graphics/SVG/1.1/DTD/svg11.dtd\">"
            << std::endl
            << "<svg width=\"" << w << "\" height=\""
            << h << "\" version=\"1.1\" "
            << "xmlns=\"http://www.w3.org/2000/svg\">" << std::endl;
    }
    ~svgout()
    {
        ofs << "</svg>" << std::endl;
    }

    void rect(
        float x, float y,
        float w, float h,
        const char* stroke = NULL,
        const char* fill = NULL,
        float opacity = 1.0f )
    {
        if( stroke == NULL ) { stroke = "none"; }
        if( fill == NULL ) { fill = "gray"; }

        ofs << "<rect "
            << "stroke=\"" << stroke << "\" fill=\"" << fill << "\" "
            << "x=\"" << x << "\" y=\"" << y << "\" "
            << "width=\"" << w << "\" height=\"" << h << "\" "
            << "opacity=\"" << opacity << "\" "
            << "/>\n";
    }

    void line(
        float x0, float y0,
        float x1, float y1,
        const char* stroke = NULL,
        const char* fill = NULL,
        float opacity = 1.0f )
    {
        if( stroke == NULL ) { stroke = "gray"; }
        if( fill == NULL ) { fill = "none"; }

        ofs << "<line "
            << "stroke=\"" << stroke << "\" fill=\"" << fill << "\" "
            << "x1=\"" << x0 << "\" y1=\"" << y0 << "\" "
            << "x2=\"" << x1 << "\" y2=\"" << y1 << "\" "
            << "opacity=\"" << opacity << "\" "
            << "/>\n";
    }

    void circle( 
        float x, float y, float r, 
        const char* stroke = NULL, 
        const char* fill = NULL, 
        float opacity = 1.0f )
    { 
        if( stroke == NULL ) { stroke = "none"; }
        if( fill == NULL ) { fill = "gray"; }
                
        ofs << "<circle "
            << "stroke=\"" << stroke << "\" fill=\"" << fill << "\" "
            << "cx=\"" << x << "\" "
            << "cy=\"" << y << "\" "
            << "r=\"" << r << "\" "
            << "opacity=\"" << opacity << "\" "
            << "/>\n" << std::endl ; 
    }

    void text(
        float x, float y,
        const char* text,
        const char* stroke = NULL,
        const char* fill = NULL )
    { 
        if( stroke == NULL ) { stroke = "none"; }
        if( fill == NULL ) { fill = "black"; }
                
        ofs << "<text "
            << "stroke=\"" << stroke << "\" fill=\"" << fill << "\" "
            << "x=\"" << x << "\" "
            << "y=\"" << y << "\""
            << ">"
            << text
            << "</text>\n";
    }
        
private:
    std::ofstream ofs;

};

#endif // SVGOUT_HPP
