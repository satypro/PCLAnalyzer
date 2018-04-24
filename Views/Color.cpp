//
// ============================================================
//
//		Color.C
//
// ============================================================
//

#include <cstdlib>
#include <cmath>
#include <string>
#include <iostream>
#include "Color.h"

#define        TRUE            1
#define        FALSE           0

//
// ============================================================
//
//		GLOBALS
//
// ============================================================
//

int             _number_of_color_names = 0 ;
int             _length_of_color_name_cache = 0 ;

//string          _color_name[1000] ;
Color           _color_value[1000] ;
int             _color_name_cache[200] ;


//
// ============================================================
//
//		CONSTRUCTOR/DESTRUCTOR
//
// ============================================================
//
Color Color ::findColor(double v, double vmin, double vmax)
{

  double rr = 1.0, gg = 1.0, bb = 1.0 ;
  double dv;

  if (v < vmin) v = vmin;
  if (v > vmax) v = vmax ;
  dv = vmax - vmin;

  if (v < vmin + 0.25*dv) {
    rr = 0;
    gg = 4*(v - vmin)/dv;
  } else if (v < vmin + 0.5*dv) {
    rr = 0;
    bb = 1 + 4*(vmin + 0.25f*dv - v) / dv;
  } else if (v < vmin + 0.75*dv) {
    rr = 4*(v - vmin - 0.5f*dv) / dv;
    bb = 0;
  } else {
    gg = 1 + 4*(vmin + 0.75f*dv - v) / dv;
    bb = 0;
  }

  return Color(rr, gg, bb) ;

 /* double vavg = (vmax+vmin)*0.5 ;
  double ratio ;
  double dv = vmax - vmin ;

  if (v <= vavg) {
    ratio = (v-vmin)*2/dv ;
    return Color(0.0, ratio, 1) ;
  }
  else {
    double ratiocomp ;
    ratio = (v-vavg)*2/dv ;
    ratiocomp = 1- ratio ;
    return Color(ratio, ratiocomp, ratiocomp) ;
  }*/

}

Color :: Color ()

   {
      _red = 0.0 ;
      _green = 0.0 ;
      _blue = 0.0 ;
      _alpha = 1.0 ;
   }

Color :: Color ( const double c1,
         const double c2,
         const double c3,
         const ColorModel model )

   {
      _set_RGB ( c1, c2, c3, model ) ;
      _alpha = 1.0 ;
   }

Color :: Color ( const double c,
         const ColorModel model )

   {
      _set_RGB ( c, c, c, model ) ;
      _alpha = 1.0 ;
   }

Color :: Color ( const double c1,
         const double c2,
         const double c3,
         const double alpha,
         const ColorModel model )

   {
      _set_RGB ( c1, c2, c3, model ) ;
      _alpha = alpha ;
   }
/*
Color :: Color ( const string& c )

   {
      if ( _number_of_color_names == 0 ) {
     int r, g, b ;
     char name[80] ;
     ifstream cname_in ;

     cname_in.open ( "/usr/home/joy/local/lib/rgb.txt", ios::in ) ;

     for ( int i = 0 ; i < 1000 ; i++ ) {
        cname_in >> r >> g >> b >> name ;
        if ( cname_in.eof() ) break ;

        _color_value[i] = Color ( (double)r / 255.0,
                      (double)g / 255.0,
                      (double)b / 255.0 )  ;

        string n = string ( name ) ;
        n.lowercase() ;
        _color_name[i] = n ;
        _number_of_color_names++ ;
        }

     cname_in.close() ;
     }

      STring clower = c ;
      clower.lowercase() ;

      if ( _length_of_color_name_cache != 0 ) {
     for ( int i = 0 ; i < _length_of_color_name_cache ; i++ ) {
        if ( _color_name[_color_name_cache[i]] == clower ) {
           _red = _color_value[_color_name_cache[i]].red() ;
           _green = _color_value[_color_name_cache[i]].green() ;
           _blue = _color_value[_color_name_cache[i]].blue() ;
           return ;
           }
        }
     }

      for ( int i = 0 ; i < _number_of_color_names ; i++ ) {
     if ( _color_name[i] == clower ) {
        _red = _color_value[i].red() ;
        _green = _color_value[i].green() ;
        _blue = _color_value[i].blue() ;
        if ( _length_of_color_name_cache < 200 ) {
           _color_name_cache[_length_of_color_name_cache] = i ;
           _length_of_color_name_cache++ ;
           }
        return ;
        }
     }

      std::cerr << "Color " << c << "[" << clower
       << "] is not found" << std::endl ;
      exit ( 200 ) ;
   }
*/
Color :: Color ( const Color& c )

   {
      _red = c._red ;
      _green = c._green ;
      _blue = c._blue ;
      _alpha = c._alpha ;
   }

Color :: ~Color () {}

//
// ============================================================
//
//		ASSIGNMENT
//
// ============================================================
//

Color& Color :: operator= ( const Color& c )

   {
      if ( this == &c ) return ( *this ) ;

      _red = c._red ;
      _green = c._green ;
      _blue = c._blue ;
      _alpha = c._alpha ;

      return ( *this ) ;
   }

//
// ============================================================
//
//		OPERATOR<<
//
// ============================================================
//

std::ostream& operator<< ( std::ostream& co, const Color& c )

   {
      return c.output ( co ) ;
   }

//
// ============================================================
//
//		OUTPUT
//
// ============================================================
//

std::ostream& Color :: output ( std::ostream& co ) const

   {
      co << "Color ( "
     << _red << ", "
     << _green << ", "
     << _blue << ", "
     << _alpha << " )" ;

      return ( co ) ;
   }

//
// ============================================================
//
//		DARKER_BY
//
// ============================================================
//

void Color :: darker_by ( const double factor )

   {
      if ( ( factor < 0.0 ) || ( factor > 1.0 ) ) {
     std::cerr << "Darker_by factor of [" << factor
          << "is out of range" << std::endl ;
     exit ( 200 ) ;
     }


      operator*= ( 1.0 - factor ) ;
   }

//
// ============================================================
//
//		LIGHTER_BY
//
// ============================================================
//

void Color :: lighter_by ( const double factor )

   {
      if ( ( factor < 0.0 ) || ( factor > 1.0 ) ) {
     std::cerr << "Lighter_by factor of [" << factor
          << "is out of range" << std::endl ;
     exit ( 200 ) ;
     }

      operator+= ( Color ( factor, factor, factor ) ) ;
   }

//
// ============================================================
//
//		BRED
//
// ============================================================
//

unsigned char Color :: bred () const

   {
      int r = (int) ( _red * 255.0 );

      return ( (unsigned char) r ) ;
   }

//
// ============================================================
//
//		BGREEN
//
// ============================================================
//

unsigned char Color :: bgreen () const

   {
      int g = (int) ( _green * 255.0 ) ;

      return ( (unsigned char) g ) ;
   }


//
// ============================================================
//
//		BBLUE
//
// ============================================================
//

unsigned char Color :: bblue () const

   {
      int b = (int) ( _blue * 255.0 ) ;

      return ( (unsigned char) b ) ;
   }

//
// ============================================================
//
//		BALPHA
//
// ============================================================
//

unsigned char Color :: balpha () const

   {
      int a = (int) ( _alpha * 255.0 ) ;

      return ( (unsigned char) a ) ;
   }

/*
  ============================================================

            CVALUE

  ============================================================
*/

static	double	cvalue ( double c1, double c2, double h )

  {
      if ( h > 360.0 ) h -= 360.0 ;

      if ( h < 0.0 ) h += 360.0 ;

      if ( h < 60.0 )
        {
           return ( c1 + ( c2 - c1 ) * h / 60.0 )  ;
        }
     else if ( h < 180.0 )
        {
           return ( c2 ) ;
        }
     else if ( h < 240.0 )
        {
           return ( c1 + ( c2 - c1 ) * ( 240.0 - h ) / 60.00 ) ;
        }
     else {
        return ( c1 ) ;
        }

  }

/*
  ============================================================

            HLS_TO_RGB

  ============================================================
*/

void Color :: hlstorgb ( const double h, const double l, const double s )

  {
      double			m1, m2,
                cvalue ( double c1,
                    double c2,
                    double h ) ;


      if ( l <= 0.5 )
        m2 = l * ( 1.0 + s ) ;
     else
        m2 = l + s - l * s ;

      m1 = 2.0 * l - m2  ;

      if ( s == 0.0 )
        {
          // Achromatic Case

        _red = l  ;
        _green = l  ;
        _blue = l  ;
        }
     else
        {
          // Chromatic Case

        _red = cvalue ( m1, m2, h + 120.0 )  ;
        _green = cvalue ( m1, m2, h )  ;
        _blue = cvalue ( m1, m2, h - 120.0 )  ;
        }

      if ( _red < 0.0 ) _red = 0.0 ;
      if ( _red > 1.0 ) _red = 1.0 ;
      if ( _green < 0.0 ) _green = 0.0 ;
      if ( _green > 1.0 ) _green = 1.0 ;
      if ( _blue < 0.0 ) _blue = 0.0 ;
      if ( _blue > 1.0 ) _blue = 1.0 ;
  }


/*
  ============================================================

            MIN3

  ============================================================
*/

static double	min3 ( double x, double y, double z )

  {
      double		min ;

      min = x ;
      if ( y < min ) min = y ;
      if ( z < min ) min = z ;

      return ( min ) ;
  }

/*
  ============================================================

            MAX3

  ============================================================
*/

static double	max3 ( double x, double y, double z )

  {
      double		max ;

      max = x ;
      if ( y > max ) max = y ;
      if ( z > max ) max = z ;

      return ( max ) ;
  }


//
// ============================================================
//
//		Hue
//
// ============================================================
//

double Color :: hue () const

  {
      double		cmax, cmin ;
      double		h = 0.0 ;
      //double l,s;

      cmax = max3 ( _red, _green, _blue ) ;
      cmin = min3 ( _red, _green, _blue ) ;

      // Lightness Calculation

      //l = ( cmax + cmin ) * 0.5 ;

      // Saturation Calculation

      if ( cmax != cmin ) {

             // The Chromatic Case

    //Fabien commented these lines because of the warning at the complilation

    //    if ( l < 0.5 )
    //   s = ( cmax - cmin ) / ( cmax + cmin ) ;
    //     else
    //   s = ( cmax - cmin ) / ( 2.0 - cmax - cmin ) ;

              // Hue Calculation

        double rc = ( cmax - _red ) / ( cmax - cmin ) ;
        double gc = ( cmax - _green ) / ( cmax - cmin ) ;
        double bc = ( cmax - _blue ) / ( cmax - cmin ) ;

        if ( _red == cmax ) h = bc - gc ;
        if ( _green == cmax ) h = 2.0 + rc - bc ;
        if ( _blue == cmax ) h = 4.0 + gc - rc ;

        h *= 60.0 ;
        if ( h < 0.0 ) h += 360.0 ;
        }

      return ( h ) ;
  }


//
// ============================================================
//
//		Lightness
//
// ============================================================
//

double Color :: lightness() const

  {
      double		cmax, cmin ;

      cmax = max3 ( _red, _green, _blue ) ;
      cmin = min3 ( _red, _green, _blue ) ;

      // Lightness Calculation

      return ( ( cmax + cmin ) * 0.5 ) ;
  }

//
// ============================================================
//
//		Saturation
//
// ============================================================
//

double Color :: saturation () const

  {
      double		cmax, cmin ;
      double		s = 0.0 ;

      cmax = max3 ( _red, _green, _blue ) ;
      cmin = min3 ( _red, _green, _blue ) ;

      // Lightness Calculation

      double lightness = ( cmax + cmin ) * 0.5 ;

      // Saturation Calculation

      if ( cmax != cmin ) {

             // The Chromatic Case

        if ( lightness < 0.5 )
           s = ( cmax - cmin ) / ( cmax + cmin ) ;
           else
           s = ( cmax - cmin ) / ( 2.0 - cmax - cmin ) ;
        }

      return ( s ) ;
  }


/*
   ==========================================================

           COMPARISON OPERATIONS

   ==========================================================
*/

int operator== ( const Color& c1, const Color& c2 )

   {
      if ( ( c1._red == c2._red ) &&
       ( c1._green == c2._green ) &&
       ( c1._blue == c2._blue ) &&
       ( c1._alpha == c2._alpha ) )
        return ( TRUE ) ;
     else
        return ( FALSE ) ;
   }

int operator!= ( const Color& c1, const Color& c2 )

   {
      if ( ( c1._red != c2._red ) ||
       ( c1._green != c2._green ) ||
       ( c1._blue != c2._blue ) ||
       ( c1._alpha != c2._alpha ) )
        return ( TRUE ) ;
     else
        return ( FALSE ) ;
   }

/*
   ==========================================================

           ARITHMETIC OPERATIONS

   ==========================================================
*/

Color operator+ ( const Color& c1, const Color& c2 )

   {
      Color cc ;

      cc._red = c1._red + c2._red ;
      cc._green = c1._green + c2._green ;
      cc._blue = c1._blue + c2._blue ;
      cc._alpha = c1._alpha ;
      if ( c2._alpha > cc._alpha ) cc._alpha = c2._alpha ;

      return ( cc ) ;
   }

Color operator- ( const Color& c1, const Color& c2 )

   {
      Color cc ;

      cc._red = c1._red - c2._red ;
      cc._green = c1._green - c2._green ;
      cc._blue = c1._blue - c2._blue ;
      cc._alpha = c1._alpha ;
      if ( c2._alpha < cc._alpha ) cc._alpha = c2._alpha ;

      return ( cc ) ;
   }

Color operator* ( const double c, const Color& v )

   {
      Color cc ;

      cc._red = c * v._red ;
      cc._green = c * v._green ;
      cc._blue = c * v._blue ;
      cc._alpha = v._alpha ;

      return ( cc ) ;
   }

Color operator* ( const Color& v, const double c )

   {
      Color cc ;

      cc._red = c * v._red ;
      cc._green = c * v._green ;
      cc._blue = c * v._blue ;
      cc._alpha = v._alpha ;

      return ( cc ) ;
   }

Color operator* ( const Color& v, const Color& w )

   {
      Color cc ;

      cc._red = w._red * v._red ;
      cc._green = w._green * v._green ;
      cc._blue = w._blue * v._blue ;
      cc._alpha = w._alpha * v._alpha ;

      return ( cc ) ;
   }

Color operator/ ( const Color& v, const double c )

   {
      Color cc ;

      cc._red = v._red / c ;
      cc._green = v._green / c ;
      cc._blue = v._blue / c ;
      cc._alpha = v._alpha ;

      return ( cc ) ;
   }

Color& Color :: operator+= ( const Color& c )

   {
      _red += c._red ;
      _green += c._green ;
      _blue += c._blue ;
      if ( _alpha < c._alpha ) _alpha = c._alpha ;

      return *this ;
   }

Color& Color :: operator-= ( const Color& c )

   {
      _red -= c._red ;
      _green -= c._green ;
      _blue -= c._blue ;
      if ( _alpha > c._alpha ) _alpha = c._alpha ;

      return *this ;
   }

Color& Color :: operator*= ( const double c )

   {
      _red *= c ;
      _green *= c ;
      _blue *= c ;

      return *this ;
   }

Color& Color :: operator/= ( const double c )

   {
      _red /= c ;
      _green /= c ;
      _blue /= c ;

      return *this ;
   }

/*
   ==========================================================

           AFFINE

   ==========================================================
*/

Color affine ( const Color& c1,
           const Color& c2,
           const double& a1,
           const double& a2 )

   {
      return ( a1 * c1 + a2 * c2 ) ;
   }

Color affine ( const Color& c1,
           const Color& c2,
           const Color& c3,
           const double& a1,
           const double& a2,
           const double& a3 )

   {
      return ( a1 * c1 + a2 * c2 + a3 * c3 ) ;
   }


/*
   ==========================================================

           _set_RGB

   ==========================================================
*/

void Color :: _set_RGB ( const double c1,
             const double c2,
             const double c3,
             const ColorModel model )

   {
     double hue;
     double lightness;
     double saturation;

      switch ( model ) {
     case RGB :

        _red = c1 ;
        if ( _red < 0.0 ) _red = 0.0 ;
        if ( _red > 1.0 ) _red = 1.0 ;

        _green = c2 ;
        if ( _green < 0.0 ) _green = 0.0 ;
        if ( _green > 1.0 ) _green = 1.0 ;

        _blue = c3 ;
        if ( _blue < 0.0 ) _blue = 0.0 ;
        if ( _blue > 1.0 ) _blue = 1.0 ;
        break ;

     case HLS :

        hue = c1 ;
        while ( hue < 0.0 ) hue += 360.0 ;
        while ( hue > 360.0 ) hue -= 360.0 ;

        lightness = c2 ;
        if ( lightness < 0.0 ) lightness = 0.0 ;
        if ( lightness > 1.0 ) lightness = 1.0 ;

        saturation = c3 ;
        if ( saturation < 0.0 ) saturation = 0.0 ;
        if ( saturation > 1.0 ) saturation = 1.0 ;

        hlstorgb ( hue, lightness, saturation ) ;
        break ;

     case HSV :

        double hue1 = c1 ;
        while ( hue1 < 0.0 ) hue1 += 360.0 ;
        while ( hue1 > 360.0 ) hue1 -= 360.0 ;

        double lightness1 = c3 ;
        if ( lightness1 < 0.0 ) lightness1 = 0.0 ;
        if ( lightness1 > 1.0 ) lightness1 = 1.0 ;

        double saturation1 = c2 ;
        if ( saturation1 < 0.0 ) saturation1 = 0.0 ;
        if ( saturation1 > 1.0 ) saturation1 = 1.0 ;

        hlstorgb ( hue1, lightness1, saturation1 ) ;
        break ;
     }
   }
