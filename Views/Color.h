#ifndef COLOR_H
#define COLOR_H
//
// ============================================================
//
//		Color
//
// ============================================================
//
//
//      Copyright (C) 1997
//                      Professor Kenneth I. Joy
//                      Computer Science Department
//                      University of California
//                      Davis, CA  95616
//
//      Permission is granted to use at your own risk and
//      distribute this software in source and binary forms
//      provided the above copyright notice and this paragraph
//      are preserved on all copies.  This software is provided
//      "as is" with no express or implied warranty.
//
//
//  ============================================================
//

#include <string.h>
#include <stdio.h>
#include <iostream>

enum ColorModel { RGB,		// RGB will be the default
          HSV,		// Hue, Saturation, Value
          HLS		// Hue, Lightness, Saturation
          } ;

class Color {

   private :

      double _red,
            _green,
            _blue ;

      double _alpha ;

      void hlstorgb ( const double, const double, const double ) ;

   private :

      void _set_RGB ( const double,
              const double,
              const double,
              const ColorModel = RGB ) ;

   public :

      Color () ;

      Color ( const double c,const ColorModel = RGB ) ;

      Color ( const double,
          const double,
          const double,
          const ColorModel = RGB ) ;

      Color ( const double,		//  Red
          const double,		//  Green
          const double,		//  Blue
          const double,		//  Alpha
          const ColorModel = RGB ) ;

      //  Copy Constructors
      Color ( const Color& ) ;

      //  Destructors
      virtual ~Color () ;

      //  Assignment
      Color& operator= ( const Color& ) ;

      // Output
      Color findColor(double v, double vmin, double vmax);

      friend std::ostream& operator<< ( std::ostream&, const Color& ) ;
      std::ostream& output ( std::ostream& ) const ;

      //  Comparison
      friend int operator== ( const Color&, const Color& ) ;
      friend int operator!= ( const Color&, const Color& ) ;

      friend Color operator+ ( const Color&, const Color& ) ;
      friend Color operator- ( const Color&, const Color& ) ;
      friend Color operator* ( const double, const Color& ) ;
      friend Color operator* ( const Color&, const double ) ;
      friend Color operator* ( const Color&, const Color& ) ;
      friend Color operator/ ( const Color&, const double ) ;

      Color& operator+= ( const Color& ) ;
      Color& operator-= ( const Color& ) ;
      Color& operator*= ( const double ) ;
      Color& operator/= ( const double ) ;

      friend Color affine ( const Color&,
                const Color&,
                const double&,
                const double& ) ;

      friend Color affine ( const Color&,
                const Color&,
                const Color&,
                const double&,
                const double&,
                const double& );

      //  Change the lightness
      void darker_by ( const double );
      void lighter_by ( const double );
      void make_opaque()
        { _alpha = 1.0 ; }

      //  Access Functions
      double hue() const ;
      double saturation() const ;
      double lightness() const ;
      double value() const { return lightness() ; }

      double red() const { return _red ; }
      double green() const { return _green ; }
      double blue() const { return _blue ; }
      double alpha() const { return _alpha ; }

      unsigned char bred() const ;
      unsigned char bgreen() const ;
      unsigned char bblue() const ;
      unsigned char balpha() const ;

      void set_red ( const double r )
            { _red = r ; }
      void set_green ( const double g )
            { _green = g ; }
      void set_blue ( const double b )
            { _blue = b ; }
      void set_alpha ( const double a )
            { _alpha = a ; }
};

const Color White (  1.0,  1.0,  1.0 ) ;
const Color Black (  0.0,  0.0,  0.0 ) ;
const Color Red (  1.0,  0.0,  0.0 ) ;
const Color Green (  0.0,  1.0,  0.0 ) ;
const Color Blue (  0.0,  0.0,  1.0 ) ;
const Color Cyan (  0.0,  1.0,  1.0 ) ;
const Color Yellow (  1.0,  1.0,  0.0 ) ;
const Color Magenta (  1.0,  0.0,  1.0 ) ;
const Color Orange (  1.0, 0.65,  0.0 ) ;

const Color Gray10 ( 0.1, 0.1, 0.1 ) ;
const Color Gray20 ( 0.2, 0.2, 0.2 ) ;
const Color Gray30 ( 0.3, 0.3, 0.3 ) ;
const Color Gray40 ( 0.4, 0.4, 0.4 ) ;
const Color Gray50 ( 0.5, 0.5, 0.5 ) ;
const Color Gray60 ( 0.6, 0.6, 0.6 ) ;
const Color Gray70 ( 0.7, 0.7, 0.7 ) ;
const Color Gray80 ( 0.8, 0.8, 0.8 ) ;
const Color Gray90 ( 0.9, 0.9, 0.9 ) ;

#endif // COLOR_H
