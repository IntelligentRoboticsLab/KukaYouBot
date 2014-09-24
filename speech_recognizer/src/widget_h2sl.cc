/**
 * @file    widget_h2sl.cc
 * @author  Thomas M. Howard (tmhoward@csail.mit.edu)
 * @version 1.0
 *
 * @section LICENSE
 *
 * TBD
 *
 * @section DESCRIPTION
 *
 * The implementation of a base class for all h2sl widgets
 */

#include <QtCore/QTime>

#include "widget_h2sl.h"

using namespace std;
using namespace h2sl;

Widget_H2SL::
Widget_H2SL( QWidget * parent ) : QWidget( parent ),
                                  _group_box_widget( NULL ) {

}

Widget_H2SL::
~Widget_H2SL() {

}

Widget_H2SL::
Widget_H2SL( const Widget_H2SL& other ) : QWidget(){

}

Widget_H2SL&
Widget_H2SL::
operator=( const Widget_H2SL& other ) {

  return (*this);
}

namespace h2sl {
  ostream&
  operator<<( ostream& out,
              const Widget_H2SL& other ) {
    return out;
  }

}
