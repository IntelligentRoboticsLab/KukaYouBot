/**
 * @file    widget_h2sl.h
 * @author  Thomas M. Howard (tmhoward@csail.mit.edu)
 * @version 1.0
 *
 * @section LICENSE
 *
 * TBD
 *
 * @section DESCRIPTION
 *
 * The interface for a base class for all h2sl widgets
 */

#ifndef H2SL_WIDGET_H2SL_H
#define H2SL_WIDGET_H2SL_H

#include <iostream>

#include <QtGui/QWidget>
#include <QtGui/QGroupBox>

namespace h2sl {
  class Widget_H2SL : public QWidget {
    Q_OBJECT
  public:
    Widget_H2SL( QWidget * parent = NULL );
    virtual ~Widget_H2SL();
    Widget_H2SL( const Widget_H2SL& other );
    Widget_H2SL& operator=( const Widget_H2SL& other );

    inline QGroupBox*& group_box_widget( void ){ return _group_box_widget; };
    inline const QGroupBox * group_box_widget( void )const{ return _group_box_widget; };
  
  protected:
    QGroupBox * _group_box_widget;

  private:

  };
  std::ostream& operator<<( std::ostream& out, const Widget_H2SL& other );
}

#endif /* H2SL_WIDGET_H2SL_H */
