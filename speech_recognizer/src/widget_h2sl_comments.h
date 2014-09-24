/**
 * @file    widget_h2sl_comments.h
 * @author  Thomas M. Howard (tmhoward@csail.mit.edu)
 * @version 1.0
 *
 * @section LICENSE
 *
 * TBD
 *
 * @section DESCRIPTION
 *
 * The interface for a class used to display and save comments
 */

#ifndef H2SL_WIDGET_H2SL_COMMENTS_H
#define H2SL_WIDGET_H2SL_COMMENTS_H

#include <iostream>

#include <QtGui/QTextBrowser>
#include <QtGui/QPushButton>

#include "widget_h2sl.h"

namespace h2sl {
  class Widget_H2SL_Comments : public Widget_H2SL {
    Q_OBJECT
  public:
    Widget_H2SL_Comments( QWidget * parent = NULL );
    virtual ~Widget_H2SL_Comments();
    Widget_H2SL_Comments( const Widget_H2SL_Comments& other );
    Widget_H2SL_Comments& operator=( const Widget_H2SL_Comments& other );

    void add_comment( const std::string& comment, const bool& error = false );

  protected slots:
    void _push_button_save_pressed( void );

  protected:
    QString _format_comment( const std::string& comment, const bool& error = false );

    QTextBrowser * _text_browser_comments;
    QPushButton * _push_button_save;

  private:

  };
  std::ostream& operator<<( std::ostream& out, const Widget_H2SL_Comments& other );
}

#endif /* H2SL_WIDGET_H2SL_COMMENTS_H */
