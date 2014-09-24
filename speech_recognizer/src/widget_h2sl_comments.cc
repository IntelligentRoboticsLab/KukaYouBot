/**
 * @file    widget_h2sl_comments.cc
 * @author  Thomas M. Howard (tmhoward@csail.mit.edu)
 * @version 1.0
 *
 * @section LICENSE
 *
 * TBD
 *
 * @section DESCRIPTION
 *
 * The implementation of a class used to display and save comments
 */

#include <fstream>

#include <QtCore/QTime>
#include <QtGui/QGroupBox>
#include <QtGui/QVBoxLayout>
#include <QtGui/QFileDialog>

#include "widget_h2sl_comments.h"

using namespace std;
using namespace h2sl;

Widget_H2SL_Comments::
Widget_H2SL_Comments( QWidget * parent ) : Widget_H2SL( parent ),
                                            _text_browser_comments( new QTextBrowser( this ) ),
                                            _push_button_save( new QPushButton( "save", this ) ){
  _text_browser_comments->setFixedHeight( 75 );

  QVBoxLayout * layout_main = new QVBoxLayout();
  layout_main->addWidget( _text_browser_comments );
  layout_main->addWidget( _push_button_save );
  
  if( parent == NULL ){
    setLayout( layout_main );
  } else {
    _group_box_widget = new QGroupBox( "comments" );
    _group_box_widget->setLayout( layout_main );
    QVBoxLayout * layout_widget = new QVBoxLayout();
    layout_widget->addWidget( _group_box_widget );
    setLayout( layout_widget ); 
  }

  connect( _push_button_save, SIGNAL( clicked() ), this, SLOT( _push_button_save_pressed() ) ); 
}

Widget_H2SL_Comments::
~Widget_H2SL_Comments() {

}

Widget_H2SL_Comments::
Widget_H2SL_Comments( const Widget_H2SL_Comments& other ) {

}

Widget_H2SL_Comments&
Widget_H2SL_Comments::
operator=( const Widget_H2SL_Comments& other ) {

  return (*this);
}

void
Widget_H2SL_Comments::
add_comment( const string& comment,
              const bool& error ){
  _text_browser_comments->append( _format_comment( comment, error ) );
  return;
}

void
Widget_H2SL_Comments::
_push_button_save_pressed( void ){
  QString filename = QFileDialog::getSaveFileName( this, "Please choose the filename for saving the contents of the comments box" );
  if( !filename.isEmpty() ){
    ofstream outfile;
    outfile.open( filename.toStdString().c_str() );
    if( !outfile.fail() ){
      outfile << _text_browser_comments->toPlainText().toStdString();
      outfile.close();
    }
  }
  return;
}

QString
Widget_H2SL_Comments::
_format_comment( const string& comment,
                  const bool& error ){
  QString formatted_comment;
  if( error ){
    formatted_comment.append( "<font color=\"red\">" );
  }
  formatted_comment.append( "(" );
  formatted_comment.append( QTime::currentTime().toString( Qt::TextDate ) );
  formatted_comment.append( ") " );
  formatted_comment.append( QString::fromStdString( comment ) );
  if( error ){
    formatted_comment.append( "</font>" );
  }
  return formatted_comment;
}

namespace h2sl {
  ostream&
  operator<<( ostream& out,
              const Widget_H2SL_Comments& other ) {
    return out;
  }

}
