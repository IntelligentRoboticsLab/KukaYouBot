/**
 * @file    speech_detector.h
 * @author  Thomas M. Howard (tmhoward@csail.mit.edu)
 * @version 1.0
 *
 * @section LICENSE
 *
 * TBD
 *
 * @section DESCRIPTION
 *
 * The interface for a class used to capture and translate speech
 */

#ifndef H2SL_SPEECH_DETECTOR_H
#define H2SL_SPEECH_DETECTOR_H

#include <iostream>

#include <QtCore/QBuffer>
#include <QtCore/QFile>
#include <QtGui/QWidget>
#include <QtGui/QComboBox>
#include <QtGui/QLineEdit>
#include <QtGui/QPushButton>
#include <QtGui/QGraphicsScene>
#include <QtGui/QGraphicsItem>
#include <QtGui/QGraphicsView>
#include <QAudioInput>
#include <QAudioDeviceInfo>

#include "ros/ros.h"
#include "std_msgs/String.h"

#include <lcm/lcm.h>

#include <google_recognition/sprec.h>

#include "widget_h2sl.h"
#include "widget_h2sl_comments.h"
#include "wav_file.h"

namespace h2sl {
  class QGraphicsItem_Microphone : public QObject, public QGraphicsItem {
    Q_OBJECT
  public:
    QGraphicsItem_Microphone( QGraphicsItem * parent = NULL );
    virtual ~QGraphicsItem_Microphone();

    void paint( QPainter * painter, const QStyleOptionGraphicsItem * option, QWidget * widget );
    QRectF boundingRect( void )const;

  signals:
    void microphone_pressed( void );
    void microphone_released( void );

  protected:
    virtual void mousePressEvent( QGraphicsSceneMouseEvent * event );
    virtual void mouseReleaseEvent( QGraphicsSceneMouseEvent * event );

    QBrush _brush;
    QPen _pen;
    bool _is_pressed;
  };

  class Speech_Detector : public Widget_H2SL {
    Q_OBJECT
  public:
    Speech_Detector( const std::string& language = "en-US", const unsigned int& samprate = 8000, const bool& verbose = false, QWidget * parent = NULL );
    virtual ~Speech_Detector();
    Speech_Detector( const Speech_Detector& other );
    Speech_Detector& operator=( const Speech_Detector& other );

  signals:
    void speech_detected( const QString& instruction );

  protected slots:
    virtual void _send_ti( const QString& msg );
    void _microphone_pressed( void );
    void _microphone_released( void );
    void _combo_box_audio_devices_changed( int index );

  protected:
    virtual void resizeEvent( QResizeEvent * event );
    void _open_audio_input( QAudioDeviceInfo& audioDeviceInfo );
    int64_t timestamp_now ();

    QComboBox * _combo_box_audio_devices;
    QGraphicsScene * _graphics_scene_record;
    QGraphicsView * _graphics_view_record;
    QGraphicsItem * _graphics_item_microphone;
    QLineEdit * _line_edit_instruction;
    Widget_H2SL_Comments * _widget_h2sl_comments;

    QList< QAudioDeviceInfo > _available_devices;
    QAudioInput * _audio_input;
    QBuffer * _buffer_audio;
    QFile * _file_audio;
    WavPcmFile *_wav_file_audio;
    std::string _last_wav_file_name;
    ros::Publisher _transcoding_publisher;

    lcm_t *lcm;

    std::string _language;
    unsigned int _samprate;
    bool _verbose;

  private:

  };
  std::ostream& operator<<( std::ostream& out, const Speech_Detector& other );
}

#endif /* H2SL_SPEECH_DETECTOR_H */
