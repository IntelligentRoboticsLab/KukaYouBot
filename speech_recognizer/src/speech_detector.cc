/**
 * @file    speech_detector.cc
 * @author  Thomas M. Howard (tmhoward@csail.mit.edu)
 * @version 1.0
 *
 * @section LICENSE
 *
 * TBD
 *
 * @section DESCRIPTION
 *
 * The implementation of a class used to capture and translate speech
 */

#include <sstream>
#include <sys/time.h>

#include <QtCore/QFile>
#include <QtGui/QLabel>
#include <QtGui/QVBoxLayout>
#include <QtGui/QGroupBox>
#include <QtGui/QGraphicsItem>

#include "speech_detector.h"

using namespace std;
using namespace h2sl;

QGraphicsItem_Microphone::
QGraphicsItem_Microphone( QGraphicsItem * parent ) : QObject(),
                                                     QGraphicsItem( parent ), 
                                                     _brush( Qt::black ), 
                                                     _pen( Qt::NoPen ), 
                                                     _is_pressed( false ) {

}

QGraphicsItem_Microphone::
~QGraphicsItem_Microphone(){

}

QRectF 
QGraphicsItem_Microphone::
boundingRect( void )const{
    return QRect( -55, -95, 110, 190 );
}
  
void 
QGraphicsItem_Microphone::
paint( QPainter * painter, const QStyleOptionGraphicsItem * option, QWidget * widget ){
    painter->setBrush( _brush );
    painter->setPen( _pen );
    painter->drawEllipse( -32.5, -87.5, 65, 65 );
    painter->drawRect( -32.5, -57.5, 65, 65 );
    painter->drawEllipse( -32.5, -27.5, 65, 65 );
    painter->setPen( QPen( _brush, 15, Qt::SolidLine, Qt::RoundCap ) );
    painter->drawArc( QRect( -45, -40, 90, 90 ), 180 * 16, 180 * 16 );
    painter->drawLine( 0, 60, 0, 80 );
    painter->drawLine( -30, 80, 30, 80 );
    painter->setPen( Qt::black );
    painter->setBrush( Qt::NoBrush );
    painter->setPen( Qt::white );
    if( _is_pressed ){
        painter->drawText( QRect( -50, -75, 100, 100 ), Qt::AlignCenter | Qt::TextWordWrap, QString( "recording" ) ); 
    } else {
        painter->drawText( QRect( -50, -75, 100, 100 ), Qt::AlignCenter | Qt::TextWordWrap, QString( "press\nand\nhold\nto\nrecord" ) ); 
    }
    return;
}

void
QGraphicsItem_Microphone:: 
mousePressEvent( QGraphicsSceneMouseEvent * event ){
    _brush = Qt::red;
    _is_pressed = true;
    emit microphone_pressed();
    update();
    return;
}

void 
QGraphicsItem_Microphone:: 
mouseReleaseEvent( QGraphicsSceneMouseEvent * event ){
    _brush = Qt::black;
    _is_pressed = false;
    emit microphone_released();
    update();
    return;
}

Speech_Detector::
Speech_Detector(const string& language,
                const unsigned int& samprate,
                const bool& verbose,
                QWidget * parent ) : Widget_H2SL( parent ),
                                     _combo_box_audio_devices( new QComboBox( this ) ),
                                     _graphics_scene_record( new QGraphicsScene( this ) ),
                                     _graphics_view_record( new QGraphicsView( _graphics_scene_record, this ) ),
                                     _graphics_item_microphone( new QGraphicsItem_Microphone() ),
                                     _line_edit_instruction( new QLineEdit( this ) ),
                                     _widget_h2sl_comments( new Widget_H2SL_Comments( this ) ),
                                     _available_devices( QAudioDeviceInfo::availableDevices( QAudio::AudioInput ) ),
                                     _audio_input( NULL ),
                                     _buffer_audio( new QBuffer() ),
                                     _file_audio( new QFile() ),
                                     _language( language ),
                                     _samprate( samprate ),
                                     _verbose ( verbose ) 
{

    _buffer_audio->open( QBuffer::ReadWrite );

    _combo_box_audio_devices->setMinimumWidth( 300 );
    _graphics_view_record->setMinimumSize( 200, 200 );
    _graphics_view_record->setStyleSheet( "background:transparent" ); 
 
    _graphics_item_microphone->setAcceptHoverEvents( true );
    _graphics_scene_record->addItem( _graphics_item_microphone );

    QLabel * label_title = new QLabel( "<b>Speech Detector</b>", this );  
    label_title->setSizePolicy( QSizePolicy::Fixed, QSizePolicy::Fixed );

    QGroupBox * group_box_audio = new QGroupBox( "audio" );
    QVBoxLayout * layout_audio = new QVBoxLayout();
    layout_audio->addWidget( _combo_box_audio_devices );
    layout_audio->addWidget( _graphics_view_record );
    layout_audio->addWidget( _line_edit_instruction );
    group_box_audio->setLayout( layout_audio );
    group_box_audio->setSizePolicy( QSizePolicy::MinimumExpanding, QSizePolicy::MinimumExpanding );

    _widget_h2sl_comments->group_box_widget()->setSizePolicy( QSizePolicy::MinimumExpanding, QSizePolicy::Fixed);

    QVBoxLayout * main_layout = new QVBoxLayout();
    main_layout->addWidget( label_title );
    main_layout->addWidget( group_box_audio );
    main_layout->addWidget( _widget_h2sl_comments->group_box_widget() );

    if( parent == NULL ){
        setLayout( main_layout );
    } else {

    }

    connect( dynamic_cast< QGraphicsItem_Microphone* >( _graphics_item_microphone ), 
            SIGNAL( microphone_pressed() ), this, SLOT( _microphone_pressed() ) );
    connect( dynamic_cast< QGraphicsItem_Microphone* >( _graphics_item_microphone ), 
            SIGNAL( microphone_released() ), this, SLOT( _microphone_released() ) );
    connect( _combo_box_audio_devices, SIGNAL( currentIndexChanged( int ) ), this, 
            SLOT( _combo_box_audio_devices_changed( int ) ) );

    for( unsigned int i = 0; i < _available_devices.size(); i++ ){
        stringstream comment;
        comment << "found <i>" << _available_devices[ i ].deviceName().toStdString() << "</i>";
        _widget_h2sl_comments->add_comment( comment.str() );
        _combo_box_audio_devices->addItem( _available_devices[ i ].deviceName() );
    }

    _widget_h2sl_comments->add_comment( "waiting for speech input" ); 

    lcm = lcm_create (NULL);
    if (!lcm) 
        cerr << "ERROR: Unable to create LCM instance" << endl;


    /*
     * Create the publisher for the transcoding message.
     */
    ros::NodeHandle node;

    _transcoding_publisher = node.advertise<std_msgs::String>("/transcoding", 1000);

    //char *response = sprec_get_text_from_json( '{"result":[{"alternative":[{"transcript":"I love you","confidence":0.9304716}],"final":true}],"result_index":0}")');


    //cout << "Single response= " << response << endl;



}

Speech_Detector::
~Speech_Detector() {
}

Speech_Detector::
Speech_Detector( const Speech_Detector& other ) {

}

Speech_Detector&
Speech_Detector::
operator=( const Speech_Detector& other ) {

    return (*this);
}

void
Speech_Detector::
_send_ti( const QString& msg ){
    return;
}

void
Speech_Detector::
_microphone_pressed( void ){
    //_file_audio->setFileName( "/tmp/test_audio.raw" );
    //_file_audio->open( QIODevice::WriteOnly | QIODevice::Truncate );

    int32_t timestamp = timestamp_now() * 1E-6;
    std::ostringstream audio_file_name;
    audio_file_name <<  "/tmp/" << timestamp << ".wav";
    cout << "Saving audio to file:" << audio_file_name.str() << endl;

    // TODO remove comment
    _last_wav_file_name = audio_file_name.str();
    //_last_wav_file_name = "/tmp/1411240705.wav";

    _wav_file_audio = new WavPcmFile (QString::fromStdString(_last_wav_file_name), _audio_input->format(), this);
    _wav_file_audio->dumpObjectInfo();

    if (_wav_file_audio->open()) {
        _widget_h2sl_comments->add_comment( "starting capture" );
        _audio_input->start(_wav_file_audio);
    }

    return;
}

void
Speech_Detector::
_microphone_released( void ){
    if( _audio_input != NULL ){
        _widget_h2sl_comments->add_comment( "ending capture" );
        _audio_input->stop();
        _wav_file_audio->close();

        // Convert to flac for Google
        if (sprec_flac_encode (_last_wav_file_name.c_str(), "/tmp/test_audio.flac"))
            cout << "Error: Unable to convert wav file to flac" << endl;
        else {
            // Send the contents of the flac file to Google
            char *flac_file_buf;
            size_t flac_file_len;
            if (_verbose)
                cout << "Getting contents of FLAC file" << endl;
            sprec_get_file_contents ("/tmp/test_audio.flac",  (void **)&flac_file_buf, &flac_file_len);



            struct sprec_server_response *resp;
            char *text;
            double confidence;
            if (_verbose)
                cout << "Sending audio data to google" << endl;
            resp = sprec_send_audio_data(flac_file_buf, flac_file_len, "_language", _samprate);




            //free(flac_file_buf);

        
            /*
             * Get the JSON from the response object,
             * then parse it to get the actual text
             */
            /*
            if (_verbose)
                fprintf (stdout, "Google returned: length data = %s\n", resp->data);

            if (resp->length > 0) {
                text = sprec_get_text_from_json(resp->data);
                confidence = sprec_get_confidence_from_json(resp->data);
            }
            sprec_free_response(resp);
        

            */



            text = "stop";
            if( strstr( resp->data, "rotate" ) != NULL ) {
                if( strstr( resp->data, "left" ) != NULL ) text = "rotate left";
                if( strstr( resp->data, "right" ) != NULL ) text = "rotate right";
            } else {
                if( strstr( resp->data, "forward" ) != NULL ) text = "forward";
                if( strstr( resp->data, "backward" ) != NULL ) text = "backward";
                if( strstr( resp->data, "left" ) != NULL ) text = "left";
                if( strstr( resp->data, "right" ) != NULL ) text = "right";
            }

            if (text)
                cout << "Text: " << text << " (Confidence = " << confidence*100 << ")" << endl;
            else
                cout << "Text: (no text recognized)" << endl;

            /*
             * Publish a String message on /transcoding that contains the
             * transcoded version of the string.
             */
            if (text) {
                std_msgs::String transcoding_msg;
                transcoding_msg.data = text;

                _transcoding_publisher.publish(transcoding_msg);

                ros::spinOnce();
            }


        }
        //_file_audio->close();
    }

    return;
}

void
Speech_Detector::
_combo_box_audio_devices_changed( int index ){
    if( ( index < _available_devices.size() ) && ( index >= 0 ) ){
        _open_audio_input( _available_devices[ index ] );
    } 
    return;
}

void
Speech_Detector::
resizeEvent( QResizeEvent * event ){
    _graphics_view_record->fitInView( _graphics_scene_record->sceneRect(), Qt::KeepAspectRatio );
    return;
}

void
Speech_Detector::
_open_audio_input( QAudioDeviceInfo& audioDeviceInfo ){
    if( _audio_input != NULL ){
        delete _audio_input;
        _audio_input = NULL;
    }
    QAudioFormat audio_format = audioDeviceInfo.preferredFormat();
    audio_format.setByteOrder( QAudioFormat::LittleEndian );
    audio_format.setSampleRate( _samprate );
    cout << "Setting samplerate to " << _samprate << "Hz" << endl;
    audio_format.setSampleSize( 16 );
    audio_format.setSampleType( QAudioFormat::SignedInt );
    audio_format.setChannelCount( 1 );

    QList< QAudioFormat::Endian > supported_byte_orders = audioDeviceInfo.supportedByteOrders();
 
    if (_verbose)
        cout << "supported_byte_orders.size(): " << supported_byte_orders.size() << endl;

    QList< int > supported_channel_counts = audioDeviceInfo.supportedChannelCounts();
    QList< int > supported_sample_rates = audioDeviceInfo.supportedSampleRates();
    QList< int > supported_sample_sizes = audioDeviceInfo.supportedSampleSizes();

    if (_verbose) {
        cout << "supported_channel_counts[" << supported_channel_counts.size() << "]:{";
        for( unsigned int i = 0; i < supported_channel_counts.size(); i++ ){
            cout << supported_channel_counts[ i ];
            if( i != ( supported_channel_counts.size() - 1 ) ){
                cout << ",";
            }
        }
        cout << "}" << endl;
        
        cout << "supported_sample_rates[" << supported_sample_rates.size() << "]:{";
        for( unsigned int i = 0; i < supported_sample_rates.size(); i++ ){
            cout << supported_sample_rates[ i ];
            if( i != ( supported_sample_rates.size() - 1 ) ){
                cout << ",";
            }
        }
        cout << "}" << endl;
        
        cout << "supported_sample_sizes[" << supported_sample_sizes.size() << "]:{";
        for( unsigned int i = 0; i < supported_sample_sizes.size(); i++ ){
            cout << supported_sample_sizes[ i ];
            if( i != ( supported_sample_sizes.size() - 1 ) ){
                cout << ",";
            }
        }
        cout << "}" << endl;
    }

    if (!supported_sample_rates.contains (_samprate)) 
        cout << "Error: Requested sample rate " << _samprate << "Hz is not supported" << endl;

    stringstream comment;
    comment << "opening <i>" << audioDeviceInfo.deviceName().toStdString() << "</u>";
    _widget_h2sl_comments->add_comment( comment.str() );
    comment.str( "" );
    comment << "codec is " << audio_format.codec().toStdString();
    _widget_h2sl_comments->add_comment( comment.str() );
    comment.str( "" );
    comment << "byte order is " << audio_format.byteOrder();
    _widget_h2sl_comments->add_comment( comment.str() );
    comment.str( "" );
    comment << "channel count is " << audio_format.channelCount();
    _widget_h2sl_comments->add_comment( comment.str() );
    comment.str( "" );
    comment << "sample rate is " << audio_format.sampleRate();
    _widget_h2sl_comments->add_comment( comment.str() );
    comment.str( "" );
    comment << "sample size is " << audio_format.sampleSize();
    _widget_h2sl_comments->add_comment( comment.str() );
    comment.str( "" );
    comment << "sample type is " << audio_format.sampleType();
    _widget_h2sl_comments->add_comment( comment.str() );
    _audio_input = new QAudioInput( audioDeviceInfo, audio_format, this );
    return;
}

int64_t
Speech_Detector::
timestamp_now () {
    struct timeval tv;
    gettimeofday (&tv, NULL);
    return (int64_t) tv.tv_sec * 1000000 + tv.tv_usec;
}

namespace h2sl {
    ostream&
    operator<<( ostream& out,
               const Speech_Detector& other ) {
        return out;
    }
}

