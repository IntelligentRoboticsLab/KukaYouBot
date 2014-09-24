/**
 * @file    wav_file.h
 * @author  Matthew Walter (mwalter@csail.mit.edu)
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

#ifndef H2SL_WAV_FILE_H
#define H2SL_WAV_FILE_H

#include <iostream>
#include <QAudioInput>
#include <QAudioDeviceInfo>


namespace h2sl {
    class WavPcmFile : public QFile {
    public:
        WavPcmFile(const QString & name, const QAudioFormat & format, QObject *parent = 0);
        bool open();
        void close();
 
    private:
        void writeHeader();
        bool hasSupportedFormat();
        QAudioFormat format;
    };
}

#endif /*  H2SL_WAV_FILE_H */
