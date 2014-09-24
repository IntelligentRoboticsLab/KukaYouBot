/****************************************************************************
** Meta object code from reading C++ file 'speech_detector.h'
**
** Created: Mon Sep 22 19:41:29 2014
**      by: The Qt Meta Object Compiler version 63 (Qt 4.8.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../speech_recognizer/src/speech_detector.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'speech_detector.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_h2sl__QGraphicsItem_Microphone[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
       2,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       2,       // signalCount

 // signals: signature, parameters, type, tag, flags
      32,   31,   31,   31, 0x05,
      53,   31,   31,   31, 0x05,

       0        // eod
};

static const char qt_meta_stringdata_h2sl__QGraphicsItem_Microphone[] = {
    "h2sl::QGraphicsItem_Microphone\0\0"
    "microphone_pressed()\0microphone_released()\0"
};

void h2sl::QGraphicsItem_Microphone::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        QGraphicsItem_Microphone *_t = static_cast<QGraphicsItem_Microphone *>(_o);
        switch (_id) {
        case 0: _t->microphone_pressed(); break;
        case 1: _t->microphone_released(); break;
        default: ;
        }
    }
    Q_UNUSED(_a);
}

const QMetaObjectExtraData h2sl::QGraphicsItem_Microphone::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject h2sl::QGraphicsItem_Microphone::staticMetaObject = {
    { &QObject::staticMetaObject, qt_meta_stringdata_h2sl__QGraphicsItem_Microphone,
      qt_meta_data_h2sl__QGraphicsItem_Microphone, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &h2sl::QGraphicsItem_Microphone::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *h2sl::QGraphicsItem_Microphone::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *h2sl::QGraphicsItem_Microphone::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_h2sl__QGraphicsItem_Microphone))
        return static_cast<void*>(const_cast< QGraphicsItem_Microphone*>(this));
    if (!strcmp(_clname, "QGraphicsItem"))
        return static_cast< QGraphicsItem*>(const_cast< QGraphicsItem_Microphone*>(this));
    return QObject::qt_metacast(_clname);
}

int h2sl::QGraphicsItem_Microphone::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 2)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 2;
    }
    return _id;
}

// SIGNAL 0
void h2sl::QGraphicsItem_Microphone::microphone_pressed()
{
    QMetaObject::activate(this, &staticMetaObject, 0, 0);
}

// SIGNAL 1
void h2sl::QGraphicsItem_Microphone::microphone_released()
{
    QMetaObject::activate(this, &staticMetaObject, 1, 0);
}
static const uint qt_meta_data_h2sl__Speech_Detector[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
       5,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       1,       // signalCount

 // signals: signature, parameters, type, tag, flags
      35,   23,   22,   22, 0x05,

 // slots: signature, parameters, type, tag, flags
      64,   60,   22,   22, 0x09,
      82,   22,   22,   22, 0x09,
     104,   22,   22,   22, 0x09,
     133,  127,   22,   22, 0x09,

       0        // eod
};

static const char qt_meta_stringdata_h2sl__Speech_Detector[] = {
    "h2sl::Speech_Detector\0\0instruction\0"
    "speech_detected(QString)\0msg\0"
    "_send_ti(QString)\0_microphone_pressed()\0"
    "_microphone_released()\0index\0"
    "_combo_box_audio_devices_changed(int)\0"
};

void h2sl::Speech_Detector::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        Speech_Detector *_t = static_cast<Speech_Detector *>(_o);
        switch (_id) {
        case 0: _t->speech_detected((*reinterpret_cast< const QString(*)>(_a[1]))); break;
        case 1: _t->_send_ti((*reinterpret_cast< const QString(*)>(_a[1]))); break;
        case 2: _t->_microphone_pressed(); break;
        case 3: _t->_microphone_released(); break;
        case 4: _t->_combo_box_audio_devices_changed((*reinterpret_cast< int(*)>(_a[1]))); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData h2sl::Speech_Detector::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject h2sl::Speech_Detector::staticMetaObject = {
    { &Widget_H2SL::staticMetaObject, qt_meta_stringdata_h2sl__Speech_Detector,
      qt_meta_data_h2sl__Speech_Detector, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &h2sl::Speech_Detector::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *h2sl::Speech_Detector::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *h2sl::Speech_Detector::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_h2sl__Speech_Detector))
        return static_cast<void*>(const_cast< Speech_Detector*>(this));
    return Widget_H2SL::qt_metacast(_clname);
}

int h2sl::Speech_Detector::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = Widget_H2SL::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 5)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 5;
    }
    return _id;
}

// SIGNAL 0
void h2sl::Speech_Detector::speech_detected(const QString & _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}
QT_END_MOC_NAMESPACE
