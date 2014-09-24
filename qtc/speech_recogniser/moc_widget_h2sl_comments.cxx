/****************************************************************************
** Meta object code from reading C++ file 'widget_h2sl_comments.h'
**
** Created: Mon Sep 22 19:41:29 2014
**      by: The Qt Meta Object Compiler version 63 (Qt 4.8.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../speech_recognizer/src/widget_h2sl_comments.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'widget_h2sl_comments.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_h2sl__Widget_H2SL_Comments[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
       1,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: signature, parameters, type, tag, flags
      28,   27,   27,   27, 0x09,

       0        // eod
};

static const char qt_meta_stringdata_h2sl__Widget_H2SL_Comments[] = {
    "h2sl::Widget_H2SL_Comments\0\0"
    "_push_button_save_pressed()\0"
};

void h2sl::Widget_H2SL_Comments::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        Widget_H2SL_Comments *_t = static_cast<Widget_H2SL_Comments *>(_o);
        switch (_id) {
        case 0: _t->_push_button_save_pressed(); break;
        default: ;
        }
    }
    Q_UNUSED(_a);
}

const QMetaObjectExtraData h2sl::Widget_H2SL_Comments::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject h2sl::Widget_H2SL_Comments::staticMetaObject = {
    { &Widget_H2SL::staticMetaObject, qt_meta_stringdata_h2sl__Widget_H2SL_Comments,
      qt_meta_data_h2sl__Widget_H2SL_Comments, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &h2sl::Widget_H2SL_Comments::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *h2sl::Widget_H2SL_Comments::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *h2sl::Widget_H2SL_Comments::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_h2sl__Widget_H2SL_Comments))
        return static_cast<void*>(const_cast< Widget_H2SL_Comments*>(this));
    return Widget_H2SL::qt_metacast(_clname);
}

int h2sl::Widget_H2SL_Comments::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = Widget_H2SL::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 1)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 1;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
