/****************************************************************************
** Meta object code from reading C++ file 'image_display.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.5.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../../../src/rviz/src/rviz/default_plugin/image_display.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'image_display.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.5.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
struct qt_meta_stringdata_rviz__ImageDisplay_t {
    QByteArrayData data[3];
    char stringdata0[43];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_rviz__ImageDisplay_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_rviz__ImageDisplay_t qt_meta_stringdata_rviz__ImageDisplay = {
    {
QT_MOC_LITERAL(0, 0, 18), // "rviz::ImageDisplay"
QT_MOC_LITERAL(1, 19, 22), // "updateNormalizeOptions"
QT_MOC_LITERAL(2, 42, 0) // ""

    },
    "rviz::ImageDisplay\0updateNormalizeOptions\0"
    ""
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_rviz__ImageDisplay[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       1,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    0,   19,    2, 0x0a /* Public */,

 // slots: parameters
    QMetaType::Void,

       0        // eod
};

void rviz::ImageDisplay::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        ImageDisplay *_t = static_cast<ImageDisplay *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->updateNormalizeOptions(); break;
        default: ;
        }
    }
    Q_UNUSED(_a);
}

const QMetaObject rviz::ImageDisplay::staticMetaObject = {
    { &ImageDisplayBase::staticMetaObject, qt_meta_stringdata_rviz__ImageDisplay.data,
      qt_meta_data_rviz__ImageDisplay,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *rviz::ImageDisplay::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *rviz::ImageDisplay::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_rviz__ImageDisplay.stringdata0))
        return static_cast<void*>(const_cast< ImageDisplay*>(this));
    return ImageDisplayBase::qt_metacast(_clname);
}

int rviz::ImageDisplay::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = ImageDisplayBase::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 1)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 1;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 1)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 1;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
