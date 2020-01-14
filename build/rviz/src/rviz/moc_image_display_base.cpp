/****************************************************************************
** Meta object code from reading C++ file 'image_display_base.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.5.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../../src/rviz/src/rviz/image/image_display_base.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'image_display_base.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.5.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
struct qt_meta_stringdata_rviz__ImageDisplayBase_t {
    QByteArrayData data[7];
    char stringdata0[99];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_rviz__ImageDisplayBase_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_rviz__ImageDisplayBase_t qt_meta_stringdata_rviz__ImageDisplayBase = {
    {
QT_MOC_LITERAL(0, 0, 22), // "rviz::ImageDisplayBase"
QT_MOC_LITERAL(1, 23, 11), // "updateTopic"
QT_MOC_LITERAL(2, 35, 0), // ""
QT_MOC_LITERAL(3, 36, 15), // "updateQueueSize"
QT_MOC_LITERAL(4, 52, 23), // "fillTransportOptionList"
QT_MOC_LITERAL(5, 76, 13), // "EnumProperty*"
QT_MOC_LITERAL(6, 90, 8) // "property"

    },
    "rviz::ImageDisplayBase\0updateTopic\0\0"
    "updateQueueSize\0fillTransportOptionList\0"
    "EnumProperty*\0property"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_rviz__ImageDisplayBase[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       3,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    0,   29,    2, 0x09 /* Protected */,
       3,    0,   30,    2, 0x09 /* Protected */,
       4,    1,   31,    2, 0x09 /* Protected */,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 5,    6,

       0        // eod
};

void rviz::ImageDisplayBase::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        ImageDisplayBase *_t = static_cast<ImageDisplayBase *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->updateTopic(); break;
        case 1: _t->updateQueueSize(); break;
        case 2: _t->fillTransportOptionList((*reinterpret_cast< EnumProperty*(*)>(_a[1]))); break;
        default: ;
        }
    }
}

const QMetaObject rviz::ImageDisplayBase::staticMetaObject = {
    { &Display::staticMetaObject, qt_meta_stringdata_rviz__ImageDisplayBase.data,
      qt_meta_data_rviz__ImageDisplayBase,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *rviz::ImageDisplayBase::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *rviz::ImageDisplayBase::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_rviz__ImageDisplayBase.stringdata0))
        return static_cast<void*>(const_cast< ImageDisplayBase*>(this));
    return Display::qt_metacast(_clname);
}

int rviz::ImageDisplayBase::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = Display::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 3)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 3;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 3)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 3;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
