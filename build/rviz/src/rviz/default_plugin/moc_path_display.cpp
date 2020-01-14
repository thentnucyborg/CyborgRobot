/****************************************************************************
** Meta object code from reading C++ file 'path_display.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.5.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../../../src/rviz/src/rviz/default_plugin/path_display.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'path_display.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.5.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
struct qt_meta_stringdata_rviz__PathDisplay_t {
    QByteArrayData data[10];
    char stringdata0[163];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_rviz__PathDisplay_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_rviz__PathDisplay_t qt_meta_stringdata_rviz__PathDisplay = {
    {
QT_MOC_LITERAL(0, 0, 17), // "rviz::PathDisplay"
QT_MOC_LITERAL(1, 18, 18), // "updateBufferLength"
QT_MOC_LITERAL(2, 37, 0), // ""
QT_MOC_LITERAL(3, 38, 11), // "updateStyle"
QT_MOC_LITERAL(4, 50, 15), // "updateLineWidth"
QT_MOC_LITERAL(5, 66, 12), // "updateOffset"
QT_MOC_LITERAL(6, 79, 15), // "updatePoseStyle"
QT_MOC_LITERAL(7, 95, 22), // "updatePoseAxisGeometry"
QT_MOC_LITERAL(8, 118, 20), // "updatePoseArrowColor"
QT_MOC_LITERAL(9, 139, 23) // "updatePoseArrowGeometry"

    },
    "rviz::PathDisplay\0updateBufferLength\0"
    "\0updateStyle\0updateLineWidth\0updateOffset\0"
    "updatePoseStyle\0updatePoseAxisGeometry\0"
    "updatePoseArrowColor\0updatePoseArrowGeometry"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_rviz__PathDisplay[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       8,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    0,   54,    2, 0x08 /* Private */,
       3,    0,   55,    2, 0x08 /* Private */,
       4,    0,   56,    2, 0x08 /* Private */,
       5,    0,   57,    2, 0x08 /* Private */,
       6,    0,   58,    2, 0x08 /* Private */,
       7,    0,   59,    2, 0x08 /* Private */,
       8,    0,   60,    2, 0x08 /* Private */,
       9,    0,   61,    2, 0x08 /* Private */,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,

       0        // eod
};

void rviz::PathDisplay::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        PathDisplay *_t = static_cast<PathDisplay *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->updateBufferLength(); break;
        case 1: _t->updateStyle(); break;
        case 2: _t->updateLineWidth(); break;
        case 3: _t->updateOffset(); break;
        case 4: _t->updatePoseStyle(); break;
        case 5: _t->updatePoseAxisGeometry(); break;
        case 6: _t->updatePoseArrowColor(); break;
        case 7: _t->updatePoseArrowGeometry(); break;
        default: ;
        }
    }
    Q_UNUSED(_a);
}

const QMetaObject rviz::PathDisplay::staticMetaObject = {
    { &MessageFilterDisplay<nav_msgs::Path>::staticMetaObject, qt_meta_stringdata_rviz__PathDisplay.data,
      qt_meta_data_rviz__PathDisplay,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *rviz::PathDisplay::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *rviz::PathDisplay::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_rviz__PathDisplay.stringdata0))
        return static_cast<void*>(const_cast< PathDisplay*>(this));
    return MessageFilterDisplay<nav_msgs::Path>::qt_metacast(_clname);
}

int rviz::PathDisplay::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = MessageFilterDisplay<nav_msgs::Path>::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 8)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 8;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 8)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 8;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
