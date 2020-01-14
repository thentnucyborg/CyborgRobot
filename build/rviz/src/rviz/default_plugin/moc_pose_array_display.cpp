/****************************************************************************
** Meta object code from reading C++ file 'pose_array_display.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.5.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../../../src/rviz/src/rviz/default_plugin/pose_array_display.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'pose_array_display.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.5.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
struct qt_meta_stringdata_rviz__PoseArrayDisplay_t {
    QByteArrayData data[7];
    char stringdata0[122];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_rviz__PoseArrayDisplay_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_rviz__PoseArrayDisplay_t qt_meta_stringdata_rviz__PoseArrayDisplay = {
    {
QT_MOC_LITERAL(0, 0, 22), // "rviz::PoseArrayDisplay"
QT_MOC_LITERAL(1, 23, 17), // "updateShapeChoice"
QT_MOC_LITERAL(2, 41, 0), // ""
QT_MOC_LITERAL(3, 42, 16), // "updateArrowColor"
QT_MOC_LITERAL(4, 59, 21), // "updateArrow2dGeometry"
QT_MOC_LITERAL(5, 81, 21), // "updateArrow3dGeometry"
QT_MOC_LITERAL(6, 103, 18) // "updateAxesGeometry"

    },
    "rviz::PoseArrayDisplay\0updateShapeChoice\0"
    "\0updateArrowColor\0updateArrow2dGeometry\0"
    "updateArrow3dGeometry\0updateAxesGeometry"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_rviz__PoseArrayDisplay[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       5,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    0,   39,    2, 0x08 /* Private */,
       3,    0,   40,    2, 0x08 /* Private */,
       4,    0,   41,    2, 0x08 /* Private */,
       5,    0,   42,    2, 0x08 /* Private */,
       6,    0,   43,    2, 0x08 /* Private */,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,

       0        // eod
};

void rviz::PoseArrayDisplay::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        PoseArrayDisplay *_t = static_cast<PoseArrayDisplay *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->updateShapeChoice(); break;
        case 1: _t->updateArrowColor(); break;
        case 2: _t->updateArrow2dGeometry(); break;
        case 3: _t->updateArrow3dGeometry(); break;
        case 4: _t->updateAxesGeometry(); break;
        default: ;
        }
    }
    Q_UNUSED(_a);
}

const QMetaObject rviz::PoseArrayDisplay::staticMetaObject = {
    { &MessageFilterDisplay<geometry_msgs::PoseArray>::staticMetaObject, qt_meta_stringdata_rviz__PoseArrayDisplay.data,
      qt_meta_data_rviz__PoseArrayDisplay,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *rviz::PoseArrayDisplay::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *rviz::PoseArrayDisplay::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_rviz__PoseArrayDisplay.stringdata0))
        return static_cast<void*>(const_cast< PoseArrayDisplay*>(this));
    return MessageFilterDisplay<geometry_msgs::PoseArray>::qt_metacast(_clname);
}

int rviz::PoseArrayDisplay::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = MessageFilterDisplay<geometry_msgs::PoseArray>::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 5)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 5;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 5)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 5;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
