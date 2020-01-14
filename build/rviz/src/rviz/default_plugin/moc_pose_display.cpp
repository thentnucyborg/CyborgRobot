/****************************************************************************
** Meta object code from reading C++ file 'pose_display.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.5.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../../../src/rviz/src/rviz/default_plugin/pose_display.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'pose_display.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.5.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
struct qt_meta_stringdata_rviz__PoseDisplay_t {
    QByteArrayData data[7];
    char stringdata0[118];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_rviz__PoseDisplay_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_rviz__PoseDisplay_t qt_meta_stringdata_rviz__PoseDisplay = {
    {
QT_MOC_LITERAL(0, 0, 17), // "rviz::PoseDisplay"
QT_MOC_LITERAL(1, 18, 21), // "updateShapeVisibility"
QT_MOC_LITERAL(2, 40, 0), // ""
QT_MOC_LITERAL(3, 41, 19), // "updateColorAndAlpha"
QT_MOC_LITERAL(4, 61, 17), // "updateShapeChoice"
QT_MOC_LITERAL(5, 79, 18), // "updateAxisGeometry"
QT_MOC_LITERAL(6, 98, 19) // "updateArrowGeometry"

    },
    "rviz::PoseDisplay\0updateShapeVisibility\0"
    "\0updateColorAndAlpha\0updateShapeChoice\0"
    "updateAxisGeometry\0updateArrowGeometry"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_rviz__PoseDisplay[] = {

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

void rviz::PoseDisplay::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        PoseDisplay *_t = static_cast<PoseDisplay *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->updateShapeVisibility(); break;
        case 1: _t->updateColorAndAlpha(); break;
        case 2: _t->updateShapeChoice(); break;
        case 3: _t->updateAxisGeometry(); break;
        case 4: _t->updateArrowGeometry(); break;
        default: ;
        }
    }
    Q_UNUSED(_a);
}

const QMetaObject rviz::PoseDisplay::staticMetaObject = {
    { &MessageFilterDisplay<geometry_msgs::PoseStamped>::staticMetaObject, qt_meta_stringdata_rviz__PoseDisplay.data,
      qt_meta_data_rviz__PoseDisplay,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *rviz::PoseDisplay::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *rviz::PoseDisplay::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_rviz__PoseDisplay.stringdata0))
        return static_cast<void*>(const_cast< PoseDisplay*>(this));
    return MessageFilterDisplay<geometry_msgs::PoseStamped>::qt_metacast(_clname);
}

int rviz::PoseDisplay::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = MessageFilterDisplay<geometry_msgs::PoseStamped>::qt_metacall(_c, _id, _a);
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
