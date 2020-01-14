/****************************************************************************
** Meta object code from reading C++ file 'robot_model_display.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.5.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../../../src/rviz/src/rviz/default_plugin/robot_model_display.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'robot_model_display.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.5.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
struct qt_meta_stringdata_rviz__RobotModelDisplay_t {
    QByteArrayData data[7];
    char stringdata0[118];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_rviz__RobotModelDisplay_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_rviz__RobotModelDisplay_t qt_meta_stringdata_rviz__RobotModelDisplay = {
    {
QT_MOC_LITERAL(0, 0, 23), // "rviz::RobotModelDisplay"
QT_MOC_LITERAL(1, 24, 19), // "updateVisualVisible"
QT_MOC_LITERAL(2, 44, 0), // ""
QT_MOC_LITERAL(3, 45, 22), // "updateCollisionVisible"
QT_MOC_LITERAL(4, 68, 14), // "updateTfPrefix"
QT_MOC_LITERAL(5, 83, 11), // "updateAlpha"
QT_MOC_LITERAL(6, 95, 22) // "updateRobotDescription"

    },
    "rviz::RobotModelDisplay\0updateVisualVisible\0"
    "\0updateCollisionVisible\0updateTfPrefix\0"
    "updateAlpha\0updateRobotDescription"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_rviz__RobotModelDisplay[] = {

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

void rviz::RobotModelDisplay::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        RobotModelDisplay *_t = static_cast<RobotModelDisplay *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->updateVisualVisible(); break;
        case 1: _t->updateCollisionVisible(); break;
        case 2: _t->updateTfPrefix(); break;
        case 3: _t->updateAlpha(); break;
        case 4: _t->updateRobotDescription(); break;
        default: ;
        }
    }
    Q_UNUSED(_a);
}

const QMetaObject rviz::RobotModelDisplay::staticMetaObject = {
    { &Display::staticMetaObject, qt_meta_stringdata_rviz__RobotModelDisplay.data,
      qt_meta_data_rviz__RobotModelDisplay,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *rviz::RobotModelDisplay::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *rviz::RobotModelDisplay::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_rviz__RobotModelDisplay.stringdata0))
        return static_cast<void*>(const_cast< RobotModelDisplay*>(this));
    return Display::qt_metacast(_clname);
}

int rviz::RobotModelDisplay::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = Display::qt_metacall(_c, _id, _a);
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
