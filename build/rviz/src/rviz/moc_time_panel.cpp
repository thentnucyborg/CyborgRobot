/****************************************************************************
** Meta object code from reading C++ file 'time_panel.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.5.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../../src/rviz/src/rviz/time_panel.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'time_panel.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.5.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
struct qt_meta_stringdata_rviz__TimePanel_t {
    QByteArrayData data[20];
    char stringdata0[214];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_rviz__TimePanel_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_rviz__TimePanel_t qt_meta_stringdata_rviz__TimePanel = {
    {
QT_MOC_LITERAL(0, 0, 15), // "rviz::TimePanel"
QT_MOC_LITERAL(1, 16, 12), // "pauseToggled"
QT_MOC_LITERAL(2, 29, 0), // ""
QT_MOC_LITERAL(3, 30, 7), // "checked"
QT_MOC_LITERAL(4, 38, 16), // "syncModeSelected"
QT_MOC_LITERAL(5, 55, 5), // "index"
QT_MOC_LITERAL(6, 61, 18), // "syncSourceSelected"
QT_MOC_LITERAL(7, 80, 19), // "experimentalToggled"
QT_MOC_LITERAL(8, 100, 6), // "update"
QT_MOC_LITERAL(9, 107, 14), // "onDisplayAdded"
QT_MOC_LITERAL(10, 122, 14), // "rviz::Display*"
QT_MOC_LITERAL(11, 137, 7), // "display"
QT_MOC_LITERAL(12, 145, 16), // "onDisplayRemoved"
QT_MOC_LITERAL(13, 162, 12), // "onTimeSignal"
QT_MOC_LITERAL(14, 175, 9), // "ros::Time"
QT_MOC_LITERAL(15, 185, 4), // "time"
QT_MOC_LITERAL(16, 190, 4), // "load"
QT_MOC_LITERAL(17, 195, 6), // "Config"
QT_MOC_LITERAL(18, 202, 6), // "config"
QT_MOC_LITERAL(19, 209, 4) // "save"

    },
    "rviz::TimePanel\0pauseToggled\0\0checked\0"
    "syncModeSelected\0index\0syncSourceSelected\0"
    "experimentalToggled\0update\0onDisplayAdded\0"
    "rviz::Display*\0display\0onDisplayRemoved\0"
    "onTimeSignal\0ros::Time\0time\0load\0"
    "Config\0config\0save"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_rviz__TimePanel[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
      10,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    1,   64,    2, 0x09 /* Protected */,
       4,    1,   67,    2, 0x09 /* Protected */,
       6,    1,   70,    2, 0x09 /* Protected */,
       7,    1,   73,    2, 0x09 /* Protected */,
       8,    0,   76,    2, 0x09 /* Protected */,
       9,    1,   77,    2, 0x09 /* Protected */,
      12,    1,   80,    2, 0x09 /* Protected */,
      13,    2,   83,    2, 0x09 /* Protected */,
      16,    1,   88,    2, 0x09 /* Protected */,
      19,    1,   91,    2, 0x09 /* Protected */,

 // slots: parameters
    QMetaType::Void, QMetaType::Bool,    3,
    QMetaType::Void, QMetaType::Int,    5,
    QMetaType::Void, QMetaType::Int,    5,
    QMetaType::Void, QMetaType::Bool,    3,
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 10,   11,
    QMetaType::Void, 0x80000000 | 10,   11,
    QMetaType::Void, 0x80000000 | 10, 0x80000000 | 14,   11,   15,
    QMetaType::Void, 0x80000000 | 17,   18,
    QMetaType::Void, 0x80000000 | 17,   18,

       0        // eod
};

void rviz::TimePanel::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        TimePanel *_t = static_cast<TimePanel *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->pauseToggled((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 1: _t->syncModeSelected((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 2: _t->syncSourceSelected((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 3: _t->experimentalToggled((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 4: _t->update(); break;
        case 5: _t->onDisplayAdded((*reinterpret_cast< rviz::Display*(*)>(_a[1]))); break;
        case 6: _t->onDisplayRemoved((*reinterpret_cast< rviz::Display*(*)>(_a[1]))); break;
        case 7: _t->onTimeSignal((*reinterpret_cast< rviz::Display*(*)>(_a[1])),(*reinterpret_cast< ros::Time(*)>(_a[2]))); break;
        case 8: _t->load((*reinterpret_cast< const Config(*)>(_a[1]))); break;
        case 9: _t->save((*reinterpret_cast< Config(*)>(_a[1]))); break;
        default: ;
        }
    }
}

const QMetaObject rviz::TimePanel::staticMetaObject = {
    { &Panel::staticMetaObject, qt_meta_stringdata_rviz__TimePanel.data,
      qt_meta_data_rviz__TimePanel,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *rviz::TimePanel::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *rviz::TimePanel::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_rviz__TimePanel.stringdata0))
        return static_cast<void*>(const_cast< TimePanel*>(this));
    return Panel::qt_metacast(_clname);
}

int rviz::TimePanel::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = Panel::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 10)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 10;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 10)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 10;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
