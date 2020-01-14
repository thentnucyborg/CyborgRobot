/****************************************************************************
** Meta object code from reading C++ file 'effort_display.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.5.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../../../src/rviz/src/rviz/default_plugin/effort_display.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'effort_display.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.5.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
struct qt_meta_stringdata_rviz__JointInfo_t {
    QByteArrayData data[3];
    char stringdata0[34];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_rviz__JointInfo_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_rviz__JointInfo_t qt_meta_stringdata_rviz__JointInfo = {
    {
QT_MOC_LITERAL(0, 0, 15), // "rviz::JointInfo"
QT_MOC_LITERAL(1, 16, 16), // "updateVisibility"
QT_MOC_LITERAL(2, 33, 0) // ""

    },
    "rviz::JointInfo\0updateVisibility\0"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_rviz__JointInfo[] = {

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

void rviz::JointInfo::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        JointInfo *_t = static_cast<JointInfo *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->updateVisibility(); break;
        default: ;
        }
    }
    Q_UNUSED(_a);
}

const QMetaObject rviz::JointInfo::staticMetaObject = {
    { &QObject::staticMetaObject, qt_meta_stringdata_rviz__JointInfo.data,
      qt_meta_data_rviz__JointInfo,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *rviz::JointInfo::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *rviz::JointInfo::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_rviz__JointInfo.stringdata0))
        return static_cast<void*>(const_cast< JointInfo*>(this));
    return QObject::qt_metacast(_clname);
}

int rviz::JointInfo::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
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
struct qt_meta_stringdata_rviz__EffortDisplay_t {
    QByteArrayData data[10];
    char stringdata0[138];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_rviz__EffortDisplay_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_rviz__EffortDisplay_t qt_meta_stringdata_rviz__EffortDisplay = {
    {
QT_MOC_LITERAL(0, 0, 19), // "rviz::EffortDisplay"
QT_MOC_LITERAL(1, 20, 19), // "updateColorAndAlpha"
QT_MOC_LITERAL(2, 40, 0), // ""
QT_MOC_LITERAL(3, 41, 19), // "updateHistoryLength"
QT_MOC_LITERAL(4, 61, 22), // "updateRobotDescription"
QT_MOC_LITERAL(5, 84, 12), // "getJointInfo"
QT_MOC_LITERAL(6, 97, 10), // "JointInfo*"
QT_MOC_LITERAL(7, 108, 11), // "std::string"
QT_MOC_LITERAL(8, 120, 5), // "joint"
QT_MOC_LITERAL(9, 126, 11) // "createJoint"

    },
    "rviz::EffortDisplay\0updateColorAndAlpha\0"
    "\0updateHistoryLength\0updateRobotDescription\0"
    "getJointInfo\0JointInfo*\0std::string\0"
    "joint\0createJoint"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_rviz__EffortDisplay[] = {

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
       5,    1,   42,    2, 0x08 /* Private */,
       9,    1,   45,    2, 0x08 /* Private */,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    0x80000000 | 6, 0x80000000 | 7,    8,
    0x80000000 | 6, 0x80000000 | 7,    8,

       0        // eod
};

void rviz::EffortDisplay::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        EffortDisplay *_t = static_cast<EffortDisplay *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->updateColorAndAlpha(); break;
        case 1: _t->updateHistoryLength(); break;
        case 2: _t->updateRobotDescription(); break;
        case 3: { JointInfo* _r = _t->getJointInfo((*reinterpret_cast< const std::string(*)>(_a[1])));
            if (_a[0]) *reinterpret_cast< JointInfo**>(_a[0]) = _r; }  break;
        case 4: { JointInfo* _r = _t->createJoint((*reinterpret_cast< const std::string(*)>(_a[1])));
            if (_a[0]) *reinterpret_cast< JointInfo**>(_a[0]) = _r; }  break;
        default: ;
        }
    }
}

const QMetaObject rviz::EffortDisplay::staticMetaObject = {
    { &rviz::MessageFilterJointStateDisplay::staticMetaObject, qt_meta_stringdata_rviz__EffortDisplay.data,
      qt_meta_data_rviz__EffortDisplay,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *rviz::EffortDisplay::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *rviz::EffortDisplay::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_rviz__EffortDisplay.stringdata0))
        return static_cast<void*>(const_cast< EffortDisplay*>(this));
    return rviz::MessageFilterJointStateDisplay::qt_metacast(_clname);
}

int rviz::EffortDisplay::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = rviz::MessageFilterJointStateDisplay::qt_metacall(_c, _id, _a);
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
