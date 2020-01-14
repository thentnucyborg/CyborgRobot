/****************************************************************************
** Meta object code from reading C++ file 'depth_cloud_display.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.5.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../../../src/rviz/src/rviz/default_plugin/depth_cloud_display.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'depth_cloud_display.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.5.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
struct qt_meta_stringdata_rviz__RosFilteredTopicProperty_t {
    QByteArrayData data[3];
    char stringdata0[46];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_rviz__RosFilteredTopicProperty_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_rviz__RosFilteredTopicProperty_t qt_meta_stringdata_rviz__RosFilteredTopicProperty = {
    {
QT_MOC_LITERAL(0, 0, 30), // "rviz::RosFilteredTopicProperty"
QT_MOC_LITERAL(1, 31, 13), // "fillTopicList"
QT_MOC_LITERAL(2, 45, 0) // ""

    },
    "rviz::RosFilteredTopicProperty\0"
    "fillTopicList\0"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_rviz__RosFilteredTopicProperty[] = {

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
       1,    0,   19,    2, 0x09 /* Protected */,

 // slots: parameters
    QMetaType::Void,

       0        // eod
};

void rviz::RosFilteredTopicProperty::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        RosFilteredTopicProperty *_t = static_cast<RosFilteredTopicProperty *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->fillTopicList(); break;
        default: ;
        }
    }
    Q_UNUSED(_a);
}

const QMetaObject rviz::RosFilteredTopicProperty::staticMetaObject = {
    { &RosTopicProperty::staticMetaObject, qt_meta_stringdata_rviz__RosFilteredTopicProperty.data,
      qt_meta_data_rviz__RosFilteredTopicProperty,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *rviz::RosFilteredTopicProperty::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *rviz::RosFilteredTopicProperty::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_rviz__RosFilteredTopicProperty.stringdata0))
        return static_cast<void*>(const_cast< RosFilteredTopicProperty*>(this));
    return RosTopicProperty::qt_metacast(_clname);
}

int rviz::RosFilteredTopicProperty::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = RosTopicProperty::qt_metacall(_c, _id, _a);
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
struct qt_meta_stringdata_rviz__DepthCloudDisplay_t {
    QByteArrayData data[12];
    char stringdata0[211];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_rviz__DepthCloudDisplay_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_rviz__DepthCloudDisplay_t qt_meta_stringdata_rviz__DepthCloudDisplay = {
    {
QT_MOC_LITERAL(0, 0, 23), // "rviz::DepthCloudDisplay"
QT_MOC_LITERAL(1, 24, 15), // "updateQueueSize"
QT_MOC_LITERAL(2, 40, 0), // ""
QT_MOC_LITERAL(3, 41, 23), // "fillTransportOptionList"
QT_MOC_LITERAL(4, 65, 13), // "EnumProperty*"
QT_MOC_LITERAL(5, 79, 8), // "property"
QT_MOC_LITERAL(6, 88, 11), // "updateTopic"
QT_MOC_LITERAL(7, 100, 17), // "updateTopicFilter"
QT_MOC_LITERAL(8, 118, 17), // "updateUseAutoSize"
QT_MOC_LITERAL(9, 136, 20), // "updateAutoSizeFactor"
QT_MOC_LITERAL(10, 157, 30), // "updateUseOcclusionCompensation"
QT_MOC_LITERAL(11, 188, 22) // "updateOcclusionTimeOut"

    },
    "rviz::DepthCloudDisplay\0updateQueueSize\0"
    "\0fillTransportOptionList\0EnumProperty*\0"
    "property\0updateTopic\0updateTopicFilter\0"
    "updateUseAutoSize\0updateAutoSizeFactor\0"
    "updateUseOcclusionCompensation\0"
    "updateOcclusionTimeOut"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_rviz__DepthCloudDisplay[] = {

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
       1,    0,   54,    2, 0x09 /* Protected */,
       3,    1,   55,    2, 0x09 /* Protected */,
       6,    0,   58,    2, 0x09 /* Protected */,
       7,    0,   59,    2, 0x09 /* Protected */,
       8,    0,   60,    2, 0x09 /* Protected */,
       9,    0,   61,    2, 0x09 /* Protected */,
      10,    0,   62,    2, 0x09 /* Protected */,
      11,    0,   63,    2, 0x09 /* Protected */,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 4,    5,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,

       0        // eod
};

void rviz::DepthCloudDisplay::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        DepthCloudDisplay *_t = static_cast<DepthCloudDisplay *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->updateQueueSize(); break;
        case 1: _t->fillTransportOptionList((*reinterpret_cast< EnumProperty*(*)>(_a[1]))); break;
        case 2: _t->updateTopic(); break;
        case 3: _t->updateTopicFilter(); break;
        case 4: _t->updateUseAutoSize(); break;
        case 5: _t->updateAutoSizeFactor(); break;
        case 6: _t->updateUseOcclusionCompensation(); break;
        case 7: _t->updateOcclusionTimeOut(); break;
        default: ;
        }
    }
}

const QMetaObject rviz::DepthCloudDisplay::staticMetaObject = {
    { &rviz::Display::staticMetaObject, qt_meta_stringdata_rviz__DepthCloudDisplay.data,
      qt_meta_data_rviz__DepthCloudDisplay,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *rviz::DepthCloudDisplay::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *rviz::DepthCloudDisplay::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_rviz__DepthCloudDisplay.stringdata0))
        return static_cast<void*>(const_cast< DepthCloudDisplay*>(this));
    return rviz::Display::qt_metacast(_clname);
}

int rviz::DepthCloudDisplay::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = rviz::Display::qt_metacall(_c, _id, _a);
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
