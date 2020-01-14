/****************************************************************************
** Meta object code from reading C++ file 'point_cloud_common.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.5.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../../../src/rviz/src/rviz/default_plugin/point_cloud_common.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'point_cloud_common.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.5.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
struct qt_meta_stringdata_rviz__PointCloudCommon_t {
    QByteArrayData data[13];
    char stringdata0[217];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_rviz__PointCloudCommon_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_rviz__PointCloudCommon_t qt_meta_stringdata_rviz__PointCloudCommon = {
    {
QT_MOC_LITERAL(0, 0, 22), // "rviz::PointCloudCommon"
QT_MOC_LITERAL(1, 23, 16), // "causeRetransform"
QT_MOC_LITERAL(2, 40, 0), // ""
QT_MOC_LITERAL(3, 41, 16), // "updateSelectable"
QT_MOC_LITERAL(4, 58, 11), // "updateStyle"
QT_MOC_LITERAL(5, 70, 19), // "updateBillboardSize"
QT_MOC_LITERAL(6, 90, 11), // "updateAlpha"
QT_MOC_LITERAL(7, 102, 20), // "updateXyzTransformer"
QT_MOC_LITERAL(8, 123, 22), // "updateColorTransformer"
QT_MOC_LITERAL(9, 146, 24), // "setXyzTransformerOptions"
QT_MOC_LITERAL(10, 171, 13), // "EnumProperty*"
QT_MOC_LITERAL(11, 185, 4), // "prop"
QT_MOC_LITERAL(12, 190, 26) // "setColorTransformerOptions"

    },
    "rviz::PointCloudCommon\0causeRetransform\0"
    "\0updateSelectable\0updateStyle\0"
    "updateBillboardSize\0updateAlpha\0"
    "updateXyzTransformer\0updateColorTransformer\0"
    "setXyzTransformerOptions\0EnumProperty*\0"
    "prop\0setColorTransformerOptions"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_rviz__PointCloudCommon[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       9,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    0,   59,    2, 0x0a /* Public */,
       3,    0,   60,    2, 0x08 /* Private */,
       4,    0,   61,    2, 0x08 /* Private */,
       5,    0,   62,    2, 0x08 /* Private */,
       6,    0,   63,    2, 0x08 /* Private */,
       7,    0,   64,    2, 0x08 /* Private */,
       8,    0,   65,    2, 0x08 /* Private */,
       9,    1,   66,    2, 0x08 /* Private */,
      12,    1,   69,    2, 0x08 /* Private */,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 10,   11,
    QMetaType::Void, 0x80000000 | 10,   11,

       0        // eod
};

void rviz::PointCloudCommon::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        PointCloudCommon *_t = static_cast<PointCloudCommon *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->causeRetransform(); break;
        case 1: _t->updateSelectable(); break;
        case 2: _t->updateStyle(); break;
        case 3: _t->updateBillboardSize(); break;
        case 4: _t->updateAlpha(); break;
        case 5: _t->updateXyzTransformer(); break;
        case 6: _t->updateColorTransformer(); break;
        case 7: _t->setXyzTransformerOptions((*reinterpret_cast< EnumProperty*(*)>(_a[1]))); break;
        case 8: _t->setColorTransformerOptions((*reinterpret_cast< EnumProperty*(*)>(_a[1]))); break;
        default: ;
        }
    }
}

const QMetaObject rviz::PointCloudCommon::staticMetaObject = {
    { &QObject::staticMetaObject, qt_meta_stringdata_rviz__PointCloudCommon.data,
      qt_meta_data_rviz__PointCloudCommon,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *rviz::PointCloudCommon::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *rviz::PointCloudCommon::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_rviz__PointCloudCommon.stringdata0))
        return static_cast<void*>(const_cast< PointCloudCommon*>(this));
    return QObject::qt_metacast(_clname);
}

int rviz::PointCloudCommon::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 9)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 9;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 9)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 9;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
