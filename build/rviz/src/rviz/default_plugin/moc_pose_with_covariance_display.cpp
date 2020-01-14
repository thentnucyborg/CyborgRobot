/****************************************************************************
** Meta object code from reading C++ file 'pose_with_covariance_display.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.5.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../../../src/rviz/src/rviz/default_plugin/pose_with_covariance_display.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'pose_with_covariance_display.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.5.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
struct qt_meta_stringdata_rviz__PoseWithCovarianceDisplay_t {
    QByteArrayData data[7];
    char stringdata0[132];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_rviz__PoseWithCovarianceDisplay_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_rviz__PoseWithCovarianceDisplay_t qt_meta_stringdata_rviz__PoseWithCovarianceDisplay = {
    {
QT_MOC_LITERAL(0, 0, 31), // "rviz::PoseWithCovarianceDisplay"
QT_MOC_LITERAL(1, 32, 21), // "updateShapeVisibility"
QT_MOC_LITERAL(2, 54, 0), // ""
QT_MOC_LITERAL(3, 55, 19), // "updateColorAndAlpha"
QT_MOC_LITERAL(4, 75, 17), // "updateShapeChoice"
QT_MOC_LITERAL(5, 93, 18), // "updateAxisGeometry"
QT_MOC_LITERAL(6, 112, 19) // "updateArrowGeometry"

    },
    "rviz::PoseWithCovarianceDisplay\0"
    "updateShapeVisibility\0\0updateColorAndAlpha\0"
    "updateShapeChoice\0updateAxisGeometry\0"
    "updateArrowGeometry"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_rviz__PoseWithCovarianceDisplay[] = {

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

void rviz::PoseWithCovarianceDisplay::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        PoseWithCovarianceDisplay *_t = static_cast<PoseWithCovarianceDisplay *>(_o);
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

const QMetaObject rviz::PoseWithCovarianceDisplay::staticMetaObject = {
    { &rviz::MessageFilterDisplay<geometry_msgs::PoseWithCovarianceStamped>::staticMetaObject, qt_meta_stringdata_rviz__PoseWithCovarianceDisplay.data,
      qt_meta_data_rviz__PoseWithCovarianceDisplay,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *rviz::PoseWithCovarianceDisplay::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *rviz::PoseWithCovarianceDisplay::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_rviz__PoseWithCovarianceDisplay.stringdata0))
        return static_cast<void*>(const_cast< PoseWithCovarianceDisplay*>(this));
    return rviz::MessageFilterDisplay<geometry_msgs::PoseWithCovarianceStamped>::qt_metacast(_clname);
}

int rviz::PoseWithCovarianceDisplay::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = rviz::MessageFilterDisplay<geometry_msgs::PoseWithCovarianceStamped>::qt_metacall(_c, _id, _a);
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
