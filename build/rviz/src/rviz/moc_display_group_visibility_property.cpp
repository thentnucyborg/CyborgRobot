/****************************************************************************
** Meta object code from reading C++ file 'display_group_visibility_property.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.5.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../../src/rviz/src/rviz/properties/display_group_visibility_property.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'display_group_visibility_property.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.5.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
struct qt_meta_stringdata_rviz__DisplayGroupVisibilityProperty_t {
    QByteArrayData data[6];
    char stringdata0[93];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_rviz__DisplayGroupVisibilityProperty_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_rviz__DisplayGroupVisibilityProperty_t qt_meta_stringdata_rviz__DisplayGroupVisibilityProperty = {
    {
QT_MOC_LITERAL(0, 0, 36), // "rviz::DisplayGroupVisibilityP..."
QT_MOC_LITERAL(1, 37, 14), // "onDisplayAdded"
QT_MOC_LITERAL(2, 52, 0), // ""
QT_MOC_LITERAL(3, 53, 14), // "rviz::Display*"
QT_MOC_LITERAL(4, 68, 7), // "display"
QT_MOC_LITERAL(5, 76, 16) // "onDisplayRemoved"

    },
    "rviz::DisplayGroupVisibilityProperty\0"
    "onDisplayAdded\0\0rviz::Display*\0display\0"
    "onDisplayRemoved"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_rviz__DisplayGroupVisibilityProperty[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       2,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    1,   24,    2, 0x0a /* Public */,
       5,    1,   27,    2, 0x0a /* Public */,

 // slots: parameters
    QMetaType::Void, 0x80000000 | 3,    4,
    QMetaType::Void, 0x80000000 | 3,    4,

       0        // eod
};

void rviz::DisplayGroupVisibilityProperty::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        DisplayGroupVisibilityProperty *_t = static_cast<DisplayGroupVisibilityProperty *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->onDisplayAdded((*reinterpret_cast< rviz::Display*(*)>(_a[1]))); break;
        case 1: _t->onDisplayRemoved((*reinterpret_cast< rviz::Display*(*)>(_a[1]))); break;
        default: ;
        }
    }
}

const QMetaObject rviz::DisplayGroupVisibilityProperty::staticMetaObject = {
    { &DisplayVisibilityProperty::staticMetaObject, qt_meta_stringdata_rviz__DisplayGroupVisibilityProperty.data,
      qt_meta_data_rviz__DisplayGroupVisibilityProperty,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *rviz::DisplayGroupVisibilityProperty::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *rviz::DisplayGroupVisibilityProperty::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_rviz__DisplayGroupVisibilityProperty.stringdata0))
        return static_cast<void*>(const_cast< DisplayGroupVisibilityProperty*>(this));
    return DisplayVisibilityProperty::qt_metacast(_clname);
}

int rviz::DisplayGroupVisibilityProperty::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = DisplayVisibilityProperty::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 2)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 2;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 2)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 2;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
