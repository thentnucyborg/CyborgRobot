/****************************************************************************
** Meta object code from reading C++ file 'enum_property.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.5.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../../src/rviz/src/rviz/properties/enum_property.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'enum_property.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.5.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
struct qt_meta_stringdata_rviz__EnumProperty_t {
    QByteArrayData data[10];
    char stringdata0[128];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_rviz__EnumProperty_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_rviz__EnumProperty_t qt_meta_stringdata_rviz__EnumProperty = {
    {
QT_MOC_LITERAL(0, 0, 18), // "rviz::EnumProperty"
QT_MOC_LITERAL(1, 19, 14), // "requestOptions"
QT_MOC_LITERAL(2, 34, 0), // ""
QT_MOC_LITERAL(3, 35, 13), // "EnumProperty*"
QT_MOC_LITERAL(4, 49, 27), // "property_in_need_of_options"
QT_MOC_LITERAL(5, 77, 9), // "setString"
QT_MOC_LITERAL(6, 87, 3), // "str"
QT_MOC_LITERAL(7, 91, 12), // "setStringStd"
QT_MOC_LITERAL(8, 104, 11), // "std::string"
QT_MOC_LITERAL(9, 116, 11) // "sortOptions"

    },
    "rviz::EnumProperty\0requestOptions\0\0"
    "EnumProperty*\0property_in_need_of_options\0"
    "setString\0str\0setStringStd\0std::string\0"
    "sortOptions"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_rviz__EnumProperty[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       4,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       1,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    1,   34,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
       5,    1,   37,    2, 0x0a /* Public */,
       7,    1,   40,    2, 0x0a /* Public */,
       9,    0,   43,    2, 0x0a /* Public */,

 // signals: parameters
    QMetaType::Void, 0x80000000 | 3,    4,

 // slots: parameters
    QMetaType::Void, QMetaType::QString,    6,
    QMetaType::Void, 0x80000000 | 8,    6,
    QMetaType::Void,

       0        // eod
};

void rviz::EnumProperty::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        EnumProperty *_t = static_cast<EnumProperty *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->requestOptions((*reinterpret_cast< EnumProperty*(*)>(_a[1]))); break;
        case 1: _t->setString((*reinterpret_cast< const QString(*)>(_a[1]))); break;
        case 2: _t->setStringStd((*reinterpret_cast< const std::string(*)>(_a[1]))); break;
        case 3: _t->sortOptions(); break;
        default: ;
        }
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        switch (_id) {
        default: *reinterpret_cast<int*>(_a[0]) = -1; break;
        case 0:
            switch (*reinterpret_cast<int*>(_a[1])) {
            default: *reinterpret_cast<int*>(_a[0]) = -1; break;
            case 0:
                *reinterpret_cast<int*>(_a[0]) = qRegisterMetaType< EnumProperty* >(); break;
            }
            break;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        void **func = reinterpret_cast<void **>(_a[1]);
        {
            typedef void (EnumProperty::*_t)(EnumProperty * );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&EnumProperty::requestOptions)) {
                *result = 0;
            }
        }
    }
}

const QMetaObject rviz::EnumProperty::staticMetaObject = {
    { &StringProperty::staticMetaObject, qt_meta_stringdata_rviz__EnumProperty.data,
      qt_meta_data_rviz__EnumProperty,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *rviz::EnumProperty::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *rviz::EnumProperty::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_rviz__EnumProperty.stringdata0))
        return static_cast<void*>(const_cast< EnumProperty*>(this));
    return StringProperty::qt_metacast(_clname);
}

int rviz::EnumProperty::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = StringProperty::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 4)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 4;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 4)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 4;
    }
    return _id;
}

// SIGNAL 0
void rviz::EnumProperty::requestOptions(EnumProperty * _t1)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}
QT_END_MOC_NAMESPACE
