/****************************************************************************
** Meta object code from reading C++ file 'editable_enum_property.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.5.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../../src/rviz/src/rviz/properties/editable_enum_property.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'editable_enum_property.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.5.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
struct qt_meta_stringdata_rviz__EditableEnumProperty_t {
    QByteArrayData data[7];
    char stringdata0[107];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_rviz__EditableEnumProperty_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_rviz__EditableEnumProperty_t qt_meta_stringdata_rviz__EditableEnumProperty = {
    {
QT_MOC_LITERAL(0, 0, 26), // "rviz::EditableEnumProperty"
QT_MOC_LITERAL(1, 27, 14), // "requestOptions"
QT_MOC_LITERAL(2, 42, 0), // ""
QT_MOC_LITERAL(3, 43, 21), // "EditableEnumProperty*"
QT_MOC_LITERAL(4, 65, 27), // "property_in_need_of_options"
QT_MOC_LITERAL(5, 93, 9), // "setString"
QT_MOC_LITERAL(6, 103, 3) // "str"

    },
    "rviz::EditableEnumProperty\0requestOptions\0"
    "\0EditableEnumProperty*\0"
    "property_in_need_of_options\0setString\0"
    "str"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_rviz__EditableEnumProperty[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       2,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       1,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    1,   24,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
       5,    1,   27,    2, 0x0a /* Public */,

 // signals: parameters
    QMetaType::Void, 0x80000000 | 3,    4,

 // slots: parameters
    QMetaType::Void, QMetaType::QString,    6,

       0        // eod
};

void rviz::EditableEnumProperty::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        EditableEnumProperty *_t = static_cast<EditableEnumProperty *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->requestOptions((*reinterpret_cast< EditableEnumProperty*(*)>(_a[1]))); break;
        case 1: _t->setString((*reinterpret_cast< const QString(*)>(_a[1]))); break;
        default: ;
        }
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        switch (_id) {
        default: *reinterpret_cast<int*>(_a[0]) = -1; break;
        case 0:
            switch (*reinterpret_cast<int*>(_a[1])) {
            default: *reinterpret_cast<int*>(_a[0]) = -1; break;
            case 0:
                *reinterpret_cast<int*>(_a[0]) = qRegisterMetaType< EditableEnumProperty* >(); break;
            }
            break;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        void **func = reinterpret_cast<void **>(_a[1]);
        {
            typedef void (EditableEnumProperty::*_t)(EditableEnumProperty * );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&EditableEnumProperty::requestOptions)) {
                *result = 0;
            }
        }
    }
}

const QMetaObject rviz::EditableEnumProperty::staticMetaObject = {
    { &StringProperty::staticMetaObject, qt_meta_stringdata_rviz__EditableEnumProperty.data,
      qt_meta_data_rviz__EditableEnumProperty,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *rviz::EditableEnumProperty::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *rviz::EditableEnumProperty::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_rviz__EditableEnumProperty.stringdata0))
        return static_cast<void*>(const_cast< EditableEnumProperty*>(this));
    return StringProperty::qt_metacast(_clname);
}

int rviz::EditableEnumProperty::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = StringProperty::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 2)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 2;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 2)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 2;
    }
    return _id;
}

// SIGNAL 0
void rviz::EditableEnumProperty::requestOptions(EditableEnumProperty * _t1)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}
QT_END_MOC_NAMESPACE
