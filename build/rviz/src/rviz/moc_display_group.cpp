/****************************************************************************
** Meta object code from reading C++ file 'display_group.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.5.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../../src/rviz/src/rviz/display_group.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'display_group.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.5.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
struct qt_meta_stringdata_rviz__DisplayGroup_t {
    QByteArrayData data[7];
    char stringdata0[87];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_rviz__DisplayGroup_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_rviz__DisplayGroup_t qt_meta_stringdata_rviz__DisplayGroup = {
    {
QT_MOC_LITERAL(0, 0, 18), // "rviz::DisplayGroup"
QT_MOC_LITERAL(1, 19, 12), // "displayAdded"
QT_MOC_LITERAL(2, 32, 0), // ""
QT_MOC_LITERAL(3, 33, 14), // "rviz::Display*"
QT_MOC_LITERAL(4, 48, 7), // "display"
QT_MOC_LITERAL(5, 56, 14), // "displayRemoved"
QT_MOC_LITERAL(6, 71, 15) // "onEnableChanged"

    },
    "rviz::DisplayGroup\0displayAdded\0\0"
    "rviz::Display*\0display\0displayRemoved\0"
    "onEnableChanged"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_rviz__DisplayGroup[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       3,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       2,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    1,   29,    2, 0x06 /* Public */,
       5,    1,   32,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
       6,    0,   35,    2, 0x0a /* Public */,

 // signals: parameters
    QMetaType::Void, 0x80000000 | 3,    4,
    QMetaType::Void, 0x80000000 | 3,    4,

 // slots: parameters
    QMetaType::Void,

       0        // eod
};

void rviz::DisplayGroup::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        DisplayGroup *_t = static_cast<DisplayGroup *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->displayAdded((*reinterpret_cast< rviz::Display*(*)>(_a[1]))); break;
        case 1: _t->displayRemoved((*reinterpret_cast< rviz::Display*(*)>(_a[1]))); break;
        case 2: _t->onEnableChanged(); break;
        default: ;
        }
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        switch (_id) {
        default: *reinterpret_cast<int*>(_a[0]) = -1; break;
        case 0:
            switch (*reinterpret_cast<int*>(_a[1])) {
            default: *reinterpret_cast<int*>(_a[0]) = -1; break;
            case 0:
                *reinterpret_cast<int*>(_a[0]) = qRegisterMetaType< rviz::Display* >(); break;
            }
            break;
        case 1:
            switch (*reinterpret_cast<int*>(_a[1])) {
            default: *reinterpret_cast<int*>(_a[0]) = -1; break;
            case 0:
                *reinterpret_cast<int*>(_a[0]) = qRegisterMetaType< rviz::Display* >(); break;
            }
            break;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        void **func = reinterpret_cast<void **>(_a[1]);
        {
            typedef void (DisplayGroup::*_t)(rviz::Display * );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&DisplayGroup::displayAdded)) {
                *result = 0;
            }
        }
        {
            typedef void (DisplayGroup::*_t)(rviz::Display * );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&DisplayGroup::displayRemoved)) {
                *result = 1;
            }
        }
    }
}

const QMetaObject rviz::DisplayGroup::staticMetaObject = {
    { &Display::staticMetaObject, qt_meta_stringdata_rviz__DisplayGroup.data,
      qt_meta_data_rviz__DisplayGroup,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *rviz::DisplayGroup::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *rviz::DisplayGroup::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_rviz__DisplayGroup.stringdata0))
        return static_cast<void*>(const_cast< DisplayGroup*>(this));
    return Display::qt_metacast(_clname);
}

int rviz::DisplayGroup::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = Display::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 3)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 3;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 3)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 3;
    }
    return _id;
}

// SIGNAL 0
void rviz::DisplayGroup::displayAdded(rviz::Display * _t1)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void rviz::DisplayGroup::displayRemoved(rviz::Display * _t1)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}
QT_END_MOC_NAMESPACE
