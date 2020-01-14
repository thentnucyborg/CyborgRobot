/****************************************************************************
** Meta object code from reading C++ file 'interactive_marker.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.5.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../../../src/rviz/src/rviz/default_plugin/interactive_markers/interactive_marker.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'interactive_marker.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.5.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
struct qt_meta_stringdata_rviz__InteractiveMarker_t {
    QByteArrayData data[13];
    char stringdata0[187];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_rviz__InteractiveMarker_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_rviz__InteractiveMarker_t qt_meta_stringdata_rviz__InteractiveMarker = {
    {
QT_MOC_LITERAL(0, 0, 23), // "rviz::InteractiveMarker"
QT_MOC_LITERAL(1, 24, 12), // "userFeedback"
QT_MOC_LITERAL(2, 37, 0), // ""
QT_MOC_LITERAL(3, 38, 46), // "visualization_msgs::Interacti..."
QT_MOC_LITERAL(4, 85, 8), // "feedback"
QT_MOC_LITERAL(5, 94, 12), // "statusUpdate"
QT_MOC_LITERAL(6, 107, 21), // "StatusProperty::Level"
QT_MOC_LITERAL(7, 129, 5), // "level"
QT_MOC_LITERAL(8, 135, 11), // "std::string"
QT_MOC_LITERAL(9, 147, 4), // "name"
QT_MOC_LITERAL(10, 152, 4), // "text"
QT_MOC_LITERAL(11, 157, 16), // "handleMenuSelect"
QT_MOC_LITERAL(12, 174, 12) // "menu_item_id"

    },
    "rviz::InteractiveMarker\0userFeedback\0"
    "\0visualization_msgs::InteractiveMarkerFeedback&\0"
    "feedback\0statusUpdate\0StatusProperty::Level\0"
    "level\0std::string\0name\0text\0"
    "handleMenuSelect\0menu_item_id"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_rviz__InteractiveMarker[] = {

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
       5,    3,   32,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
      11,    1,   39,    2, 0x09 /* Protected */,

 // signals: parameters
    QMetaType::Void, 0x80000000 | 3,    4,
    QMetaType::Void, 0x80000000 | 6, 0x80000000 | 8, 0x80000000 | 8,    7,    9,   10,

 // slots: parameters
    QMetaType::Void, QMetaType::Int,   12,

       0        // eod
};

void rviz::InteractiveMarker::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        InteractiveMarker *_t = static_cast<InteractiveMarker *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->userFeedback((*reinterpret_cast< visualization_msgs::InteractiveMarkerFeedback(*)>(_a[1]))); break;
        case 1: _t->statusUpdate((*reinterpret_cast< StatusProperty::Level(*)>(_a[1])),(*reinterpret_cast< const std::string(*)>(_a[2])),(*reinterpret_cast< const std::string(*)>(_a[3]))); break;
        case 2: _t->handleMenuSelect((*reinterpret_cast< int(*)>(_a[1]))); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        void **func = reinterpret_cast<void **>(_a[1]);
        {
            typedef void (InteractiveMarker::*_t)(visualization_msgs::InteractiveMarkerFeedback & );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&InteractiveMarker::userFeedback)) {
                *result = 0;
            }
        }
        {
            typedef void (InteractiveMarker::*_t)(StatusProperty::Level , const std::string & , const std::string & );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&InteractiveMarker::statusUpdate)) {
                *result = 1;
            }
        }
    }
}

const QMetaObject rviz::InteractiveMarker::staticMetaObject = {
    { &QObject::staticMetaObject, qt_meta_stringdata_rviz__InteractiveMarker.data,
      qt_meta_data_rviz__InteractiveMarker,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *rviz::InteractiveMarker::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *rviz::InteractiveMarker::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_rviz__InteractiveMarker.stringdata0))
        return static_cast<void*>(const_cast< InteractiveMarker*>(this));
    return QObject::qt_metacast(_clname);
}

int rviz::InteractiveMarker::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 3)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 3;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 3)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 3;
    }
    return _id;
}

// SIGNAL 0
void rviz::InteractiveMarker::userFeedback(visualization_msgs::InteractiveMarkerFeedback & _t1)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void rviz::InteractiveMarker::statusUpdate(StatusProperty::Level _t1, const std::string & _t2, const std::string & _t3)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)), const_cast<void*>(reinterpret_cast<const void*>(&_t3)) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}
QT_END_MOC_NAMESPACE
