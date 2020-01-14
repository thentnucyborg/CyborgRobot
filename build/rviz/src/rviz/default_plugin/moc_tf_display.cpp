/****************************************************************************
** Meta object code from reading C++ file 'tf_display.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.5.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../../../src/rviz/src/rviz/default_plugin/tf_display.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'tf_display.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.5.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
struct qt_meta_stringdata_rviz__TFDisplay_t {
    QByteArrayData data[6];
    char stringdata0[83];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_rviz__TFDisplay_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_rviz__TFDisplay_t qt_meta_stringdata_rviz__TFDisplay = {
    {
QT_MOC_LITERAL(0, 0, 15), // "rviz::TFDisplay"
QT_MOC_LITERAL(1, 16, 14), // "updateShowAxes"
QT_MOC_LITERAL(2, 31, 0), // ""
QT_MOC_LITERAL(3, 32, 16), // "updateShowArrows"
QT_MOC_LITERAL(4, 49, 15), // "updateShowNames"
QT_MOC_LITERAL(5, 65, 17) // "allEnabledChanged"

    },
    "rviz::TFDisplay\0updateShowAxes\0\0"
    "updateShowArrows\0updateShowNames\0"
    "allEnabledChanged"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_rviz__TFDisplay[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       4,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    0,   34,    2, 0x08 /* Private */,
       3,    0,   35,    2, 0x08 /* Private */,
       4,    0,   36,    2, 0x08 /* Private */,
       5,    0,   37,    2, 0x08 /* Private */,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,

       0        // eod
};

void rviz::TFDisplay::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        TFDisplay *_t = static_cast<TFDisplay *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->updateShowAxes(); break;
        case 1: _t->updateShowArrows(); break;
        case 2: _t->updateShowNames(); break;
        case 3: _t->allEnabledChanged(); break;
        default: ;
        }
    }
    Q_UNUSED(_a);
}

const QMetaObject rviz::TFDisplay::staticMetaObject = {
    { &Display::staticMetaObject, qt_meta_stringdata_rviz__TFDisplay.data,
      qt_meta_data_rviz__TFDisplay,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *rviz::TFDisplay::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *rviz::TFDisplay::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_rviz__TFDisplay.stringdata0))
        return static_cast<void*>(const_cast< TFDisplay*>(this));
    return Display::qt_metacast(_clname);
}

int rviz::TFDisplay::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = Display::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 4)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 4;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 4)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 4;
    }
    return _id;
}
struct qt_meta_stringdata_rviz__FrameInfo_t {
    QByteArrayData data[4];
    char stringdata0[73];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_rviz__FrameInfo_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_rviz__FrameInfo_t qt_meta_stringdata_rviz__FrameInfo = {
    {
QT_MOC_LITERAL(0, 0, 15), // "rviz::FrameInfo"
QT_MOC_LITERAL(1, 16, 25), // "updateVisibilityFromFrame"
QT_MOC_LITERAL(2, 42, 0), // ""
QT_MOC_LITERAL(3, 43, 29) // "updateVisibilityFromSelection"

    },
    "rviz::FrameInfo\0updateVisibilityFromFrame\0"
    "\0updateVisibilityFromSelection"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_rviz__FrameInfo[] = {

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
       1,    0,   24,    2, 0x0a /* Public */,
       3,    0,   25,    2, 0x0a /* Public */,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,

       0        // eod
};

void rviz::FrameInfo::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        FrameInfo *_t = static_cast<FrameInfo *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->updateVisibilityFromFrame(); break;
        case 1: _t->updateVisibilityFromSelection(); break;
        default: ;
        }
    }
    Q_UNUSED(_a);
}

const QMetaObject rviz::FrameInfo::staticMetaObject = {
    { &QObject::staticMetaObject, qt_meta_stringdata_rviz__FrameInfo.data,
      qt_meta_data_rviz__FrameInfo,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *rviz::FrameInfo::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *rviz::FrameInfo::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_rviz__FrameInfo.stringdata0))
        return static_cast<void*>(const_cast< FrameInfo*>(this));
    return QObject::qt_metacast(_clname);
}

int rviz::FrameInfo::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
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
