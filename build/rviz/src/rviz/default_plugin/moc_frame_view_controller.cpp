/****************************************************************************
** Meta object code from reading C++ file 'frame_view_controller.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.5.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../../../src/rviz/src/rviz/default_plugin/view_controllers/frame_view_controller.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'frame_view_controller.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.5.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
struct qt_meta_stringdata_rviz__FrameViewController_t {
    QByteArrayData data[5];
    char stringdata0[74];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_rviz__FrameViewController_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_rviz__FrameViewController_t qt_meta_stringdata_rviz__FrameViewController = {
    {
QT_MOC_LITERAL(0, 0, 25), // "rviz::FrameViewController"
QT_MOC_LITERAL(1, 26, 15), // "changedPosition"
QT_MOC_LITERAL(2, 42, 0), // ""
QT_MOC_LITERAL(3, 43, 18), // "changedOrientation"
QT_MOC_LITERAL(4, 62, 11) // "changedAxis"

    },
    "rviz::FrameViewController\0changedPosition\0"
    "\0changedOrientation\0changedAxis"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_rviz__FrameViewController[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       3,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    0,   29,    2, 0x09 /* Protected */,
       3,    0,   30,    2, 0x09 /* Protected */,
       4,    0,   31,    2, 0x09 /* Protected */,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,

       0        // eod
};

void rviz::FrameViewController::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        FrameViewController *_t = static_cast<FrameViewController *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->changedPosition(); break;
        case 1: _t->changedOrientation(); break;
        case 2: _t->changedAxis(); break;
        default: ;
        }
    }
    Q_UNUSED(_a);
}

const QMetaObject rviz::FrameViewController::staticMetaObject = {
    { &FramePositionTrackingViewController::staticMetaObject, qt_meta_stringdata_rviz__FrameViewController.data,
      qt_meta_data_rviz__FrameViewController,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *rviz::FrameViewController::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *rviz::FrameViewController::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_rviz__FrameViewController.stringdata0))
        return static_cast<void*>(const_cast< FrameViewController*>(this));
    return FramePositionTrackingViewController::qt_metacast(_clname);
}

int rviz::FrameViewController::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = FramePositionTrackingViewController::qt_metacall(_c, _id, _a);
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
QT_END_MOC_NAMESPACE
