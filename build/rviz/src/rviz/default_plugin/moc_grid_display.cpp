/****************************************************************************
** Meta object code from reading C++ file 'grid_display.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.5.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../../../src/rviz/src/rviz/default_plugin/grid_display.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'grid_display.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.5.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
struct qt_meta_stringdata_rviz__GridDisplay_t {
    QByteArrayData data[10];
    char stringdata0[128];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_rviz__GridDisplay_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_rviz__GridDisplay_t qt_meta_stringdata_rviz__GridDisplay = {
    {
QT_MOC_LITERAL(0, 0, 17), // "rviz::GridDisplay"
QT_MOC_LITERAL(1, 18, 15), // "updateCellCount"
QT_MOC_LITERAL(2, 34, 0), // ""
QT_MOC_LITERAL(3, 35, 14), // "updateCellSize"
QT_MOC_LITERAL(4, 50, 11), // "updateColor"
QT_MOC_LITERAL(5, 62, 12), // "updateHeight"
QT_MOC_LITERAL(6, 75, 15), // "updateLineWidth"
QT_MOC_LITERAL(7, 91, 12), // "updateOffset"
QT_MOC_LITERAL(8, 104, 11), // "updatePlane"
QT_MOC_LITERAL(9, 116, 11) // "updateStyle"

    },
    "rviz::GridDisplay\0updateCellCount\0\0"
    "updateCellSize\0updateColor\0updateHeight\0"
    "updateLineWidth\0updateOffset\0updatePlane\0"
    "updateStyle"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_rviz__GridDisplay[] = {

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
       1,    0,   54,    2, 0x08 /* Private */,
       3,    0,   55,    2, 0x08 /* Private */,
       4,    0,   56,    2, 0x08 /* Private */,
       5,    0,   57,    2, 0x08 /* Private */,
       6,    0,   58,    2, 0x08 /* Private */,
       7,    0,   59,    2, 0x08 /* Private */,
       8,    0,   60,    2, 0x08 /* Private */,
       9,    0,   61,    2, 0x08 /* Private */,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,

       0        // eod
};

void rviz::GridDisplay::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        GridDisplay *_t = static_cast<GridDisplay *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->updateCellCount(); break;
        case 1: _t->updateCellSize(); break;
        case 2: _t->updateColor(); break;
        case 3: _t->updateHeight(); break;
        case 4: _t->updateLineWidth(); break;
        case 5: _t->updateOffset(); break;
        case 6: _t->updatePlane(); break;
        case 7: _t->updateStyle(); break;
        default: ;
        }
    }
    Q_UNUSED(_a);
}

const QMetaObject rviz::GridDisplay::staticMetaObject = {
    { &Display::staticMetaObject, qt_meta_stringdata_rviz__GridDisplay.data,
      qt_meta_data_rviz__GridDisplay,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *rviz::GridDisplay::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *rviz::GridDisplay::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_rviz__GridDisplay.stringdata0))
        return static_cast<void*>(const_cast< GridDisplay*>(this));
    return Display::qt_metacast(_clname);
}

int rviz::GridDisplay::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = Display::qt_metacall(_c, _id, _a);
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
