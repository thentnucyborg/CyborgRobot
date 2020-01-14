/****************************************************************************
** Meta object code from reading C++ file 'tool_manager.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.5.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../../src/rviz/src/rviz/tool_manager.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'tool_manager.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.5.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
struct qt_meta_stringdata_rviz__ToolManager_t {
    QByteArrayData data[12];
    char stringdata0[141];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_rviz__ToolManager_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_rviz__ToolManager_t qt_meta_stringdata_rviz__ToolManager = {
    {
QT_MOC_LITERAL(0, 0, 17), // "rviz::ToolManager"
QT_MOC_LITERAL(1, 18, 13), // "configChanged"
QT_MOC_LITERAL(2, 32, 0), // ""
QT_MOC_LITERAL(3, 33, 9), // "toolAdded"
QT_MOC_LITERAL(4, 43, 5), // "Tool*"
QT_MOC_LITERAL(5, 49, 11), // "toolChanged"
QT_MOC_LITERAL(6, 61, 11), // "toolRemoved"
QT_MOC_LITERAL(7, 73, 13), // "toolRefreshed"
QT_MOC_LITERAL(8, 87, 24), // "updatePropertyVisibility"
QT_MOC_LITERAL(9, 112, 9), // "Property*"
QT_MOC_LITERAL(10, 122, 8), // "property"
QT_MOC_LITERAL(11, 131, 9) // "closeTool"

    },
    "rviz::ToolManager\0configChanged\0\0"
    "toolAdded\0Tool*\0toolChanged\0toolRemoved\0"
    "toolRefreshed\0updatePropertyVisibility\0"
    "Property*\0property\0closeTool"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_rviz__ToolManager[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       7,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       5,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    0,   49,    2, 0x06 /* Public */,
       3,    1,   50,    2, 0x06 /* Public */,
       5,    1,   53,    2, 0x06 /* Public */,
       6,    1,   56,    2, 0x06 /* Public */,
       7,    1,   59,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
       8,    1,   62,    2, 0x08 /* Private */,
      11,    0,   65,    2, 0x08 /* Private */,

 // signals: parameters
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 4,    2,
    QMetaType::Void, 0x80000000 | 4,    2,
    QMetaType::Void, 0x80000000 | 4,    2,
    QMetaType::Void, 0x80000000 | 4,    2,

 // slots: parameters
    QMetaType::Void, 0x80000000 | 9,   10,
    QMetaType::Void,

       0        // eod
};

void rviz::ToolManager::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        ToolManager *_t = static_cast<ToolManager *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->configChanged(); break;
        case 1: _t->toolAdded((*reinterpret_cast< Tool*(*)>(_a[1]))); break;
        case 2: _t->toolChanged((*reinterpret_cast< Tool*(*)>(_a[1]))); break;
        case 3: _t->toolRemoved((*reinterpret_cast< Tool*(*)>(_a[1]))); break;
        case 4: _t->toolRefreshed((*reinterpret_cast< Tool*(*)>(_a[1]))); break;
        case 5: _t->updatePropertyVisibility((*reinterpret_cast< Property*(*)>(_a[1]))); break;
        case 6: _t->closeTool(); break;
        default: ;
        }
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        switch (_id) {
        default: *reinterpret_cast<int*>(_a[0]) = -1; break;
        case 1:
            switch (*reinterpret_cast<int*>(_a[1])) {
            default: *reinterpret_cast<int*>(_a[0]) = -1; break;
            case 0:
                *reinterpret_cast<int*>(_a[0]) = qRegisterMetaType< Tool* >(); break;
            }
            break;
        case 2:
            switch (*reinterpret_cast<int*>(_a[1])) {
            default: *reinterpret_cast<int*>(_a[0]) = -1; break;
            case 0:
                *reinterpret_cast<int*>(_a[0]) = qRegisterMetaType< Tool* >(); break;
            }
            break;
        case 3:
            switch (*reinterpret_cast<int*>(_a[1])) {
            default: *reinterpret_cast<int*>(_a[0]) = -1; break;
            case 0:
                *reinterpret_cast<int*>(_a[0]) = qRegisterMetaType< Tool* >(); break;
            }
            break;
        case 4:
            switch (*reinterpret_cast<int*>(_a[1])) {
            default: *reinterpret_cast<int*>(_a[0]) = -1; break;
            case 0:
                *reinterpret_cast<int*>(_a[0]) = qRegisterMetaType< Tool* >(); break;
            }
            break;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        void **func = reinterpret_cast<void **>(_a[1]);
        {
            typedef void (ToolManager::*_t)();
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&ToolManager::configChanged)) {
                *result = 0;
            }
        }
        {
            typedef void (ToolManager::*_t)(Tool * );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&ToolManager::toolAdded)) {
                *result = 1;
            }
        }
        {
            typedef void (ToolManager::*_t)(Tool * );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&ToolManager::toolChanged)) {
                *result = 2;
            }
        }
        {
            typedef void (ToolManager::*_t)(Tool * );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&ToolManager::toolRemoved)) {
                *result = 3;
            }
        }
        {
            typedef void (ToolManager::*_t)(Tool * );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&ToolManager::toolRefreshed)) {
                *result = 4;
            }
        }
    }
}

const QMetaObject rviz::ToolManager::staticMetaObject = {
    { &QObject::staticMetaObject, qt_meta_stringdata_rviz__ToolManager.data,
      qt_meta_data_rviz__ToolManager,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *rviz::ToolManager::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *rviz::ToolManager::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_rviz__ToolManager.stringdata0))
        return static_cast<void*>(const_cast< ToolManager*>(this));
    return QObject::qt_metacast(_clname);
}

int rviz::ToolManager::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 7)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 7;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 7)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 7;
    }
    return _id;
}

// SIGNAL 0
void rviz::ToolManager::configChanged()
{
    QMetaObject::activate(this, &staticMetaObject, 0, Q_NULLPTR);
}

// SIGNAL 1
void rviz::ToolManager::toolAdded(Tool * _t1)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}

// SIGNAL 2
void rviz::ToolManager::toolChanged(Tool * _t1)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 2, _a);
}

// SIGNAL 3
void rviz::ToolManager::toolRemoved(Tool * _t1)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 3, _a);
}

// SIGNAL 4
void rviz::ToolManager::toolRefreshed(Tool * _t1)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 4, _a);
}
QT_END_MOC_NAMESPACE
