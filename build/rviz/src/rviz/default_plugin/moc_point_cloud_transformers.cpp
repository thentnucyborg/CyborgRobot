/****************************************************************************
** Meta object code from reading C++ file 'point_cloud_transformers.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.5.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../../../src/rviz/src/rviz/default_plugin/point_cloud_transformers.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'point_cloud_transformers.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.5.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
struct qt_meta_stringdata_rviz__IntensityPCTransformer_t {
    QByteArrayData data[4];
    char stringdata0[80];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_rviz__IntensityPCTransformer_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_rviz__IntensityPCTransformer_t qt_meta_stringdata_rviz__IntensityPCTransformer = {
    {
QT_MOC_LITERAL(0, 0, 28), // "rviz::IntensityPCTransformer"
QT_MOC_LITERAL(1, 29, 16), // "updateUseRainbow"
QT_MOC_LITERAL(2, 46, 0), // ""
QT_MOC_LITERAL(3, 47, 32) // "updateAutoComputeIntensityBounds"

    },
    "rviz::IntensityPCTransformer\0"
    "updateUseRainbow\0\0updateAutoComputeIntensityBounds"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_rviz__IntensityPCTransformer[] = {

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
       1,    0,   24,    2, 0x08 /* Private */,
       3,    0,   25,    2, 0x08 /* Private */,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,

       0        // eod
};

void rviz::IntensityPCTransformer::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        IntensityPCTransformer *_t = static_cast<IntensityPCTransformer *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->updateUseRainbow(); break;
        case 1: _t->updateAutoComputeIntensityBounds(); break;
        default: ;
        }
    }
    Q_UNUSED(_a);
}

const QMetaObject rviz::IntensityPCTransformer::staticMetaObject = {
    { &PointCloudTransformer::staticMetaObject, qt_meta_stringdata_rviz__IntensityPCTransformer.data,
      qt_meta_data_rviz__IntensityPCTransformer,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *rviz::IntensityPCTransformer::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *rviz::IntensityPCTransformer::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_rviz__IntensityPCTransformer.stringdata0))
        return static_cast<void*>(const_cast< IntensityPCTransformer*>(this));
    return PointCloudTransformer::qt_metacast(_clname);
}

int rviz::IntensityPCTransformer::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = PointCloudTransformer::qt_metacall(_c, _id, _a);
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
struct qt_meta_stringdata_rviz__XYZPCTransformer_t {
    QByteArrayData data[1];
    char stringdata0[23];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_rviz__XYZPCTransformer_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_rviz__XYZPCTransformer_t qt_meta_stringdata_rviz__XYZPCTransformer = {
    {
QT_MOC_LITERAL(0, 0, 22) // "rviz::XYZPCTransformer"

    },
    "rviz::XYZPCTransformer"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_rviz__XYZPCTransformer[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       0,    0, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

       0        // eod
};

void rviz::XYZPCTransformer::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    Q_UNUSED(_o);
    Q_UNUSED(_id);
    Q_UNUSED(_c);
    Q_UNUSED(_a);
}

const QMetaObject rviz::XYZPCTransformer::staticMetaObject = {
    { &PointCloudTransformer::staticMetaObject, qt_meta_stringdata_rviz__XYZPCTransformer.data,
      qt_meta_data_rviz__XYZPCTransformer,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *rviz::XYZPCTransformer::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *rviz::XYZPCTransformer::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_rviz__XYZPCTransformer.stringdata0))
        return static_cast<void*>(const_cast< XYZPCTransformer*>(this));
    return PointCloudTransformer::qt_metacast(_clname);
}

int rviz::XYZPCTransformer::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = PointCloudTransformer::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    return _id;
}
struct qt_meta_stringdata_rviz__RGB8PCTransformer_t {
    QByteArrayData data[1];
    char stringdata0[24];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_rviz__RGB8PCTransformer_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_rviz__RGB8PCTransformer_t qt_meta_stringdata_rviz__RGB8PCTransformer = {
    {
QT_MOC_LITERAL(0, 0, 23) // "rviz::RGB8PCTransformer"

    },
    "rviz::RGB8PCTransformer"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_rviz__RGB8PCTransformer[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       0,    0, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

       0        // eod
};

void rviz::RGB8PCTransformer::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    Q_UNUSED(_o);
    Q_UNUSED(_id);
    Q_UNUSED(_c);
    Q_UNUSED(_a);
}

const QMetaObject rviz::RGB8PCTransformer::staticMetaObject = {
    { &PointCloudTransformer::staticMetaObject, qt_meta_stringdata_rviz__RGB8PCTransformer.data,
      qt_meta_data_rviz__RGB8PCTransformer,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *rviz::RGB8PCTransformer::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *rviz::RGB8PCTransformer::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_rviz__RGB8PCTransformer.stringdata0))
        return static_cast<void*>(const_cast< RGB8PCTransformer*>(this));
    return PointCloudTransformer::qt_metacast(_clname);
}

int rviz::RGB8PCTransformer::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = PointCloudTransformer::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    return _id;
}
struct qt_meta_stringdata_rviz__MONO8PCTransformer_t {
    QByteArrayData data[1];
    char stringdata0[25];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_rviz__MONO8PCTransformer_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_rviz__MONO8PCTransformer_t qt_meta_stringdata_rviz__MONO8PCTransformer = {
    {
QT_MOC_LITERAL(0, 0, 24) // "rviz::MONO8PCTransformer"

    },
    "rviz::MONO8PCTransformer"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_rviz__MONO8PCTransformer[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       0,    0, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

       0        // eod
};

void rviz::MONO8PCTransformer::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    Q_UNUSED(_o);
    Q_UNUSED(_id);
    Q_UNUSED(_c);
    Q_UNUSED(_a);
}

const QMetaObject rviz::MONO8PCTransformer::staticMetaObject = {
    { &RGB8PCTransformer::staticMetaObject, qt_meta_stringdata_rviz__MONO8PCTransformer.data,
      qt_meta_data_rviz__MONO8PCTransformer,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *rviz::MONO8PCTransformer::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *rviz::MONO8PCTransformer::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_rviz__MONO8PCTransformer.stringdata0))
        return static_cast<void*>(const_cast< MONO8PCTransformer*>(this));
    return RGB8PCTransformer::qt_metacast(_clname);
}

int rviz::MONO8PCTransformer::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = RGB8PCTransformer::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    return _id;
}
struct qt_meta_stringdata_rviz__RGBF32PCTransformer_t {
    QByteArrayData data[1];
    char stringdata0[26];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_rviz__RGBF32PCTransformer_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_rviz__RGBF32PCTransformer_t qt_meta_stringdata_rviz__RGBF32PCTransformer = {
    {
QT_MOC_LITERAL(0, 0, 25) // "rviz::RGBF32PCTransformer"

    },
    "rviz::RGBF32PCTransformer"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_rviz__RGBF32PCTransformer[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       0,    0, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

       0        // eod
};

void rviz::RGBF32PCTransformer::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    Q_UNUSED(_o);
    Q_UNUSED(_id);
    Q_UNUSED(_c);
    Q_UNUSED(_a);
}

const QMetaObject rviz::RGBF32PCTransformer::staticMetaObject = {
    { &PointCloudTransformer::staticMetaObject, qt_meta_stringdata_rviz__RGBF32PCTransformer.data,
      qt_meta_data_rviz__RGBF32PCTransformer,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *rviz::RGBF32PCTransformer::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *rviz::RGBF32PCTransformer::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_rviz__RGBF32PCTransformer.stringdata0))
        return static_cast<void*>(const_cast< RGBF32PCTransformer*>(this));
    return PointCloudTransformer::qt_metacast(_clname);
}

int rviz::RGBF32PCTransformer::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = PointCloudTransformer::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    return _id;
}
struct qt_meta_stringdata_rviz__FlatColorPCTransformer_t {
    QByteArrayData data[1];
    char stringdata0[29];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_rviz__FlatColorPCTransformer_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_rviz__FlatColorPCTransformer_t qt_meta_stringdata_rviz__FlatColorPCTransformer = {
    {
QT_MOC_LITERAL(0, 0, 28) // "rviz::FlatColorPCTransformer"

    },
    "rviz::FlatColorPCTransformer"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_rviz__FlatColorPCTransformer[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       0,    0, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

       0        // eod
};

void rviz::FlatColorPCTransformer::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    Q_UNUSED(_o);
    Q_UNUSED(_id);
    Q_UNUSED(_c);
    Q_UNUSED(_a);
}

const QMetaObject rviz::FlatColorPCTransformer::staticMetaObject = {
    { &PointCloudTransformer::staticMetaObject, qt_meta_stringdata_rviz__FlatColorPCTransformer.data,
      qt_meta_data_rviz__FlatColorPCTransformer,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *rviz::FlatColorPCTransformer::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *rviz::FlatColorPCTransformer::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_rviz__FlatColorPCTransformer.stringdata0))
        return static_cast<void*>(const_cast< FlatColorPCTransformer*>(this));
    return PointCloudTransformer::qt_metacast(_clname);
}

int rviz::FlatColorPCTransformer::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = PointCloudTransformer::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    return _id;
}
struct qt_meta_stringdata_rviz__AxisColorPCTransformer_t {
    QByteArrayData data[3];
    char stringdata0[54];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_rviz__AxisColorPCTransformer_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_rviz__AxisColorPCTransformer_t qt_meta_stringdata_rviz__AxisColorPCTransformer = {
    {
QT_MOC_LITERAL(0, 0, 28), // "rviz::AxisColorPCTransformer"
QT_MOC_LITERAL(1, 29, 23), // "updateAutoComputeBounds"
QT_MOC_LITERAL(2, 53, 0) // ""

    },
    "rviz::AxisColorPCTransformer\0"
    "updateAutoComputeBounds\0"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_rviz__AxisColorPCTransformer[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       1,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    0,   19,    2, 0x08 /* Private */,

 // slots: parameters
    QMetaType::Void,

       0        // eod
};

void rviz::AxisColorPCTransformer::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        AxisColorPCTransformer *_t = static_cast<AxisColorPCTransformer *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->updateAutoComputeBounds(); break;
        default: ;
        }
    }
    Q_UNUSED(_a);
}

const QMetaObject rviz::AxisColorPCTransformer::staticMetaObject = {
    { &PointCloudTransformer::staticMetaObject, qt_meta_stringdata_rviz__AxisColorPCTransformer.data,
      qt_meta_data_rviz__AxisColorPCTransformer,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *rviz::AxisColorPCTransformer::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *rviz::AxisColorPCTransformer::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_rviz__AxisColorPCTransformer.stringdata0))
        return static_cast<void*>(const_cast< AxisColorPCTransformer*>(this));
    return PointCloudTransformer::qt_metacast(_clname);
}

int rviz::AxisColorPCTransformer::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = PointCloudTransformer::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 1)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 1;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 1)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 1;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
