/****************************************************************************
** Meta object code from reading C++ file 'add_display_dialog.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.5.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../../src/rviz/src/rviz/add_display_dialog.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'add_display_dialog.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.5.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
struct qt_meta_stringdata_rviz__AddDisplayDialog_t {
    QByteArrayData data[10];
    char stringdata0[118];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_rviz__AddDisplayDialog_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_rviz__AddDisplayDialog_t qt_meta_stringdata_rviz__AddDisplayDialog = {
    {
QT_MOC_LITERAL(0, 0, 22), // "rviz::AddDisplayDialog"
QT_MOC_LITERAL(1, 23, 6), // "accept"
QT_MOC_LITERAL(2, 30, 0), // ""
QT_MOC_LITERAL(3, 31, 17), // "onDisplaySelected"
QT_MOC_LITERAL(4, 49, 14), // "SelectionData*"
QT_MOC_LITERAL(5, 64, 4), // "data"
QT_MOC_LITERAL(6, 69, 15), // "onTopicSelected"
QT_MOC_LITERAL(7, 85, 12), // "onTabChanged"
QT_MOC_LITERAL(8, 98, 5), // "index"
QT_MOC_LITERAL(9, 104, 13) // "onNameChanged"

    },
    "rviz::AddDisplayDialog\0accept\0\0"
    "onDisplaySelected\0SelectionData*\0data\0"
    "onTopicSelected\0onTabChanged\0index\0"
    "onNameChanged"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_rviz__AddDisplayDialog[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       5,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    0,   39,    2, 0x0a /* Public */,
       3,    1,   40,    2, 0x08 /* Private */,
       6,    1,   43,    2, 0x08 /* Private */,
       7,    1,   46,    2, 0x08 /* Private */,
       9,    0,   49,    2, 0x08 /* Private */,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 4,    5,
    QMetaType::Void, 0x80000000 | 4,    5,
    QMetaType::Void, QMetaType::Int,    8,
    QMetaType::Void,

       0        // eod
};

void rviz::AddDisplayDialog::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        AddDisplayDialog *_t = static_cast<AddDisplayDialog *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->accept(); break;
        case 1: _t->onDisplaySelected((*reinterpret_cast< SelectionData*(*)>(_a[1]))); break;
        case 2: _t->onTopicSelected((*reinterpret_cast< SelectionData*(*)>(_a[1]))); break;
        case 3: _t->onTabChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 4: _t->onNameChanged(); break;
        default: ;
        }
    }
}

const QMetaObject rviz::AddDisplayDialog::staticMetaObject = {
    { &QDialog::staticMetaObject, qt_meta_stringdata_rviz__AddDisplayDialog.data,
      qt_meta_data_rviz__AddDisplayDialog,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *rviz::AddDisplayDialog::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *rviz::AddDisplayDialog::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_rviz__AddDisplayDialog.stringdata0))
        return static_cast<void*>(const_cast< AddDisplayDialog*>(this));
    return QDialog::qt_metacast(_clname);
}

int rviz::AddDisplayDialog::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QDialog::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 5)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 5;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 5)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 5;
    }
    return _id;
}
struct qt_meta_stringdata_rviz__DisplayTypeTree_t {
    QByteArrayData data[9];
    char stringdata0[108];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_rviz__DisplayTypeTree_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_rviz__DisplayTypeTree_t qt_meta_stringdata_rviz__DisplayTypeTree = {
    {
QT_MOC_LITERAL(0, 0, 21), // "rviz::DisplayTypeTree"
QT_MOC_LITERAL(1, 22, 11), // "itemChanged"
QT_MOC_LITERAL(2, 34, 0), // ""
QT_MOC_LITERAL(3, 35, 14), // "SelectionData*"
QT_MOC_LITERAL(4, 50, 9), // "selection"
QT_MOC_LITERAL(5, 60, 20), // "onCurrentItemChanged"
QT_MOC_LITERAL(6, 81, 16), // "QTreeWidgetItem*"
QT_MOC_LITERAL(7, 98, 4), // "curr"
QT_MOC_LITERAL(8, 103, 4) // "prev"

    },
    "rviz::DisplayTypeTree\0itemChanged\0\0"
    "SelectionData*\0selection\0onCurrentItemChanged\0"
    "QTreeWidgetItem*\0curr\0prev"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_rviz__DisplayTypeTree[] = {

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
       5,    2,   27,    2, 0x08 /* Private */,

 // signals: parameters
    QMetaType::Void, 0x80000000 | 3,    4,

 // slots: parameters
    QMetaType::Void, 0x80000000 | 6, 0x80000000 | 6,    7,    8,

       0        // eod
};

void rviz::DisplayTypeTree::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        DisplayTypeTree *_t = static_cast<DisplayTypeTree *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->itemChanged((*reinterpret_cast< SelectionData*(*)>(_a[1]))); break;
        case 1: _t->onCurrentItemChanged((*reinterpret_cast< QTreeWidgetItem*(*)>(_a[1])),(*reinterpret_cast< QTreeWidgetItem*(*)>(_a[2]))); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        void **func = reinterpret_cast<void **>(_a[1]);
        {
            typedef void (DisplayTypeTree::*_t)(SelectionData * );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&DisplayTypeTree::itemChanged)) {
                *result = 0;
            }
        }
    }
}

const QMetaObject rviz::DisplayTypeTree::staticMetaObject = {
    { &QTreeWidget::staticMetaObject, qt_meta_stringdata_rviz__DisplayTypeTree.data,
      qt_meta_data_rviz__DisplayTypeTree,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *rviz::DisplayTypeTree::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *rviz::DisplayTypeTree::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_rviz__DisplayTypeTree.stringdata0))
        return static_cast<void*>(const_cast< DisplayTypeTree*>(this));
    return QTreeWidget::qt_metacast(_clname);
}

int rviz::DisplayTypeTree::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QTreeWidget::qt_metacall(_c, _id, _a);
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

// SIGNAL 0
void rviz::DisplayTypeTree::itemChanged(SelectionData * _t1)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}
struct qt_meta_stringdata_rviz__TopicDisplayWidget_t {
    QByteArrayData data[14];
    char stringdata0[169];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_rviz__TopicDisplayWidget_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_rviz__TopicDisplayWidget_t qt_meta_stringdata_rviz__TopicDisplayWidget = {
    {
QT_MOC_LITERAL(0, 0, 24), // "rviz::TopicDisplayWidget"
QT_MOC_LITERAL(1, 25, 11), // "itemChanged"
QT_MOC_LITERAL(2, 37, 0), // ""
QT_MOC_LITERAL(3, 38, 14), // "SelectionData*"
QT_MOC_LITERAL(4, 53, 9), // "selection"
QT_MOC_LITERAL(5, 63, 13), // "itemActivated"
QT_MOC_LITERAL(6, 77, 16), // "QTreeWidgetItem*"
QT_MOC_LITERAL(7, 94, 4), // "item"
QT_MOC_LITERAL(8, 99, 6), // "column"
QT_MOC_LITERAL(9, 106, 12), // "stateChanged"
QT_MOC_LITERAL(10, 119, 5), // "state"
QT_MOC_LITERAL(11, 125, 20), // "onCurrentItemChanged"
QT_MOC_LITERAL(12, 146, 4), // "curr"
QT_MOC_LITERAL(13, 151, 17) // "onComboBoxClicked"

    },
    "rviz::TopicDisplayWidget\0itemChanged\0"
    "\0SelectionData*\0selection\0itemActivated\0"
    "QTreeWidgetItem*\0item\0column\0stateChanged\0"
    "state\0onCurrentItemChanged\0curr\0"
    "onComboBoxClicked"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_rviz__TopicDisplayWidget[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       5,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       2,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    1,   39,    2, 0x06 /* Public */,
       5,    2,   42,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
       9,    1,   47,    2, 0x08 /* Private */,
      11,    1,   50,    2, 0x08 /* Private */,
      13,    1,   53,    2, 0x08 /* Private */,

 // signals: parameters
    QMetaType::Void, 0x80000000 | 3,    4,
    QMetaType::Void, 0x80000000 | 6, QMetaType::Int,    7,    8,

 // slots: parameters
    QMetaType::Void, QMetaType::Int,   10,
    QMetaType::Void, 0x80000000 | 6,   12,
    QMetaType::Void, 0x80000000 | 6,   12,

       0        // eod
};

void rviz::TopicDisplayWidget::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        TopicDisplayWidget *_t = static_cast<TopicDisplayWidget *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->itemChanged((*reinterpret_cast< SelectionData*(*)>(_a[1]))); break;
        case 1: _t->itemActivated((*reinterpret_cast< QTreeWidgetItem*(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2]))); break;
        case 2: _t->stateChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 3: _t->onCurrentItemChanged((*reinterpret_cast< QTreeWidgetItem*(*)>(_a[1]))); break;
        case 4: _t->onComboBoxClicked((*reinterpret_cast< QTreeWidgetItem*(*)>(_a[1]))); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        void **func = reinterpret_cast<void **>(_a[1]);
        {
            typedef void (TopicDisplayWidget::*_t)(SelectionData * );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&TopicDisplayWidget::itemChanged)) {
                *result = 0;
            }
        }
        {
            typedef void (TopicDisplayWidget::*_t)(QTreeWidgetItem * , int );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&TopicDisplayWidget::itemActivated)) {
                *result = 1;
            }
        }
    }
}

const QMetaObject rviz::TopicDisplayWidget::staticMetaObject = {
    { &QWidget::staticMetaObject, qt_meta_stringdata_rviz__TopicDisplayWidget.data,
      qt_meta_data_rviz__TopicDisplayWidget,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *rviz::TopicDisplayWidget::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *rviz::TopicDisplayWidget::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_rviz__TopicDisplayWidget.stringdata0))
        return static_cast<void*>(const_cast< TopicDisplayWidget*>(this));
    return QWidget::qt_metacast(_clname);
}

int rviz::TopicDisplayWidget::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QWidget::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 5)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 5;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 5)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 5;
    }
    return _id;
}

// SIGNAL 0
void rviz::TopicDisplayWidget::itemChanged(SelectionData * _t1)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void rviz::TopicDisplayWidget::itemActivated(QTreeWidgetItem * _t1, int _t2)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}
struct qt_meta_stringdata_rviz__EmbeddableComboBox_t {
    QByteArrayData data[8];
    char stringdata0[85];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_rviz__EmbeddableComboBox_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_rviz__EmbeddableComboBox_t qt_meta_stringdata_rviz__EmbeddableComboBox = {
    {
QT_MOC_LITERAL(0, 0, 24), // "rviz::EmbeddableComboBox"
QT_MOC_LITERAL(1, 25, 11), // "itemClicked"
QT_MOC_LITERAL(2, 37, 0), // ""
QT_MOC_LITERAL(3, 38, 16), // "QTreeWidgetItem*"
QT_MOC_LITERAL(4, 55, 4), // "item"
QT_MOC_LITERAL(5, 60, 6), // "column"
QT_MOC_LITERAL(6, 67, 11), // "onActivated"
QT_MOC_LITERAL(7, 79, 5) // "index"

    },
    "rviz::EmbeddableComboBox\0itemClicked\0"
    "\0QTreeWidgetItem*\0item\0column\0onActivated\0"
    "index"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_rviz__EmbeddableComboBox[] = {

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
       1,    2,   24,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
       6,    1,   29,    2, 0x08 /* Private */,

 // signals: parameters
    QMetaType::Void, 0x80000000 | 3, QMetaType::Int,    4,    5,

 // slots: parameters
    QMetaType::Void, QMetaType::Int,    7,

       0        // eod
};

void rviz::EmbeddableComboBox::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        EmbeddableComboBox *_t = static_cast<EmbeddableComboBox *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->itemClicked((*reinterpret_cast< QTreeWidgetItem*(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2]))); break;
        case 1: _t->onActivated((*reinterpret_cast< int(*)>(_a[1]))); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        void **func = reinterpret_cast<void **>(_a[1]);
        {
            typedef void (EmbeddableComboBox::*_t)(QTreeWidgetItem * , int );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&EmbeddableComboBox::itemClicked)) {
                *result = 0;
            }
        }
    }
}

const QMetaObject rviz::EmbeddableComboBox::staticMetaObject = {
    { &QComboBox::staticMetaObject, qt_meta_stringdata_rviz__EmbeddableComboBox.data,
      qt_meta_data_rviz__EmbeddableComboBox,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *rviz::EmbeddableComboBox::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *rviz::EmbeddableComboBox::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_rviz__EmbeddableComboBox.stringdata0))
        return static_cast<void*>(const_cast< EmbeddableComboBox*>(this));
    return QComboBox::qt_metacast(_clname);
}

int rviz::EmbeddableComboBox::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QComboBox::qt_metacall(_c, _id, _a);
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

// SIGNAL 0
void rviz::EmbeddableComboBox::itemClicked(QTreeWidgetItem * _t1, int _t2)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}
QT_END_MOC_NAMESPACE
