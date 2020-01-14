/****************************************************************************
** Meta object code from reading C++ file 'screenshot_dialog.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.5.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../../src/rviz/src/rviz/screenshot_dialog.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'screenshot_dialog.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.5.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
struct qt_meta_stringdata_rviz__ScreenshotDialog_t {
    QByteArrayData data[13];
    char stringdata0[175];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_rviz__ScreenshotDialog_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_rviz__ScreenshotDialog_t qt_meta_stringdata_rviz__ScreenshotDialog = {
    {
QT_MOC_LITERAL(0, 0, 22), // "rviz::ScreenshotDialog"
QT_MOC_LITERAL(1, 23, 16), // "savedInDirectory"
QT_MOC_LITERAL(2, 40, 0), // ""
QT_MOC_LITERAL(3, 41, 9), // "directory"
QT_MOC_LITERAL(4, 51, 14), // "takeScreenshot"
QT_MOC_LITERAL(5, 66, 9), // "onTimeout"
QT_MOC_LITERAL(6, 76, 17), // "takeScreenshotNow"
QT_MOC_LITERAL(7, 94, 4), // "save"
QT_MOC_LITERAL(8, 99, 15), // "onButtonClicked"
QT_MOC_LITERAL(9, 115, 16), // "QAbstractButton*"
QT_MOC_LITERAL(10, 132, 7), // "clicked"
QT_MOC_LITERAL(11, 140, 17), // "setSaveFullWindow"
QT_MOC_LITERAL(12, 158, 16) // "save_full_window"

    },
    "rviz::ScreenshotDialog\0savedInDirectory\0"
    "\0directory\0takeScreenshot\0onTimeout\0"
    "takeScreenshotNow\0save\0onButtonClicked\0"
    "QAbstractButton*\0clicked\0setSaveFullWindow\0"
    "save_full_window"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_rviz__ScreenshotDialog[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       7,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       1,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    1,   49,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
       4,    0,   52,    2, 0x09 /* Protected */,
       5,    0,   53,    2, 0x09 /* Protected */,
       6,    0,   54,    2, 0x09 /* Protected */,
       7,    0,   55,    2, 0x09 /* Protected */,
       8,    1,   56,    2, 0x09 /* Protected */,
      11,    1,   59,    2, 0x09 /* Protected */,

 // signals: parameters
    QMetaType::Void, QMetaType::QString,    3,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 9,   10,
    QMetaType::Void, QMetaType::Bool,   12,

       0        // eod
};

void rviz::ScreenshotDialog::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        ScreenshotDialog *_t = static_cast<ScreenshotDialog *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->savedInDirectory((*reinterpret_cast< const QString(*)>(_a[1]))); break;
        case 1: _t->takeScreenshot(); break;
        case 2: _t->onTimeout(); break;
        case 3: _t->takeScreenshotNow(); break;
        case 4: _t->save(); break;
        case 5: _t->onButtonClicked((*reinterpret_cast< QAbstractButton*(*)>(_a[1]))); break;
        case 6: _t->setSaveFullWindow((*reinterpret_cast< bool(*)>(_a[1]))); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        void **func = reinterpret_cast<void **>(_a[1]);
        {
            typedef void (ScreenshotDialog::*_t)(const QString & );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&ScreenshotDialog::savedInDirectory)) {
                *result = 0;
            }
        }
    }
}

const QMetaObject rviz::ScreenshotDialog::staticMetaObject = {
    { &QWidget::staticMetaObject, qt_meta_stringdata_rviz__ScreenshotDialog.data,
      qt_meta_data_rviz__ScreenshotDialog,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *rviz::ScreenshotDialog::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *rviz::ScreenshotDialog::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_rviz__ScreenshotDialog.stringdata0))
        return static_cast<void*>(const_cast< ScreenshotDialog*>(this));
    return QWidget::qt_metacast(_clname);
}

int rviz::ScreenshotDialog::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QWidget::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 7)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 7;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 7)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 7;
    }
    return _id;
}

// SIGNAL 0
void rviz::ScreenshotDialog::savedInDirectory(const QString & _t1)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}
QT_END_MOC_NAMESPACE
