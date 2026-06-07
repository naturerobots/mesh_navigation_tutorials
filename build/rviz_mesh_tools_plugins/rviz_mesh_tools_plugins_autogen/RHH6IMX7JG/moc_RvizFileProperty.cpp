/****************************************************************************
** Meta object code from reading C++ file 'RvizFileProperty.hpp'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.15.13)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include <memory>
#include "../../../../mesh_tools/rviz_mesh_tools_plugins/include/rviz_mesh_tools_plugins/RvizFileProperty.hpp"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'RvizFileProperty.hpp' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.15.13. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_rviz_common__properties__FileProperty_t {
    QByteArrayData data[7];
    char stringdata0[90];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_rviz_common__properties__FileProperty_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_rviz_common__properties__FileProperty_t qt_meta_stringdata_rviz_common__properties__FileProperty = {
    {
QT_MOC_LITERAL(0, 0, 37), // "rviz_common::properties::File..."
QT_MOC_LITERAL(1, 38, 11), // "setFilename"
QT_MOC_LITERAL(2, 50, 0), // ""
QT_MOC_LITERAL(3, 51, 3), // "str"
QT_MOC_LITERAL(4, 55, 14), // "setStdFilename"
QT_MOC_LITERAL(5, 70, 11), // "std::string"
QT_MOC_LITERAL(6, 82, 7) // "std_str"

    },
    "rviz_common::properties::FileProperty\0"
    "setFilename\0\0str\0setStdFilename\0"
    "std::string\0std_str"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_rviz_common__properties__FileProperty[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
       2,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    1,   24,    2, 0x0a /* Public */,
       4,    1,   27,    2, 0x0a /* Public */,

 // slots: parameters
    QMetaType::Bool, QMetaType::QString,    3,
    QMetaType::Bool, 0x80000000 | 5,    6,

       0        // eod
};

void rviz_common::properties::FileProperty::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<FileProperty *>(_o);
        (void)_t;
        switch (_id) {
        case 0: { bool _r = _t->setFilename((*reinterpret_cast< const QString(*)>(_a[1])));
            if (_a[0]) *reinterpret_cast< bool*>(_a[0]) = std::move(_r); }  break;
        case 1: { bool _r = _t->setStdFilename((*reinterpret_cast< const std::string(*)>(_a[1])));
            if (_a[0]) *reinterpret_cast< bool*>(_a[0]) = std::move(_r); }  break;
        default: ;
        }
    }
}

QT_INIT_METAOBJECT const QMetaObject rviz_common::properties::FileProperty::staticMetaObject = { {
    QMetaObject::SuperData::link<FilePickerProperty::staticMetaObject>(),
    qt_meta_stringdata_rviz_common__properties__FileProperty.data,
    qt_meta_data_rviz_common__properties__FileProperty,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *rviz_common::properties::FileProperty::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *rviz_common::properties::FileProperty::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_rviz_common__properties__FileProperty.stringdata0))
        return static_cast<void*>(this);
    return FilePickerProperty::qt_metacast(_clname);
}

int rviz_common::properties::FileProperty::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = FilePickerProperty::qt_metacall(_c, _id, _a);
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
QT_WARNING_POP
QT_END_MOC_NAMESPACE
