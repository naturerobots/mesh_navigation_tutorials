/****************************************************************************
** Meta object code from reading C++ file 'MapDisplay.hpp'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.15.13)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include <memory>
#include "../../../../mesh_tools/rviz_mesh_tools_plugins/include/rviz_mesh_tools_plugins/MapDisplay.hpp"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'MapDisplay.hpp' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.15.13. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_rviz_mesh_tools_plugins__MapDisplay_t {
    QByteArrayData data[9];
    char stringdata0[121];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_rviz_mesh_tools_plugins__MapDisplay_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_rviz_mesh_tools_plugins__MapDisplay_t qt_meta_stringdata_rviz_mesh_tools_plugins__MapDisplay = {
    {
QT_MOC_LITERAL(0, 0, 35), // "rviz_mesh_tools_plugins::MapD..."
QT_MOC_LITERAL(1, 36, 9), // "saveLabel"
QT_MOC_LITERAL(2, 46, 0), // ""
QT_MOC_LITERAL(3, 47, 7), // "Cluster"
QT_MOC_LITERAL(4, 55, 7), // "cluster"
QT_MOC_LITERAL(5, 63, 11), // "getGeometry"
QT_MOC_LITERAL(6, 75, 20), // "shared_ptr<Geometry>"
QT_MOC_LITERAL(7, 96, 9), // "updateMap"
QT_MOC_LITERAL(8, 106, 14) // "updateMapFrame"

    },
    "rviz_mesh_tools_plugins::MapDisplay\0"
    "saveLabel\0\0Cluster\0cluster\0getGeometry\0"
    "shared_ptr<Geometry>\0updateMap\0"
    "updateMapFrame"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_rviz_mesh_tools_plugins__MapDisplay[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
       4,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    1,   34,    2, 0x0a /* Public */,
       5,    0,   37,    2, 0x0a /* Public */,
       7,    0,   38,    2, 0x08 /* Private */,
       8,    0,   39,    2, 0x08 /* Private */,

 // slots: parameters
    QMetaType::Void, 0x80000000 | 3,    4,
    0x80000000 | 6,
    QMetaType::Void,
    QMetaType::Void,

       0        // eod
};

void rviz_mesh_tools_plugins::MapDisplay::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<MapDisplay *>(_o);
        (void)_t;
        switch (_id) {
        case 0: _t->saveLabel((*reinterpret_cast< Cluster(*)>(_a[1]))); break;
        case 1: { shared_ptr<Geometry> _r = _t->getGeometry();
            if (_a[0]) *reinterpret_cast< shared_ptr<Geometry>*>(_a[0]) = std::move(_r); }  break;
        case 2: _t->updateMap(); break;
        case 3: _t->updateMapFrame(); break;
        default: ;
        }
    }
}

QT_INIT_METAOBJECT const QMetaObject rviz_mesh_tools_plugins::MapDisplay::staticMetaObject = { {
    QMetaObject::SuperData::link<rviz_common::Display::staticMetaObject>(),
    qt_meta_stringdata_rviz_mesh_tools_plugins__MapDisplay.data,
    qt_meta_data_rviz_mesh_tools_plugins__MapDisplay,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *rviz_mesh_tools_plugins::MapDisplay::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *rviz_mesh_tools_plugins::MapDisplay::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_rviz_mesh_tools_plugins__MapDisplay.stringdata0))
        return static_cast<void*>(this);
    return rviz_common::Display::qt_metacast(_clname);
}

int rviz_mesh_tools_plugins::MapDisplay::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = rviz_common::Display::qt_metacall(_c, _id, _a);
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
QT_WARNING_POP
QT_END_MOC_NAMESPACE
