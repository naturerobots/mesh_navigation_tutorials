/****************************************************************************
** Meta object code from reading C++ file 'ClusterLabelTool.hpp'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.15.13)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include <memory>
#include "../../../../mesh_tools/rviz_mesh_tools_plugins/include/rviz_mesh_tools_plugins/ClusterLabelTool.hpp"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'ClusterLabelTool.hpp' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.15.13. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_rviz_mesh_tools_plugins__ClusterLabelTool_t {
    QByteArrayData data[9];
    char stringdata0[135];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_rviz_mesh_tools_plugins__ClusterLabelTool_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_rviz_mesh_tools_plugins__ClusterLabelTool_t qt_meta_stringdata_rviz_mesh_tools_plugins__ClusterLabelTool = {
    {
QT_MOC_LITERAL(0, 0, 41), // "rviz_mesh_tools_plugins::Clus..."
QT_MOC_LITERAL(1, 42, 12), // "publishLabel"
QT_MOC_LITERAL(2, 55, 0), // ""
QT_MOC_LITERAL(3, 56, 11), // "std::string"
QT_MOC_LITERAL(4, 68, 4), // "name"
QT_MOC_LITERAL(5, 73, 16), // "getSelectedFaces"
QT_MOC_LITERAL(6, 90, 21), // "std::vector<uint32_t>"
QT_MOC_LITERAL(7, 112, 10), // "resetFaces"
QT_MOC_LITERAL(8, 123, 11) // "resetVisual"

    },
    "rviz_mesh_tools_plugins::ClusterLabelTool\0"
    "publishLabel\0\0std::string\0name\0"
    "getSelectedFaces\0std::vector<uint32_t>\0"
    "resetFaces\0resetVisual"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_rviz_mesh_tools_plugins__ClusterLabelTool[] = {

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
       7,    0,   38,    2, 0x0a /* Public */,
       8,    0,   39,    2, 0x0a /* Public */,

 // slots: parameters
    QMetaType::Void, 0x80000000 | 3,    4,
    0x80000000 | 6,
    QMetaType::Void,
    QMetaType::Void,

       0        // eod
};

void rviz_mesh_tools_plugins::ClusterLabelTool::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<ClusterLabelTool *>(_o);
        (void)_t;
        switch (_id) {
        case 0: _t->publishLabel((*reinterpret_cast< std::string(*)>(_a[1]))); break;
        case 1: { std::vector<uint32_t> _r = _t->getSelectedFaces();
            if (_a[0]) *reinterpret_cast< std::vector<uint32_t>*>(_a[0]) = std::move(_r); }  break;
        case 2: _t->resetFaces(); break;
        case 3: _t->resetVisual(); break;
        default: ;
        }
    }
}

QT_INIT_METAOBJECT const QMetaObject rviz_mesh_tools_plugins::ClusterLabelTool::staticMetaObject = { {
    QMetaObject::SuperData::link<rviz_common::Tool::staticMetaObject>(),
    qt_meta_stringdata_rviz_mesh_tools_plugins__ClusterLabelTool.data,
    qt_meta_data_rviz_mesh_tools_plugins__ClusterLabelTool,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *rviz_mesh_tools_plugins::ClusterLabelTool::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *rviz_mesh_tools_plugins::ClusterLabelTool::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_rviz_mesh_tools_plugins__ClusterLabelTool.stringdata0))
        return static_cast<void*>(this);
    return rviz_common::Tool::qt_metacast(_clname);
}

int rviz_mesh_tools_plugins::ClusterLabelTool::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = rviz_common::Tool::qt_metacall(_c, _id, _a);
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
