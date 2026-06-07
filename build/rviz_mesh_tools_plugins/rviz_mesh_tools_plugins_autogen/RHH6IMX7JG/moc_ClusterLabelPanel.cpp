/****************************************************************************
** Meta object code from reading C++ file 'ClusterLabelPanel.hpp'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.15.13)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include <memory>
#include "../../../../mesh_tools/rviz_mesh_tools_plugins/include/rviz_mesh_tools_plugins/ClusterLabelPanel.hpp"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'ClusterLabelPanel.hpp' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.15.13. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_rviz_mesh_tools_plugins__ClusterLabelPanel_t {
    QByteArrayData data[7];
    char stringdata0[108];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_rviz_mesh_tools_plugins__ClusterLabelPanel_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_rviz_mesh_tools_plugins__ClusterLabelPanel_t qt_meta_stringdata_rviz_mesh_tools_plugins__ClusterLabelPanel = {
    {
QT_MOC_LITERAL(0, 0, 42), // "rviz_mesh_tools_plugins::Clus..."
QT_MOC_LITERAL(1, 43, 14), // "setClusterName"
QT_MOC_LITERAL(2, 58, 0), // ""
QT_MOC_LITERAL(3, 59, 11), // "clusterName"
QT_MOC_LITERAL(4, 71, 17), // "updateClusterName"
QT_MOC_LITERAL(5, 89, 7), // "publish"
QT_MOC_LITERAL(6, 97, 10) // "resetFaces"

    },
    "rviz_mesh_tools_plugins::ClusterLabelPanel\0"
    "setClusterName\0\0clusterName\0"
    "updateClusterName\0publish\0resetFaces"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_rviz_mesh_tools_plugins__ClusterLabelPanel[] = {

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
       4,    0,   37,    2, 0x0a /* Public */,
       5,    0,   38,    2, 0x0a /* Public */,
       6,    0,   39,    2, 0x0a /* Public */,

 // slots: parameters
    QMetaType::Void, QMetaType::QString,    3,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,

       0        // eod
};

void rviz_mesh_tools_plugins::ClusterLabelPanel::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<ClusterLabelPanel *>(_o);
        (void)_t;
        switch (_id) {
        case 0: _t->setClusterName((*reinterpret_cast< const QString(*)>(_a[1]))); break;
        case 1: _t->updateClusterName(); break;
        case 2: _t->publish(); break;
        case 3: _t->resetFaces(); break;
        default: ;
        }
    }
}

QT_INIT_METAOBJECT const QMetaObject rviz_mesh_tools_plugins::ClusterLabelPanel::staticMetaObject = { {
    QMetaObject::SuperData::link<rviz_common::Panel::staticMetaObject>(),
    qt_meta_stringdata_rviz_mesh_tools_plugins__ClusterLabelPanel.data,
    qt_meta_data_rviz_mesh_tools_plugins__ClusterLabelPanel,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *rviz_mesh_tools_plugins::ClusterLabelPanel::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *rviz_mesh_tools_plugins::ClusterLabelPanel::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_rviz_mesh_tools_plugins__ClusterLabelPanel.stringdata0))
        return static_cast<void*>(this);
    return rviz_common::Panel::qt_metacast(_clname);
}

int rviz_mesh_tools_plugins::ClusterLabelPanel::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = rviz_common::Panel::qt_metacall(_c, _id, _a);
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
