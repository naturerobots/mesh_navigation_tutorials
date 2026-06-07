/****************************************************************************
** Meta object code from reading C++ file 'ClusterLabelDisplay.hpp'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.15.13)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include <memory>
#include "../../../../mesh_tools/rviz_mesh_tools_plugins/include/rviz_mesh_tools_plugins/ClusterLabelDisplay.hpp"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'ClusterLabelDisplay.hpp' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.15.13. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_rviz_mesh_tools_plugins__ClusterLabelDisplay_t {
    QByteArrayData data[20];
    char stringdata0[279];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_rviz_mesh_tools_plugins__ClusterLabelDisplay_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_rviz_mesh_tools_plugins__ClusterLabelDisplay_t qt_meta_stringdata_rviz_mesh_tools_plugins__ClusterLabelDisplay = {
    {
QT_MOC_LITERAL(0, 0, 44), // "rviz_mesh_tools_plugins::Clus..."
QT_MOC_LITERAL(1, 45, 14), // "signalAddLabel"
QT_MOC_LITERAL(2, 60, 0), // ""
QT_MOC_LITERAL(3, 61, 7), // "Cluster"
QT_MOC_LITERAL(4, 69, 7), // "cluster"
QT_MOC_LITERAL(5, 77, 15), // "notifyLabelTool"
QT_MOC_LITERAL(6, 93, 11), // "getGeometry"
QT_MOC_LITERAL(7, 105, 20), // "shared_ptr<Geometry>"
QT_MOC_LITERAL(8, 126, 7), // "setData"
QT_MOC_LITERAL(9, 134, 8), // "geometry"
QT_MOC_LITERAL(10, 143, 15), // "vector<Cluster>"
QT_MOC_LITERAL(11, 159, 8), // "clusters"
QT_MOC_LITERAL(12, 168, 14), // "setCullingMode"
QT_MOC_LITERAL(13, 183, 17), // "Ogre::CullingMode"
QT_MOC_LITERAL(14, 201, 4), // "mode"
QT_MOC_LITERAL(15, 206, 9), // "updateMap"
QT_MOC_LITERAL(16, 216, 12), // "updateColors"
QT_MOC_LITERAL(17, 229, 16), // "updateSphereSize"
QT_MOC_LITERAL(18, 246, 19), // "updatePhantomVisual"
QT_MOC_LITERAL(19, 266, 12) // "changeVisual"

    },
    "rviz_mesh_tools_plugins::ClusterLabelDisplay\0"
    "signalAddLabel\0\0Cluster\0cluster\0"
    "notifyLabelTool\0getGeometry\0"
    "shared_ptr<Geometry>\0setData\0geometry\0"
    "vector<Cluster>\0clusters\0setCullingMode\0"
    "Ogre::CullingMode\0mode\0updateMap\0"
    "updateColors\0updateSphereSize\0"
    "updatePhantomVisual\0changeVisual"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_rviz_mesh_tools_plugins__ClusterLabelDisplay[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
      10,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       1,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    1,   64,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
       5,    0,   67,    2, 0x0a /* Public */,
       6,    0,   68,    2, 0x0a /* Public */,
       8,    2,   69,    2, 0x0a /* Public */,
      12,    1,   74,    2, 0x0a /* Public */,
      15,    0,   77,    2, 0x08 /* Private */,
      16,    0,   78,    2, 0x08 /* Private */,
      17,    0,   79,    2, 0x08 /* Private */,
      18,    0,   80,    2, 0x08 /* Private */,
      19,    0,   81,    2, 0x08 /* Private */,

 // signals: parameters
    QMetaType::Void, 0x80000000 | 3,    4,

 // slots: parameters
    QMetaType::Void,
    0x80000000 | 7,
    QMetaType::Void, 0x80000000 | 7, 0x80000000 | 10,    9,   11,
    QMetaType::Void, 0x80000000 | 13,   14,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,

       0        // eod
};

void rviz_mesh_tools_plugins::ClusterLabelDisplay::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<ClusterLabelDisplay *>(_o);
        (void)_t;
        switch (_id) {
        case 0: _t->signalAddLabel((*reinterpret_cast< Cluster(*)>(_a[1]))); break;
        case 1: _t->notifyLabelTool(); break;
        case 2: { shared_ptr<Geometry> _r = _t->getGeometry();
            if (_a[0]) *reinterpret_cast< shared_ptr<Geometry>*>(_a[0]) = std::move(_r); }  break;
        case 3: _t->setData((*reinterpret_cast< shared_ptr<Geometry>(*)>(_a[1])),(*reinterpret_cast< vector<Cluster>(*)>(_a[2]))); break;
        case 4: _t->setCullingMode((*reinterpret_cast< Ogre::CullingMode(*)>(_a[1]))); break;
        case 5: _t->updateMap(); break;
        case 6: _t->updateColors(); break;
        case 7: _t->updateSphereSize(); break;
        case 8: _t->updatePhantomVisual(); break;
        case 9: _t->changeVisual(); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            using _t = void (ClusterLabelDisplay::*)(Cluster );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&ClusterLabelDisplay::signalAddLabel)) {
                *result = 0;
                return;
            }
        }
    }
}

QT_INIT_METAOBJECT const QMetaObject rviz_mesh_tools_plugins::ClusterLabelDisplay::staticMetaObject = { {
    QMetaObject::SuperData::link<rviz_common::Display::staticMetaObject>(),
    qt_meta_stringdata_rviz_mesh_tools_plugins__ClusterLabelDisplay.data,
    qt_meta_data_rviz_mesh_tools_plugins__ClusterLabelDisplay,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *rviz_mesh_tools_plugins::ClusterLabelDisplay::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *rviz_mesh_tools_plugins::ClusterLabelDisplay::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_rviz_mesh_tools_plugins__ClusterLabelDisplay.stringdata0))
        return static_cast<void*>(this);
    return rviz_common::Display::qt_metacast(_clname);
}

int rviz_mesh_tools_plugins::ClusterLabelDisplay::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = rviz_common::Display::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 10)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 10;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 10)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 10;
    }
    return _id;
}

// SIGNAL 0
void rviz_mesh_tools_plugins::ClusterLabelDisplay::signalAddLabel(Cluster _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
