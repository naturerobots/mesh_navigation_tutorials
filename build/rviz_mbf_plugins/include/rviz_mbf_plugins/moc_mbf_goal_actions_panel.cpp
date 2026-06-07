/****************************************************************************
** Meta object code from reading C++ file 'mbf_goal_actions_panel.hpp'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.15.13)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include <memory>
#include "../../../../move_base_flex/rviz_mbf_plugins/include/rviz_mbf_plugins/mbf_goal_actions_panel.hpp"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'mbf_goal_actions_panel.hpp' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.15.13. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_rviz_mbf_plugins__MbfGoalActionsPanel_t {
    QByteArrayData data[7];
    char stringdata0[155];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_rviz_mbf_plugins__MbfGoalActionsPanel_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_rviz_mbf_plugins__MbfGoalActionsPanel_t qt_meta_stringdata_rviz_mbf_plugins__MbfGoalActionsPanel = {
    {
QT_MOC_LITERAL(0, 0, 37), // "rviz_mbf_plugins::MbfGoalActi..."
QT_MOC_LITERAL(1, 38, 27), // "updateGoalInputSubscription"
QT_MOC_LITERAL(2, 66, 0), // ""
QT_MOC_LITERAL(3, 67, 25), // "updateGetPathActionClient"
QT_MOC_LITERAL(4, 93, 25), // "updateExePathActionClient"
QT_MOC_LITERAL(5, 119, 17), // "stopGetPathAction"
QT_MOC_LITERAL(6, 137, 17) // "stopExePathAction"

    },
    "rviz_mbf_plugins::MbfGoalActionsPanel\0"
    "updateGoalInputSubscription\0\0"
    "updateGetPathActionClient\0"
    "updateExePathActionClient\0stopGetPathAction\0"
    "stopExePathAction"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_rviz_mbf_plugins__MbfGoalActionsPanel[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
       5,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    0,   39,    2, 0x08 /* Private */,
       3,    0,   40,    2, 0x08 /* Private */,
       4,    0,   41,    2, 0x08 /* Private */,
       5,    0,   42,    2, 0x08 /* Private */,
       6,    0,   43,    2, 0x08 /* Private */,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,

       0        // eod
};

void rviz_mbf_plugins::MbfGoalActionsPanel::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<MbfGoalActionsPanel *>(_o);
        (void)_t;
        switch (_id) {
        case 0: _t->updateGoalInputSubscription(); break;
        case 1: _t->updateGetPathActionClient(); break;
        case 2: _t->updateExePathActionClient(); break;
        case 3: _t->stopGetPathAction(); break;
        case 4: _t->stopExePathAction(); break;
        default: ;
        }
    }
    (void)_a;
}

QT_INIT_METAOBJECT const QMetaObject rviz_mbf_plugins::MbfGoalActionsPanel::staticMetaObject = { {
    QMetaObject::SuperData::link<rviz_common::Panel::staticMetaObject>(),
    qt_meta_stringdata_rviz_mbf_plugins__MbfGoalActionsPanel.data,
    qt_meta_data_rviz_mbf_plugins__MbfGoalActionsPanel,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *rviz_mbf_plugins::MbfGoalActionsPanel::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *rviz_mbf_plugins::MbfGoalActionsPanel::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_rviz_mbf_plugins__MbfGoalActionsPanel.stringdata0))
        return static_cast<void*>(this);
    return rviz_common::Panel::qt_metacast(_clname);
}

int rviz_mbf_plugins::MbfGoalActionsPanel::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = rviz_common::Panel::qt_metacall(_c, _id, _a);
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
QT_WARNING_POP
QT_END_MOC_NAMESPACE
