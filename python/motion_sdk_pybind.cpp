#include "dr_timer.h"
#include "receiver.h"
#include "sender.h"
#include "lite3_types.h"
#include <functional>
#include <pybind11/pybind11.h>
#include <pybind11/functional.h>
#include <pybind11/stl.h>
#include <pybind11/stl_bind.h>
#include <pybind11/embed.h>
#include <pybind11/pytypes.h>
#include <pybind11/numpy.h>
#include <array>

namespace py = pybind11;
using namespace lite3;

PYBIND11_MODULE(deeprobotics_lite3_motion_sdk_py, m) {
  py::class_<ImuData>(m, "ImuData")
    .def(py::init<>())
    .def_readwrite("timestamp", &ImuData::timestamp)
    .def_readwrite("roll", &ImuData::roll)
    .def_readwrite("pitch", &ImuData::pitch)
    .def_readwrite("yaw", &ImuData::yaw)
    .def_readwrite("omega_x", &ImuData::omega_x)
    .def_readwrite("omega_y", &ImuData::omega_y)
    .def_readwrite("omega_z", &ImuData::omega_z)
    .def_readwrite("acc_x", &ImuData::acc_x)
    .def_readwrite("acc_y", &ImuData::acc_y)
    .def_readwrite("acc_z", &ImuData::acc_z);

  py::class_<JointData>(m, "JointData")
    .def(py::init<>())
    .def_readwrite("position", &JointData::position)
    .def_readwrite("velocity", &JointData::velocity)
    .def_readwrite("torque", &JointData::torque)
    .def_readwrite("temperature", &JointData::temperature);

  py::class_<JointCmd>(m, "JointCmd")
    .def(py::init<>())
    .def_readwrite("position", &JointCmd::position)
    .def_readwrite("velocity", &JointCmd::velocity)
    .def_readwrite("torque", &JointCmd::torque)
    .def_readwrite("kp", &JointCmd::kp)
    .def_readwrite("kd", &JointCmd::kd);

  py::class_<LegData>(m, "LegData")
    .def(py::init<>())
    .def_readwrite("joint_data", &LegData::joint_data);

  py::class_<RobotCmd>(m, "RobotCmd")
    .def(py::init<>())
    .def_readwrite("joint_cmd", &RobotCmd::joint_cmd);

  py::class_<ContactForce>(m, "ContactForce")
    .def(py::init<>())
    .def_readwrite("leg_force", &ContactForce::leg_force);
  
  py::class_<RobotData>(m, "RobotData")
    .def(py::init<>())
    .def_readwrite("tick", &RobotData::tick)
    .def_readwrite("imu", &RobotData::imu)
    .def_readwrite("joint_data", &RobotData::joint_data)
    .def_readwrite("contact_force", &RobotData::contact_force);
  

  py::class_<DRTimer>(m, "DRTimer")
    .def(py::init<>())
    .def("TimeInit", &DRTimer::TimeInit)
    .def("TimerInterrupt", &DRTimer::TimerInterrupt)
    .def("GetIntervalTime", &DRTimer::GetIntervalTime)
    .def("GetCurrentTime", &DRTimer::GetCurrentTime);

  py::class_<Receiver>(m, "Receiver")
    .def(py::init<int>())
    .def("RegisterCallBack", &Receiver::RegisterCallBack)
    .def("StartWork", &Receiver::StartWork)
    .def("GetState", &Receiver::GetState);

  py::class_<Sender>(m, "Sender")
    .def(py::init<std::string , uint16_t>())
    .def("SendCmd", &Sender::SendCmd)
    .def("ControlGet", &Sender::ControlGet)
    .def("AllJointBackZero", &Sender::AllJointBackZero)
    .def("RobotStateInit", &Sender::RobotStateInit);
}