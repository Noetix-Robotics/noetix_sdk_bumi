#include "aolion_driver.h"
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "lowcontroller.h"

namespace py = pybind11;
using namespace legged;

PYBIND11_MODULE(lowcontrol_py, m) {
        py::class_<MotorCmd>(m, "MotorCmd")
            .def(py::init<>())

            .def_readwrite("pos", &MotorCmd::pos)
            .def_readwrite("vel", &MotorCmd::vel)
            .def_readwrite("tau", &MotorCmd::tau)
            .def_readwrite("kp", &MotorCmd::kp)
            .def_readwrite("kd", &MotorCmd::kd)
            .def_readwrite("motor_id", &MotorCmd::motor_id);

        py::class_<joydata>(m, "JoyData")
            .def(py::init<>())
            .def_property_readonly("axes",
                                   [](const joydata &d) {
                                           return py::array_t<double>(
                                               {2}, {sizeof(double)}, d.axes);
                                   })
            .def_property_readonly("button", [](const joydata &d) {
                    return py::array_t<int>({14}, {sizeof(int)}, d.button);
            });

        py::class_<NingImuData>(m, "NingImuData")
            .def(py::init<>())
            .def_property_readonly("ori",
                                   [](const NingImuData &d) {
                                           return py::array_t<double>(
                                               {4}, {sizeof(double)}, d.ori);
                                   })
            .def_property_readonly("ori_cov",
                                   [](const NingImuData &d) {
                                           return py::array_t<double>(
                                               {9}, {sizeof(double)},
                                               d.ori_cov);
                                   })
            .def_property_readonly("angular_vel",
                                   [](const NingImuData &d) {
                                           return py::array_t<double>(
                                               {3}, {sizeof(double)},
                                               d.angular_vel);
                                   })
            .def_property_readonly("angular_vel_cov",
                                   [](const NingImuData &d) {
                                           return py::array_t<double>(
                                               {9}, {sizeof(double)},
                                               d.angular_vel_cov);
                                   })
            .def_property_readonly("linear_acc",
                                   [](const NingImuData &d) {
                                           return py::array_t<double>(
                                               {3}, {sizeof(double)},
                                               d.linear_acc);
                                   })
            .def_property_readonly("linear_acc_cov", [](const NingImuData &d) {
                    return py::array_t<double>({9}, {sizeof(double)},
                                               d.linear_acc_cov);
            });

        py::class_<MotorState>(m, "MotorState")
            .def(py::init<>())
            .def_readwrite("pos", &MotorState::pos)
            .def_readwrite("vel", &MotorState::vel)
            .def_readwrite("tau", &MotorState::tau)
            .def_readwrite("motor_id", &MotorState::motor_id)
            .def_readwrite("error", &MotorState::error)
            .def_readwrite("temperature", &MotorState::temperature);

        py::class_<LowController>(m, "LowController")
            .def_static("instance", &LowController::Instance,
                        py::return_value_policy::reference)

            // 初始化
            .def("init", &LowController::init)

            // 设置电机命令
            .def(
                "set_joint",
                [](LowController &self, const std::vector<MotorCmd> &cmds) {
                        if (cmds.size() != 21) {
                                throw std::runtime_error(
                                    "motorcmd size must be 21");
                        }

                        std::array<MotorCmd, 21> arr;
                        std::copy(cmds.begin(), cmds.end(), arr.begin());

                        self.set_joint(arr);
                },
                py::arg("motorcmd"))

            // 获取关节状态
            .def("get_joint_state", &LowController::get_joint_state)

            // 获取 IMU 数据
            .def("get_imu_data", &LowController::get_imu_data)

            // 获取摇杆数据
            .def("from_dds_get_joydata", &LowController::from_dds_get_joydata)

            // 根据关节名字获取索引
            .def("getJointsIndex", &LowController::getJointsIndex,
                 py::arg("jointname"));

        py::class_<AoLionDriver>(m, "AoLionDriver")
            .def(py::init<>())

            // init
            .def(
                "init",
                [](AoLionDriver &self, const std::string &port, int baudrate) {
                        return self.init(port, baudrate);
                },
                py::arg("port"), py::arg("baudrate"))

            // getremotedata
            .def("getremotedata", [](AoLionDriver &self) {
                    joydata d = self.getremotedata();
                    return d; // 直接返回，依赖你已经绑定 JoyData
            });
}
