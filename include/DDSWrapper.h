#ifndef DDSWrapper_H
#define DDSWrapper_H

#include <features.h>
#include <dds/dds.hpp>
#include <functional>
#include <thread>
#include <mutex>
#include <atomic>

#include "Robot_motorcmd.hpp"
#include "Robot_status.hpp"
#include "Robot_setmode.hpp"
#include "Robot_handTransfer.hpp"
#include "Robot_controlcmd.hpp"
namespace legged
{
  class DDSWrapper
  {
    public:
        using RobotStatusCallback = std::function<void(const RobotStatus::StatusData&)>;
        using HandReplyCallback = std::function<void(const PackageTransfer::TransferReply&)>;
        DDSWrapper(int domain_id = 0);
        ~DDSWrapper();
        void publishMotorCmdData(const RobotMotorCmd::MotorCmdArray&  state);
        void publishControlCmdData(const RobotControlCmd::ControlCmd&  state); 
        void publishModeData(const RobotSetMode::SetMode& data);
        void publishHandRequest(const PackageTransfer::TransferRequest&  request );
        void subscribeRobotStatus(RobotStatusCallback callback);
        void subscribeHandReply(HandReplyCallback callback);
    private:
        dds::domain::DomainParticipant participant_;
        dds::pub::Publisher publisher_;
        dds::sub::Subscriber subscriber_;
        

        // motor state
        dds::topic::Topic<RobotStatus::StatusData> robotstatus_topic_;
        dds::sub::DataReader<RobotStatus::StatusData> robotstatus_reader_;
        RobotStatusCallback robotstatus_callback_;
        std::thread robot_status_listener_thread_;
        std::atomic<bool> robot_status_listener_running_;
        std::mutex RobotStatus_callback_mutex_;

       
         // hand state
         dds::topic::Topic<PackageTransfer::TransferRequest> handrequest_topic_;
         dds::pub::DataWriter<PackageTransfer::TransferRequest> handrequest_writer_;
 
         //TransferRequest
         dds::topic::Topic<PackageTransfer::TransferReply> handreply_topic_;
         dds::sub::DataReader<PackageTransfer::TransferReply> handreply_reader_;
         HandReplyCallback handreply_callback_;
         std::thread handreply_listener_thread_;
         std::atomic<bool> handreply_listener_running_;
         std::mutex handreply_callback_mutex_;
        //motorcmd
        dds::topic::Topic<RobotMotorCmd::MotorCmdArray> motorcmd_topic_;
        dds::pub::DataWriter<RobotMotorCmd::MotorCmdArray> motorcmd_writer_;

        //controlcmd
        dds::topic::Topic<RobotControlCmd::ControlCmd> controlcmd_topic_;
        dds::pub::DataWriter<RobotControlCmd::ControlCmd> controlcmd_writer_;

        //setmode
        dds::topic::Topic<RobotSetMode::SetMode> mode_topic_;
        dds::pub::DataWriter<RobotSetMode::SetMode> mode_writer_;


        //qos配置
        dds::sub::qos::DataReaderQos getRobotStatusQos();
        dds::pub::qos::DataWriterQos getHandRequestQos();
        dds::sub::qos::DataReaderQos getHandReplyQos();
        dds::pub::qos::DataWriterQos getMotorCmdQos();
        dds::pub::qos::DataWriterQos getControlCmdQos();
        dds::pub::qos::DataWriterQos getModeQos();
        void init();
        void robotstatusListener();
        void handreplyListener();
        void wait_for_reader(dds::pub::DataWriter<RobotSetMode::SetMode>& writer);
     
  };




}
#endif