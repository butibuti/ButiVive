#include"stdafx.h"
#include <openvr.h>
#include<map>
#include<string>
#include<iostream>
namespace ButiVive {
using namespace ButiEngine;
class HtcViveTracker :public ITracker
{
public:

    HtcViveTracker();
    ~HtcViveTracker();

    bool Initialize(const bool arg_verbose)override;
    bool ShutDown()override;

    void Update()override;

    bool IsDeviceDetected(const std::string& arg_deviceName)override;
    void PrintAllDetectedDevices()override;
    const List<std::string>& GetAllDeviceNames()const override;
    float GetBatteryLevel(const std::string& arg_deviceName)override;
    bool EventPolling()override;
    bool GetDevicePoseQuaternion(const std::string& arg_deviceName, Vector3& arg_ref_pose, Quat& arg_ref_angle)override;
    bool GetDevicePoseMatrix(const std::string& arg_deviceName, ButiEngine::Matrix4x4& arg_ref_matrix)override;
    Velocity GetDeviceVelocity(const std::string& arg_deviceName)override;

    Dimension GetChaperoneDimensions() const override;

    std::string GetLastButtonPressedString(const std::string& arg_deviceName) override;
    VRDeviceButtonId GetLastButtonPressedEnum(const std::string& arg_deviceName)override;
    bool HapticPulse(const std::string& arg_deviceName,const std::uint32_t arg_axisIndex,const std::int16_t arg_durationMicrosec)override;
    void SetOrigin(const Matrix4x4& arg_matrix) override { m_offsetMatrix = arg_matrix.GetInverse(); }
    void SetOrigin(const std::int32_t arg_deviceIndex) override {

        vr::TrackedDevicePose_t current_device_pose = m_devicePoses[arg_deviceIndex];
        if (current_device_pose.bDeviceIsConnected && current_device_pose.bPoseIsValid) {
            auto deviceMatrix = VRMatrixToMatrix(current_device_pose.mDeviceToAbsoluteTracking);
            m_offsetMatrix *= deviceMatrix.Inverse();
        }
    }
    const Matrix4x4& GetOrigin()const override { return m_offsetMatrix; }
private:
    vr::IVRSystem* m_p_vrSystem;

    vr::IVRChaperone* m_vrChaperone;
    vr::TrackedDevicePose_t m_devicePoses[vr::k_unMaxTrackedDeviceCount];
    std::map<std::string, std::uint32_t> m_devicesIndex;
    List<std::string>m_list_devicesNames, m_activeDeviceNames;
    List<vr::EVRButtonId>m_list_lastButtonPressed;
    std::uint32_t m_maxDevices;
    std::int32_t m_hmdCounts = 1, m_controllerCounts = 1, m_trackerCounts = 1, m_trackReferenceCounts = 1, m_nullCounts = 1;

    void InitializeDeviceMap();
    Matrix4x4 m_offsetMatrix;

    EventFlags m_events;
    bool m_isVerbose;
    const std::uint32_t MAX_PULSE_DURATION = 3999;

    static constexpr const char* NAME_HMD = "hmd";
    static constexpr const char* NAME_CONTROLLER = "controller";
    static constexpr const char* NAME_TRACKER = "tracker";
    static constexpr const char* NAME_TREFERENCE = "tracking_reference";
    static constexpr const char* NAME_NULL = "invalid";

    Matrix4x4 VRMatrixToMatrix(const vr::HmdMatrix34_t& arg_Matrix);
    bool UpdateDevicePosition(const std::int32_t arg_deviceIndex);


    std::string GetDeviceClass(const std::int32_t arg_deviceIndex);
    std::string SetDeviceName(const std::int32_t arg_deviceIndex);
    bool AddNewDevice(const std::int32_t arg_deviceIndex);
    bool DeleteDevice(const std::int32_t arg_deviceIndex);


};
HtcViveTracker::HtcViveTracker()
{
}
HtcViveTracker::~HtcViveTracker()
{
    ShutDown();
}
bool HtcViveTracker::Initialize(const bool arg_verbose)
{
    m_isVerbose = arg_verbose;
    bool runtime_ok = vr::VR_IsRuntimeInstalled();
    bool hmd_present = vr::VR_IsHmdPresent();
    vr::EVRInitError er;
    m_p_vrSystem = vr::VR_Init(&er, vr::VRApplication_Background);
    std::string init_error = vr::VR_GetVRInitErrorAsSymbol(er);

    if (m_isVerbose) {
        std::cout << "VR is runtime installed : " << runtime_ok << std::endl;
        std::cout << "VR is HMD present : " << hmd_present << std::endl;
        std::cout << "VR init error : " << er << init_error << std::endl;
    }

    if (runtime_ok && hmd_present && er == vr::VRInitError_None) {
        m_maxDevices = vr::k_unMaxTrackedDeviceCount;
        for (std::int32_t index = 0; index<m_maxDevices; index++) {
            m_list_devicesNames.push_back("");
            m_list_lastButtonPressed.Add(vr::k_EButton_Max);
        }
        m_vrChaperone = (vr::IVRChaperone*)vr::VR_GetGenericInterface(vr::IVRChaperone_Version, &er);
        if (er == 0) {
            if (m_isVerbose) std::cout << "Chaperone initialized correctly" << std::endl;
        }
        else {
            if (m_isVerbose) std::cout << "Problem initializing chaperone" << std::endl;
            return false;
        }
        Update();
        InitializeDeviceMap();
        return true;
    }
    else {
        if (m_isVerbose) {
            std::cout << "Problem initializing VR" << std::endl;
        }
        return false;
    }
}
bool HtcViveTracker::ShutDown()
{
    if (m_p_vrSystem) {
        vr::VR_Shutdown();
        if (m_isVerbose) {
            std::cout << "Device is shut down" << std::endl;
        }
        return true;
    }
    else {
        return false;
    }
}
void HtcViveTracker::Update()
{
    if (!m_p_vrSystem) { return; }
    m_p_vrSystem->GetDeviceToAbsoluteTrackingPose(vr::TrackingUniverseStanding, 0, m_devicePoses, m_maxDevices);
    for (vr::TrackedDeviceIndex_t device_index = vr::k_unTrackedDeviceIndex_Hmd; device_index < m_maxDevices; ++device_index) {
        if (m_devicePoses[device_index].bDeviceIsConnected && m_devicePoses[device_index].bPoseIsValid) {
            if (m_isVerbose) {
                std::string info = ("device[" + std::to_string(device_index) + "]: " + GetDeviceClass(device_index) + " " + std::to_string(m_devicePoses[device_index].eTrackingResult));
                std::cout << info << std::endl;
            }
        }
    }
}
bool HtcViveTracker::IsDeviceDetected(const std::string& arg_deviceName)
{
    if (m_devicesIndex.find(arg_deviceName) == m_devicesIndex.end()) {
        return false;
    }
    uint32_t device_index = m_devicesIndex[arg_deviceName];
    if (device_index < m_maxDevices) {
        return m_p_vrSystem->IsTrackedDeviceConnected(device_index);
    }
    else return false;
}
void HtcViveTracker::PrintAllDetectedDevices()
{
    for (vr::TrackedDeviceIndex_t device_index = vr::k_unTrackedDeviceIndex_Hmd; device_index < m_maxDevices; ++device_index) {
        if (m_devicePoses[device_index].bDeviceIsConnected && m_devicePoses[device_index].bPoseIsValid) {
            std::string arg_deviceName = m_list_devicesNames[device_index];
            std::string info = ("device[" + std::to_string(device_index) + "]: " + arg_deviceName + " is connected");
            std::cout << info << std::endl;
        }
    }
}
const List<std::string>& HtcViveTracker::GetAllDeviceNames()const
{
    return m_activeDeviceNames;
}
float HtcViveTracker::GetBatteryLevel(const std::string& arg_deviceName)
{
    if (m_devicesIndex.find(arg_deviceName) == m_devicesIndex.end()) {
        return 0;
    }
    float level = 1.0;
    std::uint32_t device_id = m_devicesIndex[arg_deviceName];
    vr::ETrackedDeviceProperty device_property = vr::Prop_DeviceBatteryPercentage_Float;
    vr::ETrackedPropertyError error;
    std::string device_class = GetDeviceClass(device_id);
    //Only return battery if device is not connected (controller or tracker)
    if (device_class == NAME_CONTROLLER || device_class == NAME_TRACKER) {
        if (device_id < m_maxDevices) {
            level = m_p_vrSystem->GetFloatTrackedDeviceProperty(device_id, device_property, &error);
        }
    }
    return level;
}
bool HtcViveTracker::EventPolling()
{
    if (!m_p_vrSystem) { return false; }
    vr::VREvent_t event;
    if (m_p_vrSystem->PollNextEvent(&event, sizeof(event))) {
        std::cout << m_p_vrSystem->GetEventTypeNameFromEnum((vr::EVREventType)event.eventType) << std::endl;
        switch (event.eventType) {
        case vr::VREvent_TrackedDeviceActivated:
            AddNewDevice(event.trackedDeviceIndex);
            break;
        case vr::VREvent_TrackedDeviceDeactivated:
            DeleteDevice(event.trackedDeviceIndex);
            break;
        case vr::VREvent_ButtonPress:
            m_events = EventFlags::BUTTONPRESS;
            m_list_lastButtonPressed[event.trackedDeviceIndex] = (vr::EVRButtonId)event.data.controller.button;
            break;
        case vr::VREvent_ButtonUnpress:
            m_events = EventFlags::BUTTONPRESS;
            break;
        case vr::VREvent_ButtonTouch:
            break;
        case vr::VREvent_TrackedDeviceRoleChanged:
            break;

        default:
            break;
        }
        return true;
    }
    return false;
}
bool HtcViveTracker::GetDevicePoseQuaternion(const std::string& arg_deviceName, Vector3& arg_ref_pose, Quat& arg_ref_angle)
{
    Matrix4x4 mat;
    if (GetDevicePoseMatrix(arg_deviceName, mat)) {
        arg_ref_pose = mat.GetPosition();
        arg_ref_angle = mat.RemovePosition().ToQuat();
        return true;
    }

    return false;
}
bool HtcViveTracker::GetDevicePoseMatrix(const std::string& arg_deviceName, ButiEngine::Matrix4x4& arg_ref_matrix) {

    if (m_devicesIndex.find(arg_deviceName) == m_devicesIndex.end()) {
        return false;
    }
    std::int32_t device_index = m_devicesIndex[arg_deviceName];
    if (UpdateDevicePosition(device_index)) {
        vr::TrackedDevicePose_t current_device_pose = m_devicePoses[device_index];
        if (current_device_pose.bDeviceIsConnected && current_device_pose.bPoseIsValid) {
            arg_ref_matrix =VRMatrixToMatrix( current_device_pose.mDeviceToAbsoluteTracking);
            arg_ref_matrix *= m_offsetMatrix;
            return true;
        }
    }
    return false;
}
Velocity HtcViveTracker::GetDeviceVelocity(const std::string& arg_deviceName)
{
    Velocity velocity;
    m_p_vrSystem->GetDeviceToAbsoluteTrackingPose(vr::TrackingUniverseStanding, 0, m_devicePoses, m_maxDevices);
    uint32_t device_index = m_devicesIndex[arg_deviceName];
    if (device_index < m_maxDevices) {
        if (m_devicePoses[device_index].bDeviceIsConnected && m_devicePoses[device_index].eTrackingResult == vr::TrackingResult_Running_OK) {
            for (std::int32_t i = 0; i < 3; ++i) {
                velocity.linear_velocity.x = m_devicePoses[device_index].vVelocity.v[0];
                velocity.linear_velocity.y = m_devicePoses[device_index].vVelocity.v[1];
                velocity.linear_velocity.z = m_devicePoses[device_index].vVelocity.v[2];
                velocity.angular_velocity.x = m_devicePoses[device_index].vAngularVelocity.v[0];
                velocity.angular_velocity.y = m_devicePoses[device_index].vAngularVelocity.v[1];
                velocity.angular_velocity.z = m_devicePoses[device_index].vAngularVelocity.v[2];
            }
        }
    }
    return velocity;
}
Dimension HtcViveTracker::GetChaperoneDimensions() const
{
    vr::HmdQuad_t rect;
    Dimension chaperone_dimension;
    std::vector<float>pose(3);
    if (m_vrChaperone->GetPlayAreaSize(&chaperone_dimension.size_x, &chaperone_dimension.size_z)) {
        if (m_vrChaperone->GetPlayAreaRect(&rect)) {
            chaperone_dimension.corner1.x = rect.vCorners[0].v[0];
            chaperone_dimension.corner1.y = rect.vCorners[0].v[1];
            chaperone_dimension.corner1.z = rect.vCorners[0].v[2];

            chaperone_dimension.corner2.x = rect.vCorners[1].v[0];
            chaperone_dimension.corner2.y = rect.vCorners[1].v[1];
            chaperone_dimension.corner2.z = rect.vCorners[1].v[2];

            chaperone_dimension.corner3.x = rect.vCorners[2].v[0];
            chaperone_dimension.corner3.y = rect.vCorners[2].v[1];
            chaperone_dimension.corner3.z = rect.vCorners[2].v[2];

            chaperone_dimension.corner4.x = rect.vCorners[3].v[0];
            chaperone_dimension.corner4.y = rect.vCorners[3].v[1];
            chaperone_dimension.corner4.z = rect.vCorners[3].v[2];

        }
    }
    return chaperone_dimension;
}
std::string HtcViveTracker::GetLastButtonPressedString(const std::string& arg_deviceName)
{
    return m_p_vrSystem->GetButtonIdNameFromEnum(static_cast<vr::EVRButtonId>( GetLastButtonPressedEnum(arg_deviceName)));
}
VRDeviceButtonId HtcViveTracker::GetLastButtonPressedEnum(const std::string& arg_deviceName)
{
    if (m_devicesIndex.find(arg_deviceName) == m_devicesIndex.end()) {
        return VRDeviceButtonId::Max;
    }
    std::uint32_t device_index = m_devicesIndex[arg_deviceName];
    return static_cast<VRDeviceButtonId>( m_list_lastButtonPressed[device_index]);
}
bool HtcViveTracker::HapticPulse(const std::string& arg_deviceName, const std::uint32_t arg_axisIndex, std::int16_t arg_durationMicrosec)
{
    if (m_devicesIndex.find(arg_deviceName) == m_devicesIndex.end()) {
        return false;
    }
    if (arg_durationMicrosec > MAX_PULSE_DURATION) arg_durationMicrosec = MAX_PULSE_DURATION;
    uint32_t device_index = m_devicesIndex[arg_deviceName];
    m_p_vrSystem->TriggerHapticPulse(device_index, arg_axisIndex, arg_durationMicrosec);
    return true;
}
void HtcViveTracker::InitializeDeviceMap()
{
    std::int32_t num_detected_devices = m_maxDevices;
    if (m_isVerbose) {
        std::cout << "Detected devices:" << std::endl;
    }
    std::string arg_deviceName;
    for (std::int32_t i = 0; i < num_detected_devices; ++i) {
        if (m_devicePoses[i].bDeviceIsConnected && m_devicePoses[i].bPoseIsValid) {
            arg_deviceName = SetDeviceName(i);
            if (m_isVerbose) std::cout << i << " :  " << arg_deviceName << std::endl;
            m_devicesIndex[arg_deviceName] = i;
            m_list_devicesNames[i] = arg_deviceName;
        }
    }

    m_activeDeviceNames.Clear();
    for (std::int32_t index=0; index < m_list_devicesNames.GetSize(); index++) {
        if (m_list_devicesNames[index].size()) {
            m_activeDeviceNames.Add(m_list_devicesNames[index]);
        }
    }
}
Matrix4x4 HtcViveTracker::VRMatrixToMatrix(const vr::HmdMatrix34_t& arg_Matrix)
{
    Matrix4x4 output;
    memcpy_s(output.m, sizeof(Matrix4x4) , arg_Matrix.m, sizeof(vr::HmdMatrix34_t));

    output.Transpose();
    output._43 *= -1;
    return output;
}

bool HtcViveTracker::UpdateDevicePosition(const std::int32_t arg_deviceIndex)
{
    vr::ETrackedDeviceClass device_class = m_p_vrSystem->GetTrackedDeviceClass(arg_deviceIndex);
    vr::TrackedDevicePose_t tracked_device_pose;
    vr::VRControllerState_t controller_state;
    m_p_vrSystem->GetDeviceToAbsoluteTrackingPose(vr::TrackingUniverseStanding, 0, m_devicePoses, m_maxDevices);
    if (device_class == vr::ETrackedDeviceClass::TrackedDeviceClass_Controller or device_class == vr::ETrackedDeviceClass::TrackedDeviceClass_GenericTracker) {
        m_p_vrSystem->GetControllerStateWithPose(vr::TrackingUniverseStanding, arg_deviceIndex, &controller_state, sizeof(controller_state), &tracked_device_pose);
        m_devicePoses[arg_deviceIndex] = tracked_device_pose;
    }
    else if (device_class == vr::ETrackedDeviceClass::TrackedDeviceClass_Invalid) return false;
    return true;
}
std::string HtcViveTracker::GetDeviceClass(const std::int32_t arg_deviceIndex)
{
    vr::ETrackedDeviceClass tracked_device_class = m_p_vrSystem->GetTrackedDeviceClass(arg_deviceIndex);
    if (tracked_device_class == vr::ETrackedDeviceClass::TrackedDeviceClass_Controller) {
        return NAME_CONTROLLER;
    }
    else if (tracked_device_class == vr::ETrackedDeviceClass::TrackedDeviceClass_HMD) {
        return NAME_HMD;
    }
    else if (tracked_device_class == vr::ETrackedDeviceClass::TrackedDeviceClass_TrackingReference) {
        return NAME_TREFERENCE;
    }
    else if (tracked_device_class == vr::ETrackedDeviceClass::TrackedDeviceClass_GenericTracker) {
        return NAME_TRACKER;
    }
    else if (tracked_device_class == vr::ETrackedDeviceClass::TrackedDeviceClass_Invalid) {
        return NAME_NULL;
    }
    else return NAME_NULL;
}
std::string HtcViveTracker::SetDeviceName(const std::int32_t arg_deviceIndex)
{
    std::string class_name = GetDeviceClass(arg_deviceIndex);
    std::string arg_deviceName;
    if (class_name == NAME_CONTROLLER) {
        arg_deviceName = class_name + "_" + std::to_string(m_controllerCounts);
        m_controllerCounts++;
    }
    else if (class_name == NAME_HMD) {
        arg_deviceName = class_name + "_" + std::to_string(m_hmdCounts);
        m_hmdCounts++;
    }
    else if (class_name == NAME_TREFERENCE) {
        arg_deviceName = class_name + "_" + std::to_string(m_trackReferenceCounts);
        m_trackReferenceCounts++;
    }
    else if (class_name == NAME_TRACKER) {
        arg_deviceName = class_name + "_" + std::to_string(m_trackerCounts);
        m_trackerCounts++;
    }
    else if (class_name == NAME_NULL) {
        arg_deviceName = class_name + "_" + std::to_string(m_nullCounts);
        m_nullCounts++;
    }
    else arg_deviceName = NAME_NULL;

    return arg_deviceName;
}
bool HtcViveTracker::AddNewDevice(const std::int32_t arg_deviceIndex)
{
    std::string name = SetDeviceName(arg_deviceIndex);
    m_devicesIndex[name] = arg_deviceIndex;
    m_list_devicesNames[arg_deviceIndex] = name;
    m_activeDeviceNames.Clear();
    for (std::int32_t index=0; index < m_list_devicesNames.GetSize(); index++) {
        if (m_list_devicesNames[index].size()) {
            m_activeDeviceNames.Add(m_list_devicesNames[index]);
        }
    }
    return true;
}
bool HtcViveTracker::DeleteDevice(const std::int32_t arg_deviceIndex)
{
    std::string name = m_list_devicesNames[arg_deviceIndex];
    m_list_devicesNames[arg_deviceIndex] = "";
    m_devicesIndex.erase(name);
    m_activeDeviceNames.Clear();
    for (std::int32_t index=0; index < m_list_devicesNames.GetSize(); index++) {
        if (m_list_devicesNames[index].size()) {
            m_activeDeviceNames.Add(m_list_devicesNames[index]);
        }
    }
    return true;
}
Value_ptr<ITracker> ButiVive::CreateTracker()
{
    auto output = make_value<HtcViveTracker>();
    output->Initialize(true);

    return output;
}
}