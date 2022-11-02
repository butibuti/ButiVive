#ifndef BUTIVIVE_COMMON_H
#define BUTIVIVE_COMMON_H
#include"ButiMath/ButiMath.h"
#include"ButiMemorySystem/ButiMemorySystem/ButiPtr.h"
#include"ButiMemorySystem/ButiMemorySystem/ButiList.h"

#ifdef BUTIVIVE_EXPORTS
#define BUTIVIVE_API __declspec(dllexport)
#else
#define BUTIVIVE_API __declspec(dllimport)
#endif

namespace ButiVive {
struct Velocity {
    ButiEngine::Vector3 linear_velocity;
    ButiEngine::Vector3 angular_velocity;
};

struct Dimension {
    ButiEngine::Vector3 corner1;
    ButiEngine::Vector3 corner2;
    ButiEngine::Vector3 corner3;
    ButiEngine::Vector3 corner4;

    float size_x;
    float size_z;
};

enum class EventFlags {
    BUTTONPRESS,
    BUTTONUNPRESS
};

enum class ButtonFlags {
    Button_OUT,
    Button_GND,
    Button_GRIP,
    Button_TRIGGER,
    Button_TOUCHPAD,
    Button_MENU,
    Button_SYSTEM,
    Button_OTHER
};
enum class VRDeviceButtonId
{
    System = 0,
    ApplicationMenu = 1,
    Grip = 2,
    DPad_Left = 3,
    DPad_Up = 4,
    DPad_Right = 5,
    DPad_Down = 6,
    A = 7,

    ProximitySensor = 31,

    Axis0 = 32,
    Axis1 = 33,
    Axis2 = 34,
    Axis3 = 35,
    Axis4 = 36,

    SteamVR_Touchpad = Axis0,
    SteamVR_Trigger = Axis1,

    Dashboard_Back = Grip,

    IndexController_A = Grip,
    IndexController_B = ApplicationMenu,
    IndexController_JoyStick = Axis3,

    Max = 64
};
class ITracker {
public:
    virtual bool Initialize()= 0;
    virtual bool ShutDown()= 0;

    virtual void Update()= 0;

    virtual bool IsDeviceDetected(const std::string& arg_deviceName)= 0;
    virtual void PrintAllDetectedDevices()= 0;
    virtual const ButiEngine::List<std::string>& GetAllDeviceNames()const = 0;
    virtual float GetBatteryLevel(const std::string& arg_deviceName)= 0;
    virtual bool EventPolling()= 0;
    virtual bool GetDevicePoseQuaternion(const std::string& arg_deviceName, ButiEngine::Vector3& arg_ref_pose, ButiEngine::Quat& arg_ref_angle) = 0;
    virtual bool GetDevicePoseMatrix(const std::string& arg_deviceName, ButiEngine::Matrix4x4& arg_ref_matrix)= 0;
    virtual Velocity GetDeviceVelocity(const std::string& arg_deviceName)= 0;

    virtual Dimension GetChaperoneDimensions() const= 0;

    virtual std::string GetLastButtonPressedString(const std::string& arg_deviceName)= 0;
    virtual VRDeviceButtonId GetLastButtonPressedEnum(const std::string& arg_deviceName) = 0;
    virtual bool HapticPulse(const std::string& arg_deviceName,const std::uint32_t arg_axisIndex,const std::int16_t arg_durationMicrosec)= 0;
    virtual void SetOrigin(const ButiEngine::Matrix4x4& arg_matrix) = 0;
    virtual void SetOrigin(const std::int32_t arg_deviceIndex)=0;
    virtual const ButiEngine::Matrix4x4& GetOrigin()const=0;
};
BUTIVIVE_API  ButiEngine::Value_ptr<ITracker> CreateTracker();
#ifdef _DEBUG
BUTIVIVE_API void SetPrintFunction(void (*arg_printFunction)(const std::string&));
BUTIVIVE_API void SetIsDebugPrint(const bool arg_isDebugPrintEnable);
#endif
}

#endif // !BUTIVIVE_COMMON_H
