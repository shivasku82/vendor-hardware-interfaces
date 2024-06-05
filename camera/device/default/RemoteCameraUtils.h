#ifndef ANDROID_HARDWARE_CAMERA_DEVICE_REMOTECAMUTILS_H
#define ANDROID_HARDWARE_CAMERA_DEVICE_REMOTECAMUTILS_H

#include <android/hardware/graphics/common/1.0/types.h>
#include <android/hardware/graphics/mapper/2.0/IMapper.h>
#include <inttypes.h>
#include <mutex>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <string>
#include "tinyxml2.h"  // XML parsing
#include "utils/LightRefBase.h"
#include "utils/Timers.h"
#include <CameraMetadata.h>
#include <HandleImporter.h>

#include <sys/mman.h>
#include <linux/videodev2.h>

#include "ExternalCameraUtils.h"

namespace android {
namespace hardware {
namespace camera {
namespace device {
namespace implementation {

#define DEFAULT_WIDTH 1920
#define DEFAULT_HEIGHT 1080
#define DEFAULT_FPS 30

enum ResolutionType {
    RESOLUTION_MAX = 0,
    RESOLUTION_COMPOSE,
    RESOLUTION_CROP,
    RESOLUTION_TARGET,

};

enum IRMode {
    UNKNOWN_MODE=-1,
    ADAPTIVE_DAY_MODE=0,
    ADAPTIVE_NIGHT_MODE,
    HOSTCONTROL_DAY_MODE,
    HOSTCONTROL_NIGHT_MODE,
};

struct McFormat {
    int entity;
    int pad;
    int stream;
    int formatType;
    int selCmd;
    int top;
    int left;
    int width;
    int height;
    enum ResolutionType type;
    std::string entityName;
    unsigned int pixelCode;
};

struct McLink {
    int srcEntity;
    int srcPad;
    int sinkEntity;
    int sinkPad;
    bool enable;
    std::string srcEntityName;
    std::string sinkEntityName;
};

struct RemoteCameraConfig {
    std::string path;
    std::string driver;
    uint32_t width;
    uint32_t height;
    uint32_t fps;
    uint32_t numVideoBuffers;
    uint32_t numStillBuffers;
    std::string format;
    std::vector<std::string> virtIds;
    std::string subdev;
    IRMode irMode;
};

class RemoteCameraProviderConfig{
public:
    RemoteCameraProviderConfig() {}
    virtual ~RemoteCameraProviderConfig() {
        for (auto iter : mCameraConfigs) {
            delete iter;
            iter = nullptr;
        }

        mCameraConfigs.clear();
    }
    const std::string kDefaultCfgPath;

    void loadFromCfg(const char* cfgPath);
    void printCfg();

    int mCameraIdOffset;
    std::string mLoopbackPairId;
    std::string mLoopbackSrcCamera;
    RemoteCameraConfig mLoopbackConfig;

    std::unordered_set<std::string> mIgnoreDevices;
    std::unordered_set<RemoteCameraConfig*> mCameraConfigs;

    std::vector<McFormat> mMCFormats;
    std::vector<McLink> mMCLinks;

private:
    int loadDeviceCfg(tinyxml2::XMLElement* deviceCfg);
    int loadLoopbackCfg(tinyxml2::XMLElement* loopbackCfg);
    int loadMediaCtrl(tinyxml2::XMLElement* mediaCtrlCfg);
};

}  // namespace implementation
}  // namespace device
}  // namespace camera
}  // namespace hardware
}  // namespace android

#endif
