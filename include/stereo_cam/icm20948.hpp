#ifndef STEREO_CAM_ICM20948_HPP
#define STEREO_CAM_ICM20948_HPP

#include <cstdint>
#include <memory>
#include <string>

namespace stereo_cam {

enum class AccelRange {
    G2 = 2,
    G4 = 4,
    G8 = 8,
    G16 = 16
};

enum class GyroRange {
    DPS250 = 250,
    DPS500 = 500,
    DPS1000 = 1000,
    DPS2000 = 2000
};

class ICM20948 {
public:
    ICM20948(const std::string& device = "/dev/i2c-1", uint8_t address = 0x68);
    ~ICM20948();

    bool init();
    bool configureAccel(AccelRange range);
    bool configureGyro(GyroRange range);
    bool enableMagnetometer(bool enable);

    bool readAccel(float& x, float& y, float& z);
    bool readGyro(float& x, float& y, float& z);
    bool readMag(float& x, float& y, float& z);
    bool readTemp(float& temp);

private:
    class Impl;
    std::unique_ptr<Impl> pimpl_;
};

} // namespace stereo_cam

#endif // STEREO_CAM_ICM20948_HPP 