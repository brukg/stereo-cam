#include "stereo_cam/icm20948.hpp"
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <cmath>
#include <iostream>

namespace stereo_cam {

// Constants from reference code
constexpr uint8_t I2C_ADD_ICM20948 = 0x68;
constexpr uint8_t I2C_ADD_ICM20948_AK09916 = 0x0C;
constexpr uint8_t I2C_ADD_ICM20948_AK09916_READ = 0x80;
constexpr uint8_t I2C_ADD_ICM20948_AK09916_WRITE = 0x00;

// Register addresses
constexpr uint8_t REG_ADD_WIA = 0x00;
constexpr uint8_t REG_VAL_WIA = 0xEA;
constexpr uint8_t REG_ADD_USER_CTRL = 0x03;
constexpr uint8_t REG_VAL_BIT_I2C_MST_EN = 0x20;
constexpr uint8_t REG_ADD_REG_BANK_SEL = 0x7F;
constexpr uint8_t REG_VAL_REG_BANK_0 = 0x00;
constexpr uint8_t REG_VAL_REG_BANK_3 = 0x30;

// Magnetometer registers
constexpr uint8_t REG_ADD_MAG_WIA1 = 0x00;
constexpr uint8_t REG_VAL_MAG_WIA1 = 0x48;
constexpr uint8_t REG_ADD_MAG_WIA2 = 0x01;
constexpr uint8_t REG_VAL_MAG_WIA2 = 0x09;
constexpr uint8_t REG_ADD_MAG_ST2 = 0x10;
constexpr uint8_t REG_ADD_MAG_DATA = 0x11;
constexpr uint8_t REG_ADD_MAG_CNTL2 = 0x31;
constexpr uint8_t REG_VAL_MAG_MODE_20HZ = 0x04;
constexpr uint8_t REG_VAL_MAG_MODE_100HZ = 0x05;

// I2C slave registers
constexpr uint8_t REG_ADD_I2C_SLV0_ADDR = 0x03;
constexpr uint8_t REG_ADD_I2C_SLV0_REG = 0x04;
constexpr uint8_t REG_ADD_I2C_SLV0_CTRL = 0x05;
constexpr uint8_t REG_VAL_BIT_SLV0_EN = 0x80;
constexpr uint8_t REG_VAL_BIT_MASK_LEN = 0x07;
constexpr uint8_t REG_ADD_EXT_SENS_DATA_00 = 0x3B;

constexpr uint8_t REG_ADD_I2C_SLV1_ADDR = 0x07;
constexpr uint8_t REG_ADD_I2C_SLV1_REG = 0x08;
constexpr uint8_t REG_ADD_I2C_SLV1_CTRL = 0x09;
constexpr uint8_t REG_ADD_I2C_SLV1_DO = 0x0A;

constexpr uint8_t MAG_DATA_LEN = 6;
constexpr float MAG_SSF_AT_FS_4900uT = 0.15f;

// Add these with other constants
constexpr uint8_t REG_ADD_PWR_MIGMT_1 = 0x06;
constexpr uint8_t REG_VAL_ALL_RGE_RESET = 0x80;
constexpr uint8_t REG_VAL_RUN_MODE = 0x01;

struct ICM20948_ST_AVG_DATA {
    uint8_t u8Index;
    int16_t s16AvgBuffer[8];
};

class ICM20948::Impl {
public:
    Impl(const std::string& device, uint8_t address) 
        : device_(device), address_(address), fd_(-1) {
        gyro_offset_ = {0, 0, 0};
    }
    
    ~Impl() {
        if (fd_ >= 0) {
            close(fd_);
        }
    }

    bool init() {
        fd_ = open(device_.c_str(), O_RDWR);
        if (fd_ < 0) return false;
        
        if (ioctl(fd_, I2C_SLAVE, address_) < 0) {
            close(fd_);
            fd_ = -1;
            return false;
        }

        // Reset device
        writeRegister(0x06, 0x80);
        usleep(10000);
        writeRegister(0x06, 0x01);

        // Configure gyro and accel
        writeRegister(0x7F, 0x20); // Switch to bank 2
        writeRegister(0x01, 0x06); // Gyro config: ±1000dps
        writeRegister(0x14, 0x06); // Accel config: ±2g
        writeRegister(0x7F, 0x00); // Switch back to bank 0

        // Initialize magnetometer
        initMagnetometer();
        
        // Calculate gyro offset
        calculateGyroOffset();
        
        return true;
    }

    bool readAccel(float& x, float& y, float& z) {
        uint8_t data[6];
        if (!readRegisters(0x2D, data, 6)) return false;

        int16_t raw_x = (data[0] << 8) | data[1];
        int16_t raw_y = (data[2] << 8) | data[3];
        int16_t raw_z = (data[4] << 8) | data[5];

        // Convert to g (±2g range)
        x = raw_x / 16384.0f;
        y = raw_y / 16384.0f;
        z = raw_z / 16384.0f;

        return true;
    }

    bool readGyro(float& x, float& y, float& z) {
        uint8_t data[6];
        if (!readRegisters(0x33, data, 6)) return false;

        int16_t raw_x = ((data[0] << 8) | data[1]) - gyro_offset_.x;
        int16_t raw_y = ((data[2] << 8) | data[3]) - gyro_offset_.y;
        int16_t raw_z = ((data[4] << 8) | data[5]) - gyro_offset_.z;

        // Convert to degrees per second (±1000dps range)
        x = raw_x / 32.8f;
        y = raw_y / 32.8f;
        z = raw_z / 32.8f;

        return true;
    }

    bool readMag(float& x, float& y, float& z) {
        uint8_t counter = 20;
        uint8_t u8Data[MAG_DATA_LEN];
        int16_t s16Buf[3] = {0}; 
        uint8_t u8Ret;
        static ICM20948_ST_AVG_DATA sstAvgBuf[3];

        while(counter > 0) {
            usleep(10*1000);
            icm20948ReadSecondary(I2C_ADD_ICM20948_AK09916 | I2C_ADD_ICM20948_AK09916_READ, 
                                 REG_ADD_MAG_ST2, 1, &u8Ret);
            
            if ((u8Ret & 0x01) != 0)
                break;
            
            counter--;
        }
        
        if(counter != 0) {
            icm20948ReadSecondary(I2C_ADD_ICM20948_AK09916 | I2C_ADD_ICM20948_AK09916_READ, 
                                 REG_ADD_MAG_DATA, 
                                 MAG_DATA_LEN,
                                 u8Data);


            s16Buf[0] = ((int16_t)u8Data[1]<<8) | u8Data[0];
            s16Buf[1] = ((int16_t)u8Data[3]<<8) | u8Data[2];
            s16Buf[2] = ((int16_t)u8Data[5]<<8) | u8Data[4];

            

            // Use averaging like in the sample code
            int32_t s32OutBuf[3] = {0};
            for(uint8_t i = 0; i < 3; i++) {
                icm20948CalAvgValue(&sstAvgBuf[i].u8Index, sstAvgBuf[i].s16AvgBuffer, s16Buf[i], &s32OutBuf[i]);
            }

            
            x = s32OutBuf[0] * 0.15f;
            y = -s32OutBuf[1] * 0.15f;
            z = -s32OutBuf[2] * 0.15f;
        } else {
            return false;
        }

        return true;
    }

    bool readTemp(float& temp) {
        uint8_t data[2];
        if (!readRegisters(0x35, data, 2)) return false;

        int16_t raw_temp = (data[0] << 8) | data[1];

        // Convert to degrees Celsius
        temp = (raw_temp / 333.87f) + 25.0f;

        return true;
    }

private:
    std::string device_;
    uint8_t address_;
    int fd_;

    struct GyroOffset {
        int16_t x, y, z;
    } gyro_offset_;

    void writeRegister(uint8_t reg, uint8_t value) {
        uint8_t buf[2] = {reg, value};
        write(fd_, buf, 2);
    }

    bool readRegisters(uint8_t reg, uint8_t* data, size_t length) {
        write(fd_, &reg, 1);
        return read(fd_, data, length) == static_cast<ssize_t>(length);
    }

    uint8_t readRegister(uint8_t reg) {
        uint8_t value;
        write(fd_, &reg, 1);
        read(fd_, &value, 1);
        return value;
    }

    void initMagnetometer() {
        // Reset magnetometer first
        writeRegister(REG_ADD_REG_BANK_SEL, REG_VAL_REG_BANK_0);
        writeRegister(REG_ADD_PWR_MIGMT_1, REG_VAL_ALL_RGE_RESET);
        usleep(10*1000);
        writeRegister(REG_ADD_PWR_MIGMT_1, REG_VAL_RUN_MODE);

        // Switch to bank 3
        writeRegister(REG_ADD_REG_BANK_SEL, REG_VAL_REG_BANK_3);
        
        // Configure I2C master
        writeRegister(0x01, 0x4D);
        writeRegister(0x02, 0x01);
        writeRegister(0x05, 0x81);
        
        // Switch to bank 0
        writeRegister(REG_ADD_REG_BANK_SEL, REG_VAL_REG_BANK_0);

        usleep(100*1000);

        // Check magnetometer ID
        if (!icm20948MagCheck()) {
            std::cerr << "Magnetometer check failed" << std::endl;
            return;
        }

        // Set magnetometer mode
        icm20948WriteSecondary(I2C_ADD_ICM20948_AK09916 | I2C_ADD_ICM20948_AK09916_WRITE,
                              REG_ADD_MAG_CNTL2, REG_VAL_MAG_MODE_20HZ);

        std::cout << "Magnetometer initialized successfully" << std::endl;
    }

    bool icm20948MagCheck() {
        uint8_t u8Ret[2];
        
        icm20948ReadSecondary(I2C_ADD_ICM20948_AK09916 | I2C_ADD_ICM20948_AK09916_READ,
                             REG_ADD_MAG_WIA1, 2, u8Ret);
        if ((u8Ret[0] == REG_VAL_MAG_WIA1) && (u8Ret[1] == REG_VAL_MAG_WIA2)) {
            return true;
        }
        return false;
    }

    void calculateGyroOffset() {
        int32_t x_sum = 0, y_sum = 0, z_sum = 0;
        const int samples = 32;

        for (int i = 0; i < samples; i++) {
            uint8_t data[6];
            if (readRegisters(0x33, data, 6)) {
                x_sum += (data[0] << 8) | data[1];
                y_sum += (data[2] << 8) | data[3];
                z_sum += (data[4] << 8) | data[5];
            }
            usleep(10000);
        }

        gyro_offset_.x = x_sum / samples;
        gyro_offset_.y = y_sum / samples;
        gyro_offset_.z = z_sum / samples;
    }

    void icm20948ReadSecondary(uint8_t u8I2CAddr, uint8_t u8RegAddr, uint8_t u8Len, uint8_t *pu8data) {
        uint8_t u8Temp;

        writeRegister(REG_ADD_REG_BANK_SEL, REG_VAL_REG_BANK_3); // Switch to bank 3
        writeRegister(REG_ADD_I2C_SLV0_ADDR, u8I2CAddr);
        writeRegister(REG_ADD_I2C_SLV0_REG, u8RegAddr);
        writeRegister(REG_ADD_I2C_SLV0_CTRL, REG_VAL_BIT_SLV0_EN | u8Len);

        writeRegister(REG_ADD_REG_BANK_SEL, REG_VAL_REG_BANK_0); // Switch to bank 0
        
        u8Temp = readRegister(REG_ADD_USER_CTRL);
        u8Temp |= REG_VAL_BIT_I2C_MST_EN;
        writeRegister(REG_ADD_USER_CTRL, u8Temp);
        usleep(5*1000);
        u8Temp &= ~REG_VAL_BIT_I2C_MST_EN;
        writeRegister(REG_ADD_USER_CTRL, u8Temp);

        for(uint8_t i = 0; i < u8Len; i++) {
            *(pu8data + i) = readRegister(REG_ADD_EXT_SENS_DATA_00 + i);
        }

        writeRegister(REG_ADD_REG_BANK_SEL, REG_VAL_REG_BANK_3); // Switch to bank 3
        u8Temp = readRegister(REG_ADD_I2C_SLV0_CTRL);
        u8Temp &= ~((REG_VAL_BIT_I2C_MST_EN) & (REG_VAL_BIT_MASK_LEN));
        writeRegister(REG_ADD_I2C_SLV0_CTRL, u8Temp);
        
        writeRegister(REG_ADD_REG_BANK_SEL, REG_VAL_REG_BANK_0); // Switch to bank 0
    }

    void icm20948WriteSecondary(uint8_t u8I2CAddr, uint8_t u8RegAddr, uint8_t u8data) {
        uint8_t u8Temp;

        writeRegister(REG_ADD_REG_BANK_SEL, REG_VAL_REG_BANK_3); // Switch to bank 3
        writeRegister(REG_ADD_I2C_SLV1_ADDR, u8I2CAddr);
        writeRegister(REG_ADD_I2C_SLV1_REG, u8RegAddr);
        writeRegister(REG_ADD_I2C_SLV1_DO, u8data);
        writeRegister(REG_ADD_I2C_SLV1_CTRL, REG_VAL_BIT_SLV0_EN | 1);

        writeRegister(REG_ADD_REG_BANK_SEL, REG_VAL_REG_BANK_0); // Switch to bank 0

        u8Temp = readRegister(REG_ADD_USER_CTRL);
        u8Temp |= REG_VAL_BIT_I2C_MST_EN;
        writeRegister(REG_ADD_USER_CTRL, u8Temp);
        usleep(5*1000);
        u8Temp &= ~REG_VAL_BIT_I2C_MST_EN;
        writeRegister(REG_ADD_USER_CTRL, u8Temp);

        writeRegister(REG_ADD_REG_BANK_SEL, REG_VAL_REG_BANK_3); // Switch to bank 3
        u8Temp = readRegister(REG_ADD_I2C_SLV0_CTRL);
        u8Temp &= ~((REG_VAL_BIT_I2C_MST_EN) & (REG_VAL_BIT_MASK_LEN));
        writeRegister(REG_ADD_I2C_SLV0_CTRL, u8Temp);

        writeRegister(REG_ADD_REG_BANK_SEL, REG_VAL_REG_BANK_0); // Switch to bank 0
    }

    void icm20948CalAvgValue(uint8_t *pIndex, int16_t *pAvgBuffer, int16_t InVal, int32_t *pOutVal) {
        *(pAvgBuffer + ((*pIndex) ++)) = InVal;
        *pIndex &= 0x07;
        
        *pOutVal = 0;
        for(uint8_t i = 0; i < 8; i++) {
            *pOutVal += *(pAvgBuffer + i);
        }
        *pOutVal >>= 3;
    }
};

ICM20948::ICM20948(const std::string& device, uint8_t address)
    : pimpl_(std::make_unique<Impl>(device, address)) {}

ICM20948::~ICM20948() = default;

bool ICM20948::init() { return pimpl_->init(); }

bool ICM20948::configureAccel(AccelRange range) { return true; }
bool ICM20948::configureGyro(GyroRange range) { return true; }
bool ICM20948::enableMagnetometer(bool enable) { return true; }
bool ICM20948::readAccel(float& x, float& y, float& z) { return pimpl_->readAccel(x, y, z); }
bool ICM20948::readGyro(float& x, float& y, float& z) { return pimpl_->readGyro(x, y, z); }
bool ICM20948::readMag(float& x, float& y, float& z) { return pimpl_->readMag(x, y, z); }
bool ICM20948::readTemp(float& temp) { return pimpl_->readTemp(temp); }

} // namespace stereo_cam 