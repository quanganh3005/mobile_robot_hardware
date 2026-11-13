// Copyright (c) 2021 Juan Miguel Jimeno
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef DEFAULT_IMU
#define DEFAULT_IMU

//include IMU base interface
#include "imu_interface.h"

//include sensor API headers
#include "I2Cdev.h"
#include "ADXL345.h"
#include "ITG3200.h"
#include "HMC5883L.h"
#include "MPU6050.h"
#include "MPU9150.h"
#include "MPU9250.h"
#include "QMI8658.h"

class GY85IMU: public IMUInterface 
{
    private:
        //constants specific to the sensor
        const float accel_scale_ = 1 / 256.0;
        const float gyro_scale_ = 1 / 14.375;

        // driver objects to be used
        ADXL345 accelerometer_;
        ITG3200 gyroscope_;

        // returned vector for sensor reading
        geometry_msgs__msg__Vector3 accel_;
        geometry_msgs__msg__Vector3 gyro_;

    public:
        GY85IMU()
        {
            // accel_cov_ = 0.001; //you can overwrite the convariance values here
            // gyro_cov_ = 0.001; //you can overwrite the convariance values here
        }

        bool startSensor() override
        {
            // here you can override startSensor() function and use the sensor's driver API
            // to initialize and test the sensor's connection during boot time
            Wire.begin();
            bool ret;
            accelerometer_.initialize();
            ret = accelerometer_.testConnection();
            if(!ret)
                return false;

            gyroscope_.initialize();
            ret = gyroscope_.testConnection();
            if(!ret)
                return false;

            return true;
        }

        geometry_msgs__msg__Vector3 readAccelerometer() override
        {
            // here you can override readAccelerometer function and use the sensor's driver API
            // to grab the data from accelerometer and return as a Vector3 object
            int16_t ax, ay, az;
            
            accelerometer_.getAcceleration(&ax, &ay, &az);

            accel_.x = ax * (double) accel_scale_ * g_to_accel_;
            accel_.y = ay * (double) accel_scale_ * g_to_accel_;
            accel_.z = az * (double) accel_scale_ * g_to_accel_;

            return accel_;
        }

        geometry_msgs__msg__Vector3 readGyroscope() override
        {
            // here you can override readAccelerometer function and use the sensor's driver API
            // to grab the data from gyroscope and return as a Vector3 object
            int16_t gx, gy, gz;

            gyroscope_.getRotation(&gx, &gy, &gz);

            gyro_.x = gx * (double) gyro_scale_ * DEG_TO_RAD;
            gyro_.y = gy * (double) gyro_scale_ * DEG_TO_RAD;
            gyro_.z = gz * (double) gyro_scale_ * DEG_TO_RAD;

            return gyro_;
        }
};


class MPU6050IMU: public IMUInterface 
{
    private:
        // MPU6050 default scale
        const float accel_scale_ = 1 / 16384.0;
        const float gyro_scale_ = 1 / 131.0;

        MPU6050 accelerometer_;
        MPU6050 gyroscope_;

        geometry_msgs__msg__Vector3 accel_;
        geometry_msgs__msg__Vector3 gyro_;

    public:
        MPU6050IMU()
        {
        }

        bool startSensor() override
        {
            Wire.begin();

            bool ret;

            accelerometer_.initialize();
            ret = accelerometer_.testConnection();

            if(!ret)
                return false;

            gyroscope_.initialize();
            ret = gyroscope_.testConnection();

            if(!ret)
                return false;

            return true;
        }

        geometry_msgs__msg__Vector3 readAccelerometer() override
        {
            int16_t ax, ay, az;

            accelerometer_.getAcceleration(&ax, &ay, &az);

            // =====================================================
            // 1. Convert raw MPU6050 -> m/s^2
            // =====================================================

            double raw_x =
                ax * (double) accel_scale_ * g_to_accel_;

            double raw_y =
                ay * (double) accel_scale_ * g_to_accel_;

            double raw_z =
                az * (double) accel_scale_ * g_to_accel_;


            // =====================================================
            // 2. Accelerometer bias
            //    From imu_calib.yaml
            // =====================================================

            // =====================================================
            // 2. Accelerometer bias
            //    From imu_calib.yaml
            // =====================================================

            const double bx = 0.37746151004210671;
            const double by = -0.047627139199379671;
            const double bz = -0.0024628240252435188;

            double dx = raw_x - bx;
            double dy = raw_y - by;
            double dz = raw_z - bz;


            // =====================================================
            // 3. Accelerometer scale/misalignment matrix
            //    From imu_calib.yaml
            //
            //    accel_scale:
            //
            //    [ m00 m01 m02 ]
            //    [ m10 m11 m12 ]
            //    [ m20 m21 m22 ]
            // =====================================================

            const double m00 = 0.99848409460197785;
            const double m01 = -0.05213062521940004;
            const double m02 = -0.027544776044800823;

            const double m10 = -0.0012303726888448266;
            const double m11 = 0.99691986958515566;
            const double m12 = 0.060741328476576324;

            const double m20 = 0.021764002275797709;
            const double m21 = 0.05627569030006354;
            const double m22 = 1.0126984912292694;


            // =====================================================
            // 4. Apply calibration matrix
            // =====================================================

            accel_.x =
                m00 * dx +
                m01 * dy +
                m02 * dz;

            accel_.y =
                m10 * dx +
                m11 * dy +
                m12 * dz;

            accel_.z =
                m20 * dx +
                m21 * dy +
                m22 * dz;


            return accel_;
        }


        geometry_msgs__msg__Vector3 readGyroscope() override
        {
            int16_t gx, gy, gz;

            gyroscope_.getRotation(&gx, &gy, &gz);


            // =====================================================
            // 1. Convert raw MPU6050 -> rad/s
            // =====================================================

            double raw_gx =
                gx * (double) gyro_scale_ * DEG_TO_RAD;

            double raw_gy =
                gy * (double) gyro_scale_ * DEG_TO_RAD;

            double raw_gz =
                gz * (double) gyro_scale_ * DEG_TO_RAD;


            // =====================================================
            // 2. Gyroscope bias
            //
            // Calibration result:
            // bias = [-0.000, -0.006, 0.000]
            // =====================================================

            const double gyro_bias_x = -0.000;
            const double gyro_bias_y = -0.006;
            const double gyro_bias_z = 0.000;


            // Remove gyro bias

            gyro_.x = raw_gx - gyro_bias_x;
            gyro_.y = raw_gy - gyro_bias_y;
            gyro_.z = raw_gz - gyro_bias_z;


            return gyro_;
        }
};

class MPU9150IMU: public IMUInterface 
{
    private:
        const float accel_scale_ = 1 / 16384.0;
        const float gyro_scale_ = 1 / 131.0;

        MPU9150 accelerometer_;
        MPU9150 gyroscope_;

        geometry_msgs__msg__Vector3 accel_;
        geometry_msgs__msg__Vector3 gyro_;

    public:
        MPU9150IMU()
        {
        }

        bool startSensor() override
        {
            Wire.begin();
            bool ret;
            accelerometer_.initialize();
            ret = accelerometer_.testConnection();
            if(!ret)
                return false;

            gyroscope_.initialize();
            ret = gyroscope_.testConnection();
            if(!ret)
                return false;

            return true;
        }

        geometry_msgs__msg__Vector3 readAccelerometer() override
        {
            int16_t ax, ay, az;
            
            accelerometer_.getAcceleration(&ax, &ay, &az);

            accel_.x = ax * (double) accel_scale_ * g_to_accel_;
            accel_.y = ay * (double) accel_scale_ * g_to_accel_;
            accel_.z = az * (double) accel_scale_ * g_to_accel_;

            return accel_;
        }

        geometry_msgs__msg__Vector3 readGyroscope() override
        {
            int16_t gx, gy, gz;

            gyroscope_.getRotation(&gx, &gy, &gz);

            gyro_.x = gx * (double) gyro_scale_ * DEG_TO_RAD;
            gyro_.y = gy * (double) gyro_scale_ * DEG_TO_RAD;
            gyro_.z = gz * (double) gyro_scale_ * DEG_TO_RAD;

            return gyro_;
        }
};

class MPU9250IMU: public IMUInterface 
{
    private:
        const float accel_scale_ = 1 / 16384.0;
        const float gyro_scale_ = 1 / 131.0;

        MPU9250 accelerometer_;
        MPU9250 gyroscope_;

        geometry_msgs__msg__Vector3 accel_;
        geometry_msgs__msg__Vector3 gyro_;

    public:
        MPU9250IMU()
        {
        }

        bool startSensor() override
        {
            Wire.begin();
            bool ret;
            accelerometer_.initialize();
            ret = accelerometer_.testConnection();
            if(!ret)
                return false;

            gyroscope_.initialize();
            ret = gyroscope_.testConnection();
            if(!ret)
                return false;

            return true;
        }

        geometry_msgs__msg__Vector3 readAccelerometer() override
        {
            int16_t ax, ay, az;
            
            accelerometer_.getAcceleration(&ax, &ay, &az);

            accel_.x = ax * (double) accel_scale_ * g_to_accel_;
            accel_.y = ay * (double) accel_scale_ * g_to_accel_;
            accel_.z = az * (double) accel_scale_ * g_to_accel_;

            return accel_;
        }

        geometry_msgs__msg__Vector3 readGyroscope() override
        {
            int16_t gx, gy, gz;

            gyroscope_.getRotation(&gx, &gy, &gz);

            gyro_.x = gx * (double) gyro_scale_ * DEG_TO_RAD;
            gyro_.y = gy * (double) gyro_scale_ * DEG_TO_RAD;
            gyro_.z = gz * (double) gyro_scale_ * DEG_TO_RAD;

            return gyro_;
        }
};

class FakeIMU: public IMUInterface 
{
    private:
        geometry_msgs__msg__Vector3 accel_;
        geometry_msgs__msg__Vector3 gyro_;

    public:
        FakeIMU()
        {
        }

        bool startSensor() override
        {
            return true;
        }

        geometry_msgs__msg__Vector3 readAccelerometer() override
        {
            return accel_;
        }

        geometry_msgs__msg__Vector3 readGyroscope() override
        {
            return gyro_;
        }
};

class QMI8658IMU: public IMUInterface 
{
    private:
        QMI8658 qmi8658_;

        geometry_msgs__msg__Vector3 accel_;
        geometry_msgs__msg__Vector3 gyro_;

    public:
        QMI8658IMU()
        {
        }

        bool startSensor() override
        {
            Wire.begin();
	    if (qmi8658_.begin() == 0){
	        // Serial.println("qmi8658_init fail");
	        return false;
	    }
	    return true;
        }

        geometry_msgs__msg__Vector3 readAccelerometer() override
        {
	    float ac[3];
            qmi8658_.read_acc(ac);
            accel_.x = ac[0];
            accel_.y = ac[1];
            accel_.z = ac[2];
            return accel_;
        }

        geometry_msgs__msg__Vector3 readGyroscope() override
        {
	    float gy[3];
            qmi8658_.read_gyro(gy);
            gyro_.x = gy[0];
            gyro_.y = gy[1];
            gyro_.z = gy[2];
            return gyro_;
        }
};

#endif
