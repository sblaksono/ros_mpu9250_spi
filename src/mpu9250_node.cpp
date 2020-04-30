/*
    ros_mpu9250_spi : mpu9250_node

    MPU9250 with SPI interface, tested running on Raspberry Pi 3B+.
    Sensor is connected to spidev0.1.

    some code and files were taken from these works:
    https://github.com/wolfeidau/ros-mpu9250-node
    https://github.com/emlid/Navio2
*/

#include <string>
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/Temperature.h"
#include "ros_mpu9250/ahrs.h"
#include "MPU9250.h"

#define G_SI 9.80665
#define PI   3.14159

using namespace std;

class ImuNode 
{
    public:
        ros::NodeHandle node_handle;
        ros::Publisher imu_data_pub;
        ros::Publisher ahrs_data_pub;
        ros::Publisher temp_data_pub;
        sensor_msgs::Imu imu_data;
        ros_mpu9250::ahrs ahrs_data;
        sensor_msgs::Temperature temp_data;
        string frame_id;
        bool publish_temperature = false;
        double rate;
        float ax, ay, az;
        float vx, vy, vz;
        float ox, oy, oz;
        MPU9250 *sensor;
    	float q0, q1, q2, q3;
	    float gyroOffset[3];
    	float twoKi;
    	float twoKp;
    	float integralFBx, integralFBy, integralFBz;

        explicit ImuNode(ros::NodeHandle nh)
            : node_handle(nh)
        {
            node_handle.param("frame_id", frame_id, std::string("imu"));
            node_handle.param("rate", rate, 100.0);

            ROS_INFO("frame_id: %s", frame_id.c_str());
            ROS_INFO("rate: %f [Hz]", rate);
            ROS_INFO("publish_temperature: %s", (publish_temperature ? "true" : "false"));

            q0 = 1; q1 = 0; q2 = 0, q3 = 0; twoKi = 0; twoKp =2;

            imu_data_pub = node_handle.advertise<sensor_msgs::Imu>("imu/data_raw", 100);
            ahrs_data_pub = node_handle.advertise<ros_mpu9250::ahrs>("ahrs", 100);
            if (publish_temperature)
            {
                temp_data_pub = node_handle.advertise<sensor_msgs::Temperature>("temperature", 100);
            }
        }

        bool init(void) {
            sensor = new MPU9250();
            if (!sensor->probe()) {
                ROS_INFO("Sensor not enabled");
                return false;
            }
            sensor->initialize();
            return true;
        }

        void update_imu(void)
        {
            sensor->update();
            sensor->read_accelerometer(&ax, &ay, &az);
            sensor->read_gyroscope(&vx, &vy, &vz);
            sensor->read_magnetometer(&ox, &oy, &oz);
        }

        void publish_imu_data(void)
        {
            imu_data.header.frame_id = frame_id;
            imu_data.header.stamp = ros::Time::now();

            imu_data.linear_acceleration.x = ax;
            imu_data.linear_acceleration.y = ay;
            imu_data.linear_acceleration.z = az;

            imu_data.angular_velocity.x = vx;
            imu_data.angular_velocity.y = vy;
            imu_data.angular_velocity.z = vz;

            imu_data.orientation.x = ox;
            imu_data.orientation.y = oy;
            imu_data.orientation.z = oz;

            imu_data_pub.publish(imu_data);
        }

        void publish_temp_data(void)
        {
            temp_data.header.frame_id = frame_id;
            temp_data.header.stamp = ros::Time::now();

            temp_data.temperature = sensor->read_temperature();
            temp_data.variance = 0;

            temp_data_pub.publish(temp_data);
        }

        void setGyroOffset()
        {
            float offset[3] = {0.0, 0.0, 0.0};
            float gx, gy, gz;

            ROS_INFO("Beginning Gyro calibration...");
            for(int i = 0; i < 100; i++)
            {
                sensor->update();
                sensor->read_gyroscope(&gx, &gy, &gz);

                gx *= 180 / PI;
                gy *= 180 / PI;
                gz *= 180 / PI;

                offset[0] += gx * 0.0175;
                offset[1] += gy * 0.0175;
                offset[2] += gz * 0.0175;

                usleep(10000);
            }
            offset[0] /= 100.0;
            offset[1] /= 100.0;
            offset[2] /= 100.0;

            ROS_INFO("Offsets are: %f %f %f", offset[0], offset[1], offset[2]);

            gyroOffset[0] = offset[0];
            gyroOffset[1] = offset[1];
            gyroOffset[2] = offset[2];
        }


        float invSqrt(float x)
        {
            float halfx = 0.5f * x;
            float y = x;
            long i = *(long*)&y;
            i = 0x5f3759df - (i>>1);
            y = *(float*)&i;
            y = y * (1.5f - (halfx * y * y));
            return y;
        }

        void update_ahrs(float dt)
        {
            float recipNorm;
            float halfvx, halfvy, halfvz;
            float halfex, halfey, halfez;
            float qa, qb, qc;

            float cx, cy, cz;
            float gx, gy, gz;

            cx = ax / G_SI;
            cy = ay / G_SI;
            cz = az / G_SI;
            gx = vx * (180 / PI) * 0.0175;
            gy = vy * (180 / PI) * 0.0175;
            gz = vz * (180 / PI) * 0.0175;

            gx -= gyroOffset[0];
            gy -= gyroOffset[1];
            gz -= gyroOffset[2];

            // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
            if(!((cx == 0.0f) && (cy == 0.0f) && (cz == 0.0f))) {

                // Normalise accelerometer measurement
                recipNorm = invSqrt(cx * cx + cy * cy + cz * cz);
                cx *= recipNorm;
                cy *= recipNorm;
                cz *= recipNorm;

                // Estimated direction of gravity and vector perpendicular to magnetic flux
                halfvx = q1 * q3 - q0 * q2;
                halfvy = q0 * q1 + q2 * q3;
                halfvz = q0 * q0 - 0.5f + q3 * q3;

                // Error is sum of cross product between estimated and measured direction of gravity
                halfex = (cy * halfvz - cz * halfvy);
                halfey = (cz * halfvx - cx * halfvz);
                halfez = (cx * halfvy - cy * halfvx);

                // Compute and apply integral feedback if enabled
                if(twoKi > 0.0f) {
                    integralFBx += twoKi * halfex * dt;	// integral error scaled by Ki
                    integralFBy += twoKi * halfey * dt;
                    integralFBz += twoKi * halfez * dt;
                    gx += integralFBx;	// apply integral feedback
                    gy += integralFBy;
                    gz += integralFBz;
                }
                else {
                    integralFBx = 0.0f;	// prevent integral windup
                    integralFBy = 0.0f;
                    integralFBz = 0.0f;
                }

                // Apply proportional feedback
                gx += twoKp * halfex;
                gy += twoKp * halfey;
                gz += twoKp * halfez;
            }

            // Integrate rate of change of quaternion
            gx *= (0.5f * dt);		// pre-multiply common factors
            gy *= (0.5f * dt);
            gz *= (0.5f * dt);
            qa = q0;
            qb = q1;
            qc = q2;
            q0 += (-qb * gx - qc * gy - q3 * gz);
            q1 += (qa * gx + qc * gz - q3 * gy);
            q2 += (qa * gy - qb * gz + q3 * gx);
            q3 += (qa * gz + qb * gy - qc * gx);

            // Normalise quaternion
            recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
            q0 *= recipNorm;
            q1 *= recipNorm;
            q2 *= recipNorm;
            q3 *= recipNorm;
        }

        void publish_ahrs_data(void)
        {
            ahrs_data.roll = atan2(2 * (q0 * q1 + q2 * q3), 1 - 2 * (q1 * q1 + q2 * q2)) * 180.0 / M_PI;
            ahrs_data.pitch = asin(2 * (q0 * q2 - q3 * q1)) * 180.0 / M_PI;
            ahrs_data.yaw = atan2(2 * (q0 * q3 + q1 * q2), 1 - 2 * (q2 * q2 + q3 * q3)) * 180.0 / M_PI;        

            ahrs_data_pub.publish(ahrs_data);
        } 

        void spin(void)
        {
            struct timeval tv;
            float dt;
            static unsigned long previoustime, currenttime;

            setGyroOffset();

            ros::Rate loop_rate(rate);
            while (ros::ok())
            {
                update_imu();
                publish_imu_data();

	            gettimeofday(&tv, NULL);
	            previoustime = currenttime;
	            currenttime = 1000000 * tv.tv_sec + tv.tv_usec;
	            dt = (currenttime - previoustime) / 1000000.0;
	            if(dt < 1/1300.0) usleep((1/1300.0 - dt) * 1000000);
                gettimeofday(&tv, NULL);
                currenttime = 1000000 * tv.tv_sec + tv.tv_usec;
	            dt = (currenttime - previoustime) / 1000000.0;

                update_ahrs(dt);
                publish_ahrs_data();

                if (publish_temperature) publish_temp_data();

                ros::spinOnce();
                loop_rate.sleep();
            }
        }

};

int main(int argc, char **argv) {

    ros::init(argc, argv, "ros_mpu9250");
    ros::NodeHandle nh;
    ImuNode node(nh);

    if (!node.init()) return EXIT_FAILURE;

    node.spin();

    return 0;
}
