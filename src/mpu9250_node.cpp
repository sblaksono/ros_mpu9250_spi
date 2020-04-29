#include <string>
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/Temperature.h"
#include "MPU9250.h"

using namespace std;

class ImuNode 
{
    public:
        ros::NodeHandle node_handle;
        ros::Publisher imu_data_pub;
        ros::Publisher temp_data_pub;
        sensor_msgs::Imu imu_data;
        sensor_msgs::Temperature temp_data;
        string device;
        string frame_id;
        bool publish_temperature = false;
        double rate;
        float ax, ay, az;
        float vx, vy, vz;
        float ox, oy, oz;
        MPU9250 *sensor;

        explicit ImuNode(ros::NodeHandle nh)
            : node_handle(nh)
        {
            node_handle.param("device", device, std::string("mpu9250"));
            node_handle.param("frame_id", frame_id, std::string("imu"));
            node_handle.param("rate", rate, 100.0);

            ROS_INFO("device: %s", device.c_str());
            ROS_INFO("frame_id: %s", frame_id.c_str());
            ROS_INFO("rate: %f [Hz]", rate);
            ROS_INFO("publish_temperature: %s", (publish_temperature ? "true" : "false"));

            imu_data_pub = node_handle.advertise<sensor_msgs::Imu>("data_raw", 100);
            if (publish_temperature)
            {
                temp_data_pub = node_handle.advertise<sensor_msgs::Temperature>("temperature", 100);
            }
        }

        bool init(void) {
            sensor = new MPU9250();
            if (!sensor->probe()) {
                printf("Sensor not enabled\n");
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

        void spin(void)
        {
            ros::Rate loop_rate(rate);
            while (ros::ok())
            {
                update_imu();
                publish_imu_data();

                if (publish_temperature) publish_temp_data();

                ros::spinOnce();
                loop_rate.sleep();
            }
        }

};

int main(int argc, char **argv) {

    ros::init(argc, argv, "mpu9250");
    ros::NodeHandle nh("~");
    ImuNode node(nh);

    if (!node.init())
    {
        printf("Sensor not enabled\n");
        return EXIT_FAILURE;
    }

    node.spin();

    return 0;
}
