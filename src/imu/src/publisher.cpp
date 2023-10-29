#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "i2c.h"
#include "wit_c_sdk.h"
#include <stdio.h>

#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/imu.hpp>

using namespace std::chrono_literals;

#define TIMEOUT 3
#define RETRY 3

static int fd;

#define ACC_UPDATE 0x01
#define GYRO_UPDATE 0x02
#define ANGLE_UPDATE 0x04
#define MAG_UPDATE 0x08
#define READ_UPDATE 0x80
static volatile char s_cDataUpdate = 0;

static void AutoScanSensor(void);
static void CopeSensorData(uint32_t uiReg, uint32_t uiRegNum);
static void Delayms(uint16_t ucMs);
static int i2c_read(u8 addr, u8 reg, u8 *data, u32 len);
static int i2c_write(u8 addr, u8 reg, u8 *data, u32 len);

void euler_to_quat(float roll, float pitch, float yaw, float* q) 
{
    float cy = cos(yaw * 0.5);
    float sy = sin(yaw * 0.5);
    float cp = cos(pitch * 0.5);
    float sp = sin(pitch * 0.5);
    float cr = cos(roll * 0.5);
    float sr = sin(roll * 0.5);

    q[0] = cy * cp * cr + sy * sp * sr;
    q[1] = cy * cp * sr - sy * sp * cr;
    q[2] = sy * cp * sr + cy * sp * cr;
    q[3] = sy * cp * cr - cy * sp * sr;
}

static int
__i2c_send(int fd, struct i2c_rdwr_ioctl_data *data)
{
	if (fd < 0)
		return -1;

	if (data == NULL)
		return -1;

	if (data->msgs == NULL || data->nmsgs == 0)
		return -1;
	
	return ioctl(fd, I2C_RDWR, (unsigned long)data) ;
}

static int
__i2c_set(int fd, unsigned int timeout, unsigned int retry)
{
	if (fd == 0 )
		return -1;

	ioctl(fd, I2C_TIMEOUT, timeout ? timeout : I2C_DEFAULT_TIMEOUT);

	ioctl(fd, I2C_RETRIES, retry ? retry : I2C_DEFAULT_RETRY);
	
	return 0;
}

int
i2c_read_data(u8 addr, u8 reg, u8 *val, u32 len)
{
	int i,ret = 0;

	struct i2c_rdwr_ioctl_data *data;

	if ((data = (struct i2c_rdwr_ioctl_data *)malloc(sizeof(struct i2c_rdwr_ioctl_data))) == 
NULL)
	return -1;

	data->nmsgs = 2;
	if ((data->msgs = (struct i2c_msg *)malloc(data->nmsgs * sizeof(struct i2c_msg))) == 
NULL) {
		ret = -1;
		goto errexit3;
	}
	if ((data->msgs[0].buf = (unsigned char *)malloc(sizeof(unsigned char))) == NULL) {
		ret = -1;
		goto errexit2;
	}
	if ((data->msgs[1].buf = (unsigned char *)malloc(sizeof(unsigned char))) == NULL) {
		ret = -1;
		goto errexit1;
	}

	data->msgs[0].addr = addr;
	data->msgs[0].flags = 0;
	data->msgs[0].len = 1;
	data->msgs[0].buf[0] = reg;

	data->msgs[1].addr = addr;
	data->msgs[1].flags = I2C_M_RD;
	data->msgs[1].len = len;
	data->msgs[1].buf[0] = 0;

	if ((ret = __i2c_send(fd, data)) < 0)
		goto errexit0;

	for(i = 0 ;i < data->msgs[1].len; i++)
		val[i] = data->msgs[1].buf[i];

errexit0:
	free(data->msgs[1].buf);
errexit1:
	free(data->msgs[0].buf);
errexit2:
	free(data->msgs);
errexit3:
	free(data);

	return ret;
}

int
i2c_write_data(u8 addr, u8 reg, u8 *val, u32 len)
{
	int ret = 0, i;

	struct i2c_rdwr_ioctl_data *data;

	if ((data = (struct i2c_rdwr_ioctl_data *)malloc(sizeof(struct i2c_rdwr_ioctl_data))) == 
NULL)
		return -1;

	data->nmsgs = 1;
	if ((data->msgs = (struct i2c_msg *)malloc(data->nmsgs * sizeof(struct i2c_msg))) == NULL) 
{
		ret = -1;
		goto errexit2;
	}
	if ((data->msgs[0].buf = (unsigned char *)malloc((len+1) * sizeof(unsigned char))) == NULL) {
		ret = -1;
		goto errexit1;
	}

	data->msgs[0].addr = addr;
	data->msgs[0].flags = 0;
	data->msgs[0].len = 2;
	data->msgs[0].buf[0] = reg;
	for(i = 0; i < len; i++)
	{
		data->msgs[0].buf[1+i] = val[i];
	}

	if ((ret = __i2c_send(fd, data)) < 0)
		goto errexit0;

errexit0:
	free(data->msgs[0].buf);
errexit1:
	free(data->msgs);
errexit2:
	free(data);

	return ret;
}

int
i2c_open(const char* dev, unsigned int timeout, unsigned int retry)
{
	if ((fd = open(dev, O_RDWR)) < 0)
		return fd;
	
	__i2c_set(fd, timeout, retry);

	return fd;
}

void
i2c_close(int fd)
{
	if (fd < 0)
		return;

	close(fd);
}


class MinimalPublisher : public rclcpp::Node
{
public:
  MinimalPublisher()
      : Node("minimal_publisher"), count_(0)
  {
    is_cal_ = false;
    publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("imu", 10);
    timer_ = this->create_wall_timer(
        20ms, std::bind(&MinimalPublisher::timer_callback, this));
    const char dev[20] = "/dev/i2c-1";
    fd = i2c_open(dev, 3, 3);
    WitInit(WIT_PROTOCOL_I2C, 0x50);
    WitI2cFuncRegister(i2c_write, i2c_read);
    WitRegisterCallBack(CopeSensorData);
    WitDelayMsRegister(Delayms);
    AutoScanSensor();
  }

private:
  float x_cal_;
  float y_cal_;
  float z_cal_;
  bool is_cal_;

  void timer_callback()
  {
    float fAcc[3], fGyro[3], fAngle[3];
    int i;
    WitReadReg(AX, 12);
    if (s_cDataUpdate)
    {
      for (i = 0; i < 3; i++)
      {
        fAcc[i] = sReg[AX + i] / 32768.0f * 16.0f;
        fGyro[i] = sReg[GX + i] / 32768.0f * 2000.0f;
        fAngle[i] = sReg[Roll + i] / 32768.0f * 180.0f;
      }
      if (s_cDataUpdate & ACC_UPDATE)
      {
        s_cDataUpdate &= ~ACC_UPDATE;
      }
      if (s_cDataUpdate & GYRO_UPDATE)
      {
        s_cDataUpdate &= ~GYRO_UPDATE;
      }
      if (s_cDataUpdate & ANGLE_UPDATE)
      {
        s_cDataUpdate &= ~ANGLE_UPDATE;
      }

      sensor_msgs::msg::Imu msg;
      msg.header.stamp = this->get_clock()->now();
      msg.header.frame_id = "imu_link";

      if (!is_cal_) {
        x_cal_ = fAngle[0] * 0.01745329251;
        y_cal_ = fAngle[1] * 0.01745329251;
        z_cal_ = fAngle[2] * 0.01745329251;
        is_cal_ = true;
      }

      float q[4];
      euler_to_quat(
        (fAngle[0] * 0.01745329251) - x_cal_, 
        (fAngle[1] * 0.01745329251) - y_cal_, 
        (fAngle[2] * 0.01745329251) - z_cal_, 
        q
      );

      printf("z angle = %f\n", fAngle[2]);

      msg.orientation.x = q[1];
      msg.orientation.y = q[2];
      msg.orientation.z = q[3];
      msg.orientation.w = q[0];
      msg.orientation_covariance = {
        0.1, 0.0, 0.0,
        0.0, 0.1, 0.0,
        0.0, 0.0, 0.1
      };

      msg.angular_velocity.x = fGyro[0] * 0.01745329251;
      msg.angular_velocity.y = fGyro[1] * 0.01745329251;
      msg.angular_velocity.z = fGyro[2] * 0.01745329251;
      msg.angular_velocity_covariance = {
        0.1, 0.0, 0.0,
        0.0, 0.1, 0.0,
        0.0, 0.0, 0.1
      };

      msg.linear_acceleration.x = fAcc[0] * 9.80665;
      msg.linear_acceleration.y = fAcc[1] * 9.80665;
      msg.linear_acceleration.z = fAcc[2] * 9.80665;
      msg.linear_acceleration_covariance = {
        0.1, 0.0, 0.0,
        0.0, 0.1, 0.0,
        0.0, 0.0, 0.1
      };

      this->publisher_->publish(msg);
    }
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
  size_t count_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}

static int i2c_read(u8 addr, u8 reg, u8 *data, u32 len)
{
  if (i2c_read_data(addr >> 1, reg, data, len) < 0)
    return 0;
  return 1;
}
static int i2c_write(u8 addr, u8 reg, u8 *data, u32 len)
{
  if (i2c_write_data(addr >> 1, reg, data, len) < 0)
    return 0;
  return 1;
}
static void CopeSensorData(uint32_t uiReg, uint32_t uiRegNum)
{
  int i;
  for (i = 0; i < uiRegNum; i++)
  {
    switch (uiReg)
    {
      //            case AX:
      //            case AY:
    case AZ:
      s_cDataUpdate |= ACC_UPDATE;
      break;
      //            case GX:
      //            case GY:
    case GZ:
      s_cDataUpdate |= GYRO_UPDATE;
      break;
      //            case HX:
      //            case HY:
    case HZ:
      s_cDataUpdate |= MAG_UPDATE;
      break;
      //            case Roll:
      //            case Pitch:
    case Yaw:
      s_cDataUpdate |= ANGLE_UPDATE;
      break;
    default:
      s_cDataUpdate |= READ_UPDATE;
      break;
    }
    uiReg++;
  }
}

static void Delayms(uint16_t ucMs)
{
  usleep(ucMs * 1000);
}

static void AutoScanSensor(void)
{
  int i, iRetry;

  for (i = 0; i < 0x7F; i++)
  {
    WitInit(WIT_PROTOCOL_I2C, i);
    iRetry = 2;
    do
    {
      s_cDataUpdate = 0;
      WitReadReg(AX, 3);
      usleep(5);
      if (s_cDataUpdate != 0)
      {
        printf("find %02X addr sensor\r\n", i);
        return;
      }
      iRetry--;
    } while (iRetry);
  }
  printf("can not find sensor\r\n");
  printf("please check your connection\r\n");
}
