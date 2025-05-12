#include <linux/module.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <linux/uaccess.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/kdev_t.h>
#include <linux/slab.h>
#include <linux/kernel.h>
#include <asm/div64.h>
#include <linux/uaccess.h>
#include <linux/delay.h>

// Định nghĩa tên thiết bị và lớp thiết bị
#define DEV_NAME "bmp180"
#define DEV_CLASS "bmp180_class"

// Cấu hình ban đầu cho BMP180
static int temperature_oversampling = 3;  // temperature_oversampling x8 (osrs_t[011])
static int pressure_oversampling = 3;     // pressure_oversampling x8 (osrs_p[011])
static int normal_mode = 0;               // normal mode = 00 (chế độ bình thường)

// Các thanh ghi của BMP180
#define BMP180_REG_TEMP_MSB 0xF6
#define BMP180_REG_PRESS_MSB 0xF6
#define BMP180_REG_CTRL_MEAS 0xF4
#define BMP180_REG_SOFT_RESET 0xE0

// Các lệnh IOCTL cho BMP180
#define BMP180_IOCTL_MAGIC 'b'
#define BMP180_IOCTL_READ_TEMP  _IOR(BMP180_IOCTL_MAGIC, 1, int)
#define BMP180_IOCTL_READ_PRESS _IOR(BMP180_IOCTL_MAGIC, 2, int)

// Khai báo adapter và client I2C
static struct i2c_adapter *bmp180_i2c_adapter = NULL;
static struct i2c_client *bmp180_i2c_client = NULL;

#define I2C_BUS_AVAILABLE 1        // Bus I2C khả dụng trên Raspberry Pi
#define SLAVE_DEVICE_NAME "BMP180" // Tên thiết bị và trình điều khiển
#define BMP180_SLAVE_ADDRESS 0x77  // Địa chỉ I2C của cảm biến BMP180

// Hàm ghi giá trị vào thanh ghi
static void write_register_value(struct i2c_client *client, u8 reg, u8 value) {
    i2c_smbus_write_byte_data(client, reg, value);
}

// Cấu hình trình điều khiển I2C
static struct i2c_driver bmp180_i2c_driver = {
    .driver = {
        .name = SLAVE_DEVICE_NAME,  // Sửa tên thiết bị nếu cần
        .owner = THIS_MODULE
    }
};

// Thông tin bảng I2C
static struct i2c_board_info bmp180_i2c_board_info = {
    I2C_BOARD_INFO(SLAVE_DEVICE_NAME, BMP180_SLAVE_ADDRESS)  // Cập nhật tên thiết bị và địa chỉ
};

// Khai báo số thiết bị, lớp thiết bị và cdev
dev_t dev_num = 0;
static struct class *dev_class;
static struct cdev bmp180_cdev;

// Khai báo các hàm thao tác với thiết bị
static int bmp180_open(struct inode *device_file, struct file *instance);
static int bmp180_close(struct inode *device_file, struct file *instance);

// Macro để nối hai byte
#define CONCAT_BYTES(msb, lsb) (((uint16_t)msb << 8) | (uint16_t)lsb)

// Các biến hiệu chỉnh, đọc khi khởi tạo

// Hiệu chỉnh nhiệt độ
unsigned short AC5;
unsigned short AC6;
signed short MC;
signed short MD;

// Hiệu chỉnh áp suất
signed short AC1;
signed short AC2;
signed short AC3;
unsigned short AC4;
signed short B1;
signed short B2;
signed short MB;

// Biến toàn cục t_fine, được tính trong read_temperature() và dùng trong read_pressure()
int t_fine;

// Hàm đọc áp suất từ BMP180 (đã hiệu chỉnh bằng t_fine), trả về đơn vị Pascal (Pa)
long read_pressure_bmp180(int oss)
{
    long up, p;
    long x1, x2, x3, b3, b6;
    unsigned long b4, b7;

    write_register_value(bmp180_i2c_client, BMP180_REG_CTRL_MEAS, 0x34 + (oss << 6));
    msleep(2 + (3 << oss));
    // Đọc giá trị áp suất chưa bù (UP) từ thanh ghi 0xF6, 0xF7, 0xF8
    u8 msb, lsb, xlsb;
    msb = (u8)(0xFF &i2c_smbus_read_byte_data(bmp180_i2c_client, 0xF6));
    lsb = (u8)(0xFF &i2c_smbus_read_byte_data(bmp180_i2c_client, 0xF7));
    xlsb = (u8)(0xFF &i2c_smbus_read_byte_data(bmp180_i2c_client, 0xF8));

    up = (((long)msb << 16) + ((long)lsb << 8) + (long)xlsb) >> (8 - oss);

    // Bắt đầu tính toán áp suất từ UP và các hệ số hiệu chỉnh
    b6 = t_fine - 4000;

    x1 = (B2 * ((b6 * b6) >> 12)) >> 11;
    x2 = (AC2 * b6) >> 11;
    x3 = x1 + x2;
    b3 = (((((long)AC1) * 4 + x3) << oss) + 2) >> 2;

    x1 = (AC3 * b6) >> 13;
    x2 = (B1 * ((b6 * b6) >> 12)) >> 16;
    x3 = ((x1 + x2) + 2) >> 2;
    b4 = (AC4 * (unsigned long)(x3 + 32768)) >> 15;
    b7 = ((unsigned long)(up - b3) * (50000 >> oss));

    if (b7 < 0x80000000)
        p = (b7 << 1) / b4;
    else
        p = (b7 / b4) << 1;

    x1 = (p >> 8) * (p >> 8);
    x1 = (x1 * 3038) >> 16;
    x2 = (-7357 * p) >> 16;
    p = p + ((x1 + x2 + 3791) >> 4);

    return p;  // đơn vị: Pascal (Pa)
}


// Hàm đọc nhiệt độ từ BMP180, trả về giá trị nhiệt độ x100 (ví dụ: 3001 = 30.01°C)
long read_temperature_bmp180(void)
{
    int ut;
    int x1, x2;
    long temp;

    write_register_value(bmp180_i2c_client, BMP180_REG_CTRL_MEAS, 0x2E);
    msleep(5);

    u8 msb = i2c_smbus_read_byte_data(bmp180_i2c_client, 0xF6);
    u8 lsb = i2c_smbus_read_byte_data(bmp180_i2c_client, 0xF7);
    ut = ((msb << 8) | lsb);

    x1 = ((ut - (int)AC6) * (int)AC5) >> 15;
    x2 = ((int)MC << 11) / (x1 + (int)MD);
    t_fine = x1 + x2;

    temp = (t_fine + 8) >> 4;
    return temp * 10;
}

// Hàm đọc các giá trị hiệu chuẩn từ BMP180
void read_calibration_bmp180(void)
{
    unsigned char data_read[22];  // BMP180 có 22 byte hiệu chuẩn
    int i;

    // Đọc 22 byte dữ liệu hiệu chuẩn từ địa chỉ 0xAA
    for (i = 0; i < 22; i++)
    {
        data_read[i] = i2c_smbus_read_byte_data(bmp180_i2c_client, 0xAA + i);
    }

    // Gán các giá trị hiệu chuẩn vào các biến toàn cục (hiệu chuẩn cho nhiệt độ và áp suất)
    AC1 = CONCAT_BYTES(data_read[0], data_read[1]);
    AC2 = CONCAT_BYTES(data_read[2], data_read[3]);
    AC3 = CONCAT_BYTES(data_read[4], data_read[5]);
    AC4 = CONCAT_BYTES(data_read[6], data_read[7]);
    AC5 = CONCAT_BYTES(data_read[8], data_read[9]);
    AC6 = CONCAT_BYTES(data_read[10], data_read[11]);
    
    B1 = CONCAT_BYTES(data_read[12], data_read[13]);
    B2 = CONCAT_BYTES(data_read[14], data_read[15]);
    
    MB = CONCAT_BYTES(data_read[16], data_read[17]);
    MC = CONCAT_BYTES(data_read[18], data_read[19]);
    MD = CONCAT_BYTES(data_read[20], data_read[21]);

    pr_info("bmp180: AC1 = %d, AC2 = %d, AC3 = %d, AC4 = %d, AC5 = %d, AC6 = %d\n",
            AC1, AC2, AC3, AC4, AC5, AC6);
    pr_info("bmp180: B1 = %d, B2 = %d, MB = %d, MC = %d, MD = %d\n",
            B1, B2, MB, MC, MD);
}


// Hàm mở thiết bị BMP180
static int bmp180_open(struct inode *device_file, struct file *instance)
{
    pr_info("bmp180: open was called\n");
    return 0;
}

// Hàm đóng thiết bị BMP180
static int bmp180_close(struct inode *device_file, struct file *instance)
{
    pr_info("bmp180: close was called\n");
    return 0;
}

// Hàm xử lý các lệnh IOCTL
static long bmp180_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    long signed int t = 0;
    long unsigned int p = 0;
    
    switch (cmd) {
    case BMP180_IOCTL_READ_TEMP:
        // Đọc nhiệt độ
        t = read_temperature_bmp180();  
        if (copy_to_user((long signed int *)arg, &t, sizeof(t)))
        {
            pr_err("bmp180: failed to copy temperature to user\n");
            return -EFAULT;
        }
        break;

    case BMP180_IOCTL_READ_PRESS:
        // Đọc áp suất từ BMP180
        p = read_pressure_bmp180(pressure_oversampling);  
        if (copy_to_user((long unsigned int *)arg, &p, sizeof(p)))
        {
            pr_err("bmp180: failed to copy pressure to user\n");
            return -EFAULT;
        }
        break;
    default:
        return -EINVAL;
    }
    return 0;
}

// Khai báo các thao tác tập tin cho thiết bị BMP180
static struct file_operations fops = {
    .owner = THIS_MODULE,
    .open = bmp180_open,        
    .release = bmp180_close,    
    .unlocked_ioctl = bmp180_ioctl,  
};


// Hàm khởi tạo module cho BMP180
static int init_driver(void)
{
  int ret = -1;
  u8 id;
  pr_info("bmp180: initializing...\n");

  // Cấp phát số major cho thiết bị
  if (alloc_chrdev_region(&dev_num, 0, 1, DEV_NAME) < 0)
  {
    pr_err("bmp180: could not allocate major number\n");
    return -1;
  }

  // Tạo lớp thiết bị
  dev_class = class_create(THIS_MODULE, DEV_CLASS);
  if (IS_ERR(dev_class))
  {
    pr_err("bmp180: failed to create class\n");
    goto ClassError;
  }

  // Tạo thiết bị
  if (IS_ERR(device_create(dev_class, NULL, dev_num, NULL, DEV_NAME)))
  {
    pr_err("bmp180: failed to create device\n");
    goto DeviceError;
  }

  // Khởi tạo cdev và thao tác file
  cdev_init(&bmp180_cdev, &fops);

  // Thêm thiết bị vào hệ thống
  if (cdev_add(&bmp180_cdev, dev_num, 1) == -1)
  {
    pr_err("bmp180: failed to add device");
    goto KernelError;
  }

  // Lấy adapter I2C khả dụng
  bmp180_i2c_adapter = i2c_get_adapter(I2C_BUS_AVAILABLE);
  if (bmp180_i2c_adapter != NULL)   // Kiểm tra xem adapter có tồn tại không
  {
    // Tạo client I2C mới với adapter vừa lấy và thông tin board
    bmp180_i2c_client = i2c_new_client_device(bmp180_i2c_adapter, &bmp180_i2c_board_info);
    if (bmp180_i2c_client != NULL)      // Kiểm tra xem client có tồn tại không
    {
        // Thêm driver I2C cho thiết bị BMP180
      int add_i2c = i2c_add_driver(&bmp180_i2c_driver);
      if (add_i2c != -1)    // Kiểm tra xem driver có được thêm thành công không
      {
        ret = 0;  // Nếu thành công, đặt giá trị trả về là 0
      }
      else
      {
        pr_err("bmp180: failed to add i2c driver \n");
      }
    }
    // Giải phóng adapter I2C
    i2c_put_adapter(bmp180_i2c_adapter);
  }

  // Đọc id của cảm biến BMP180
  id = i2c_smbus_read_byte_data(bmp180_i2c_client, 0xD0);
  pr_info(" id is 0x%x\n", id);

  // Cấu hình thanh ghi ctrl_meas tại địa chỉ 0xF4
  // Thiết lập temperature_oversampling cho nhiệt độ
  // Thiết lập pressure_oversampling áp suất 
// Thiết lập chế độ normal_mode
write_register_value(bmp180_i2c_client, 0xF4, (temperature_oversampling << 5) | (pressure_oversampling << 2) | normal_mode);

// Đọc và thiết lập các thanh ghi hiệu chỉnh
read_calibration_bmp180();

pr_info("bmp180: successfully init module\n");
return ret;

KernelError:
  device_destroy(dev_class, dev_num); // Hủy thiết bị nếu xảy ra lỗi kernel
DeviceError:
  class_destroy(dev_class);           // Hủy lớp thiết bị nếu xảy ra lỗi tạo thiết bị
ClassError:
  unregister_chrdev_region(dev_num, 1);// Hủy đăng ký số major nếu xảy ra lỗi tạo lớp
  return -1;
}


// Hàm kết thúc module
static void exit_driver(void)
{
  i2c_unregister_device(bmp180_i2c_client); // Hủy đăng ký thiết bị i2c
  i2c_del_driver(&bmp180_i2c_driver);       // Hủy driver i2c
  cdev_del(&bmp180_cdev);                   // Hủy char device
  device_destroy(dev_class, dev_num);       // Hủy thiết bị
  class_destroy(dev_class);                 // Hủy lớp thiết bị
  unregister_chrdev_region(dev_num, 1);     // Hủy đăng ký số major
  pr_info("bmp180: successfully removed module\n");
  return;
}

module_init(init_driver);
module_exit(exit_driver);

// Thông tin về module
MODULE_LICENSE("GPL");
MODULE_AUTHOR("CDT");
MODULE_DESCRIPTION("bmp180 sensor driver");

