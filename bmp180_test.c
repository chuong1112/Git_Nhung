#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>

#define DEVICE_FILE "/dev/bmp180"

// IOCTL commands for BMP180
#define BMP180_IOCTL_MAGIC 'b'
#define BMP180_IOCTL_READ_TEMP _IOR(BMP180_IOCTL_MAGIC, 1, int)
#define BMP180_IOCTL_READ_PRESS _IOR(BMP180_IOCTL_MAGIC, 2, int)

int main() {
    int fd;
    long signed int temp;
    long unsigned int press;

    // Open the device file
    fd = open(DEVICE_FILE, O_RDONLY);
    if (fd == -1) {
        perror("Failed to open the device file");
        return EXIT_FAILURE;
    }

    while(1){
        // Read temperature
        if (ioctl(fd, BMP180_IOCTL_READ_TEMP, &temp) == -1) {
            perror("Failed to read temperature");
            close(fd);
            return EXIT_FAILURE;
        }
        printf("Temperature: %ld.%02ld °C\n", temp / 10, temp % 10);

        // Read pressure
        if (ioctl(fd, BMP180_IOCTL_READ_PRESS, &press) == -1) {
            perror("Failed to read pressure");
            close(fd);
            return EXIT_FAILURE;
        }
        printf("Pressure: %ld hPa\n", press / 100); // BMP180 pressure is in Pa, so divide by 100 for hPa

        sleep(2);
    }
    
    // Close the device file
    close(fd);
    return EXIT_SUCCESS;
}
