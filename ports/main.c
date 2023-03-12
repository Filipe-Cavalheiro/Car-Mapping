#include <errno.h>
#include <fcntl.h> 
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <stdio.h>
#include <stdint.h>

#define BAUDRATE B9600
#define PORTNAME "/dev/ttyACM0"

int set_interface_attribs (int fd);

int main(){
    int fd = open (PORTNAME, O_RDWR | O_NOCTTY | O_SYNC );
    if (fd < 0){
        fprintf(stderr, "error %d opening %s: %s", errno, PORTNAME, strerror (errno));
        return -1;
    }

    set_interface_attribs (fd);    // set speed to 9600 bps, 8N1 (no parity)

    char buf [128];
    uint8_t bytes_read;

    //soft restart the atmega328p
    char kickstart = '1';
    write(fd, &kickstart, sizeof(kickstart));

    while(1){
        memset(buf, '\0', sizeof(buf)); 
        bytes_read = read(fd, buf, sizeof buf);
        printf("N bytes: %d Output: %s", bytes_read, buf);
        /*
        for(int i = 0; i < 4; ++i)
            printf("%c", buf[i]);
        printf("\n");
        */

    }
    return 0;
}

int set_interface_attribs (int fd){
    struct termios settings;
    if (tcgetattr (fd, &settings)){
        fprintf(stderr, "error %d from tcgetattr", errno);
        return -1;
    }
    cfsetospeed (&settings, BAUDRATE);
    cfsetispeed (&settings, BAUDRATE);

    settings.c_cflag |= CS8;     // 8-bit chars
    settings.c_cflag &= ~CSIZE; // disable IGNBRK for mismatched speed tests; otherwise receive break
    settings.c_cflag |= (CLOCAL | CREAD);       // ignore modem controls, enable reading
    settings.c_cflag &= ~(PARENB | PARODD);     // no parity
    settings.c_cflag &= ~CSTOPB;                 //2 stop bits
    settings.c_cflag &= ~CRTSCTS;               //no hardware flow control

    settings.c_iflag &= ~IGNBRK;                // disable break processing
    settings.c_iflag &= ~(IXON | IXOFF | IXANY); // no Software flow control

    settings.c_lflag = 0;           // no signaling chars, no echo, no canonical processing

    settings.c_cc[VMIN]  = 5;       // 4 chars to read block
    settings.c_cc[VTIME] = 0;       // 0.0 seconds read timeout

    settings.c_oflag = 0;           // no remapping, no delays


    if (tcsetattr (fd, TCSANOW, &settings)){
        fprintf(stderr, "error %d from tcsetattr", errno);
        return -1;
    }
    return 0;
}
