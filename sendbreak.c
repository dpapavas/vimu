#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>

int main(int argc,char** argv) {
    if (argc < 2) {
        printf("Usage: %s DEVICE [ DURATION ] \n", argv[0]);
    } else {
        int fd;

        fd = open(argv[1], O_RDWR | O_NONBLOCK);
        tcsendbreak(fd, argc < 3 ? 0 : atoi(argv[2]));
        close(fd);
    }

    return 0;
}
