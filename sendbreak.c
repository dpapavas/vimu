#include <termios.h>
#include <fcntl.h>
#include <stdio.h>

int main(int argc,char** argv) {
    if (argc<2) {
        printf("Usage: %s DEVICE \n",argv[0]);
    } else {
        int tty_fd = open(argv[1], O_RDWR | O_NONBLOCK);
        tcsendbreak(tty_fd, 0);
    }

    return 0;
}
