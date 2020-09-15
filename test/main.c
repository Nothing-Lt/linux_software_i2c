#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <fcntl.h>

int main(int argc, char** argv)
{
    char buf[BUFSIZ];
    int fd = -1;
    off_t off;

    fd = open("/dev/soft_iic",O_RDWR);
    if(-1 == fd){
        printf("failed\r\n");
              return -1;
    }

    buf[0] = '1';
    buf[1] = '\0';

    off = atoi(argv[1]);
    off = (off << 16) | atoi(argv[2]);
    printf("%lx\n",off);
    lseek(fd,off,SEEK_SET);

    write(fd,buf,2);

    close(fd);

    printf("123");

    return 0;
}
