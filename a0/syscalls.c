#include <stddef.h>
#include <sys/stat.h>


// https://sourceware.org/newlib/libc.html#Syscalls
// bare minimum ones to support
extern "C" {
    int _write(int fd, char* ptr, int len) {
        (void)fd;
        (void)ptr;
        (void)len;
        return -1;
    }

    int _read(int fd, char* ptr, int len) {
        (void)fd;
        (void)ptr;
        (void)len;
        return -1;
    }

    int _close(int fd) {
        (void)fd;
        return -1;
    }

    int _fstat(int fd, struct stat* st) {
        (void)fd;
        (void)st;
        return -1;
    }

    int _isatty(int fd) {
        (void)fd;
        return -1;
    }

    int _lseek(int fd, int ptr, int dir) {
        (void)fd;
        (void)ptr;
        (void)dir;
        return -1;
    }

    int _kill(int pid, int sig) {
        (void)pid;
        (void)sig;
        return -1;
    }

    int _getpid(void) {
        return -1;
    }

    void* _sbrk(ptrdiff_t incr) {
        (void)incr;
        return (void*)-1;
    }

    void _exit(int status) {
        (void)status;
        for (;;) {
        }
    }
}
