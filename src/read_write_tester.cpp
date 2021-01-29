#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <iostream>
//#include <cstring>

int main() {
   
// WRITE    
    unsigned int reg = 3;//0x2D;
    printf("register: %d\n", reg);
    
    unsigned char bytes[10] = {0};
    bytes[0] = 0b00101111;
    bytes[1] = 0x2D;
    bytes[2] = 0b10001110;

    int const out {open("test_writefile.txt", O_WRONLY)};
    if (out == -1) {
        std::cout << "Cannot open write file" << std::endl;
        return 1;
    }
    std::cout << "Write file opened" << std::endl;
    
    ssize_t const w { write(out, bytes, sizeof(bytes))}; 
    if (w!=sizeof(bytes)) {
        std::cout << "Could not write full array" << std::endl;
        return 1;
    }
    if (w<0) {
        std::cout << "Write error" << std::endl;
        return 1;
    }
    close (out);
    std::cout << "Written" << std::endl; 
    
//READ
    int const in {open("test_writefile.txt", O_RDONLY)};
    if (in == -1) {
        std::cout << "Cannot open read file" << std::endl;
        return 1;
    }
    std::cout << "Read file opened" << std::endl;
    
    unsigned char a[10] = {0};
    //ssize_t const r { read(in, a, sizeof(a))};
    ssize_t const r { read(in, a, sizeof(a))};
    if (r!=sizeof(a)) {
        std::cout << "Could not read full array" << std::endl;
        printf("r = %ld", r);
        printf("\n");
        //return 1;
    }
    if (r<0) {
        std::cout << "Read error" << std::endl;
        return 1;
    }
    close (in);
    
    for(unsigned int i(0); i<sizeof(a); ++i) {
        printf("%d ", a[i]);
    }
    printf("\n");

    return 0;
}
















