#include <iostream>

int global_int = 10;


void test_fcn(int pass_var)
{
    std::cout << pass_var << std::endl;
    return;
}

int read(int addr, int *pass_var)
{
    //pass_var = 5;
    std::cout << "pass_var: " << *pass_var << std::endl;
return 0;
}

int write(int fd, const void *buf, size_t count)
{
    printf("In buffer: %p", buf);
    printf("\n");
return 0;
}

void writer(unsigned char buf, int size){
    unsigned char outbuffer[1] = {0};
    outbuffer[0] = buf;
    printf("size of outbuffer: %ld \n", sizeof(outbuffer));
    for(int i=0; i<sizeof(outbuffer); i++) {
        printf("outbuffer[%d]: %d \n", i, outbuffer[i]);
    }
return;
}

int main()
{
    int ADXL375_POWER_CTL = 0x2D;
    unsigned char buffer[60] = {0b00001000,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,1,2,3,4,5,6,78,9,1,2,3,4,5,6,7,8,9,1,2,3,4,5,6,7,8,9,1,2,3};
    int pass_var = 10;
    std::cout << "Hello World" << std::endl;
    //write(0, buffer, 10);
    //read(10, &pass_var);
    printf("size of buffer: %ld", sizeof(buffer));
    printf("\n");
    //for(int i=0; i<sizeof(buffer); i++) {
    //    printf("buffer[%d]: %d \n", i, buffer[i]);
    //}
    writer(buffer[0], sizeof(buffer));
return 0;
}

































