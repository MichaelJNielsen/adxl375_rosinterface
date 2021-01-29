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
    std::cout << "in write with buffer: " << buf << std::endl;
return 0;
}


int main()
{
    int ADXL375_POWER_CTL = 0x2D;
    unsigned char buffer[60] = {0};
    int pass_var = 10;
    std::cout << "Hello World" << std::endl;
    write(0, buffer, 10);
    //read(10, &pass_var);
return 0;
}

































