#include <iostream>

int global_int = 10;

void test_fcn(int pass_var)
{
    std::cout << pass_var << std::endl;
    return;
}

int main()
{
    std::cout << "Hello World" << std::endl;
    int i = 0;
    while (true)
    {
        i++;
        //std::cout << i << std::endl;
        if (i % 2 == 0)
        {
            test_fcn(global_int);
        }
    }
    return 0;
}


