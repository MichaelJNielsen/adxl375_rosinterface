#include <iostream>

int main()
{
    std::cout << "Hello World" << std::endl;
    int i = 0;
    while (true)
    {
        i++;
        std::cout << i << std::endl;
        if (i % 2 == 0)
        {
            std::cout << "if" << std::endl;
        }
    }
    return 0;
}
