import core;
#include <iostream>
int main()
{
	Core::Mat img("/data/Kitti/00/image_0/000000.png");
	std::cout << "Hello, Core: " << img.size << std::endl;
	img.show();
	Core::Wait(0);
	return 0;
}