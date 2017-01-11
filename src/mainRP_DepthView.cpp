#include "./RP_DepthView/RP_DepthView.h"

int main(int argc, char* argv[])
{
	
	
	std::cout << " Usage : robot_number [int] // voxelGridScaler [float] // tresholdFilter [float] // method [int=0]." << std::endl;
	
	int robot_number = 0;
	if(argc>1)
	{
		robot_number = atoi(argv[1]);
	}
	
	float voxelGridScaler = 0.25f;
	if( argc>2)
	{
		voxelGridScaler = atof(argv[2]);
	}
	
	float tresholdFilter = 0.3f;
	if( argc>3)
	{
		tresholdFilter = atof(argv[3]);
	}
	
	int method = 0;	//ordered standard clipping
	if( argc>4)
	{
		method = atoi(argv[4]);
	}
	
	
	ros::init(argc, argv,std::string("RP_DepthView_"+std::to_string(robot_number)).c_str() );

	RP_DepthView dv(robot_number, voxelGridScaler, tresholdFilter, method);
	
	ros::spin();
	
	return 0;
	
}

