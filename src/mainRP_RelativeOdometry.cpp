#include "./RP_RelativeOdometry/RP_RelativeOdometry.h"

int main(int argc, char* argv[])
{
	
	
	std::cout << " Usage : robot_number [int] // method [int=0] // verbose [int=0]." << std::endl;
	
	int robot_number = 0;
	if(argc>1)
	{
		robot_number = atoi(argv[1]);
	}
	
	int method = 0;	//ordered standard clipping
	if( argc>2)
	{
		method = atoi(argv[2]);
	}
	
	bool verbose = false;
	if( argc>3)
	{
		verbose = (atoi(argv[3])==1?true:false);
	}
	
	
	ros::init(argc, argv,std::string("RP_RelativeOdometry_"+std::to_string(robot_number)).c_str() );

	RP_RelativeOdometry ro(robot_number,method,verbose);
	
	ros::spin();
	
	return 0;
	
}

