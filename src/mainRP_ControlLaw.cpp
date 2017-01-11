#include "./RP_ControlLaw/RP_ControlLaw.h"

int main(int argc, char* argv[])
{
	
	
	std::cout << " Usage : tresholdDistAccount [float] // emergencyBreak [int=0] // tresholdDistFarEnough [float ] // tresholdDistPair [float] // ... // verbose [int=1] " << std::endl;
	
	int robot_number = 0;
	
	float tresholdDistAccount = 1.0f;
	if(argc>1)
	{
		tresholdDistAccount = atof(argv[1]);
	}
	
	bool emergencyBreak = false;
	if(argc>2)
	{
		emergencyBreak = (atoi(argv[2])==1?true:false);
	}
	
	float tresholdDistFarEnough = 5.0f;
	if(argc>3)
	{
		tresholdDistFarEnough = atof(argv[3]);
	}
	
	float tresholdDistPair = 0.1f;
	if(argc>4)
	{
		tresholdDistPair = atof(argv[4]);
	}
	
	
	float Pang=0.1f;//1.0f;
	if(argc>5)
	{
		Pang = atof(argv[5]);
	}
	
	float Iang=0.0f;
	if(argc>6)
	{
		Iang = atof(argv[6]);
	}
	
	
	float Plin=0.1f;
	if(argc>7)
	{
		Plin = atof(argv[7]);
	}
	
	float Ilin=0.0f;
	if(argc>8)
	{
		Ilin = atof(argv[8]);
	}
	
	bool verbose = true;
	if(argc>9)
	{
		verbose = (atoi(argv[9])==1?true:false);
	}
	
	
	
	ros::init(argc, argv,std::string("RP_ControlLaw_"+std::to_string(robot_number)).c_str() );

	RP_ControlLaw cl(robot_number, emergencyBreak, tresholdDistAccount, tresholdDistFarEnough, tresholdDistPair, Pang, Iang, Plin, Ilin, verbose);
	
	ros::spin();
	
	return 0;
	
}

