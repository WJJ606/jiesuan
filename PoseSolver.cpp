#include "PoseSolver.hpp"

using namespace cv;
using namespace std;

PoseSolver::PoseSolver(const char* filePath, int camId) 
{
	FileStorage fsRead;
	fsRead.open(filePath, FileStorage::READ);
	if (!fsRead.isOpened())
	{
		cout << "Failed to open xml" << endl;
	}

	//fsRead["Y_DISTANCE_BETWEEN_GUN_AND_CAM"] >> GUN_CAM_DISTANCE_Y;

	Mat cameraMatrix;
	Mat distortionCoeffs;
	switch (camId)
	{
	case 1:
		fsRead["CAMERA_MATRIX_1"] >> cameraMatrix;
		fsRead["DISTORTION_COEFF_1"] >> distortionCoeffs;
		break;
	default:
		cout << "WRONG CAMID GIVEN!" << endl;
		break;
	}
	setCameraParams(cameraMatrix, distortionCoeffs);
	setObjPoints(smallArmor,135,55);
	setObjPoints(bigArmor,230,55);
	fsRead.release();
}

PoseSolver::~PoseSolver(void)
{

}

void PoseSolver::setCameraParams(const Mat& camMatrix, const Mat& distCoeffs)
{
	instantMatrix = camMatrix;
	distortionCoeffs = distCoeffs;
}

int PoseSolver::readFile(const char* filePath, int camId)
{
	FileStorage fsRead;
	fsRead.open(filePath, FileStorage::READ);
	if (!fsRead.isOpened())
	{
		cout << "Failed to open xml" << endl;
		return -1;
	}

	//fsRead["Y_DISTANCE_BETWEEN_GUN_AND_CAM"] >> GUN_CAM_DISTANCE_Y;

	Mat cameraMatrix;
	Mat distortionCoeffs;
	switch (camId)
	{
	case 1:
		fsRead["CAMERA_MATRIX_1"] >> cameraMatrix;
		fsRead["DISTORTION_COEFF_1"] >> distortionCoeffs;
		break;
	default:
		cout << "WRONG CAMID GIVEN!" << endl;
		break;
	}
	setCameraParams(cameraMatrix, distortionCoeffs);
	fsRead.release();
	return 0;
}

void PoseSolver::setObjPoints(ArmorType type, double width, double height)
{
	double centerX = width / 2.0;
	double centerY = height / 2.0;
	switch (type)
	{
	case smallArmor:
	    smallObjPoints.push_back(Point3f(-centerX, centerY, 0));   //tl top left左上
		smallObjPoints.push_back(Point3f(-centerX, -centerY, 0));  //bl below left左下
		smallObjPoints.push_back(Point3f(centerX, -centerY, 0));   //br below right右下
		smallObjPoints.push_back(Point3f(centerX, centerY, 0));	//tr top right右上
		break;

	case bigArmor:
	    bigObjPoints.push_back(Point3f(-centerX, centerY, 0));   //tl top left左上
		bigObjPoints.push_back(Point3f(-centerX, -centerY, 0));  //br below right左下
		bigObjPoints.push_back(Point3f(centerX, -centerY, 0));   //bl below left右下
		bigObjPoints.push_back(Point3f(centerX, centerY, 0));    //tr top right右上
		break;
	default: break;
	}
}

void PoseSolver::getImgpPoints(std::vector<Point2f> image_points)
{
	imagePoints.clear();
	for(int i=0;i<4;i++)
	{
		imagePoints.emplace_back(image_points[i]);
		//cout<<endl<<imagePoints[i]<<endl;
	}

}
Eigen::Vector3d PredictorKalman::pnp_get_pc(const cv::Point2f p[4], int armor_number) {
    static const std::vector<cv::Point3d> pw_small = {  // 单位：m
            {-0.066, 0.027,  0.},
            {-0.066, -0.027, 0.},
            {0.066,  -0.027, 0.},
            {0.066,  0.027,  0.}
    };
    static const std::vector<cv::Point3d> pw_big = {    // 单位：m
            {-0.115, 0.029,  0.},
            {-0.115, -0.029, 0.},
            {0.115,  -0.029, 0.},
            {0.115,  0.029,  0.}
    };













Eigen::Vector3f PoseSolver::solvePose(int armorType)
{	
	
	switch (armorType)
	{
	case smallArmor:

        solvePnP(smallObjPoints, imagePoints, instantMatrix, distortionCoeffs, rvec, tvec, false, SOLVEPNP_ITERATIVE);
		cout<<"=============小装甲板============"<<endl;
		break;
	case bigArmor:
		solveaPnP(bigObjPoints, imagePoints, instantMatrix, distortionCoeffs, rvec, tvec, false, SOLVEPNP_ITERATIVE);
		cout<<"=============大装甲板============"<<endl;
		break;
	default:
		break;
	}

    float x_pos= static_cast<float>(tvec.at<double>(0, 0)) + camera2gun_offest.x;
    float y_pos = static_cast<float>(tvec.at<double>(1, 0)) + camera2gun_offest.y;
    float z_pos = static_cast<float>(tvec.at<double>(2, 0)) + camera2gun_offest.z;


    float tan_pitch = y_pos / sqrt(x_pos * x_pos + z_pos * z_pos);
    float tan_yaw = x_pos / z_pos;
    float distance=sqrt(x_pos * x_pos + y_pos * y_pos + z_pos * z_pos);
    fps=0.001;
    float speed=distance/fps;//fps
    double a = 9.8 * 9.8 * 0.25;
    double b = speed*speed-distance*9.8*cos(M_PI_2+atan(z_pos,distance));
    double c = distance * distance;
    // 带入求根公式，解出t^2
    double t_2 = (- sqrt(b * b - 4 * a * c) - b) / (2 * a);

    // 解出抬枪高度，即子弹下坠高度
    double height = 0.5 * 9.8 * t_2;

    Eigen::Vector3f result{x_pos,y_pos, z_pos + height}; // 抬枪后预测点

	return result;
}


Eigen::Vector2f PoseSolver::solvePose(int armorType,Eigen::Quaternionf q)
{
	switch (armorType)
	{
	case smallArmor:
		solvePnP(smallObjPoints, imagePoints, instantMatrix, distortionCoeffs, rvec, tvec, false, SOLVEPNP_ITERATIVE); 
		cout<<"=============小装甲板============"<<endl;
		break;
	case bigArmor:
		solvePnP(bigObjPoints, imagePoints, instantMatrix, distortionCoeffs, rvec, tvec, false, SOLVEPNP_ITERATIVE); 
		cout<<"=============大装甲板============"<<endl;
		break;
	default:
		break;
	}

	cv::Point3f earth_coord(camera2earth(q,tvec));
	//std::cout<<"==============earth_coord================="<<endl<<earth_coord<<endl;

	cv::Point3f camera_coord(earth2camera(q,earth_coord));

	std::cout<<"==============camera_coord================="<<endl<<camera_coord<<endl;
	float tan_pitch=earth_coord.z/sqrt(earth_coord.x*earth_coord.x + earth_coord.z*earth_coord.z);
	float tan_yaw=earth_coord.x/earth_coord.z;

	Eigen::Vector2f result(-atan(tan_pitch) * 180 / CV_PI,atan(tan_yaw) * 180 / CV_PI);

	return result;

}



cv::Point3f PoseSolver::getCameraPose()
{
	camera_coord = cv::Point3f(tvec.at<double>(0, 0),tvec.at<double>(1, 0),tvec.at<double>(2, 0));
	return camera_coord;
}

void PoseSolver::showPredict(cv::Mat image2show, cv::Mat predict_coord)
{

    cv::Point2f pixel_coord;

	pixel_coord=cv::Point2f(predict_coord.at<double>(0,0),predict_coord.at<double>(1,0));
	
	cout<<"=================pixel_coord=================="<<endl<<pixel_coord<<endl;

	cv::circle(image2show, pixel_coord, 10, cv::Scalar(0, 255, 255), 5);


 }



cv::Mat PoseSolver::camera2pixel(cv::Point3f camera_coord)
{
	cv::Mat camera_coord_mat;
	
	camera_coord_mat = (cv::Mat_<double>(3, 1) << camera_coord.x,
                                                  camera_coord.y,
                                                  camera_coord.z);													 

	return instantMatrix * camera_coord_mat / camera_coord.z;

}

cv::Point3f PoseSolver::camera2earth(Eigen::Quaternionf q, cv::Point3f camera_coord)
{
	camera_coord += camera2imu_offest;
    Eigen::Quaternionf p(0, camera_coord.z, -camera_coord.x, -camera_coord.y);

    Eigen::Quaternionf result = q * p *q.inverse();
    return cv::Point3f(result.x(), result.y(), result.z());
}

cv::Point3f PoseSolver::earth2camera(Eigen::Quaternionf q, cv::Point3f earth_coord)
{
    Eigen::Quaternionf p(0, earth_coord.x, earth_coord.y, earth_coord.z);

    Eigen::Quaternionf result = q.inverse() * p * q;
	
	cv::Point3f sss(-result.y(),-result.z(),result.x());
	std::cout<<"-------------------------------"<<sss<<endl;
    
	return cv::Point3f(-result.y(), -result.z(), result.x()) - camera2imu_offest;
}

cv::Mat PoseSolver::earth2pixel(Eigen::Quaternionf q1, cv::Point3f earth_coord)
{
	cv::Mat camera_coord_mat;
	
	Eigen::Quaternionf p(0, earth_coord.x, earth_coord.y, earth_coord.z);

    Eigen::Quaternionf camera_coord_temp = q1.inverse() * p * q1;

	camera_coord_mat = (cv::Mat_<double>(3, 1) << camera_coord_temp.x(),
                                                  camera_coord_temp.y(),
                                                  camera_coord_temp.z());

	return instantMatrix * camera_coord_mat / camera_coord_temp.z();

}

cv::Point3f PoseSolver::camera2earth(Eigen::Quaternionf q, cv::Mat tvec)
{	
	std::cout<<"==============tvec==============="<<endl<<tvec<<endl;
	cv::Point3f camera_coord(static_cast<float>(tvec.at<double>(0,0)),
							 static_cast<float>(tvec.at<double>(1,0)),
							 static_cast<float>(tvec.at<double>(2,0)));
	
	camera_coord += camera2imu_offest;
	
	//
	cv::Point3f imu_coord(camera_coord.x, camera_coord.z, -camera_coord.y);
	
	std::cout<<"==============imu_coord_mat==============="<<endl<<imu_coord<<endl;
    
	Eigen::Quaternionf p(0, camera_coord.z, -camera_coord.x, -camera_coord.y);

    Eigen::Quaternionf result = q * p *q.inverse();

	cv::Point3f xxx(result.x(),result.y(),result.z());
	std::cout<<"==============earth_coord==============="<<endl<<xxx<<endl;

    return cv::Point3f(result.x(), result.y(), result.z());
}

