//#define ROS
//#define CAM_NEW_ON

#ifdef ROS
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sys/wait.h>
#include <termio.h>
#include <unistd.h>
#include "std_msgs/Int16.h"

#endif

#include <iostream>

#ifndef ROS
#include <opencv2/opencv.hpp>
#endif

#ifdef ROS
#include <cv.hpp>
#include <iostream>
#include <stdio.h>
#include <fcntl.h>
#include <time.h>
#endif
//#define CV_PI 3.141592


using namespace cv;
using namespace std;

int main(){

#ifdef ROS
VideoCapture ppp(0);
#endif


#ifndef ROS
VideoCapture ppp("anti_video.mp4");
#endif
Mat Distort;
Mat img;
Mat canny_t;
Mat Hsv;
Mat _hsv[3];
Mat bin;
Mat Gray;
Mat whiteboard;

Mat Test1;

//namedWindow("uDistort");

namedWindow("ssss");
moveWindow("ssss", 370, 0);

namedWindow("canny_t");
moveWindow("canny_t", 0, 400);

//namedWindow("HSV_Detect");
//moveWindow("HSV_Detect", 370, 300);

//namedWindow("Bin");
//moveWindow("Bin", 600, 0);

//namedWindow("Gray");
//moveWindow("Gray", 700, 300);

namedWindow("Test1");

double canny_t1 = 80;
double canny_t2 = 180;
double gaussian_sgm = 2.0;

//왜곡보정
double dCamMat[]={ 336.922391, 0, 327.279999, 0, 336.831463, 227.170825, 0, 0, 1};
double dDstMat[]={ -0.316110, 0.078925, -0.006606, 0.002313};
Mat matCameraMatrix = Mat( 3, 3, CV_64FC1, (void*)dCamMat);
Mat matDistortionMatrix = Mat(1, 4, CV_64FC1, (void*)dDstMat);

//픽셀 추출
int pixel[320] = {0, };
//test
int pixel_test[320] = {0, };


//line
vector<Vec4i> lines;

//line 따로 저장
//vector<Vec4i> n_line[3];

//ROS
#ifdef ROS
ros::init(argc,argv,"cam_msg_publisher");
ros::NodeHandle nh;
std_msgs::Int16 cam_msg;
ros::Publisher pub = nh.advertise<std_msgs::Int16>("cam_msg" , 100);
ros::Rate loop_rate(50);
cout <<"start" <<endl;
#endif




while(true)
{
//label:
ppp >> img;
ppp >> Distort;
if(img.empty()==true)
break;

//srand(time(NULL));



//가우시안 필터
GaussianBlur(img, img, Size(3,3), gaussian_sgm);

//왜곡 보정
undistort(img, Distort, matCameraMatrix, matDistortionMatrix); 

//겉 오차 제거
/*for (int i=0; i<290; i++){
	Distort.at<uchar>(i,0)= 0;
	Distort.at<uchar>(i,1)=0;
	Distort.at<uchar>(i,390) = 0;
	Distort.at<uchar>(i,389)=0;
}
*/
//HSV에서 S 검출
cvtColor(Distort, Hsv, COLOR_BGR2HSV);
cvtColor(img, Gray, COLOR_BGR2GRAY);
//Hsv = Imread(img, IMREAD_COLOR);
split(Hsv, _hsv);
//imshow("HSV_Detect", _hsv[2]);

//HSV의 V영역을 이진화
adaptiveThreshold(Gray, bin, 255, ADAPTIVE_THRESH_MEAN_C, THRESH_BINARY ,5, 9);
adaptiveThreshold(_hsv[2], Gray, 255, ADAPTIVE_THRESH_MEAN_C, THRESH_BINARY, 5, 8);
//imshow("Bin", bin);
//imshow("Gray", Gray);

//Warping 전의 이미지 상의 좌표 

vector<Point2f> corners(4);
corners[0]=Point2f(0, 166);
corners[1]=Point2f(400, 166);
corners[2]=Point2f(-350, 290); //279
corners[3]=Point2f(730, 290);

Size warpSize(200, 320);

Mat warpImg(warpSize, Gray.type());

//Warping 후의 좌표

vector<Point2f> warpCorners(4);
warpCorners[0]=Point2f(0, 0);
warpCorners[1]=Point2f(warpImg.cols, 0);
warpCorners[2]=Point2f(0, warpImg.rows);
warpCorners[3]=Point2f(warpImg.cols, warpImg.rows);

//Transformation Matrix 구하기
Mat trans=getPerspectiveTransform(corners, warpCorners);

//Warping
warpPerspective(Gray, warpImg, trans, warpSize, INTER_LINEAR, BORDER_REPLICATE, 2 );
//imshow("warpImg", warpImg);

//Canny
Canny(Distort, canny_t, canny_t1, canny_t2, 3, true);
//Canny Original
Canny(img, Test1, canny_t1, canny_t2, 3, true);

//Test
warpPerspective(canny_t, canny_t, trans, warpSize, INTER_LINEAR, BORDER_REPLICATE, 2);
warpPerspective(Test1, Test1, trans, warpSize, INTER_LINEAR, BORDER_REPLICATE, 2);
//imshow("Test1", Test1);

canny_t = canny_t | Test1;

//line
HoughLinesP(canny_t, lines, 1, CV_PI / 180, 30, 60, 10); //Test1

for(size_t i=0; i< lines.size(); i++){
	
	line(img, Point(lines[i][0], lines[i][1]), Point(lines[i][2], lines[i][3]), Scalar(rand()/155+10, rand()/135+100, rand()/105+132), 1);
} //라인 찾기
imshow("Test1", Test1);

//line slope
int slopes [4];	//0:left, 1: straight, 2: right
float slope;

//위에 올릴것
//vector<Vec4i>c_line[3];
float avg_slope[3];
//vector<Vec4i>l_line, r_line;
for(int i=0; i<lines.size(); i++)
{
	int x1 = lines[i][0];
	int y1 = lines[i][1];
	int x2 = lines[i][2];
	int y2 = lines[i][3];


	if(x1 == x2)
		slopes[1]++;
	else if (((33< x1) && (x1 < 167)) && ((33 < x2) && (x2 < 167))){
		slope = (y2-y1) / (float)(x2-x1);
		if (slope > 10.3 || slope < -10.3){	//직선
			line(img, Point(lines[i][0], lines[i][1]), Point(lines[i][2], lines[i][3]), Scalar(0, 255, 0) ,1);
			slopes[1] ++;
			avg_slope[3] = avg_slope[3] + slope;
			//c_line[1].push_back(lines[i]);	//선 저장
		}
		else if (slope <= 4 && slope >= -4){	//무시
			//line(img, Point(lines[i][0], lines[i][1]), Point(lines[i][2]), lines[i][3]), Scalar(100, 5, 255) ,1);
			slopes[3] ++;
		}
		else if (slope < 0){	//오른쪽
			line(img, Point(lines[i][0], lines[i][1]), Point(lines[i][2], lines[i][3]), Scalar(0, 0, 255) ,1);
			slopes[2]++;
			//c_line[2].push_back(lines[i]);
			avg_slope[2] = avg_slope[2] + slope;
		}
		else if (slope > 0){	//왼쪽
			line(img, Point(lines[i][0], lines[i][1]), Point(lines[i][2], lines[i][3]), Scalar(255, 0, 0) ,1);
			slopes[0]++;
			//c_line[0].push_back(lines[i]);
			avg_slope[0] = avg_slope[0] + slope;
		}
	     }
	
	//line(img, Point(lines[i][0], lines[i][1]), Point(lines[i][2], lines[i][3]), Scalar(100, 0,255), 1);
}


//제일 많은것이 현 주행 상태 : 0: 왼쪽 1: 직진, 2: 오른쪽
int status;
if( slopes[0] > slopes[1]){
	if(slopes[0] > slopes[2])
		status = 0;
	else
		status = 2;
}
else {
	if(slopes[1] > slopes[2])
		status = 1;
	else
		status = 2;
}


//평균 기울기 구하기
if(status != 1){
	//avg_slope[i] = (avg_slope[i] / (float) c_line[i].size()) //평균 기울기
	avg_slope[status] = avg_slope[status] / (float) slopes[status]; //평균 기울기
/*
	for(int i=0; i<c_line[status].size(); i++){	// 왼쪽 오른쪽 구분
		if(y1>avg_slope[i] * x1)	//오른쪽
			r_line.push_back(c_lines[i]);
		else
			l_line.push_back(c_lines[i]);
	}
	*/
}
else 
avg_slope[status] = 999;
/*
else {
	for(int i=0; i<c_line[1].size(); i++){
	if(x1 > 100)	//오른쪽, 100은 평균
		r_line.push_back(c_lines[i]);
	else
		l_line.push_back(c_lines[i]);
	}
}
float avg_lx;	//왼쪽 x값 평균
float avg_rx;	//오른쪽 x값 평균
float avg_ly;
float avg_ry;
for(int i=0; i<l_line.size(); i++){
	avg_lx = l_line[i][0] + l_line[i][2];
	avg_ly = l_line[i][1] + l+line[i][3];
}
for(int i=0; i<r_line.size(); i++){
	avg_rx = r_line[i][0] + r_line[i][2];
	avg_ry = r_line[i][1] + r_line[i][3];
}
avg_lx = avg_lx / (float) l_line.size();
avg_ly = avg_ly / (float) l_line.size();
avg_rx = avg_rx / (float) r_line.size();
avg_ry = avg_ry / (float) r_line.size();

avg_slope
int match_l = 0;	//선분 검출
int match_r = 0;
while(true){
	for(int i=0; i<10; i++){
	int j = rand() / l_line.size();
	if((l_line[j][0]
}
	int k = rand() / r_line.size();


}
}
*/
//평균 기울기 구함 : avg_slope[status]

//현재 주행 상태
circle(img, Point2f(50+20*status, 220-slopes[status] + 5), 2, Scalar(100, 100, 100), 2);

string text =std::to_string(avg_slope[status]);
Point myPoint;
myPoint.x=10;
myPoint.y=40;
int font = 2;
double font_scale = 1.2;

putText(img, text, myPoint, font, font_scale, Scalar(105,66,150));
//cout << avg_slope[status];
//표시
circle(img, Point2f(50+20*0, 220-slopes[0]+30), 1, Scalar(255,0,0), 2);
circle(img, Point2f(50+20*1, 220-slopes[1]+30), 1, Scalar(0,255,0), 2);
circle(img, Point2f(50+20*2, 220-slopes[2]+30), 1, Scalar(0,0,255), 2);
/*
int fr_no = 1;

//differ 값  추정
//경사가 높을수록 직진
int differ = 0;
//if(avg_slope[status] > 10)
//	avg_slope[status] = 0;
if(avg_slope[status] > 8)
	avg_slope[status] = 8;
if(avg_slope[status] < -8)
	avg_slope[status] = -8;
if (avg_slope[status] < 0)
	differ = 6.25 * avg_slope[status] + 50;
else
	differ = -6.25 * avg_slope[status] + 50;

//ROS
#ifdef ROS
cam_msg.data = differ;
pub.publish(cam_msg);
loop_rate.sleep()
#else
std::cout << fr_no <<":" << differ <<endl;
#endif
*/

for(int i=0; i<3; i++){
//circle(img, Point2f(10+ 20*i,270-slopes[i]+30), 1, Scalar(0,0,255), 2);
//초기화
slopes[i]=0;
}

//중앙선, 가로선
line(img, Point(100, 0), Point(100, 320), Scalar(255, 5, 255) ,1);
line(img, Point(0, 200), Point(160, 200), Scalar(255, 5, 255) ,1);
line(img, Point(200, 0), Point(200, 320), Scalar(255, 5, 255) ,1);


//카운트
int u_count = 0;
int d_count = 0;


//y축 흰 픽셀 갯수
for(int j=0; j<320; j++){
	for(int k=50; k<150; k++){
		if(canny_t.at<uchar>(j,k) > 0)
			pixel[j]++;
		//if(Test1.at<uchar>(j,k) > 0)
			//pixel_test[j]++;
				}
			}
for(int l=0; l<160; l++){
	if(pixel[l] > 16){
	line(img, Point(270-5*pixel[l],l), Point( 270-5*pixel[l+1],l+1), Scalar(0,255,255), 2, 8);
	circle(img,Point2f(270-5*pixel[l],l), 1, Scalar(0,255,0), 2);
	u_count ++;
}

for(int l=160; l<320; l++){
        if(pixel[l] > 16){
        line(img, Point(270-5*pixel[l],l), Point( 270-5*pixel[l+1],l+1), Scalar(0,255,255), 2, 8);
        circle(img,Point2f(270-5*pixel[l],l), 1, Scalar(0,255,0), 2);
	d_count --;
}
//circle(img, Point2f(100+ (pixel[l]-pixel_test[l]), l), 2, Scalar(10,200,10), 2);
}
//for(int i=0; i<corners.size(); i++)
//circle(img, corners[i], 3, Scalar(0, 255, 0), 3);


int fr_no = 1;

//differ 값  추정
//경사가 높을수록 직진
int differ = 0;
//if(avg_slope[status] > 10)
//      avg_slope[status] = 0;
if(avg_slope[status] > 8)
        avg_slope[status] = 8;
if(avg_slope[status] < -8)
        avg_slope[status] = -8;
if (avg_slope[status] < 0)
        differ = 6.25 * avg_slope[status] + 50;
else
        differ = -6.25 * avg_slope[status] + 50;

//stop sign

if((u_count >= 5) && (d_count >= 5))
	differ = 9999;







//ROS
#ifdef ROS
cam_msg.data = differ;
pub.publish(cam_msg);
loop_rate.sleep()
#else
std::cout << fr_no <<":" << differ <<endl;
#endif











for(int i=0; i < 320; i++){
pixel[i] = 0;
pixel_test[i]=0;
}
//Distort = (canny_t & Test1) | (canny_t ^ Test1) & (canny_t ^ _hsv[2]);
//Distort = (0.8* (canny_t | Test1) & (0.8 * (Test1 ^ canny_t)));
//imshow("uDistort", Distort);
imshow("ssss", img);
imshow("canny_t", canny_t);
while(true){
if(waitKey(10)== 'a'){
//while(true){
//if(waitKey(10) == 'b')
//break;
//}
//continue;
break;
}
}
}
}

