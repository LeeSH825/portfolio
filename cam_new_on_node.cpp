#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sys/wait.h>
#include <termio.h>
#include <unistd.h>
#include "std_msgs/Int16.h"
#include <cv.hpp>

#include <iostream>
#include <cmath>
#include "opencv2/opencv.hpp"
//#define __CV4
//#define __START //print start line
//#define __DIFF //print diff
//#define __HSV

using namespace std;
using namespace cv;

#if !defined(__CV4)
#define BGR2HSV CV_BGR2HSV
#define HSV2RGB CV_HSV2RGB
#define BGR2GRAY CV_BGR2GRAY
#else
#define BGR2HSV COLOR_BGR2HSV
#define HSV2RGB COLOR_HSV2RGB
#define BGR2GRAY COLOR_BGR2GRAY
#endif

double line_ingradient(int x1, int y1, int x2, int y2)
{
  return (double)(y1 - y2) / (x1 - x2);
}

void init_windows(double width, double height, double roi_height)
{

  namedWindow("original");

  namedWindow("white");
  moveWindow("white", 0, height * 1.4);

  namedWindow("gray");
  moveWindow("gray", width * 1.2, height * 1.4);

  namedWindow("canny_t");
  moveWindow("canny_t", width * 1.2 * 2, height * 1.4);

  namedWindow("binary");
  moveWindow("binary", width * 1.2, 0);
}

int main(int argc, char *argv[])
{
  //VideoCapture cap("/home/cseecar/catkin_ws/video/clockwise.mp4");
  VideoCapture cap(0);
  Mat frame, gray, gaussian, canny_t, hsv, binary;

  // ROS
  ros::init(argc,argv,"cam_msg_publisher");
  ros::NodeHandle nh;
  std_msgs::Int16 cam_msg;
  ros::Publisher pub = nh.advertise<std_msgs::Int16>("cam_msg",100);

  int init_past=1;
  ros::Rate loop_rate(50);
  cout<< "start" <<endl;
  //ROS end
 Scalar s_white(255, 255, 255);
  Scalar s_black(0, 0, 0);
  Scalar s_red(0, 0, 255);
  Scalar s_green(0, 255, 0);
  Scalar s_blue(255, 0, 0);

  double canny_t1 = 50;
  double canny_t2 = 150;

  double width = cap.get(CAP_PROP_FRAME_WIDTH);
  double height = cap.get(CAP_PROP_FRAME_HEIGHT);
  int roi_height = cvRound(height / 2.5);

  double gaussian_sgm = 2.0;
  int line_tsh = 20;
  double line_min_length = 20;
  double line_max_gap = 20;
  int mopology_itr = 1;

  double angle;
  int alpha = 100;



  int mid_x_bf = width / 2;
  int dis_x_bf = 0;

  int white_threshold =2000;
  bool start = false;
  int turn = 0;
  int obj_turn = 1;
  vector<Vec4i> lines;

  Mat white(Size(width, roi_height), CV_8UC3, s_white);

/*
#ifdef __SHOWUI
  init_windows(width, height, roi_height);
#endif
*/

  Point before_left_line[2];
 Point before_right_line[2];

  while (1)
  {
    cap >> frame;
    if (frame.empty())
    {
      waitKey(0);
      break;
    }

    //Before Line Detecting Logic
    cvtColor(frame.rowRange(height - roi_height, height).clone(), gray, BGR2GRAY);
    GaussianBlur(gray, gaussian, Size(3, 3), gaussian_sgm);
    threshold(gray.colRange(width / 4, width * 3 / 4), binary, 0, 255, THRESH_OTSU);
    Canny(gaussian, canny_t, canny_t1, canny_t2);
    HoughLinesP(canny_t, lines, 1, CV_PI / 180, line_tsh, line_min_length, line_max_gap);

    //find start line
    int count_white[4] = {0,0,0,0};
    for (int h = 0; h < roi_height; h++)
    {
      for (int w = 0; w < width / 2; w++)
      {
        if (binary.at<bool>(Point(w,h)) == 255)
        {
          if (w < (width / 4) && h < (roi_height / 2))
            count_white[0]++;
          else if (w >= (width/4) && h < (roi_height / 2))
            count_white[1]++;
          else if (w < (width / 4) && h >=(roi_height / 2))
            count_white[2]++;
          else if (w >= (width / 4) && h>= (roi_height / 2))
            count_white[3]++;
        }
      }
    }
/*
#ifdef __SHOWUI
    line(binary, Point(width/4,0),Point(width/4,roi_height),Scalar(255,255,255),3);
    line(binary,Point(0,roi_height/2),Point(width/2,roi_height/2),Scalar(255,255,255),3);
    cout << count_white[0] << ' ' << count_white[1]<< ' ' << count_white[2] << ' ' << count_white[3] <<  endl;
    cout << abs((width/4)*(roi_height/2) - count_white[0]*2) - abs((width/4)*(roi_height/2) - count_white[1]*2) << ' '
    << abs((width/4)*(roi_height/2) - count_white[2]*2) - abs((width/4)*(roi_height/2) - count_white[3]*2) << endl;
#endif
*/
    if(start)
    {
      if(!(count_white[0] > white_threshold && count_white[1] > white_threshold))
          start = false;
    }
    else if(count_white[0] > white_threshold && count_white[1] > white_threshold)
    {
      start = true;
      turn++;
    }
    //start가 true일때는 출발선이 있을때, false일때는 출발선이 없을때, turn은 돈 바퀴수,(수정이 필요할수 있음)

#ifdef __START
    if(start)
    cout << start <<' '<< turn << endl;
    else
    cout << 0 << ' ' <<  turn << endl;
#endif

    // Line Detecting Logics
    int left_min = width;
    int right_min = width;

    int right_min_x = cvRound(width / 4);
    int left_max_x = cvRound((width / 4) * 3);

    Point left_line[2];
    bool check_left = false;
    Point right_line[2];
    bool check_right = false;
    Point parallel_line[2];

    for (vector<Vec4i>::iterator pl = lines.begin();
         pl != lines.end(); ++pl) {
      Vec4i& l = *pl;
      angle = line_ingradient(l[0], l[1], l[2], l[3]);
      if (fabs(angle) > 0.5 && fabs(angle) < 5.6)
      {
        line(white, Point(l[0], l[1]), Point(l[2], l[3]), s_red, 1, LINE_AA);

        if (angle < 0)
        {
          if (left_max_x > l[0] && left_max_x > l[2])
          {
            int line_ = ((roi_height / 3) - l[1]) * ((l[0] - l[2]) / (l[1] - l[3])) + l[0];
            if (left_min > abs(line_ - width / 2))
            {
              left_min = abs(line_ - width / 2);
              left_line[0] = Point(l[0], l[1]);
              left_line[1] = Point(l[2], l[3]);
              check_left = true;
            }
          }
        }
        else if (angle > 0)
        {
          if (right_min_x < l[0] && right_min_x < l[2])
          {
            int line_ = ((roi_height / 3) - l[1]) * ((l[0] - l[2]) / (l[1] - l[3])) + l[0];
            if (right_min > abs(line_ - width / 2))
            {
              right_min = abs(line_ - width / 2);
              right_line[0] = Point(l[0], l[1]);
              right_line[1] = Point(l[2], l[3]);
              check_right = true;
            }
          }
        }
      }
    }

    //If can't find line
    if (check_left)
    {
      before_left_line[0] = left_line[0];
      before_left_line[1] = left_line[1];
    }
    else
    {
      left_line[0] = before_left_line[0];
      left_line[1] = before_left_line[1];
    }

    if (check_right)
    {
      before_right_line[0] = right_line[0];
      before_right_line[1] = right_line[1];
    }
    else
    {
      right_line[0] = before_right_line[0];
      right_line[1] = before_right_line[1];
    }

    //Making long line logics
    int left_dx = left_line[0].x - left_line[1].x;
    int left_dy = left_line[0].y - left_line[1].y;
    int right_dx = right_line[0].x - right_line[1].x;
    int right_dy = right_line[0].y - right_line[1].y;

    left_line[0].x += abs(left_dx) * alpha;
    left_line[0].y -= abs(left_dy) * alpha;
    left_line[1].x -= abs(left_dx) * alpha;
    left_line[1].y += abs(left_dy) * alpha;

    right_line[0].x += abs(right_dx) * alpha;
    right_line[0].y += abs(right_dy) * alpha;
    right_line[1].x -= abs(right_dx) * alpha;
    right_line[1].y -= abs(right_dy) * alpha;

    //Formular to find Vanishing point
    double left_gredient = (double)left_dy / left_dx;
    double right_gradient = (double)right_dy / right_dx;
    double left_y_itc = left_line[0].y - left_gredient * left_line[0].x;
    double right_y_itc = right_line[0].y - right_gradient * right_line[0].x;

	int left_dis = abs(((roi_height-left_y_itc)/left_gredient) - width/2);
    int right_dis = abs(((roi_height-right_y_itc)/right_gradient)-width/2);

    int dis_x = right_dis - left_dis;	
    int mid_x = -(left_y_itc - right_y_itc) / (left_gredient - right_gradient);
    int mid_y = left_gredient * mid_x + left_y_itc;

    if (mid_y > 0 && mid_y < roi_height){
      mid_x = mid_x_bf;
	  dis_x = dis_x_bf;
}
    else{
      mid_x_bf = mid_x;
	  dis_x_bf = dis_x;
}


/*
#ifdef __SHOWUI
    // Draw line
    line(white, left_line[0], left_line[1], s_black, 2);
    line(white, right_line[0], right_line[1], s_black, 2);
    line(white, Point(mid_x, 0), Point(mid_x, roi_height), s_blue, 2);
#endif
*/

    // Result
    // string str = string("x: ") + to_string((width / 2 - mid_x));
    // putText(white, str, Point(10, 20), 1, 1, s_black);
	int msg_data;

	msg_data = (int)(dis_x*0.65);
    // ROS
    cam_msg.data = msg_data;
    pub.publish(cam_msg);
    //loop_rate.sleep();
    // ROS end

#ifdef __DIFF
    //    cout << msg_data << endl;
#endif

/*
#ifdef __SHOWUI
    // Frame showing logics
    imshow("original", frame);
    imshow("gray", gray);
    imshow("canny_t", canny_t);
    imshow("white", white);
    imshow("binary", binary);
#endif
*/

    // Set whiteboard empty
    white.setTo(s_white);

    //Space := pause ESC := end
    int ke = waitKey(10);
    if (ke == ' ')
      waitKey();
    else if (ke == 27)
      break;
  }
  return 0;
}
