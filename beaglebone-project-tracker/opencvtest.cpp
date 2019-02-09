#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <iostream>
#include <ctype.h>
#include <stdio.h>

using namespace cv;
using namespace std;

Mat image;

int flag=1;
bool selectObject = false;
int trackObject = 0;
Point origin,center;
Rect selection;
int vmin = 10, vmax = 256, smin = 30;
void drive(Point);

static void init()
{
	system("sudo echo 48 > /sys/class/gpio/export");
	system("sudo echo 49 > /sys/class/gpio/export");
	system("sudo echo 51 > /sys/class/gpio/export");
	system("sudo echo 60 > /sys/class/gpio/export");
	system("sudo echo out > /sys/class/gpio/gpio48/direction");
	system("sudo echo out > /sys/class/gpio/gpio49/direction");
	system("sudo echo out > /sys/class/gpio/gpio51/direction");
	system("sudo echo out > /sys/class/gpio/gpio60/direction");
}

static void intro()
{
system("clear");
printf("------------------------------\n");
printf("Final Year Project (2014-15)\n");
printf("Object Tracking With BeagleBoneBlack\n");
printf("#linux	#OpenCV	#C,C++	#OpenSource\n");
printf("------------------------------\n");
printf("Team Members : \n");
printf("------------------------------\n");
printf("Akash Sinha       (1106831006)\n");
printf("Nirmit Seth       (1106831042)\n");
printf("Ratnodai Singh    (1106831061)\n");
printf("Vaibhav Chaudhary (1106831087)\n");
printf("------------------------------\n");
}

static void onMouse( int event, int x, int y, int, void* )
{
    if( selectObject )
    {
        selection.x = MIN(x, origin.x);
        selection.y = MIN(y, origin.y);
        selection.width = std::abs(x - origin.x);
        selection.height = std::abs(y - origin.y);

        selection &= Rect(0, 0, image.cols, image.rows);
    }

    switch( event )
    {
        case CV_EVENT_LBUTTONDOWN:
            origin = Point(x,y);
            selection = Rect(x,y,0,0);
            selectObject = true;
            break;
        case CV_EVENT_LBUTTONUP:
            selectObject = false;
            if( selection.width > 0 && selection.height > 0 )
                trackObject = -1;
            break;
    }
}

static void help()
{
    cout << "\nObject Tracking \n"
    "You select a colored object and the algorithm tracks it.\n"
    "This takes the input from the webcam\n"
    "Usage: \n"
    "$ ./main [camera number]\n";

    cout << "\n\nKeyboard input options: \n"
    "\tESC - quit the program\n"
    "\ts - stop the tracking\n"
    "\tp - pause video\n"
    "\nTo start tracking an object, select the rectangular region around it with the mouse\n\n";
}

int main( int argc, const char** argv )
{
    intro();
    help();
    init();
    VideoCapture cap;
    Rect trackWindow;
    int hsize = 16;
    float hranges[] = {0,180};
    const float* phranges = hranges;

    int camNum = 0;
    if(argc == 2)
        camNum = atoi(argv[1]);

    cap.open(camNum);
	printf("%f\t%f\n",cap.get(3),cap.get(4));

    if( !cap.isOpened() )
    {
        help();
        cout << "***Could not initialize capturing...***\n";
        cout << "Current parameter's value: " << camNum << endl;
        return -1;
    }

    namedWindow( "CamShift Object Tracker", 0 );
    setMouseCallback( "CamShift Object Tracker", onMouse, 0 );

    Mat frame, hsv, hue, mask, hist, histimg = Mat::zeros(200, 320, CV_8UC3), backproj;
    bool paused = false;

    for(;;)
    {
        if( !paused )
        {
            cap >> frame;
            if( frame.empty() )
                break;
        }

        frame.copyTo(image);

        if( !paused )
        {
            cvtColor(image, hsv, CV_BGR2HSV);

            if( trackObject )
            {
                int _vmin = vmin, _vmax = vmax;

                inRange(hsv, Scalar(0, smin, MIN(_vmin,_vmax)),
                        Scalar(180, 256, MAX(_vmin, _vmax)), mask);
                int ch[] = {0, 0};
                hue.create(hsv.size(), hsv.depth());
                mixChannels(&hsv, 1, &hue, 1, ch, 1);

                if( trackObject < 0 )
                {
                    Mat roi(hue, selection), maskroi(mask, selection);
                    calcHist(&roi, 1, 0, maskroi, hist, 1, &hsize, &phranges);
                    normalize(hist, hist, 0, 255, CV_MINMAX);

                    trackWindow = selection;
                    trackObject = 1;


                }

                calcBackProject(&hue, 1, 0, hist, backproj, &phranges);
                backproj &= mask;
                RotatedRect trackBox = CamShift(backproj, trackWindow,
                                                TermCriteria( CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 10, 1 ));

		//printf("%f\t%f\n",trackBox.size.width,trackBox.size.height);
                center=trackBox.center;
                ellipse( image, trackBox, Scalar(0,0,255), 3, CV_AA );
		drive(center);
            }
        }
        else if( trackObject < 0 )
            paused = false;

        if( selectObject && selection.width > 0 && selection.height > 0 )
        {
            Mat roi(image, selection);
            bitwise_not(roi, roi);
        }

        imshow( "CamShift Object Tracker", image );
        char c = (char)waitKey(10);
	//printf("%d\t%d\n",center.x,center.y);

	if( c == 27 )
            break;
        switch(c)
        {
            case 's':
                trackObject = 0;
                histimg = Scalar::all(0);
                break;

            case 'p':
                paused = !paused;
                break;

            default:
                ;
        }
    }

    return 0;
}

static void gpio()
{
	system("echo -n gpio48 : && cat /sys/class/gpio/gpio48/value");
	system("echo -n gpio49 : && cat /sys/class/gpio/gpio49/value");
	system("echo -n gpio51 : && cat /sys/class/gpio/gpio51/value");
	system("echo -n gpio60 : && cat /sys/class/gpio/gpio60/value");
}

void drive(Point ctrl)
{

	if(ctrl.x<150)
	{
		if(flag!=1)
		{
		system("clear");
		intro();
		printf("turn left\n");
		system("echo 1 > /sys/class/gpio/gpio48/value");
		system("echo 1 > /sys/class/gpio/gpio49/value");
		system("echo 0 > /sys/class/gpio/gpio51/value");
		system("echo 0 > /sys/class/gpio/gpio60/value");
		flag=1;
		gpio();
		}
	}
	else if(ctrl.x>490)
	{
		if(flag!=2)
		{
		system("clear");
		intro();
		printf("turn right\n");
		system("echo 0 > /sys/class/gpio/gpio48/value");
		system("echo 0 > /sys/class/gpio/gpio49/value");
		system("echo 1 > /sys/class/gpio/gpio51/value");
		system("echo 1 > /sys/class/gpio/gpio60/value");
		flag=2;
		gpio();
		}
	}

	else if(ctrl.y<125)
	{
		if(flag!=3)
		{
		system("clear");
		intro();
		printf("drive back\n");
		system("echo 0 > /sys/class/gpio/gpio48/value");
		system("echo 0 > /sys/class/gpio/gpio49/value");
		system("echo 1 > /sys/class/gpio/gpio51/value");
		system("echo 1 > /sys/class/gpio/gpio60/value");
		flag=3;
		gpio();
		}
	}

	else if(ctrl.y>355)
	{
		if(flag!=4)
		{
		system("clear");
		intro();
		printf("drive froward\n");
		system("echo 0 > /sys/class/gpio/gpio48/value");
		system("echo 0 > /sys/class/gpio/gpio49/value");
		system("echo 1 > /sys/class/gpio/gpio51/value");
		system("echo 1 > /sys/class/gpio/gpio60/value");
		flag=4;
		gpio();
		}
	}
	else
	{
		if(flag!=5)
		{
		system("clear");
		intro();
		printf("ruk ja thand rakh\n");
		system("echo 1 > /sys/class/gpio/gpio48/value");
		system("echo 0 > /sys/class/gpio/gpio49/value");
		system("echo 1 > /sys/class/gpio/gpio51/value");
		system("echo 0 > /sys/class/gpio/gpio60/value");
		flag=5;
		gpio();
		}
	}
}
