#include <iostream>	// for standard I/O
#include <string>   // for strings

#include <opencv2/core/core.hpp>        // Basic OpenCV structures (cv::Mat)
#include <opencv2/highgui/highgui.hpp>  // Video write

using namespace std;
using namespace cv;

static void help()
{
    cout
        << "------------------------------------------------------------------------------" << endl
        << "This program shows how to write video files."                                   << endl
        << "Usage:"                                                                         << endl
        << "./video-write inputvideoName"                                                   << endl
        << "------------------------------------------------------------------------------" << endl
        << endl;
}

int main(int argc, char *argv[])
{
    help();

    if (argc != 3)
    {
        cout << "Not enough parameters" << endl;
        return -1;
    }

    const char *source = argv[1]; // the video source
    const string target = argv[2]; // the target file name

    VideoCapture inputVideo;              // Open input
    if (!inputVideo.open(atoi(source)))
    {
        cout  << "Could not open the input video: " << target << endl;
        return -1;
    }

    Size S = Size((int) inputVideo.get(CV_CAP_PROP_FRAME_WIDTH),    // Acquire input size
                  (int) inputVideo.get(CV_CAP_PROP_FRAME_HEIGHT));

    VideoWriter outputVideo;                                        // Open the output
    outputVideo.open(target, CV_FOURCC('M','J','P','G'), 25, S, true);

    if (!outputVideo.isOpened())
    {
        cout  << "Could not open the output video for write: " << target << endl;
        return -1;
    }

    cout << "Input frame resolution: Width=" << S.width << "  Height=" << S.height << endl;

    Mat src;

    for(;;) //Show the image captured in the window and repeat
    {
        inputVideo >> src;              // read
        if (src.empty()) break;

       //outputVideo.write(res); //save or
       outputVideo << src;
    }

    cout << "Finished writing" << endl;
    return 0;
}
