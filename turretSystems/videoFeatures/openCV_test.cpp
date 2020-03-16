/*
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/imgproc.hpp>

#include <fstream>
#include <iostream>
#include <map>
*/

#include "pch.h"

#include<opencv2/opencv.hpp>
#include<iostream>

using namespace std;
using namespace cv;

/*	Function Prototype	*/
void detectPikachu(Mat);

/*	Cascade Clasifier Types	*/
CascadeClassifier pikachu_cascade;
CascadeClassifier charmander_cascade;


int main(int argc, const char ** argv)
{	
	/*	Get command line arguments  */
	CommandLineParser parser(argc, argv, "{pikachu_path |C:/Users/xavmo/Pictures/Pikachu_Doll/pikachu_cascade.xml| Path to Pikachu Cascades.}"
										 "{camera |0| Going to use Camera 0. }");

	String pika_cascade = samples::findFile(parser.get<String>("pikachu_path"));


	/*	Load Cascade[s] */
	if (!pikachu_cascade.load(pika_cascade))
	{
		cout << "Error Loading Cascade\n" ;
		return -1;
	}

	/*	Load Camera */
	int camera_device = parser.get<int>("camera");	// Get Camera default camera
	VideoCapture capture;							// Class for video capturing
	capture.open(camera_device);					// Read the video stream

	if (!capture.isOpened())
	{
		cout << "--(!)Error opening video capture\n";
		return -1;
	}

	/*	Start Capturing the video stream	*/
	Mat frame;
	while (capture.read(frame))
	{
		if (frame.empty())
		{
			cout << "--(!) No captured frame -- Break!\n";
			break;
		}

		//-- 3. Apply the classifier to the frame
		detectPikachu(frame);

		if (waitKey(10) == 27)
		{
			break; // escape
		}
	}
	waitKey(0);

	return 0;
}




void detectPikachu(Mat frame)
{
	Mat frame_gray;

	cvtColor(frame, frame_gray, COLOR_BGR2GRAY);	// Convert image into gray
	equalizeHist(frame_gray, frame_gray);			// Convert image to grayscale

	/*	Detect a Pikachu	*/
	std::vector<Rect> pikachu_outlines;
	pikachu_cascade.detectMultiScale(frame_gray, pikachu_outlines);

	for (size_t i = 0; i < pikachu_outlines.size(); i++)
	{
		Point center(pikachu_outlines[i].x + pikachu_outlines[i].width / 2, pikachu_outlines[i].y + pikachu_outlines[i].height / 2);
		ellipse(frame, center, Size(pikachu_outlines[i].width / 2, pikachu_outlines[i].height / 2), 0, 0, 360, Scalar(255, 0, 255), 4);

		//Mat faceROI = frame_gray(pikachu_outlines[i]);
	}

	//-- Show what you got
	imshow("Capture - Face detection", frame);
}