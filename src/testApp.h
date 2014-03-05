#pragma once

#include "ofMain.h"
#include "ofxOpenCv.h"
#include "ofxKinect.h"
#include "ofxPanel.h"

class testApp : public ofBaseApp {
public:

	void setup();
	void update();
	void draw();
	void exit();
	void drawPointCloud();
	void keyPressed(int key);
	void mouseMoved(int x, int y);
	void mouseDragged(int x, int y, int button);
	void mousePressed(int x, int y, int button);
	void mouseReleased(int x, int y, int button);
	void windowResized(int w, int h);

	ofxKinect kinect;
	ofEasyCam easyCam; // used for viewing the point cloud

    ofImage selectedImg,
            selectedNearImg,
            selectedFarImg;
	ofxCvColorImage colorImg;
	ofxCvGrayscaleImage cvGrayImage1, // grayscale depth image
                        cvGrayImage2, // grayscale depth image
                        cvGrayThreshNear, // the near thresholded image
                        cvGrayThreshMid1, // the mid thresholded image
                        cvGrayThreshMid2, // the mid thresholded image
                        cvGrayThreshFar, // the far thresholded image
                        cvGrayBg,
                        cvSelectedNearImg,
                        cvSelectedFarImg,
                        cvSubtractedImg;
	ofxCvContourFinder contourFinder1,
                       contourFinder2,
                       contourFinder3;

    //GUI
    ofxPanel gui;
	ofxIntSlider threshSlider1,
                 threshSlider2,
                 maxSizeSlider,
                 minSizeSlider;

	bool bDrawPointCloud,
         getNewBg;
	int angle;

	vector<ofVec2f> corners;
    ofVec2f mousePt;
    ofVec3f centroid;
    vector<ofPoint> gesture_lines;
    ofRectangle rect;
};
