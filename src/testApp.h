#pragma once

#include "ofMain.h"
#include "ofxOpenCv.h"
#include "ofxKinect.h"
#include "ofxPanel.h"
#include "ofxBox2d.h"

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

    ofImage selectedImg1,
            selectedImg2,
            selectedImg3,
            selectedImg4,
            selectedImg5,
            selectedImg6;
	ofxCvGrayscaleImage cvGrayImage1, // grayscale depth image
                        cvGrayImage2, // grayscale depth image
                        cvGrayImage3,
                        cvGrayImage4,
                        cvGrayImage5,
                        cvGrayImage6,
                        cvGrayThresh1,
                        cvGrayThresh2,
                        cvGrayThresh3,
                        cvGrayThresh4,
                        cvGrayThresh5,
                        cvGrayThresh6,
                        cvGrayThresh1F,
                        cvGrayThresh2F,
                        cvGrayThresh3F,
                        cvGrayThresh4F,
                        cvGrayThresh5F,
                        cvGrayThresh6F,
                        cvSelectedImg1,
                        cvSelectedImg2,
                        cvSelectedImg3,
                        cvSelectedImg4,
                        cvSelectedImg5,
                        cvSelectedImg6;
	ofxCvContourFinder contourFinder1,
                       contourFinder2,
                       contourFinder3,
                       contourFinder4,
                       contourFinder5,
                       contourFinder6;

    //GUI
    ofxPanel gui;
	ofxIntSlider threshSlider1,
                 threshSlider2,
                 threshSlider3,
                 threshSlider4,
                 threshSlider5,
                 threshSlider6,
                 maxSizeSlider,
                 minSizeSlider;

	bool bDrawPointCloud,
         findWin,
         drawSel;
	int angle;

	vector<ofVec2f> corners;
    ofVec2f mousePt;
    ofVec3f centroid;
    vector<ofPoint> gesture_lines;
    ofPoint furthest,
            blob_pts;
    ofRectangle rect;

    ofxBox2d box2d;
    vector    <ofPtr<ofxBox2dCircle> >	circles;		  //	default box2d circles
	vector	  <ofPtr<ofxBox2dRect> >	boxes;			  //	defalut box2d rects
};
