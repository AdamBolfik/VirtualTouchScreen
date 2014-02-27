#include "testApp.h"


//--------------------------------------------------------------
void testApp::setup() {
	ofSetLogLevel(OF_LOG_VERBOSE);

	// enable depth->video image calibration
	kinect.setRegistration(true);
	kinect.init();
	kinect.open();

    // get space for all of the images
	colorImg.allocate(kinect.width, kinect.height);
	cvGrayImage1.allocate(kinect.width, kinect.height);
	cvGrayImage2.allocate(kinect.width, kinect.height);
	cvGrayThreshNear.allocate(kinect.width, kinect.height);
	cvGrayThreshMid1.allocate(kinect.width, kinect.height);
	cvGrayThreshMid2.allocate(kinect.width, kinect.height);
	cvGrayThreshFar.allocate(kinect.width, kinect.height);
	cvGrayBg.allocate(kinect.width, kinect.height);

//	ofSetFrameRate(60);

	// zero the tilt on startup
	angle = 0;
	kinect.setCameraTiltAngle(angle);

	bDrawPointCloud = false;
	getNewBg = true;

	ofSetWindowShape(kinect.width * 2, kinect.height * 2);

	// GUI
    gui.setup();
//    gui.add(threshSlider1.setup("Thresh 1", 177, 0, 255));
//    gui.add(threshSlider2.setup("Thresh 2", 168, 0, 255));
    gui.add(threshSlider1.setup("ThreshHold", 200, 0, 255));
    gui.add(maxSizeSlider.setup("Max Size", 4000, 300, kinect.width*kinect.height/4));
    gui.add(minSizeSlider.setup("Min Size", 100, 10, 300));
}

//--------------------------------------------------------------
void testApp::update() {

	ofBackground(100, 100, 100);

	kinect.update();
	// there is a new frame and we are connected
	if(getNewBg){
        cvGrayBg.setFromPixels(kinect.getDepthPixels(), kinect.width, kinect.height);
        getNewBg = false;
	}
	if(kinect.isFrameNew()) {
		// load grayscale depth image from the kinect source
		cvGrayImage1.setFromPixels(kinect.getDepthPixels(), kinect.width, kinect.height);

        cvGrayThreshNear = cvGrayImage1;
        cvGrayThreshMid1 = cvGrayImage1;
        cvGrayThreshMid2 = cvGrayImage1;
        cvGrayThreshFar = cvGrayImage1;

        cvGrayThreshNear.threshold(threshSlider1, true);
        cvGrayThreshMid1.threshold(threshSlider1 - 5);
        cvGrayThreshMid2.threshold(threshSlider1 - 6, true);
        cvGrayThreshFar.threshold(threshSlider1 - 11);
        cvAnd(cvGrayThreshNear.getCvImage(), cvGrayThreshMid1.getCvImage(), cvGrayImage1.getCvImage(), NULL);
        cvAnd(cvGrayThreshMid2.getCvImage(), cvGrayThreshFar.getCvImage(), cvGrayImage2.getCvImage(), NULL);

        // allocate ofImg, set ofImg from cvImg, crop ofImg, change to cvimage
        if(corners.size() == 2){
//            selectedImg.allocate(kinect.width, kinect.height);
//            selectedImg.setFromPixels(kinect.getPixels(), kinect.width, kinect.height, OF_IMAGE_COLOR);
//            selectedImg.crop(corners[0].x, corners[0].y, corners[1].x - corners[0].x, corners[1].y - corners[0].y);
            // Near Image
            selectedNearImg.allocate(cvGrayImage1.width, cvGrayImage1.height, OF_IMAGE_GRAYSCALE);
            selectedNearImg.setFromPixels(cvGrayImage1.getPixels(), cvGrayImage1.width, cvGrayImage1.height, OF_IMAGE_GRAYSCALE);
            selectedNearImg.crop(corners[0].x, corners[0].y, corners[1].x - corners[0].x, corners[1].y - corners[0].y);
            cvSelectedNearImg.setFromPixels(selectedNearImg.getPixels(), corners[1].x - corners[0].x, corners[1].y - corners[0].y);
            // Far Image
            selectedFarImg.allocate(cvGrayImage2.width, cvGrayImage2.height, OF_IMAGE_GRAYSCALE);
            selectedFarImg.setFromPixels(cvGrayImage2.getPixels(), cvGrayImage2.width, cvGrayImage2.height, OF_IMAGE_GRAYSCALE);
            selectedFarImg.crop(corners[0].x, corners[0].y, corners[1].x - corners[0].x, corners[1].y - corners[0].y);
            cvSelectedFarImg.setFromPixels(selectedFarImg.getPixels(), corners[1].x - corners[0].x, corners[1].y - corners[0].y);

            cvSubtractedImg.allocate(cvSelectedFarImg.getWidth(), cvSelectedFarImg.getHeight());
            cvSubtractedImg.absDiff(cvSelectedNearImg, cvSelectedFarImg);

            // find contours which are between the size of 20 pixels and 1/3 the w*h pixels.
            // also, find holes is set to true so we will get interior contours as well....
            contourFinder1.findContours(cvSelectedNearImg, minSizeSlider, maxSizeSlider, 2, false);
            contourFinder2.findContours(cvSelectedFarImg, minSizeSlider, maxSizeSlider, 2, false);
            contourFinder3.findContours(cvSubtractedImg, minSizeSlider, maxSizeSlider, 2, false);
        }

		// update the cv images
		cvGrayImage1.flagImageChanged();
		cvGrayImage2.flagImageChanged();
        // Dilate
//        grayImage1.dilate_3x3();
//        grayImage2.dilate_3x3();
	}
}

//--------------------------------------------------------------
void testApp::draw() {

	ofSetColor(255, 255, 255);

	if(bDrawPointCloud) {
		easyCam.begin();
		drawPointCloud();
		easyCam.end();
	} else {
		// draw from the live kinect
		cvGrayImage1.draw(0, 0);
        cvGrayImage2.draw(kinect.width, 0);
//        kinect.draw(kinect.width, kinect.height);
//		  kinect.drawDepth(kinect.width, 0, 400, 300);

		// if two points are selected, draw a rectangle
        if(corners.size() == 2){
            ofNoFill();
            ofRect(corners[0].x, corners[0].y, corners[1].x - corners[0].x, corners[1].y - corners[0].y);
//            selectedImg.draw(kinect.width, 0);
            cvSelectedNearImg.draw(0, kinect.height);
            cvSelectedFarImg.draw(cvSelectedNearImg.width, kinect.height);
            cvSubtractedImg.draw(cvSelectedNearImg.width * 2, kinect.height);
            contourFinder1.draw(0, kinect.height);
            contourFinder2.draw(cvSelectedNearImg.width, kinect.height);
            contourFinder3.draw(cvSelectedNearImg.width * 2, kinect.height);
        }
	}

    // draw crosshair on cursor
    ofSetLineWidth(2);
	ofLine(mousePt.x+10, mousePt.y, mousePt.x-10, mousePt.y);
	ofLine(mousePt.x, mousePt.y+10, mousePt.x, mousePt.y-10);

    // values printed to screen
//    stringstream reportStream;
//    reportStream << "sensor-emitter dist: " << kinect.getSensorEmitterDistance() << "cm" << endl
//	<< "sensor-camera dist:  " << kinect.getSensorCameraDistance() << "cm" << endl
//	<< "zero plane pixel size: " << kinect.getZeroPlanePixelSize() << "mm" << endl
//	<< "zero plane dist: " << kinect.getZeroPlaneDistance() << "mm" << endl
//	<< "fps: " << ofGetFrameRate() << endl;
//    if(kinect.hasCamTiltControl()) {
//    	reportStream << "press UP and DOWN to change the tilt angle: " << angle << " degrees" << endl
//        << "press 1-5 & 0 to change the led mode" << endl;
//    }
//
//	ofDrawBitmapString(reportStream.str(), 20, 500);

	// GUI
	gui.draw();
}

//--------------------------------------------------------------
void testApp::drawPointCloud() {
	int w = 640;
	int h = 480;
	ofMesh mesh;
	mesh.setMode(OF_PRIMITIVE_POINTS);
	int step = 2;
	for(int y = 0; y < h; y += step) {
		for(int x = 0; x < w; x += step) {
			if(kinect.getDistanceAt(x, y) > 0) {
				mesh.addColor(kinect.getColorAt(x,y));
				mesh.addVertex(kinect.getWorldCoordinateAt(x, y));
			}
		}
	}
	glPointSize(3);
	ofPushMatrix();
	// the projected points are 'upside down' and 'backwards'
	ofScale(1, -1, -1);
	ofTranslate(0, 0, -1000); // center the points a bit
	ofEnableDepthTest();
	mesh.drawVertices();
	ofDisableDepthTest();
	ofPopMatrix();
}

//--------------------------------------------------------------
void testApp::exit() {
	kinect.setCameraTiltAngle(0); // zero the tilt on exit
	kinect.close();
}

//--------------------------------------------------------------
void testApp::keyPressed (int key) {
	switch (key) {

        case'r':
            getNewBg = true; // get new background image
            break;

		case'p':
			bDrawPointCloud = !bDrawPointCloud; // Draw the pointcloud
			break;

		case 'w':
			kinect.enableDepthNearValueWhite(!kinect.isDepthNearValueWhite());
			break;

		case OF_KEY_UP:
			angle++;
			if(angle>30) angle=30;
			kinect.setCameraTiltAngle(angle);
			break;

		case OF_KEY_DOWN:
			angle--;
			if(angle<-30) angle=-30;
			kinect.setCameraTiltAngle(angle);
			break;
	}
}

//--------------------------------------------------------------
void testApp::mouseMoved(int x, int y) {
    mousePt.x = x;
    mousePt.y = y;
}

//--------------------------------------------------------------
void testApp::mouseDragged(int x, int y, int button){
    cout << x << ", " << y << ", " << button << endl;
}

//--------------------------------------------------------------
void testApp::mousePressed(int x, int y, int button){
    if(button == 0){
        if(corners.size() != 2)
            corners.push_back(ofVec2f(x, y));
        else if(corners.size() == 2){
            rect = ofRectangle(corners[0].x, corners[0].y, corners[1].x - corners[0].x, corners[1].y - corners[0].y);
        }
    }
    else{
        corners.clear();
    }
}

//--------------------------------------------------------------
void testApp::mouseReleased(int x, int y, int button){
}

//--------------------------------------------------------------
void testApp::windowResized(int w, int h){
}
