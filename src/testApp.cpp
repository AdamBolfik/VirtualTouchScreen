#include "testApp.h"


//--------------------------------------------------------------
void testApp::setup() {
	ofSetLogLevel(OF_LOG_VERBOSE);

	// enable depth->video image calibration
	kinect.setRegistration(true);
	kinect.init();
	kinect.open();

	colorImg.allocate(kinect.width, kinect.height);
	grayImage1.allocate(kinect.width, kinect.height);
	grayImage2.allocate(kinect.width, kinect.height);
	grayThreshNear.allocate(kinect.width, kinect.height);
	grayThreshMid1.allocate(kinect.width, kinect.height);
	grayThreshMid2.allocate(kinect.width, kinect.height);
	grayThreshFar.allocate(kinect.width, kinect.height);

	//ofSetFrameRate(60);

	// zero the tilt on startup
	angle = 0;
	kinect.setCameraTiltAngle(angle);

	// start from the front
	bDrawPointCloud = false;

	ofSetWindowShape(kinect.width * 2, kinect.height * 2);

	// GUI
    gui.setup();
    gui.add(threshSlider1.setup("Thresh 1", 181, 0, 255));
    gui.add(threshSlider2.setup("Thresh 2", 170, 0, 255));
}

//--------------------------------------------------------------
void testApp::update() {

	ofBackground(100, 100, 100);

	kinect.update();
	// there is a new frame and we are connected
	if(kinect.isFrameNew()) {

		// load grayscale depth image from the kinect source
		grayImage1.setFromPixels(kinect.getDepthPixels(), kinect.width, kinect.height);

        if(corners.size() == 2){
            //selectedImg.allocate(kinect.width, kinect.height);
            selectedImg.setFromPixels(kinect.getPixels(), kinect.width, kinect.height, OF_IMAGE_COLOR);
            selectedImg.crop(corners[0].x, corners[0].y, corners[1].x - corners[0].x, corners[1].y - corners[0].y);
        }

		// we do two thresholds - one for the far plane and one for the near plane
		// we then do a cvAnd to get the pixels which are a union of the two thresholds

        grayThreshNear = grayImage1;
        grayThreshMid1 = grayImage1;
        grayThreshMid2 = grayImage1;
        grayThreshFar = grayImage1;
        grayThreshNear.threshold(threshSlider1, true);
        grayThreshMid1.threshold(threshSlider1 - 5);
        grayThreshMid2.threshold(threshSlider2, true);
        grayThreshFar.threshold(threshSlider2 - 5);
        cvAnd(grayThreshNear.getCvImage(), grayThreshMid1.getCvImage(), grayImage1.getCvImage(), NULL);
        cvAnd(grayThreshMid2.getCvImage(), grayThreshFar.getCvImage(), grayImage2.getCvImage(), NULL);

		// update the cv images
		grayImage1.flagImageChanged();
		grayImage2.flagImageChanged();
        // Dilate
        grayImage1.dilate_3x3();
        grayImage2.dilate_3x3();

		// find contours which are between the size of 20 pixels and 1/3 the w*h pixels.
		// also, find holes is set to true so we will get interior contours as well....
		contourFinder1.findContours(grayImage1, 100, (kinect.width*kinect.height)/2, 2, false);
		contourFinder2.findContours(grayImage2, 100, (kinect.width*kinect.height)/2, 2, false);
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
		grayImage1.draw(0, 0);
        grayImage2.draw(kinect.width, 0);
        kinect.draw(kinect.width, kinect.height);
		contourFinder1.draw(0, 0);
		contourFinder2.draw(kinect.width, 0);
		//kinect.drawDepth(kinect.width, 0, 400, 300);

		// if two points are selected, draw a rectangle
        if(corners.size() == 2){
            ofNoFill();
            //ofSetColor(0, 255, 255, 50);
            ofRect(corners[0].x, corners[0].y, corners[1].x - corners[0].x, corners[1].y - corners[0].y);
            selectedImg.draw(kinect.width, 0);
        }
	}

    // draw crosshair on cursor
    ofSetLineWidth(2);
	ofLine(mousePt.x+10, mousePt.y, mousePt.x-10, mousePt.y);
	ofLine(mousePt.x, mousePt.y+10, mousePt.x, mousePt.y-10);

    // Values
    stringstream reportStream;
    reportStream << "sensor-emitter dist: " << kinect.getSensorEmitterDistance() << "cm" << endl
	<< "sensor-camera dist:  " << kinect.getSensorCameraDistance() << "cm" << endl
	<< "zero plane pixel size: " << kinect.getZeroPlanePixelSize() << "mm" << endl
	<< "zero plane dist: " << kinect.getZeroPlaneDistance() << "mm" << endl
	<< "fps: " << ofGetFrameRate() << endl;

    if(kinect.hasCamTiltControl()) {
    	reportStream << "press UP and DOWN to change the tilt angle: " << angle << " degrees" << endl
        << "press 1-5 & 0 to change the led mode" << endl;
    }

	ofDrawBitmapString(reportStream.str(), 20, 500);

	//GUI
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

		case'p':
			bDrawPointCloud = !bDrawPointCloud;
			break;

		case 'w':
			kinect.enableDepthNearValueWhite(!kinect.isDepthNearValueWhite());
			break;

		case 'o':
			kinect.setCameraTiltAngle(angle); // go back to prev tilt
			kinect.open();
			break;

		case 'c':
			kinect.setCameraTiltAngle(0); // zero the tilt
			kinect.close();
			break;

		case '1':
			kinect.setLed(ofxKinect::LED_GREEN);
			break;

		case '2':
			kinect.setLed(ofxKinect::LED_YELLOW);
			break;

		case '3':
			kinect.setLed(ofxKinect::LED_RED);
			break;

		case '4':
			kinect.setLed(ofxKinect::LED_BLINK_GREEN);
			break;

		case '5':
			kinect.setLed(ofxKinect::LED_BLINK_YELLOW_RED);
			break;

		case '0':
			kinect.setLed(ofxKinect::LED_OFF);
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
