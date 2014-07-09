#include "testApp.h"

//--------------------------------------------------------------
void testApp::setup() {
	ofSetLogLevel(OF_LOG_VERBOSE);

	// enable depth->video image calibration
	kinect.setRegistration(true);
	kinect.init();
	kinect.open();

    // get space for all of the images
	cvGrayImage1.allocate(kinect.width, kinect.height);
	cvGrayImage2.allocate(kinect.width, kinect.height);
	cvGrayImage3.allocate(kinect.width, kinect.height);
	cvGrayImage4.allocate(kinect.width, kinect.height);
	cvGrayImage5.allocate(kinect.width, kinect.height);
	cvGrayImage6.allocate(kinect.width, kinect.height);
	cvGrayThresh1.allocate(kinect.width, kinect.height);
	cvGrayThresh2.allocate(kinect.width, kinect.height);
	cvGrayThresh3.allocate(kinect.width, kinect.height);
	cvGrayThresh4.allocate(kinect.width, kinect.height);
	cvGrayThresh5.allocate(kinect.width, kinect.height);
	cvGrayThresh6.allocate(kinect.width, kinect.height);
	cvGrayThresh1F.allocate(kinect.width, kinect.height);
	cvGrayThresh2F.allocate(kinect.width, kinect.height);
	cvGrayThresh3F.allocate(kinect.width, kinect.height);
	cvGrayThresh4F.allocate(kinect.width, kinect.height);
	cvGrayThresh5F.allocate(kinect.width, kinect.height);
	cvGrayThresh6F.allocate(kinect.width, kinect.height);


	ofSetFrameRate(30);

	// zero the tilt on startup
	angle = 0;
	kinect.setCameraTiltAngle(angle);

	bDrawPointCloud = false;
	findWin = false;
	drawSel = true;

	ofSetWindowShape(kinect.width + 200, kinect.height);

	// GUI
    gui.setup();
    gui.add(threshSlider1.setup("Thresh 1", 200, 0, 255));
    gui.add(threshSlider2.setup("Thresh 2", 200, 0, 255));
    gui.add(threshSlider3.setup("Thresh 3", 200, 0, 255));
    gui.add(threshSlider4.setup("Thresh 4", 200, 0, 255));
    gui.add(threshSlider5.setup("Thresh 5", 200, 0, 255));
    gui.add(threshSlider6.setup("Thresh 6", 200, 0, 255));
//    gui.add(maxSizeSlider.setup("Max Size", 4000, 300, kinect.width*kinect.height/4));
    gui.add(maxSizeSlider.setup("Max Size", 2000, 300, 4000));
    gui.add(minSizeSlider.setup("Min Size", 100, 10, 300));
}

//--------------------------------------------------------------
void testApp::update() {

	ofBackground(100, 100, 100);

	kinect.update();
	// there is a new frame and we are connected
	if(kinect.isFrameNew()) {
		// load grayscale depth image from the kinect source
		cvGrayImage1.setFromPixels(kinect.getDepthPixels(), kinect.width, kinect.height);

        // load a base picture into each image
        cvGrayThresh1 = cvGrayImage1;
        cvGrayThresh2 = cvGrayImage1;
        cvGrayThresh3 = cvGrayImage1;
        cvGrayThresh4 = cvGrayImage1;
        cvGrayThresh5 = cvGrayImage1;
        cvGrayThresh6 = cvGrayImage1;
        cvGrayThresh1F = cvGrayImage1;
        cvGrayThresh2F = cvGrayImage1;
        cvGrayThresh3F = cvGrayImage1;
        cvGrayThresh4F = cvGrayImage1;
        cvGrayThresh5F = cvGrayImage1;
        cvGrayThresh6F = cvGrayImage1;

        // Create pairs of threshold images
        cvGrayThresh1.threshold(threshSlider1, true);
        cvGrayThresh1F.threshold(threshSlider1 - 2);
        cvGrayThresh2.threshold(threshSlider2, true);
        cvGrayThresh2F.threshold(threshSlider2 - 2);
        cvGrayThresh3.threshold(threshSlider3, true);
        cvGrayThresh3F.threshold(threshSlider3 -2);
        cvGrayThresh4.threshold(threshSlider4, true);
        cvGrayThresh4F.threshold(threshSlider4 - 2);
        cvGrayThresh5.threshold(threshSlider5, true);
        cvGrayThresh5F.threshold(threshSlider5 - 2);
        cvGrayThresh6.threshold(threshSlider6, true);
        cvGrayThresh6F.threshold(threshSlider6 - 2);

        // Find similarities between a pair of images
        cvAnd(cvGrayThresh1.getCvImage(), cvGrayThresh1F.getCvImage(), cvGrayImage1.getCvImage(), NULL);
        cvAnd(cvGrayThresh2.getCvImage(), cvGrayThresh2F.getCvImage(), cvGrayImage2.getCvImage(), NULL);
        cvAnd(cvGrayThresh3.getCvImage(), cvGrayThresh3F.getCvImage(), cvGrayImage3.getCvImage(), NULL);
        cvAnd(cvGrayThresh4.getCvImage(), cvGrayThresh4F.getCvImage(), cvGrayImage4.getCvImage(), NULL);
        cvAnd(cvGrayThresh5.getCvImage(), cvGrayThresh5F.getCvImage(), cvGrayImage5.getCvImage(), NULL);
        cvAnd(cvGrayThresh6.getCvImage(), cvGrayThresh6F.getCvImage(), cvGrayImage6.getCvImage(), NULL);

        // allocate and load each selected image
        if(corners.size() >= 2){
            selectedImg1.allocate(cvGrayImage1.width, cvGrayImage1.height, OF_IMAGE_GRAYSCALE);
            selectedImg1.setFromPixels(cvGrayImage1.getPixels(), cvGrayImage1.width, cvGrayImage1.height, OF_IMAGE_GRAYSCALE);
            selectedImg1.crop(corners[0].x, corners[0].y, corners[1].x - corners[0].x, corners[1].y - corners[0].y);
            cvSelectedImg1.setFromPixels(selectedImg1.getPixels(), corners[1].x - corners[0].x, corners[1].y - corners[0].y);
            contourFinder1.findContours(cvSelectedImg1, minSizeSlider, maxSizeSlider, 1, false);
            if(contourFinder1.nBlobs == 1 && findWin){
                system("xdotool search 'Web Page -- Home --' windowactivate");
            }

            if(corners.size() >= 4){
                selectedImg2.allocate(cvGrayImage2.width, cvGrayImage2.height, OF_IMAGE_GRAYSCALE);
                selectedImg2.setFromPixels(cvGrayImage2.getPixels(), cvGrayImage2.width, cvGrayImage2.height, OF_IMAGE_GRAYSCALE);
                selectedImg2.crop(corners[2].x, corners[2].y, corners[3].x - corners[2].x, corners[3].y - corners[2].y);
                cvSelectedImg2.setFromPixels(selectedImg2.getPixels(), corners[3].x - corners[2].x, corners[3].y - corners[2].y);
                contourFinder2.findContours(cvSelectedImg2, minSizeSlider, maxSizeSlider, 2, false);
                if(contourFinder2.nBlobs == 1 && findWin){
                    system("xdotool search 'Web Page -- Python --' windowactivate");
                }

                if(corners.size() >= 6){
                    selectedImg3.allocate(cvGrayImage3.width, cvGrayImage3.height, OF_IMAGE_GRAYSCALE);
                    selectedImg3.setFromPixels(cvGrayImage3.getPixels(), cvGrayImage3.width, cvGrayImage3.height, OF_IMAGE_GRAYSCALE);
                    selectedImg3.crop(corners[4].x, corners[4].y, corners[5].x - corners[4].x, corners[5].y - corners[4].y);
                    cvSelectedImg3.setFromPixels(selectedImg3.getPixels(), corners[5].x - corners[4].x, corners[5].y - corners[4].y);
                    contourFinder3.findContours(cvSelectedImg3, minSizeSlider, maxSizeSlider, 2, false);
                    if(contourFinder3.nBlobs == 1 && findWin){
                        system("xdotool search 'Web Page -- Cpp --' windowactivate");
                    }

                    if(corners.size() >= 8){
                        selectedImg4.allocate(cvGrayImage4.width, cvGrayImage4.height, OF_IMAGE_GRAYSCALE);
                        selectedImg4.setFromPixels(cvGrayImage4.getPixels(), cvGrayImage4.width, cvGrayImage4.height, OF_IMAGE_GRAYSCALE);
                        selectedImg4.crop(corners[6].x, corners[6].y, corners[7].x - corners[6].x, corners[7].y - corners[6].y);
                        cvSelectedImg4.setFromPixels(selectedImg4.getPixels(), corners[7].x - corners[6].x, corners[7].y - corners[6].y);
                        contourFinder4.findContours(cvSelectedImg4, minSizeSlider, maxSizeSlider, 2, false);
                        if(contourFinder4.nBlobs == 1 && findWin){
                            system("xdotool search 'Web Page -- Contact --' windowactivate");
                        }

                        if(corners.size() >= 10){
                            selectedImg5.allocate(cvGrayImage5.width, cvGrayImage5.height, OF_IMAGE_GRAYSCALE);
                            selectedImg5.setFromPixels(cvGrayImage5.getPixels(), cvGrayImage5.width, cvGrayImage5.height, OF_IMAGE_GRAYSCALE);
                            selectedImg5.crop(corners[8].x, corners[8].y, corners[9].x - corners[8].x, corners[9].y - corners[8].y);
                            cvSelectedImg5.setFromPixels(selectedImg5.getPixels(), corners[9].x - corners[8].x, corners[9].y - corners[8].y);
                            contourFinder5.findContours(cvSelectedImg5, minSizeSlider, maxSizeSlider, 2, false);
                            if(contourFinder5.nBlobs == 1 && findWin){
                                system("xdotool search 'Web Page -- Java --' windowactivate");
                            }

                            if(corners.size() == 12){
                                selectedImg6.allocate(cvGrayImage6.width, cvGrayImage6.height, OF_IMAGE_GRAYSCALE);
                                selectedImg6.setFromPixels(cvGrayImage6.getPixels(), cvGrayImage6.width, cvGrayImage6.height, OF_IMAGE_GRAYSCALE);
                                selectedImg6.crop(corners[10].x, corners[10].y, corners[11].x - corners[10].x, corners[11].y - corners[10].y);
                                cvSelectedImg6.setFromPixels(selectedImg6.getPixels(), corners[11].x - corners[10].x, corners[11].y - corners[10].y);
                                contourFinder6.findContours(cvSelectedImg6, minSizeSlider, maxSizeSlider, 2, false);
                                if(contourFinder6.nBlobs == 1 && findWin){
                                    system("xdotool search 'Web Page -- Android --' windowactivate");
                                }
                            }
                        }
                    }
                }
            }

//            cvSelectedImg1.dilate_3x3();
//            cvSelectedImg2.dilate_3x3();
//            cvSelectedImg3.dilate_3x3();
        }

		// update the cv images
		cvGrayImage1.flagImageChanged();
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
        kinect.draw(0, 0);

		// Check if ok to draw
		if(drawSel){
		// if a pair of points are selected, draw a rectangle and the corresponding image
            if(corners.size() >= 2){
                ofNoFill();
                ofRect(corners[0].x, corners[0].y, corners[1].x - corners[0].x, corners[1].y - corners[0].y);
                cvSelectedImg1.draw(corners[0].x, corners[0].y);
//              contourFinder1.draw(0, kinect.height + 5);
                if(corners.size() >= 4){
                    ofRect(corners[2].x, corners[2].y, corners[3].x - corners[2].x, corners[3].y - corners[2].y);
                    cvSelectedImg2.draw(corners[2].x, corners[2].y);
                    if(corners.size() >= 6){
                        ofRect(corners[4].x, corners[4].y, corners[5].x - corners[4].x, corners[5].y - corners[4].y);
                        cvSelectedImg3.draw(corners[4].x, corners[4].y);
                        if(corners.size() >= 8){
                            ofRect(corners[6].x, corners[6].y, corners[7].x - corners[6].x, corners[7].y - corners[6].y);
                            cvSelectedImg4.draw(corners[6].x, corners[6].y);
                            if(corners.size() >= 10){
                                ofRect(corners[8].x, corners[8].y, corners[9].x - corners[8].x, corners[9].y - corners[8].y);
                                cvSelectedImg5.draw(corners[8].x, corners[8].y);
                                if(corners.size() == 12){
                                    ofRect(corners[10].x, corners[10].y, corners[11].x - corners[10].x, corners[11].y - corners[10].y);
                                    cvSelectedImg6.draw(corners[10].x, corners[10].y);
                                }
                            }
                        }
                    }
                }
            }
        }
	}

    // draw crosshair on cursor
    ofSetColor(255, 255, 255);
    ofSetLineWidth(2);
	ofLine(mousePt.x+10, mousePt.y, mousePt.x-10, mousePt.y);
	ofLine(mousePt.x, mousePt.y+10, mousePt.x, mousePt.y-10);

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

        case' ':
            gesture_lines.clear();
            break;

        case's':
			findWin = !findWin; // begin or halt window switch
			break;

        case'd':
			drawSel = !drawSel; // begin or halt drawing selections
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
        if(corners.size() < 12)
            corners.push_back(ofVec2f(x, y));
    }
    else{
        if(corners.size() > 0)
            corners.pop_back();
    }
}

//--------------------------------------------------------------
void testApp::mouseReleased(int x, int y, int button){
}

//--------------------------------------------------------------
void testApp::windowResized(int w, int h){
}
