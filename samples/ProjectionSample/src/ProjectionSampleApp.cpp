#include "cinder/app/App.h"
#include "cinder/app/RendererGl.h"
#include "cinder/gl/gl.h"
#include "Projection.h"
#include "cinder/Camera.h"
#include "cinder/Utilities.h"


#include <stdlib.h> //for rand()

using namespace ci;
using namespace ci::app;
using namespace std;
using namespace itp;

class ProjectionSampleApp : public App {
  public:
	void setup() override;
	void mouseDown( MouseEvent event ) override;
	void update() override;
	void draw() override;

	// CAMERAS
	CameraPersp	mPcam;
	CameraOrtho mOcam;
	float mPcamDistance, mOcamDistance;
	vec3 mEye, mCenter, mUp;

	Projection persp, ortho;	
};

void ProjectionSampleApp::setup()
{
	mPcamDistance = 400.0f;
	mEye = vec3(0.0f, 0.0f, mPcamDistance);
	mCenter = vec3(0.0f, 1.0f, 0.0f);
	mUp = vec3(0.0f, 1.0f, 0.0f);

	mPcam.setPerspective(75.0f, getWindowAspectRatio(), 5.0f, 2000.0f);
	mOcam.setOrtho(0, getWindowWidth(), getWindowHeight(), 0, -1, 1);

	persp = Projection(mPcam, getWindowSize());
	ortho = Projection(mOcam, getWindowSize());	
}

void ProjectionSampleApp::mouseDown( MouseEvent event )
{
}

void ProjectionSampleApp::update()
{
	// Update Cameras
	mEye = vec3(0.0f, 0.0f, mPcamDistance);
	mPcam.lookAt(mEye, mCenter, mUp);

	// Compute orthogonal cam parameters
	float tParamX = getWindowWidth() / 2.0;
	float tParamY = getWindowHeight() / 2.0;

	mOcam.setOrtho(-tParamX, tParamX, -tParamY, tParamY, -1000, 1000);
	mOcam.lookAt(vec3(0, 2, 0), vec3(0, 0, 0));

	// Test World to Screen projection. Ensure Projection.setView() is setup.
	vec3 randomPoint = vec3(rand() % 100, rand() % 100, rand() % 100);
	ci::app::console() << "Point in World space: " << randomPoint << endl;
	ci::app::console() << "Perspective screen space: " << persp.worldToScreen(randomPoint) << endl;
	ci::app::console() << "Orthogonal screen space: " << ortho.worldToScreen(randomPoint) << endl << endl;
	ci::app::console() << "=====================================" << endl << endl;
}

void ProjectionSampleApp::draw()
{
	gl::clear( Color( 0, 0, 0 ) );

}

CINDER_APP(ProjectionSampleApp, RendererGl, [](App::Settings* settings)
{
	settings->prepareWindow(Window::Format().size(1280, 720).title("ITP Kinect Recording Tools"));
	settings->setFrameRate(60.0f);
} )
