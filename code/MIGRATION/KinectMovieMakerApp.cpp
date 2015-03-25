#include "Controller.h"

using namespace std;
using namespace ci;
using namespace ci::app;

class KinectMovieMakerApp : public ci::app::AppBasic {
public:

	void prepareSettings(ci::app::AppBasic::Settings* settings) override;
	
	void setup() override;
	void update() override;
	void draw() override;

	void fileDrop(ci::app::FileDropEvent event) override;
	void keyDown(ci::app::KeyEvent event) override;

private:

	Controller::Ref mController;

	int mCurrentFrame; // todo temp
};

void KinectMovieMakerApp::prepareSettings(Settings* settings)
{
	settings->prepareWindow(ci::app::Window::Format().size(1024, 768).title("ITP FoS Sketch"));
	settings->setFrameRate(60.0f);
}

void KinectMovieMakerApp::setup()
{
	mCurrentFrame = 0; // todo temp

	// Create controller:
	mController = Controller::create( this, "controller" );
}

void KinectMovieMakerApp::update()
{
	// Update controller:
	mController->update();
}

void KinectMovieMakerApp::draw()
{
	// Prepare context:
	gl::viewport(getWindowSize());
	gl::clear();
	gl::setMatricesWindow(getWindowSize());
	gl::enableAlphaBlending();
	// Draw controller:
	mController->draw();


	// TODO: debug save frame:
	//ci::writeImage(getHomeDirectory() / "cinder" / "saveImage_" / (toString(mCurrentFrame) + ".png"), copyWindowSurface());
	//mCurrentFrame++;
}

void KinectMovieMakerApp::fileDrop(ci::app::FileDropEvent event)
{
	mController->fileDrop(event);
}

void KinectMovieMakerApp::keyDown(ci::app::KeyEvent event)
{
}

CINDER_APP_BASIC(KinectMovieMakerApp, RendererGl)
