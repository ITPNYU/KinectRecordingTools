#include "cinder/app/AppNative.h"
#include "cinder/app/RendererGl.h"
#include "cinder/gl/gl.h"

#include "cinder/Cinder.h"
#include "cinder/app/App.h"
#include "cinder/app/RendererGl.h"
#include "cinder/gl/Texture.h"
#include "cinder/Capture.h"

#include "Timer.h"
#include "Track.h"
#include "TrackGroup.h"
#include "TypeTrack.h"
#include "Controller.h"

using namespace ci;
using namespace ci::app;
using namespace std;

class RecorderApp : public AppNative {
  public:
	void setup() override;
	void mouseDown(MouseEvent event) override;
	void keyUp(KeyEvent event) override;
	void update() override;
	void draw() override;
	void shutdown() override;
};

void RecorderApp::setup()
{
	sequence::Timer::Ref t = sequence::Timer::create();
}

void RecorderApp::mouseDown( MouseEvent event )
{
}

void RecorderApp::keyUp(KeyEvent event)
{

}

void RecorderApp::update()
{
}

void RecorderApp::draw()
{
}

void RecorderApp::shutdown()
{

}

CINDER_APP_NATIVE( RecorderApp, RendererGl )
