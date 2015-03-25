#include "cinder/app/App.h"
#include "cinder/app/RendererGl.h"
#include "cinder/gl/gl.h"
#include "cinder/gl/Fbo.h"

#include "Kinect2.h"

#include <KinectProcessingGlsl.h>
#include <multitrack/Controller.h>

#define RAW_FRAME_WIDTH  1920
#define RAW_FRAME_HEIGHT 1080

using namespace ci;
using namespace ci::app;
using namespace std;

class HelloKinectMultitrackApp : public App {
  public:
	void setup() override;
	void update() override;
	void draw() override;
	void cleanup() override;

	void mouseDown(MouseEvent event) override;
	void keyUp(KeyEvent event) override;

	void renderSilhouette(bool iDrawSkeletons = false);

	long long							mTimeStamp;
	long long							mTimeStampPrev;

	ci::gl::GlslProgRef					mGlslProg;

	Kinect2::DeviceRef					mDevice;

	Kinect2::BodyFrame					mBodyFrame;

	ci::Channel8uRef					mChannelBody;
	ci::Surface8uRef					mSurfaceColor;
	ci::Channel16uRef					mChannelDepth;

	ci::Surface32fRef					mSurfaceLookup;

	ci::gl::TextureRef					mTextureBody;
	ci::gl::TextureRef					mTextureColor;
	ci::gl::TextureRef					mTextureDepth;
	ci::gl::TextureRef					mTextureLookup;

	ci::gl::FboRef						mSilhouetteFbo;

	itp::multitrack::Timer::Ref			mMultitrackTimer;
	itp::multitrack::Track::Ref			mMultitrackCursor;
	itp::multitrack::TrackGroup::Ref	mMultitrackSequence;
	itp::multitrack::Controller::Ref	mMultitrackController;
};

void HelloKinectMultitrackApp::setup()
{
	// Enable texture mode:
	ci::gl::enable(GL_TEXTURE_2D);
	// Initialize timestamp:
	mTimeStamp = 0L;
	mTimeStampPrev = mTimeStamp;
	// Setup shader:
	try {
		mGlslProg = itp::createKinectAlignSilhouetteShader();
	}
	catch (ci::gl::GlslProgCompileExc ex) {
		ci::app::console() << "GLSL Error: " << ex.what() << std::endl;
		quit();
	}
	catch (ci::gl::GlslNullProgramExc ex) {
		ci::app::console() << "GLSL Error: " << ex.what() << std::endl;
		quit();
	}
	catch (...) {
		ci::app::console() << "Unknown GLSL Error" << std::endl;
		quit();
	}
	// Initialize Kinect and register callbacks:
	mDevice = Kinect2::Device::create();
	mDevice->start();
	mDevice->connectBodyEventHandler([&](const Kinect2::BodyFrame& frame)
	{
		mBodyFrame = frame;
	});
	mDevice->connectBodyIndexEventHandler([&](const Kinect2::BodyIndexFrame& frame)
	{
		mChannelBody = frame.getChannel();
	});
	mDevice->connectColorEventHandler([&](const Kinect2::ColorFrame& frame)
	{
		mSurfaceColor = frame.getSurface();
	});
	mDevice->connectDepthEventHandler([&](const Kinect2::DepthFrame& frame)
	{
		mChannelDepth = frame.getChannel();
		mTimeStamp = frame.getTimeStamp();
	});
	// Setup FBO:
	ci::gl::Fbo::Format tSilhouetteFboFormat;
	mSilhouetteFbo = ci::gl::Fbo::create(RAW_FRAME_WIDTH, RAW_FRAME_HEIGHT, tSilhouetteFboFormat.colorTexture());
	// Setup multitrack controller:
	mMultitrackController = itp::multitrack::Controller::create();
	mMultitrackController->start();
}

void HelloKinectMultitrackApp::update()
{
	// Check whether depth-to-color mapping update is needed:
	if ((mTimeStamp != mTimeStampPrev) && mSurfaceColor && mChannelDepth) {
		// Update timestamp:
		mTimeStampPrev = mTimeStamp;
		// Initialize lookup surface:
		mSurfaceLookup = ci::Surface32f::create(mChannelDepth->getWidth(), mChannelDepth->getHeight(), false, ci::SurfaceChannelOrder::RGB);
		// Get depth-to-color mapping points:
		std::vector<ci::ivec2> tMappingPoints = mDevice->mapDepthToColor(mChannelDepth);
		// Get color frame dimension:
		ci::vec2 tColorFrameDim(Kinect2::ColorFrame().getSize());
		// Prepare iterators:
		ci::Surface32f::Iter iter = mSurfaceLookup->getIter();
		std::vector<ci::ivec2>::iterator v = tMappingPoints.begin();
		// Create lookup mapping:
		while (iter.line()) {
			while (iter.pixel()) {
				iter.r() = (float)v->x / tColorFrameDim.x;
				iter.g() = 1.0f - (float)v->y / tColorFrameDim.y;
				iter.b() = 0.0f;
				++v;
			}
		}
	}
	// Update multitrack controller:
	mMultitrackController->update();
}

void HelloKinectMultitrackApp::draw()
{
	gl::clear(Color(0, 0, 0));
	gl::enableAlphaBlending();
	gl::setMatricesWindow(getWindowSize());
	gl::enable(GL_TEXTURE_2D);
	// Draw multitrack controller:
	mMultitrackController->draw();
}

void HelloKinectMultitrackApp::cleanup()
{
	mMultitrackController->stop();
}

void HelloKinectMultitrackApp::mouseDown(MouseEvent event)
{
}

void HelloKinectMultitrackApp::keyUp(KeyEvent event)
{
	switch (event.getChar()) {
	case 'r': {
		mMultitrackController->cancelRecorder();
		mMultitrackController->start();
		break;
	}
	case 'a': {
		// Create recorder callback lambda:
		auto tRecorderCallbackFn = [&](void) -> ci::SurfaceRef
		{
			renderSilhouette(false);
			return std::make_shared<Surface8u>(mSilhouetteFbo->readPixels8u(mSilhouetteFbo->getBounds()));
		};
		// Create player callback lambda:
		auto tPlayerCallbackFn = [&](const ci::SurfaceRef& iSurface) -> void
		{
			ci::gl::draw(ci::gl::Texture::create(*(iSurface.get())), ci::app::getWindowBounds());
		};
		// Create recorder track:
		mMultitrackController->addRecorder<ci::SurfaceRef>(tRecorderCallbackFn, tPlayerCallbackFn);
		//
		break;
	}
	case 'c': {
		mMultitrackController->completeRecorder();
		break;
	}
	default: { break; }
	}
}

void HelloKinectMultitrackApp::renderSilhouette(bool iDrawSkeletons)
{
	gl::ScopedFramebuffer fbScp(mSilhouetteFbo);
	gl::clear( ColorA( 0, 0, 0, 0 ) ); 
	gl::ScopedViewport scpVp(ivec2(0), mSilhouetteFbo->getSize());
	gl::setMatricesWindow(mSilhouetteFbo->getSize());
	// Check for necessary inputs:
	if (mSurfaceColor && mChannelDepth && mSurfaceLookup && mChannelBody) {
		gl::enable(GL_TEXTURE_2D);
		// Generate color texture:
		if (mTextureColor) {
			mTextureColor->update(*(mSurfaceColor.get()));
		}
		else {
			mTextureColor = ci::gl::Texture::create(*(mSurfaceColor.get()));
		}
		// Bind color texture:
		mTextureColor->bind(0);
		// Generate depth texture:
		if (mTextureDepth) {
			mTextureDepth->update(*(Kinect2::channel16To8(mChannelDepth).get()));
		}
		else {
			mTextureDepth = ci::gl::Texture::create(*(Kinect2::channel16To8(mChannelDepth).get()));
		}
		// Bind depth texture:
		mTextureDepth->bind(1);
		// Generate lookup texture:
		if (mTextureLookup) {
			mTextureLookup->update(*(mSurfaceLookup.get()));
		}
		else {
			mTextureLookup = ci::gl::Texture::create(*(mSurfaceLookup.get()), ci::gl::Texture::Format().dataType(GL_FLOAT));
		}
		// Bind lookup texture:
		mTextureLookup->bind(2);
		// Generate body-index texture:
		if (mTextureBody) {
			mTextureBody->update(*(mChannelBody.get()));
		}
		else {
			mTextureBody = ci::gl::Texture::create(*(mChannelBody.get()));
		}
		// Bind body-index texture:
		mTextureBody->bind(3);
		// Bind shader and draw:
		{
			// Bind shader:
			ci::gl::ScopedGlslProg shaderBind(mGlslProg);
			// Bind uniforms:
			ci::gl::setDefaultShaderVars();
			mGlslProg->uniform("uTextureColor", 0);
			mGlslProg->uniform("uTextureLookup", 2);
			mGlslProg->uniform("uTextureBody", 3);
			mGlslProg->uniform("uSilhouette", false);
			// Set color:
			ci::gl::color(1.0, 1.0, 1.0, 1.0);
			// Draw rect:
			ci::gl::drawSolidRect(mSilhouetteFbo->getBounds());
		}
		// Unbind textures:
		mTextureColor->unbind();
		mTextureDepth->unbind();
		mTextureLookup->unbind();
		// Draw skeletons:
		if (iDrawSkeletons) {
			gl::ScopedMatrices scopeMatrices;
			gl::scale(vec2(mSilhouetteFbo->getSize()) / vec2(mChannelBody->getSize()));
			gl::disable(GL_TEXTURE_2D);
			for (const Kinect2::Body& body : mBodyFrame.getBodies()) {
				if (body.isTracked()) {
					gl::color(ColorAf::white());
					for (const auto& joint : body.getJointMap()) {
						if (joint.second.getTrackingState() == TrackingState::TrackingState_Tracked) {
							vec2 pos(mDevice->mapCameraToDepth(joint.second.getPosition()));
							gl::drawSolidCircle(pos, 5.0f, 32);
							vec2 parent(mDevice->mapCameraToDepth(
								body.getJointMap().at(joint.second.getParentJoint()).getPosition()
								));
							gl::drawLine(pos, parent);
						}
					}
				}
			}
		}
	}
}

CINDER_APP(HelloKinectMultitrackApp, RendererGl, [](App::Settings* settings)
{
	settings->prepareWindow(Window::Format().size(1024, 768).title("ITP Kinect Recording Tools"));
	settings->setFrameRate(60.0f);
} )
