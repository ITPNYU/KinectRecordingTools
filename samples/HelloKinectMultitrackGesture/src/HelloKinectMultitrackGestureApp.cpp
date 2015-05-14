#include "cinder/app/App.h"
#include "cinder/app/RendererGl.h"
#include "cinder/gl/gl.h"
#include "cinder/gl/Fbo.h"
#include "cinder/ImageIo.h"
#include "cinder/Utilities.h"
#include "cinder/Timeline.h"

#include "Kinect2.h"

#include <KinectProcessingGlsl.h>
#include <multitrack/Controller.h>

#include <foil/oss/gesture/recognizer.hpp>

using namespace ci;
using namespace ci::app;
using namespace std;

#define RAW_FRAME_WIDTH  960
#define RAW_FRAME_HEIGHT 540

static const double kRecognitionThreshold  = 0.85;

enum AppState
{
	NONE,
	IDLE,
	WAIT_FOR_SINGLE_USER,
	ESTABLISH_IDLE_POSE,
	ESTABLISH_CONTROL_POSE,
	BEGIN_RECORDING,
	RECORDING_TRACK
};

struct GestureHolder
{
	AppState	m_targ;
	size_t		m_pos;
	size_t		m_neg;

	GestureHolder(AppState targ = AppState::NONE)
		: m_targ(targ), m_pos(0), m_neg(0) { /* no-op */ }

	// TODO UNUSED WORK IN PROGRESS !
};


class HelloKinectMultitrackGestureApp : public App {
public:
	void setup() override;
	void update() override;
	void draw() override;
	void cleanup() override;

	void mouseDown(MouseEvent event) override;
	void keyUp(KeyEvent event) override;

	void transitionToState(AppState targetState, float transitionDuration, const std::string& updateMsg);
	void startStateTransition();
	void updateStateTransition(const std::string& updateMsg);
	void finishStateTransition(AppState targetState);

	void addTrack();

	bool addGestureTemplate(const std::string& poseName);

	void renderSilhouetteGpu();

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
	ci::gl::TextureRef					mTextureLookup;
	ci::gl::FboRef						mSilhouetteFbo;

	itp::multitrack::Controller::Ref	mMultitrackController;

	ci::Font							mFont;
	std::string							mInfoLabel;
	AppState							mAppState;
	size_t								mActiveBodyCount;

	float								mStateTransitionShort;
	float								mStateTransitionMedium;
	float								mStateTransitionLong;

	bool								mInStateTransition;
	ci::Anim<float>						mTransitionAnim;

	bool								mEstablishedPoseIdle;
	bool								mEstablishedPoseControl;

	foil::gesture::Recognizer			mRecognizer;
};

void HelloKinectMultitrackGestureApp::setup()
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
		mActiveBodyCount = 0;
		for (const auto& body : frame.getBodies()) {
			if (body.calcConfidence() > 0.5) {
				mActiveBodyCount++;
			}
		}
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
	mMultitrackController = itp::multitrack::Controller::create(getHomeDirectory() / "Desktop" / "Tests");
	mMultitrackController->start();
	// Set default state transition duration:
	mStateTransitionShort  = 2.0f;
	mStateTransitionMedium = 4.0f;
	mStateTransitionLong   = 6.0f;
	// Initialize application state:
	mFont = Font("Helvetica", 40);
	mInfoLabel = "";
	mAppState = AppState::WAIT_FOR_SINGLE_USER;
	mActiveBodyCount = 0;
	mInStateTransition = false;
	mEstablishedPoseIdle = false;
	mEstablishedPoseControl = false;
	// Add initial track:
	addTrack();
}

void HelloKinectMultitrackGestureApp::update()
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

	// If in state transition, block new transitions:
	if (!mInStateTransition) {
		// Handle states:
		if (mActiveBodyCount != 1) {
			mAppState = AppState::WAIT_FOR_SINGLE_USER;
			mInfoLabel = "I see " + std::to_string(mActiveBodyCount) + " users, but need one.";
		}
		else if (mAppState == AppState::WAIT_FOR_SINGLE_USER) {
			transitionToState(AppState::IDLE, mStateTransitionMedium, "Oh hello! We'll get started in ");
		}
		else if (mAppState == AppState::IDLE) {
			if (!mEstablishedPoseIdle) {
				transitionToState(AppState::ESTABLISH_IDLE_POSE, mStateTransitionLong, "Establishing IDLE POSE in ");
			}
			else if (!mEstablishedPoseControl) {
				transitionToState(AppState::ESTABLISH_CONTROL_POSE, mStateTransitionLong, "Establishing CONTROL POSE in ");
			}
			else {
				// Check whether recognizer has templates:
				if (mRecognizer.hasTemplates()) {
					// Get point cloud:
					itp::multitrack::PointCloud tCloud = itp::multitrack::PointCloud(mBodyFrame, mDevice);
					// Check for correct point count for single body:
					if (tCloud.mPoints.size() == 25) {
						// Get gesture guess:
						foil::gesture::Result tResult = mRecognizer.recognizeBest({ tCloud.mPoints });
						// Check for control gesture:
						if (tResult.mName == "CONTROL" && tResult.mScore >= kRecognitionThreshold) {
							transitionToState(AppState::BEGIN_RECORDING, mStateTransitionLong, "Recording performance in ");
						}
						else {
							mInfoLabel = "Best guess: " + tResult.mName + " " + std::to_string(tResult.mScore);
						}
					}
				}
			}
		}
		else if (mAppState == AppState::ESTABLISH_IDLE_POSE) {
			if (addGestureTemplate("IDLE")) {
				mEstablishedPoseIdle = true;
			}
			transitionToState(AppState::IDLE, mStateTransitionShort, "Thanks! We'll be back in ");
		}
		else if (mAppState == AppState::ESTABLISH_CONTROL_POSE) {
			if (addGestureTemplate("CONTROL")) {
				mEstablishedPoseControl = true;
			}
			transitionToState(AppState::IDLE, mStateTransitionShort, "Thanks! We'll be back in ");
		}
		else if (mAppState == AppState::BEGIN_RECORDING) {
			mMultitrackController->start();
			mAppState = AppState::RECORDING_TRACK;
			mInfoLabel = "";
		}
		else if (mAppState == AppState::RECORDING_TRACK) {
			// Check whether recognizer has templates:
			if (mRecognizer.hasTemplates()) {
				// Get point cloud:
				itp::multitrack::PointCloud tCloud = itp::multitrack::PointCloud(mBodyFrame, mDevice);
				// Check for correct point count for single body:
				if (tCloud.mPoints.size() == 25) {
					// Get gesture guess:
					foil::gesture::Result tResult = mRecognizer.recognizeBest({ tCloud.mPoints });
					// Check for control gesture:
					if (tResult.mName == "CONTROL" && tResult.mScore >= kRecognitionThreshold) {
						// Complete track:
						mMultitrackController->completeRecorder();
						// Setup next preview track and return to idle state:
						addTrack();
						transitionToState(AppState::IDLE, mStateTransitionShort, "Thanks! We'll be back in ");
					}
				}
			}
		}
	}
	// Update multitrack controller:
	mMultitrackController->update();
}

void HelloKinectMultitrackGestureApp::draw()
{
	gl::clear(Color(0, 0, 0));
	gl::enableAlphaBlending();
	gl::setMatricesWindow(getWindowSize());
	gl::enable(GL_TEXTURE_2D);
	// Draw controller:
	mMultitrackController->draw();
	// Draw info:
	if (!mInfoLabel.empty())
		gl::drawStringCentered(mInfoLabel, vec2(getWindowWidth() * 0.5f, getWindowHeight() - 100.0f), Color(1, 1, 1), mFont);
	// Draw time:
	if (mAppState == AppState::RECORDING_TRACK)
		gl::drawString(std::to_string(mMultitrackController->getTimer()->getPlayhead()), vec2(25, 25), Color(1, 1, 1), mFont);
}

void HelloKinectMultitrackGestureApp::cleanup()
{
	mMultitrackController->stop();
}

void HelloKinectMultitrackGestureApp::mouseDown(MouseEvent event)
{
}

void HelloKinectMultitrackGestureApp::keyUp(KeyEvent event)
{
}

void HelloKinectMultitrackGestureApp::transitionToState(AppState targetState, float transitionDuration, const std::string& updateMsg)
{
	mTransitionAnim = transitionDuration;
	ci::app::timeline().apply(&mTransitionAnim, 0.0f, transitionDuration, EaseNone())
		.startFn(std::bind(&HelloKinectMultitrackGestureApp::startStateTransition, this))
		.updateFn(std::bind(&HelloKinectMultitrackGestureApp::updateStateTransition, this, updateMsg))
		.finishFn(std::bind(&HelloKinectMultitrackGestureApp::finishStateTransition, this, targetState))
		;
}

void HelloKinectMultitrackGestureApp::startStateTransition()
{
	mInStateTransition = true;
}

void HelloKinectMultitrackGestureApp::updateStateTransition(const std::string& updateMsg)
{
	mInfoLabel = updateMsg + std::to_string(static_cast<int>(mTransitionAnim)) + " seconds";
}

void HelloKinectMultitrackGestureApp::finishStateTransition(AppState targetState)
{
	mAppState = targetState;
	mInStateTransition = false;
}

void HelloKinectMultitrackGestureApp::addTrack()
{
	// Create image recorder callback lambda:
	auto tImgRecorderCallbackFn = [&](void) -> ci::SurfaceRef
	{
		renderSilhouetteGpu();
		return std::make_shared<Surface8u>(mSilhouetteFbo->readPixels8u(mSilhouetteFbo->getBounds()));
	};
	// Create image player callback lambda:
	auto tImgPlayerCallbackFn = [&](const ci::SurfaceRef& iSurface) -> void
	{
		if (iSurface.get() == NULL) return;
		gl::enable(GL_TEXTURE_2D);
		ci::gl::draw(ci::gl::Texture::create(*(iSurface.get())), ci::app::getWindowBounds());
	};
	// Create image recorder track:
	mMultitrackController->addRecorder<ci::SurfaceRef>(tImgRecorderCallbackFn, tImgPlayerCallbackFn);
	/*
	// Create body recorder callback lambda:
	auto tBodyRecorderCallbackFn = [&](void) -> itp::multitrack::PointCloudRef
	{
		return std::make_shared<itp::multitrack::PointCloud>(itp::multitrack::PointCloud(mBodyFrame, mDevice));
	};
	// Create body player callback lambda:
	auto tBodyPlayerCallbackFn = [&](const itp::multitrack::PointCloudRef& iFrame) -> void
	{
		if (iFrame.get() == NULL || mChannelBody.get() == NULL) return;
		gl::ScopedMatrices scopeMatrices;
		gl::scale(vec2(getWindowSize()) / vec2(mChannelBody->getSize()));
		gl::disable(GL_TEXTURE_2D);
		gl::color(ColorAf::white());
		for (const auto& pt : iFrame->mPoints) {
			gl::drawSolidCircle(pt, 3.0f, 10);
		}
	};
	// Create body recorder track:
	mMultitrackController->addRecorder<itp::multitrack::PointCloudRef>(tBodyRecorderCallbackFn, tBodyPlayerCallbackFn);
	*/
}

bool HelloKinectMultitrackGestureApp::addGestureTemplate(const std::string& poseName)
{
	// Get point cloud:
	itp::multitrack::PointCloud tCloud = itp::multitrack::PointCloud(mBodyFrame, mDevice);
	// Check for correct point count for single body:
	if (tCloud.mPoints.size() == 25) {
		mRecognizer.addTemplate(poseName, { tCloud.mPoints });
		return true;
	}
	return false;
}

void HelloKinectMultitrackGestureApp::renderSilhouetteGpu()
{
	gl::ScopedFramebuffer fbScp(mSilhouetteFbo);
	gl::clear(ColorA(0, 0, 0, 0));
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
		// Generate lookup texture:
		if (mTextureLookup) {
			mTextureLookup->update(*(mSurfaceLookup.get()));
		}
		else {
			mTextureLookup = ci::gl::Texture::create(*(mSurfaceLookup.get()), ci::gl::Texture::Format().dataType(GL_FLOAT));
		}
		// Bind lookup texture:
		mTextureLookup->bind(1);
		// Generate body-index texture:
		if (mTextureBody) {
			mTextureBody->update(*(mChannelBody.get()));
		}
		else {
			mTextureBody = ci::gl::Texture::create(*(mChannelBody.get()));
		}
		// Bind body-index texture:
		mTextureBody->bind(2);
		// Bind shader and draw:
		{
			// Bind shader:
			ci::gl::ScopedGlslProg shaderBind(mGlslProg);
			// Bind uniforms:
			ci::gl::setDefaultShaderVars();
			mGlslProg->uniform("uTextureColor", 0);
			mGlslProg->uniform("uTextureLookup", 1);
			mGlslProg->uniform("uTextureBody", 2);
			mGlslProg->uniform("uSilhouette", false);
			// Set color:
			ci::gl::color(1.0, 1.0, 1.0, 1.0);
			// Draw rect:
			ci::gl::drawSolidRect(mSilhouetteFbo->getBounds());
		}
		// Unbind textures:
		mTextureColor->unbind();
		mTextureLookup->unbind();
	}
}

CINDER_APP(HelloKinectMultitrackGestureApp, RendererGl, [](App::Settings* settings)
{
	settings->prepareWindow(Window::Format().size(1024, 768).title("ITP Kinect Recording Tools"));
	settings->setFrameRate(60.0f);
})
