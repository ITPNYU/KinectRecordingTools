#include "cinder/app/App.h"
#include "cinder/app/RendererGl.h"
#include "cinder/gl/gl.h"
#include "cinder/gl/Fbo.h"
#include "cinder/ImageIo.h"
#include "cinder/Utilities.h"

#include "Kinect2.h"

#include <KinectProcessingGlsl.h>
#include <multitrack/Controller.h>

#include <foil/oss/gesture/recognizer.hpp>

#define RAW_FRAME_WIDTH  1920
#define RAW_FRAME_HEIGHT 1080

using namespace ci;
using namespace ci::app;
using namespace std;

enum AppState
{
	IDLE,
	WAIT_FOR_SINGLE_USER,
	ESTABLISH_IDLE_POSE,
	ESTABLISH_CONTROL_POSE,
	BEGIN_RECORDING,
	END_RECORDING,
	RECORDING_TRACK
};

static const double kDirectiveCountdownSec = 3.0;
static const double kRecognitionThreshold  = 0.85;

class HelloKinectMultitrackGestureApp : public App {
public:
	void setup() override;
	void update() override;
	void draw() override;
	void cleanup() override;

	void mouseDown(MouseEvent event) override;
	void keyUp(KeyEvent event) override;

	void startRecording();
	void completeRecording();
	void cancelRecording();

	bool addGestureTemplate(const std::string& poseName);
	void renderSilhouette();

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

	itp::multitrack::Controller::Ref	mMultitrackController;

	ci::Font							mFont;
	std::string							mInfoLabel;
	AppState							mAppState;
	double								mStateStartTime;
	size_t								mActiveBodyCount;

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
	// Setup application state:
	mFont = Font("Helvetica", 24);
	mInfoLabel = "";
	mAppState = AppState::WAIT_FOR_SINGLE_USER;
	mStateStartTime = 0.0;
	mActiveBodyCount = 0;
	mEstablishedPoseIdle = false;
	mEstablishedPoseControl = false;
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
	// Check for single user:
	if (mActiveBodyCount == 1) {
		if (mAppState == AppState::WAIT_FOR_SINGLE_USER) {
			mAppState = AppState::IDLE;
			mInfoLabel = "";
			//mMultitrackController->cancelRecorder();
			//mMultitrackController->stop();
			//mMultitrackController->resetTimer();
		}
		else if (mAppState == AppState::IDLE) {
			if (!mEstablishedPoseIdle) {
				mAppState = AppState::ESTABLISH_IDLE_POSE;
				mStateStartTime = getElapsedSeconds();
			}
			else if (!mEstablishedPoseControl) {
				mAppState = AppState::ESTABLISH_CONTROL_POSE;
				mStateStartTime = getElapsedSeconds();
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
							mAppState = AppState::BEGIN_RECORDING;
							mInfoLabel = "";
							mStateStartTime = getElapsedSeconds();
						}
						else {
							mInfoLabel = "Best guess: " + tResult.mName + " " + std::to_string(tResult.mScore);
						}
					}
				}
			}
		}
		else if (mAppState == AppState::ESTABLISH_IDLE_POSE) {
			double tDelta = getElapsedSeconds() - mStateStartTime;
			if (kDirectiveCountdownSec - tDelta <= 0.0) {
				if (addGestureTemplate("IDLE")) {
					mEstablishedPoseIdle = true;
				}
				mAppState = AppState::IDLE;
				mInfoLabel = "";
			}
			else {
				mInfoLabel = std::to_string(kDirectiveCountdownSec - tDelta) + " seconds remaining to establish idle pose";
			}
		}
		else if (mAppState == AppState::ESTABLISH_CONTROL_POSE) {
			double tDelta = getElapsedSeconds() - mStateStartTime;
			if (kDirectiveCountdownSec - tDelta <= 0.0) {
				if (addGestureTemplate("CONTROL")) {
					mEstablishedPoseControl = true;
				}
				mAppState = AppState::IDLE;
				mInfoLabel = "";
			}
			else {
				mInfoLabel = std::to_string(kDirectiveCountdownSec - tDelta) + " seconds remaining to establish control pose";
			}
		}
		else if (mAppState == AppState::BEGIN_RECORDING) {
			double tDelta = getElapsedSeconds() - mStateStartTime;
			if (kDirectiveCountdownSec - tDelta <= 0.0) {
				mAppState = AppState::RECORDING_TRACK;
				mInfoLabel = "";
				startRecording();
			}
			else {
				mInfoLabel = std::to_string(kDirectiveCountdownSec - tDelta) + " seconds before recording begins";
			}
		}
		else if (mAppState == AppState::END_RECORDING) {
			double tDelta = getElapsedSeconds() - mStateStartTime;
			if (kDirectiveCountdownSec - tDelta <= 0.0) {
				mAppState = AppState::IDLE;
			}
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
						mAppState = AppState::END_RECORDING;
						mInfoLabel = "";
						mStateStartTime = getElapsedSeconds();
						completeRecording();
					}
				}
			}
		}
	}
	else {
		mAppState = AppState::WAIT_FOR_SINGLE_USER;
		mInfoLabel = "I see " + std::to_string(mActiveBodyCount) + " users, but need one.";
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
	//
	if (!mInfoLabel.empty())
		gl::drawStringCentered(mInfoLabel, vec2(getWindowWidth() * 0.5f, getWindowHeight() - 100.0f), Color(1, 1, 1), mFont);
	//
	switch (mAppState) {
		case AppState::WAIT_FOR_SINGLE_USER: {
			break;
		}
		default: {
			mMultitrackController->draw();
			break;
		}
	}
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
	switch (event.getChar()) {
	case 'r': {
		cancelRecording();
		break;
	}
	case 'a': {
		startRecording();
		break;
	}
	case 'c': {
		completeRecording();
		break;
	}
	default: { break; }
	}
}

void HelloKinectMultitrackGestureApp::startRecording()
{
	// Create image recorder callback lambda:
	auto tImgRecorderCallbackFn = [&](void) -> ci::SurfaceRef
	{
		renderSilhouette();
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
			gl::drawSolidCircle(pt, 5.0f, 32);
		}
	};
	// Create body recorder track:
	mMultitrackController->addRecorder<itp::multitrack::PointCloudRef>(tBodyRecorderCallbackFn, tBodyPlayerCallbackFn);
}

void HelloKinectMultitrackGestureApp::completeRecording()
{
	mMultitrackController->completeRecorder();
	mMultitrackController->start();
}

void HelloKinectMultitrackGestureApp::cancelRecording()
{
	mMultitrackController->cancelRecorder();
	mMultitrackController->start();
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

void HelloKinectMultitrackGestureApp::renderSilhouette()
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
	}
}

CINDER_APP(HelloKinectMultitrackGestureApp, RendererGl, [](App::Settings* settings)
{
	settings->prepareWindow(Window::Format().size(1024, 768).title("ITP Kinect Recording Tools"));
	settings->setFrameRate(60.0f);
})
