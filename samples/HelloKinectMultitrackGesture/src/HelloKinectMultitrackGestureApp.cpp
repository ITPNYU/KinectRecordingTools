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

static const size_t kRawFrameWidth = 960;
static const size_t kRawFrameHeight = 540;

static const size_t kBodyPointCount = 25;

static const double kRecognitionThreshold  = 0.85;
static const size_t kRecognitionSamples    = 30;
static const size_t kRecognitionSamplesMin = 25;

static const double kSceneDurationSec = 20.0;

enum AppState
{
	NONE,
	HOME,
	CHOOSE_ACTIVITY,
	WAIT_FOR_SINGLE_USER,
	ESTABLISH_IDLE_POSE,
	ESTABLISH_CONTROL_POSE,
	ESTABLISH_ACTOR_POSE,
	BEGIN_ACTOR,
	RECORD_ACTOR
};

struct CaptionImage
{
	typedef std::deque<CaptionImage> Deque;

	ci::gl::TextureRef	mTex;
	std::string			mMsg;
	ci::vec2			mDim;
};

class HelloKinectMultitrackGestureApp : public App {
public:
	void setup() override;
	void update() override;
	void draw() override;
	void cleanup() override;

	void mouseDown(MouseEvent event) override;
	void keyUp(KeyEvent event) override;

	void drawCaption(const CaptionImage& caption);
	void drawCaptions(const CaptionImage::Deque& captions);

	void transitionToState(AppState targetState, float transitionDuration, const std::string& updateMsg);
	void startStateTransition();
	void updateStateTransition(const std::string& updateMsg);
	void finishStateTransition(AppState targetState);

	void receiveLoopCallback();

	void addTrack();

	bool addGestureTemplate(const std::string& poseName);
	bool analyzeGesture(std::string* recognizedGesture);

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
	bool								mEstablishedPoseActor;

	ci::gl::TextureRef					mTexturePoseIdle;
	ci::gl::TextureRef					mTexturePoseControl;
	ci::gl::TextureRef					mTexturePoseActor;

	CaptionImage::Deque					mActiveCaptions;

	foil::gesture::Recognizer			mRecognizer;
	foil::gesture::Result::Deque		mRecognizerBuffer;
};

void HelloKinectMultitrackGestureApp::drawCaption(const CaptionImage& caption)
{
	Rectf rect = Rectf(vec2(0.0, 0.0), caption.mDim);
	gl::color(1, 1, 1);
	gl::drawSolidRect(rect);
	gl::draw(caption.mTex, rect);
	gl::drawString(caption.mMsg, vec2(caption.mDim.x + 20.0, caption.mDim.y * 0.5), Color(1, 1, 1), mFont);
}

void HelloKinectMultitrackGestureApp::drawCaptions(const CaptionImage::Deque& captions)
{
	float padding = 5.0; // TODO: externalize

	vec2 offset(0.0, 0.0);
	for (const auto& caption : captions) {
		gl::pushMatrices();
		gl::translate(offset);
		drawCaption(caption);
		gl::popMatrices();
		offset.y += caption.mDim.y + padding;
	}
}

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
	mSilhouetteFbo = ci::gl::Fbo::create(kRawFrameWidth, kRawFrameHeight, tSilhouetteFboFormat.colorTexture());
	// Setup multitrack controller:
	mMultitrackController = itp::multitrack::Controller::create(getHomeDirectory() / "Desktop" / "Tests");
	mMultitrackController->getTimer()->setLoopCallback(std::bind(&HelloKinectMultitrackGestureApp::receiveLoopCallback, this));
	mMultitrackController->getTimer()->setLoopMarker(kSceneDurationSec);
	mMultitrackController->getTimer()->start();
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
	mEstablishedPoseActor = false;
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
			if (mAppState != AppState::WAIT_FOR_SINGLE_USER) {
				mAppState = AppState::WAIT_FOR_SINGLE_USER;
				mActiveCaptions.clear();
			}
			mInfoLabel = "I see " + std::to_string(mActiveBodyCount) + " users, but need one.";
		}
		else if (mAppState == AppState::WAIT_FOR_SINGLE_USER) {
			transitionToState(AppState::HOME, mStateTransitionMedium, "Oh hello! We'll get started in ");
		}
		else if (mAppState == AppState::HOME) {
			if (!mEstablishedPoseIdle) {
				transitionToState(AppState::ESTABLISH_IDLE_POSE, mStateTransitionLong, "Establishing IDLE POSE in ");
			}
			else if (!mEstablishedPoseControl) {
				transitionToState(AppState::ESTABLISH_CONTROL_POSE, mStateTransitionLong, "Establishing CONTROL POSE in ");
			}
			else if (!mEstablishedPoseActor) {
				transitionToState(AppState::ESTABLISH_ACTOR_POSE, mStateTransitionLong, "Establishing ACTOR POSE in ");
			}
			else {
				mMultitrackController->cancelRecorder();
				addTrack();
				mMultitrackController->getTimer()->start();
				mAppState = AppState::CHOOSE_ACTIVITY;
				mInfoLabel = "What's next?";
				mActiveCaptions = {
					{ mTexturePoseControl, "Start a new movie", vec2(240, 135) },
					{ mTexturePoseActor, "Add an actor", vec2(240, 135) }
				};
			}
		}
		else if (mAppState == AppState::CHOOSE_ACTIVITY) {
			// Analyze gesture:
			std::string recognizedGesture;
			if (analyzeGesture(&recognizedGesture)) {
				// Check for control gesture:
				if (recognizedGesture == "CONTROL") {
					mMultitrackController->resetSequence();
					mActiveCaptions.clear();
					transitionToState(AppState::HOME, mStateTransitionMedium, "Starting a new movie in ");
				}
				// Check for actor gesture:
				else if (recognizedGesture == "ACTOR") {
					mActiveCaptions.clear();
					mMultitrackController->getTimer()->stop();
					transitionToState(AppState::BEGIN_ACTOR, mStateTransitionLong, "You're on in ");
				}
			}
		}
		else if (mAppState == AppState::ESTABLISH_IDLE_POSE) {
			if (addGestureTemplate("IDLE")) {
				mTexturePoseIdle = ci::gl::Texture::create(mSilhouetteFbo->readPixels8u(mSilhouetteFbo->getBounds()));
				mEstablishedPoseIdle = true;
			}
			transitionToState(AppState::HOME, mStateTransitionShort, "Thanks! We'll be back in ");
		}
		else if (mAppState == AppState::ESTABLISH_CONTROL_POSE) {
			if (addGestureTemplate("CONTROL")) {
				mTexturePoseControl = ci::gl::Texture::create(mSilhouetteFbo->readPixels8u(mSilhouetteFbo->getBounds()));
				mEstablishedPoseControl = true;
			}
			transitionToState(AppState::HOME, mStateTransitionShort, "Thanks! We'll be back in ");
		}
		else if (mAppState == AppState::ESTABLISH_ACTOR_POSE) {
			if (addGestureTemplate("ACTOR")) {
				mTexturePoseActor = ci::gl::Texture::create(mSilhouetteFbo->readPixels8u(mSilhouetteFbo->getBounds()));
				mEstablishedPoseActor = true;
			}
			transitionToState(AppState::HOME, mStateTransitionShort, "Thanks! We'll be back in ");
		}
		else if (mAppState == AppState::BEGIN_ACTOR) {
			mMultitrackController->getTimer()->start();
			mMultitrackController->start();
			mAppState = AppState::RECORD_ACTOR;
			mInfoLabel = "";
		}
		else if (mAppState == AppState::RECORD_ACTOR) {
			// Analyze gesture:
			std::string recognizedGesture;
			if (analyzeGesture(&recognizedGesture)) {
				// Check for control gesture:
				if (recognizedGesture == "CONTROL") {
					// Complete track:
					mMultitrackController->completeRecorder();
					// Stop timer:
					mMultitrackController->getTimer()->stop();
					// Return to home state:
					transitionToState(AppState::HOME, mStateTransitionShort, "I see you're an actor and a director. We'll be back in ");
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
	// Draw active captions:
	if (!mActiveCaptions.empty()) {
		drawCaptions(mActiveCaptions);
	}
	// Draw info:
	if (!mInfoLabel.empty())
		gl::drawStringCentered(mInfoLabel, vec2(getWindowWidth() * 0.5f, getWindowHeight() - 100.0f), Color(1, 1, 1), mFont);
	// Draw time:
	if (mAppState == AppState::RECORD_ACTOR) {
		const double& playhead = mMultitrackController->getTimer()->getPlayhead();
		if (playhead >= 0.0) {
			gl::drawString(std::to_string(playhead), vec2(25, 25), Color(1, 1, 1), mFont);
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

void HelloKinectMultitrackGestureApp::receiveLoopCallback()
{
	switch (mAppState) {
		case AppState::RECORD_ACTOR: {
			// Complete track:
			mMultitrackController->completeRecorder();
			// Stop timer:
			mMultitrackController->getTimer()->stop();
			// Return to home state:
			transitionToState(AppState::HOME, mStateTransitionShort, "Cut! We'll be back in ");
			break;
		}
		default: { break; }
	}
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
	if (tCloud.mPoints.size() == kBodyPointCount) {
		mRecognizer.addTemplate(poseName, { tCloud.mPoints });
		return true;
	}
	return false;
}

bool HelloKinectMultitrackGestureApp::analyzeGesture(std::string* recognizedGesture)
{
	// Check whether recognizer has templates:
	if (mRecognizer.hasTemplates()) {
		// Get point cloud:
		itp::multitrack::PointCloud tCloud = itp::multitrack::PointCloud(mBodyFrame, mDevice);
		// Check for correct point count for single body:
		if (tCloud.mPoints.size() == kBodyPointCount) {
			// Get gesture guess:
			foil::gesture::Result tResult = mRecognizer.recognizeBest({ tCloud.mPoints });
			// Check whether sample count has been reached:
			if (mRecognizerBuffer.size() == kRecognitionSamples) {
				mRecognizerBuffer.pop_front();
				mRecognizerBuffer.push_back(tResult);
				// Compute histogram (TODO optimize):
				std::map< std::string, std::pair<size_t, float> > histo;
				for (const auto& item : mRecognizerBuffer) {
					std::map< std::string, std::pair<size_t, float> >::iterator findIt = histo.find( item.mName );
					if (findIt == histo.end()) {
						histo[item.mName] = std::pair<size_t, float>(1, item.mScore);
					}
					else {
						(*findIt).second.first++;
						(*findIt).second.second += item.mScore;
					}
				}
				size_t bestCount = 0;
				std::map< std::string, std::pair<size_t, float> >::const_iterator bestIt = histo.cend();
				for (std::map< std::string, std::pair<size_t, float> >::const_iterator it = histo.cbegin(); it != histo.cend(); it++) {
					if ( (*it).second.first > bestCount ) {
						bestCount = (*it).second.first;
						bestIt = it;
					}
				}
				// Check recognition match criteria:
				if (bestCount >= kRecognitionSamplesMin && bestIt != histo.cend()) {
					float score = (*bestIt).second.second / static_cast<float>(bestCount);
					if (score >= kRecognitionThreshold) {
						// Set the output recognition label:
						*recognizedGesture = (*bestIt).first;
						// Clear the buffer for next process:
						mRecognizerBuffer.clear();
						// Return recognition success:
						return true;
					}
				}
			}
			// Otherwise, just add new result:
			else {
				mRecognizerBuffer.push_back(tResult);
			}
		}
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
