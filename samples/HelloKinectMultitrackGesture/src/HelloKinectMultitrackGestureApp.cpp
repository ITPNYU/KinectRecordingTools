#include "cinder/app/App.h"
#include "cinder/app/RendererGl.h"
#include "cinder/audio/Voice.h"
#include "cinder/audio/Source.h"
#include "cinder/gl/gl.h"
#include "cinder/gl/Fbo.h"
#include "cinder/ImageIo.h"
#include "cinder/Utilities.h"
#include "cinder/Timeline.h"
#include "cinder/Rand.h"

#include "Kinect2.h"
#include <KinectProcessingGlsl.h>

#include <multitrack/Track.h>
#include <multitrack/Mediator.h>

#include <foil/oss/gesture/recognizer.hpp>

using namespace ci;
using namespace ci::app;
using namespace std;

static const size_t kRawFrameWidth = 960;
static const size_t kRawFrameHeight = 540;

static const double kCaptionHeight = 325.0;

static const size_t kBodyPointCount = 25;

static const double kRecognitionThreshold  = 0.85;
static const size_t kRecognitionSamples    = 30;
static const size_t kRecognitionSamplesMin = 25;

static const size_t kSelectItemFramesMin = 100;

static const double kSceneDurationSec = 30.0;
static const double kShotDurationMin  = 3.0;
static const double kShotDurationMax  = 8.0;

static const double kStateTransitionShort = 2.0f;
static const double kStateTransitionMedium = 4.0f;
static const double kStateTransitionLong = 5.0f;

namespace itp { namespace multitrack {

	struct CaptionImage
	{
		typedef std::deque<CaptionImage> Deque;

		ci::gl::TextureRef	mTex;
		std::string			mMsg;
	};

	static inline void drawCaptions(const CaptionImage::Deque& captions, const ci::Font& font, float itemHeight)
	{
		if (captions.empty()) return;
		// Get caption count:
		size_t captionCount = captions.size();
		// Compute padding:
		float padding = (getWindowHeight() - itemHeight * captionCount) / (captionCount + 1.0f);
		// Compute item dimensions:
		ci::vec2 dim;
		dim.y = itemHeight;
		dim.x = dim.y * (static_cast<float>(kRawFrameWidth) / static_cast<float>(kRawFrameHeight));
		// Draw captions:
		vec2 offset(0.0, padding);
		for (const auto& caption : captions) {
			gl::pushMatrices();
			gl::translate(offset);

			Rectf rect = Rectf(vec2(0.0, 0.0), dim);
			gl::color(0.0, 0.0, 0.0, 0.75);
			gl::drawSolidRect(rect);
			gl::color(1.0, 1.0, 1.0, 1.0);
			gl::draw(caption.mTex, rect);
			gl::drawStrokedRect(rect);
			gl::drawStringCentered(caption.mMsg, vec2((rect.x1 + rect.x2) * 0.5, rect.y2 - 50.0), Color(1, 1, 1), font);

			gl::popMatrices();
			offset.y += dim.y + padding;
		}
	}

	static inline std::string findAndReplace(const std::string& str, const std::string& toRemove, const std::string& toInsert)
	{
		std::string s = str;
		size_t pos = s.find(toRemove);
		if (pos == std::string::npos) return s;
		return s.replace(pos, toRemove.length(), toInsert);
	}

	/** @brief abstract base class for mode types */
	class Mode : public std::enable_shared_from_this<Mode> {
	public:

		typedef std::shared_ptr<Mode>				Ref;
		typedef std::shared_ptr<const Mode>			ConstRef;

		typedef std::shared_ptr<class Controller>	ControllerRef;

	protected:

		ControllerRef		mController;		//!< internal reference to the controller

		ci::Font			mFont;				//!< primary font
		std::string			mLabel;				//!< primary label

		/** @brief default constructor */
		Mode(const ControllerRef& controller, const ci::Font& font = ci::Font("Helvetica", 40), const std::string& label = "") :
			mController(controller),
			mFont(font),
			mLabel(label)
		{ /* no-op */ }

	public:

		/** @brief virtual destructor */
		virtual ~Mode() { /* no-op */ }

		/** @brief returns const shared_ptr to mode */
		Mode::ConstRef getRef() const
		{
			return shared_from_this();
		}

		/** @brief returns shared_ptr to mode */
		Mode::Ref getRef()
		{
			return shared_from_this();
		}

		/** @brief dynamically casts and returns this entity as const shared_ptr to templated type */
		template <typename T> std::shared_ptr<const T> getRef() const
		{
			return std::dynamic_pointer_cast<T>(shared_from_this());
		}

		/** @brief dynamically casts and returns this entity as shared_ptr to templated type */
		template <typename T> std::shared_ptr<T> getRef()
		{
			return std::dynamic_pointer_cast<T>(shared_from_this());
		}

		/** @brief pure virtual update method */
		virtual void update() = 0;

		/** @brief pure virtual draw method */
		virtual void draw() = 0;

		/** @brief overloadable event method */
		virtual void onEvent(const std::string& str) { /* no-op */ }
	};

	/** @brief controller */
	class Controller : public std::enable_shared_from_this<Controller> {
	public:

		typedef std::shared_ptr<Controller>			Ref;
		typedef std::shared_ptr<const Controller>	ConstRef;

	private:

		long long									mTimeStamp;
		long long									mTimeStampPrev;

		ci::gl::GlslProgRef							mGlslProg;

		Kinect2::DeviceRef							mDevice;

		Kinect2::BodyFrame							mBodyFrame;

		ci::Channel8uRef							mChannelBody;
		ci::Surface8uRef							mSurfaceColor;
		ci::Channel16uRef							mChannelDepth;

		ci::Surface32fRef							mSurfaceLookup;

		ci::gl::TextureRef							mTextureBody;
		ci::gl::TextureRef							mTextureColor;
		ci::gl::TextureRef							mTextureLookup;
		ci::gl::FboRef								mSilhouetteFbo;

		size_t										mUidCounter;

		size_t										mActiveBodyCount;

		std::map<std::string, ci::gl::TextureRef>	mPoseArchetypes;

		foil::gesture::Recognizer					mRecognizer;
		foil::gesture::Result::Deque				mRecognizerBuffer;

		std::vector<ci::fs::path>					mBackgroundPaths;
		std::vector<ci::fs::path>					mSoundtrackPaths;

		Timer::Ref									mTimer;
		ci::fs::path								mDirectory;

		Track::Group::RefDeque						mSequence;

		Mode::Ref									mModeCurr;
		Mode::Ref									mModeNext;

		ci::audio::VoiceRef							mSoundtrack;

		/** @brief default constructor */
		Controller() { /* no-op */ }

		void initialize()
		{
			// Seed random number generator:
			ci::randSeed((unsigned)time(NULL));
			// Enable texture mode:
			ci::gl::enable(GL_TEXTURE_2D);
			// Initialize timestamp:
			mTimeStamp = 0L;
			mTimeStampPrev = mTimeStamp;
			// Setup shader:
			mGlslProg = itp::createKinectAlignSilhouetteShader();
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
			// Get background asset directory:
			ci::fs::path bgAssetDir = getAssetDirectories().front() / "background";
			// Iterate over directory:
			if (fs::is_directory(bgAssetDir)) {
				fs::directory_iterator dirEnd;
				for (fs::directory_iterator it(bgAssetDir); it != dirEnd; it++) {
					if ((*it).path().extension() == ".png") {
						mBackgroundPaths.push_back(*it);
					}
				}
			}
			// Get audio asset directory:
			ci::fs::path audioAssetDir = getAssetDirectories().front() / "audio";
			// Iterate over directory:
			if (fs::is_directory(audioAssetDir)) {
				fs::directory_iterator dirEnd;
				for (fs::directory_iterator it(audioAssetDir); it != dirEnd; it++) {
					if ((*it).path().extension() == ".mp3") {
						mSoundtrackPaths.push_back(*it);
					}
				}
			}
			// Set scratch directory:
			mDirectory = getHomeDirectory() / "Desktop" / "Tests";
			mUidCounter = 0;
			// Create timer:
			mTimer = itp::multitrack::Timer::create();
			mTimer->setLoopCallback(std::bind(&Controller::receiveLoopCallback, this));
			mTimer->setLoopMarker(kSceneDurationSec);
			// Initialize body count:
			mActiveBodyCount = 0;
			// Create initial movie:
			startNewMovie();
			// Set initial mode:
			setMode("WaitForUserMode");
		}

	public:

		/** @brief static creational method */
		template <typename ... Args> static Controller::Ref create(Args&& ... args)
		{
			Controller::Ref controller = Controller::Ref(new Controller(std::forward<Args>(args)...));
			controller->initialize();
			return controller;
		}

		/** @brief returns const shared_ptr to controller */
		Controller::ConstRef getRef() const
		{
			return shared_from_this();
		}

		/** @brief returns shared_ptr to controller */
		Controller::Ref getRef()
		{
			return shared_from_this();
		}

		Timer::Ref getTimer() const
		{
			return mTimer;
		}

		const ci::fs::path& getDirectory() const
		{
			return mDirectory;
		}

		const std::vector<ci::fs::path>& getBackgroundPaths() const
		{
			return mBackgroundPaths;
		}

		const size_t& getActiveBodyCount() const
		{
			return mActiveBodyCount;
		}

		Kinect2::DeviceRef getKinect() const
		{
			return mDevice;
		}

		const Kinect2::BodyFrame& getBodyFrame() const
		{
			return mBodyFrame;
		}

		ci::Rectf getFboRect() const
		{
			return ci::Rectf(0, 0, kRawFrameWidth, kRawFrameHeight).getCenteredFit(getWindowBounds(), true);
		}

		ci::gl::FboRef getSilhouetteFbo() const
		{
			return mSilhouetteFbo;
		}

		size_t getNextUid()
		{
			size_t t = mUidCounter;
			mUidCounter++;
			return t;
		}

		bool hasPoseArchetype(const std::string& name) const
		{
			return (mPoseArchetypes.find(name) != mPoseArchetypes.cend());
		}

		ci::gl::TextureRef getPoseArchetype(const std::string& name) const
		{
			std::map<std::string, ci::gl::TextureRef>::const_iterator findIt = mPoseArchetypes.find(name);
			if (findIt != mPoseArchetypes.cend()) return (*findIt).second;
			return nullptr;
		}

		void setPoseArchetype(const std::string& name, const ci::gl::TextureRef& pose)
		{
			mPoseArchetypes[name] = pose;
		}

		void setMode(const Mode::Ref& mode)
		{
			mModeNext = mode;
		}

		void setMode(const std::string& name);

		/** @brief update method */
		void update()
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
			// Update timer:
			mTimer->update();
			// Goto next mode, if applicable:
			if (mModeNext) {
				mModeCurr = mModeNext;
				mModeNext.reset();
			}
			// Update mode:
			if (mModeCurr) mModeCurr->update();
		}

		/** @brief draw method */
		void draw()
		{
			gl::clear(Color(0, 0, 0));
			gl::enableAlphaBlending();
			gl::setMatricesWindow(getWindowSize());
			ci::gl::color(1.0, 1.0, 1.0, 1.0);
			// Draw mode:
			if (mModeCurr) mModeCurr->draw();
		}

		/** @brief update sequence method */
		void updateSequence()
		{
			for (auto& curr : mSequence) {
				curr->update();
			}
		}

		/** @brief draw sequence method */
		void drawSequence()
		{
			for (auto& curr : mSequence) {
				curr->draw();
			}
		}

		/** @brief start soundtrack method */
		void startSoundtrack()
		{
			if (mSoundtrack) {
				mSoundtrack->stop();
				mSoundtrack->start();
			}
		}

		/** @brief pause soundtrack method */
		void pauseSoundtrack()
		{
			if (mSoundtrack) {
				mSoundtrack->pause();
			}
		}

		/** @brief stop soundtrack method */
		void stopSoundtrack()
		{
			if (mSoundtrack) {
				mSoundtrack->stop();
			}
		}

		/** @brief start timer method */
		void startTimer(bool handleSoundtrack)
		{
			if (handleSoundtrack) startSoundtrack();
			mTimer->start();
		}

		/** @brief pause timer method */
		void pauseTimer()
		{
			pauseSoundtrack();
			mTimer->pause();
		}

		/** @brief stop timer method */
		void stopTimer()
		{
			stopSoundtrack();
			mTimer->stop();
		}

		/** @brief loop callback method */
		void receiveLoopCallback()
		{
			// Notify mode:
			if (mModeCurr) {
				mModeCurr->onEvent("LOOP");
			}
		}

		/** @brief clears current sequence and begins new one */
		void startNewMovie()
		{
			// Stop timer:
			stopTimer();
			// Clear current sequence:
			mSequence.clear();
			// Reset id counter:
			mUidCounter = 0;
			// Pick new soundtrack randomly:
			if (!mSoundtrackPaths.empty()) {
				ci::fs::path currPath = mSoundtrackPaths[ci::randInt(0, mSoundtrackPaths.size())];
				mSoundtrack = audio::Voice::create(audio::load(ci::DataSourcePath::create(currPath)));
				mSoundtrack->setVolume(1.0);
				mSoundtrack->setPan(0.5);
				stopSoundtrack();
			}
		}

		Track::Group::Ref createTrackCinematographer(size_t frameCount)
		{
			// Create group:
			Track::Group::Ref tGroup = Track::Group::create("track_bg_group");
			// Create image recorder callback lambda:
			auto tImgRecorderCallbackFn = [&](void) -> ci::SurfaceRef
			{
				return nullptr;
			};
			// Create image player callback lambda:
			auto tImgPlayerCallbackFn = [&](const ci::SurfaceRef& iSurface) -> void
			{
				if (iSurface.get() == NULL) return;
				gl::enable(GL_TEXTURE_2D);
				ci::gl::TextureRef tex = ci::gl::Texture::create(*(iSurface.get()));
				ci::gl::draw(tex, getFboRect());
			};
			// Create typed track:
			itp::multitrack::Track::Ref tTrack = itp::multitrack::TrackT<ci::SurfaceRef>::create(
				mDirectory,
				"track_bg",
				mTimer,
				tImgRecorderCallbackFn,
				tImgPlayerCallbackFn,
				frameCount);
			// Initialize player:
			tTrack->gotoPlayMode();
			// Add track to group:
			tGroup->push(tTrack);
			// Return output:
			return tGroup;
		}

		void removeTrackCinematographer()
		{
			for (Track::Group::RefDeque::const_iterator it = mSequence.cbegin(); it != mSequence.cend(); it++) {
				if ("track_bg_group" == (*it)->getName()) {
					mSequence.erase(it);
					return;
				}
			}
		}

		Track::Group::Ref createTrackSilhouette(const std::string& name, bool active)
		{
			// Create group:
			Track::Group::Ref tGroup = Track::Group::create(name + "_group");
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
				ci::gl::TextureRef tex = ci::gl::Texture::create(*(iSurface.get()));
				ci::gl::draw(tex, getFboRect());
			};
			// Create typed track:
			itp::multitrack::Track::Ref tTrack = itp::multitrack::TrackT<ci::SurfaceRef>::create(
				mDirectory,
				name,
				mTimer,
				tImgRecorderCallbackFn,
				tImgPlayerCallbackFn);
			// Initialize recorder:
			tTrack->gotoRecordMode(active);
			// Add track to group:
			tGroup->push(tTrack);
			// Return output:
			return tGroup;

			/* // TODO: for skeleton recording...
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

		void addTrackGroup(const Track::Group::Ref& group, bool toBack)
		{
			// Check validity:
			if (group) {
				// Check framecount:
				if (group->getFrameCount() > 0) {
					// Add to sequence:
					if (toBack) mSequence.push_back(group);
					else mSequence.push_front(group);
					// Set to play mode:
					group->gotoPlayMode();
				}
				// Stop timer:
				stopTimer();
			}
		}

		bool addGestureTemplate(const std::string& poseName)
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

		foil::gesture::Result guessGesture()
		{
			// Check whether recognizer has templates:
			if (mRecognizer.hasTemplates()) {
				// Get point cloud:
				itp::multitrack::PointCloud tCloud = itp::multitrack::PointCloud(mBodyFrame, mDevice);
				// Check for correct point count for single body:
				if (tCloud.mPoints.size() == kBodyPointCount) {
					// Get gesture guess:
					return mRecognizer.recognizeBest({ tCloud.mPoints });
				}
			}
			return foil::gesture::Result("NONE", 0.0f);
		}

		bool analyzeGesture(std::string* recognizedGesture)
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
							std::map< std::string, std::pair<size_t, float> >::iterator findIt = histo.find(item.mName);
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
							if ((*it).second.first > bestCount) {
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

		void renderSilhouetteGpu()
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
	};


	class TransitionCardMode : public Mode {
	private:

		double			mStart;
		double			mDuration;
		std::string		mMessage;
		std::string		mTargetMode;

		TransitionCardMode(const Controller::Ref& controller, double duration, const std::string& msg, const std::string& targetMode) :
			Mode(controller, ci::Font("Helvetica", 80), ""),
			mStart(ci::app::getElapsedSeconds()),
			mDuration(duration),
			mMessage(msg),
			mTargetMode(targetMode)
		{
			mController->stopTimer();
		}

	public:

		template <typename ... Args> static Mode::Ref create(Args&& ... args)
		{
			return Mode::Ref(new TransitionCardMode(std::forward<Args>(args)...));
		}

		void update() 
		{
			double timeElap = ci::app::getElapsedSeconds() - mStart;
			if (timeElap >= mDuration) {
				mController->setMode(mTargetMode);
			}
			else {
				mLabel = findAndReplace(mMessage, "$", std::to_string(static_cast<int>(mDuration - timeElap + 1)));
			}
		}

		void draw()
		{
			ci::gl::drawStringCentered(mLabel, vec2(getWindowWidth() * 0.5f, getWindowHeight() * 0.5f), Color(1, 1, 1), mFont);
		}
	};

	class EstablishPoseMode : public Mode {
	private:

		double				mStart;
		std::string			mName;
		Track::Group::Ref	mPreview;

		EstablishPoseMode(const Controller::Ref& controller, const std::string& name) :
			Mode(controller, ci::Font("Helvetica", 60), "What's next?"),
			mName(name),
			mStart(ci::app::getElapsedSeconds()),
			mPreview(mController->createTrackSilhouette("Preview",false))
		{
			mController->stopTimer();
		}

	public:

		static Mode::Ref create(const Controller::Ref& controller, const std::string& name)
		{
			return Mode::Ref(new EstablishPoseMode(controller, name));
		}

		void update()
		{
			mPreview->update();

			double timeElap = ci::app::getElapsedSeconds() - mStart;
			double duration = kStateTransitionLong;
			if (timeElap >= duration) {
				ci::gl::FboRef fbo = mController->getSilhouetteFbo();
				if (mController->addGestureTemplate(mName)) {
					mController->setPoseArchetype(mName, ci::gl::Texture::create(fbo->readPixels8u(fbo->getBounds())));
				}
				mController->setMode("HomeMode");
			}
			else if (mController->getActiveBodyCount() != 1) {
				mController->setMode("WaitForUserMode");
			}
			else {
				mLabel = "Establishing " + mName + " pose in " + std::to_string(static_cast<int>(duration - timeElap + 1));
			}
		}

		void draw()
		{
			mPreview->draw();
			ci::gl::drawStringCentered(mLabel, vec2(getWindowWidth() * 0.5f, getWindowHeight() * 0.5f), Color(1, 1, 1), mFont);
		}
	};

	class WaitForUserMode : public Mode {
	private:

		WaitForUserMode(const Controller::Ref& controller) :
			Mode(controller, ci::Font("Helvetica", 80), "")
		{ 
			mController->stopTimer();
		}

	public:

		static Mode::Ref create(const Controller::Ref& controller)
		{
			return Mode::Ref(new WaitForUserMode(controller));
		}

		void update()
		{ 
			if (mController->getActiveBodyCount() != 1) {
				if (mController->getActiveBodyCount() == 0) mLabel = "Is anyone there?";
				else mLabel = "I see " + std::to_string(mController->getActiveBodyCount()) + " users, but need one.";
			}
			else {
				mController->setMode(TransitionCardMode::create(
					mController,
					kStateTransitionShort,
					"Oh hello!",
					"HomeMode"));
			}
		}

		void draw()
		{
			ci::gl::drawStringCentered(mLabel, vec2(getWindowWidth() * 0.5f, getWindowHeight() * 0.5f), Color(1, 1, 1), mFont);
		}
	};

	class HomeMode : public Mode {
	private:

		Track::Group::Ref		mPreview;
		CaptionImage::Deque		mCaptions;

		HomeMode(const Controller::Ref& controller) :
			Mode(controller, ci::Font("Helvetica", 40), ""),
			mPreview(mController->createTrackSilhouette("Preview",false))
		{ 
			mController->startTimer(false);
		}

	public:

		static Mode::Ref create(const Controller::Ref& controller)
		{
			return Mode::Ref(new HomeMode(controller));
		}

		void update()
		{
			mController->updateSequence();
			mPreview->update();
			
			if (!mController->hasPoseArchetype("IDLE")) {
				mController->setMode("EstablishIdlePoseMode");
			}
			else if (!mController->hasPoseArchetype("CONTROL")) {
				mController->setMode("EstablishControlPoseMode");
			}
			else if (!mController->hasPoseArchetype("ACTOR")) {
				mController->setMode("EstablishActorPoseMode");
			}
			else if (!mController->hasPoseArchetype("CINEMATOGRAPHER")) {
				mController->setMode("EstablishCinematographerPoseMode");
			}
			else {
				// Initialize, if necessary:
				if (mCaptions.empty()) {
					// Set captions:
					mCaptions = {
						{ mController->getPoseArchetype("CONTROL"), "NEW MOVIE" },
						{ mController->getPoseArchetype("ACTOR"), "ACTOR" },
						{ mController->getPoseArchetype("CINEMATOGRAPHER"), "CINEMATOGRAPHER" },
					};
					// Start soundtrack:
					mController->startSoundtrack();
				}
				// Handle single user:
				if (mController->getActiveBodyCount() == 1) {
					// Set label:
					mLabel = "";
					// Analyze gesture:
					std::string recognizedGesture;
					if (mController->analyzeGesture(&recognizedGesture)) {
						// Check for control gesture:
						if (recognizedGesture == "CONTROL") {
							mController->startNewMovie();
							mController->setMode(TransitionCardMode::create(
								mController,
								kStateTransitionLong,
								"Starting a new movie in $",
								"HomeMode"));
						}
						// Check for actor gesture:
						else if (recognizedGesture == "ACTOR") {
							mController->setMode(TransitionCardMode::create(
								mController,
								kStateTransitionLong,
								"You're on in $",
								"PerformActorMode"));
						}
						// Check for actor gesture:
						else if (recognizedGesture == "CINEMATOGRAPHER") {
							mController->setMode(TransitionCardMode::create(
								mController,
								kStateTransitionLong,
								"Shooting in $",
								"PerformCinematographerMode"));
						}
					}
				}
				// Handle default:
				else {
					// Set label:
					mLabel = "Ready when you are.";
				}
			}
		}

		void draw()
		{
			mController->drawSequence();
			mPreview->draw();
			ci::gl::drawStringCentered(mLabel, vec2(getWindowWidth() * 0.5f, getWindowHeight() - 100.0f), Color(1, 1, 1), mFont);
			if (!mCaptions.empty() && mController->getActiveBodyCount() == 1) drawCaptions(mCaptions, mFont, kCaptionHeight);
		}

		void onEvent(const std::string& str)
		{
			if ("LOOP" == str) {
				mController->startSoundtrack();
			}
		}
	};

	class PerformActorMode : public Mode {
	private:

		Track::Group::Ref	mRecorder;

		PerformActorMode(const Controller::Ref& controller) :
			Mode(controller, ci::Font("Helvetica", 40), ""),
			mRecorder(mController->createTrackSilhouette("track_" + std::to_string(mController->getNextUid()),true))
		{ 
			mController->startTimer(true);
		}

	public:

		static Mode::Ref create(const Controller::Ref& controller)
		{
			return Mode::Ref(new PerformActorMode(controller));
		}

		void update()
		{
			mController->updateSequence();
			mRecorder->update();
			// Analyze gesture:
			std::string recognizedGesture;
			if (mController->analyzeGesture(&recognizedGesture)) {
				// Check for control gesture:
				if (recognizedGesture == "CONTROL") {
					complete();
				}
			}
			else {
				mLabel = std::to_string(mController->getTimer()->getPlayhead());
			}
		}

		void draw()
		{
			mController->drawSequence();
			mRecorder->draw();
			ci::gl::drawStringCentered(mLabel, vec2(getWindowWidth() * 0.5f, getWindowHeight() - 100.0f), Color(1, 1, 1), mFont);
		}

		void onEvent(const std::string& str) 
		{ 
			if ("LOOP" == str) {
				complete();
			}
		}

		void complete()
		{
			mRecorder->gotoPlayMode();
			mController->addTrackGroup(mRecorder, true);
			mController->setMode(TransitionCardMode::create(
				mController,
				kStateTransitionLong,
				"Cut! We'll be back in $",
				"HomeMode"));
		}
	};

	class PerformCinematographerMode : public Mode {
	private:

		enum Submode
		{
			WAIT_FOR_NEW_MARKER,
			CHOOSE_IMAGE
		};

		struct ItemInfo
		{
			typedef std::deque<ItemInfo> Deque;

			double						mTime;
			size_t						mIdx;
			ci::gl::TextureRef			mTex;

			struct sort_by_time_a { bool operator()(const ItemInfo& a, const ItemInfo& b) { return (a.mTime < b.mTime); } };
			struct sort_by_time_d { bool operator()(const ItemInfo& a, const ItemInfo& b) { return (a.mTime > b.mTime); } };

			static void sort(ItemInfo::Deque& items, bool ascending)
			{
				if (ascending) { std::sort(items.begin(), items.end(), sort_by_time_a()); }
				else           { std::sort(items.begin(), items.end(), sort_by_time_d()); }
			}

			ItemInfo() { /* no-op */ }

			ItemInfo(double itemTime, size_t itemIdx, const ci::gl::TextureRef& itemTex) :
				mTime(itemTime),
				mIdx(itemIdx),
				mTex(itemTex)
			{ /* no-op */ }
		};

		struct InfoManager
		{
			ItemInfo::Deque				mInfoDeque;
			ItemInfo::Deque::iterator	mInfoIterator;
			bool						mInitialized;

			InfoManager() : mInitialized(false) { /* no-op */ }

			void sort()
			{
				ItemInfo::sort(mInfoDeque, true);
				mInitialized = false;
			}

			size_t getFrameCount() const
			{
				return mInfoDeque.size();
			}

			void addItem(const ItemInfo& item)
			{
				mInfoDeque.push_back(item);
				sort();
			}

			void update(double playhead)
			{
				// Get iterator:
				mInfoIterator = mInfoDeque.begin();
				// Iterate over frames:
				while (mInfoIterator != mInfoDeque.end()) {
					ItemInfo::Deque::iterator iterNext = mInfoIterator + 1;
					if (iterNext != mInfoDeque.end() && playhead < iterNext->mTime) {
						break;
					}
					mInfoIterator = iterNext;
				}
				// Set initialization flag:
				mInitialized = (mInfoIterator != mInfoDeque.end());
			}

			void draw(const ci::Rectf& rect)
			{
				if (!mInitialized || mInfoIterator->mTex.get() == NULL) return;
				gl::draw(mInfoIterator->mTex, rect);
			}

			void save(const std::string& uid, const std::vector<ci::fs::path>& imagePaths)
			{
				// Sort info:
				sort();
				// Note: The following procedure assumes safety check was performed elsewhere...
				// Remove the throw-away:
				mInfoDeque.pop_back();
				// Get a copy of the back item:
				ItemInfo backCopy = mInfoDeque.back();
				// Set copy's time stamp to end of sequence:
				backCopy.mTime = kSceneDurationSec;
				// Insert the rigged copy:
				mInfoDeque.push_back(backCopy);
				// Get paths:
				ci::fs::path rootDir = getHomeDirectory() / "Desktop" / "Tests";
				ci::fs::path currDir = rootDir / uid;
				ci::fs::path infoPth = rootDir / (uid + "_info.txt");
				// Check if directory already exists:
				if (ci::fs::exists(currDir)) {
					// If path exists but is not a directory, throw:
					if (!fs::is_directory(currDir)) {
						throw std::runtime_error("Could not open \'" + currDir.string() + "\' as a directory");
					}
				}
				// Create directory:
				else if (!boost::filesystem::create_directory(currDir)) {
					throw std::runtime_error("Could not create \'" + currDir.string() + "\' as a directory");
				}
				// Try to open info file:
				std::ofstream tInfoFile;
				tInfoFile.open(infoPth.string());
				if (tInfoFile.is_open()) {
					size_t itemCount = mInfoDeque.size();
					for (size_t i = 0; i < itemCount; i++) {
						// Write image:
						ci::Surface surf = ci::Surface(loadImage(imagePaths[mInfoDeque[i].mIdx]));
						ci::writeImage(currDir / ("frame_" + std::to_string(i) + ".png"), surf);
						// Write frame to info file:
						tInfoFile << std::to_string(mInfoDeque[i].mTime) + " frame_" + std::to_string(i) + ".png" << std::endl;
					}
					// Close info file:
					tInfoFile.close();
				}
				else {
					throw std::runtime_error("Application could not open file: \'" + infoPth.string() + "\'");
				}
			}
		};

		Track::Group::Ref					mPreview;
		CaptionImage::Deque					mCaptions;
		Submode								mSubmode;
		std::vector<ci::gl::TextureRef>		mBackgroundImages;

		size_t								mSelectionCurr;
		size_t								mDecisionFramecount;

		InfoManager							mInfoManager;

		PerformCinematographerMode(const Controller::Ref& controller) :
			Mode(controller, ci::Font("Helvetica", 40), ""),
			mPreview(mController->createTrackSilhouette("PREVIEW",false)),
			mSelectionCurr(SIZE_MAX)
		{
			// Load background images:
			const std::vector<ci::fs::path>& imagePaths = mController->getBackgroundPaths();
			for (const auto& imagePath : imagePaths) {
				mBackgroundImages.push_back(ci::gl::Texture::create(ci::Surface(loadImage(imagePath))));
			}
			// Set captions:
			mCaptions = {
				{ mController->getPoseArchetype("CONTROL"), "GO HOME" },
				{ mController->getPoseArchetype("CINEMATOGRAPHER"), "ADD SHOT" }
			};
			// Initialize info manager:
			mInfoManager = InfoManager();
			// Force throw-away final frame:
			mInfoManager.addItem(ItemInfo(kSceneDurationSec + 1.0f, 0, mBackgroundImages[0]));
			// Remove previous cinematographer:
			mController->removeTrackCinematographer();
			// Goto start of sequence:
			mController->startTimer(true);
			// Prepare initial marker:
			prepareMarker();
		}

		void prepareMarker()
		{
			// Reset decision frame counter:
			mDecisionFramecount = 0;
			// Set submode:
			mSubmode = Submode::CHOOSE_IMAGE;
			// Pause timer:
			mController->pauseTimer();
		}

	public:

		static Mode::Ref create(const Controller::Ref& controller)
		{
			return Mode::Ref(new PerformCinematographerMode(controller));
		}

		void update()
		{
			mInfoManager.update(mController->getTimer()->getPlayhead());
			mController->updateSequence();
			mPreview->update();
			if (mController->getActiveBodyCount() != 1) {
				mController->setMode("WaitForUserMode");
			}
			else {
				switch (mSubmode) {
					case Submode::WAIT_FOR_NEW_MARKER: {
						mLabel = std::to_string(mController->getTimer()->getPlayhead());
						// Analyze gesture:
						std::string recognizedGesture;
						if (mController->analyzeGesture(&recognizedGesture)) {
							// Check for control gesture:
							if (recognizedGesture == "CONTROL") {
								complete();
								return;
							}
						}
						// Guess gesture:
						foil::gesture::Result bestResult = mController->guessGesture();
						// Check threshold:
						if (bestResult.mScore >= kRecognitionThreshold) {
							if ("CINEMATOGRAPHER" == bestResult.mName) {
								// Prepare new marker:
								prepareMarker();
							}
						}
						break;
					}
					case Submode::CHOOSE_IMAGE: {
						// Set range map (TODO if mapping seems wrong on location, change these):
						float rangeMapMin =  300.0;
						float rangeMapMax = 1700.0;
						// 
						std::vector<Kinect2::Body> bodies = mController->getBodyFrame().getBodies();
						if (!bodies.empty()) {
							for (const Kinect2::Body& body : bodies) {
								if (body.isTracked()) {
									ci::vec3 posRaw = body.getJointMap().at(JointType_HandRight).getPosition();
									ci::ivec2 pos = mController->getKinect()->mapCameraToColor(posRaw);
									ci::Rectf rect = mController->getFboRect();
									size_t selectionCurr = static_cast<size_t>(ci::lmap<float>((float)pos.x, rangeMapMin, rangeMapMax, 0.0f, (float)mBackgroundImages.size()));
									if (mSelectionCurr == selectionCurr) {
										if (mDecisionFramecount == kSelectItemFramesMin) {
											// Update info deque:
											mInfoManager.addItem(ItemInfo(mController->getTimer()->getPlayhead(), mSelectionCurr, mBackgroundImages[mSelectionCurr]));
											// Restart sequence and process:
											mDecisionFramecount = 0;
											mSelectionCurr = SIZE_MAX;
											mController->startTimer(true);
											mSubmode = Submode::WAIT_FOR_NEW_MARKER;
										}
										else {
											mDecisionFramecount++;
										}
									}
									else {
										mDecisionFramecount = 0;
										mSelectionCurr = selectionCurr;
									}
									break;
								}
							}
						}
						mLabel = "Please hold your RIGHT HAND over a shot location";
						break;
					}
					default: {
						mLabel = std::to_string(mController->getTimer()->getPlayhead());
						break;
					}
				}
			}
		}

		void draw()
		{
			mInfoManager.draw(mController->getFboRect());
			mController->drawSequence();
			mPreview->draw();
			ci::gl::drawStringCentered(mLabel, vec2(getWindowWidth() * 0.5f, getWindowHeight() - 100.0f), Color(1, 1, 1), mFont);
			switch (mSubmode) {
				case Submode::WAIT_FOR_NEW_MARKER: {
					if (!mCaptions.empty()) drawCaptions(mCaptions, mFont, kCaptionHeight);
					break;
				}
				case Submode::CHOOSE_IMAGE: {
					float itemX = 0.0f;
					size_t imgCount = mBackgroundImages.size();
					float itemWidth = static_cast<float>(getWindowWidth()) / static_cast<float>(imgCount);
					float completeRatio = static_cast<float>(mDecisionFramecount) / static_cast<float>(kSelectItemFramesMin);
					for (size_t i = 0; i < imgCount; i++) {
						float itemHeight = itemWidth / mBackgroundImages[i]->getAspectRatio();
						float itemY = (getWindowHeight() - itemHeight) * 0.5f;
						ci::Rectf rect = ci::Rectf(itemX, itemY, itemX + itemWidth, itemY + itemHeight);
						// Draw image:
						if (mSelectionCurr != SIZE_MAX) {
							gl::color(1.0, 1.0, 1.0, 0.0);
						}
						if (i == mSelectionCurr) {
							gl::color(1.0, 1.0, 1.0, 0.75 + completeRatio * 0.75);
							gl::draw(mBackgroundImages[i], rect);
							gl::color(0.0, 1.0, 0.0, 1.0);
							gl::drawStrokedRect(rect);
						}
						else  {
							gl::color(1.0, 1.0, 1.0, 0.75 - completeRatio * 0.75);
							gl::draw(mBackgroundImages[i], rect);
						}
						// Advance:
						itemX += itemWidth;
					}
					break;
				}
				default: {
					break;
				}
			}
		}

		void onEvent(const std::string& str)
		{
			if ("LOOP" == str) {
				complete();
			}
		}

		void complete()
		{
			// Escape if we don't have the throw-away and at least one other frame:
			if (mInfoManager.getFrameCount() < 2) {
				mController->setMode(TransitionCardMode::create(
					mController,
					kStateTransitionLong,
					"Not in the mood for cinematography?",
					"HomeMode"));
				return;
			}
			// Save track:
			mInfoManager.save("track_bg", mController->getBackgroundPaths());
			// Add group to sequence:
			mController->addTrackGroup(mController->createTrackCinematographer(mInfoManager.getFrameCount()), false);
			// Return home:
			mController->setMode(TransitionCardMode::create(
				mController,
				kStateTransitionLong,
				"Cut! We'll be back in $",
				"HomeMode"));
		}
	};

	void Controller::setMode(const std::string& name)
	{
		if ("WaitForUserMode" == name) {
			mModeNext = WaitForUserMode::create(getRef());
		}
		else if ("HomeMode" == name) {
			mModeNext = HomeMode::create(getRef());
		}
		else if ("EstablishIdlePoseMode" == name) {
			mModeNext = EstablishPoseMode::create(getRef(), "IDLE");
		}
		else if ("EstablishControlPoseMode" == name) {
			mModeNext = EstablishPoseMode::create(getRef(), "CONTROL");
		}
		else if ("EstablishActorPoseMode" == name) {
			mModeNext = EstablishPoseMode::create(getRef(), "ACTOR");
		}
		else if ("EstablishCinematographerPoseMode" == name) {
			mModeNext = EstablishPoseMode::create(getRef(), "CINEMATOGRAPHER");
		}
		else if ("PerformActorMode" == name) {
			mModeNext = PerformActorMode::create(getRef());
		}
		else if ("PerformCinematographerMode" == name) {
			mModeNext = PerformCinematographerMode::create(getRef());
		}
		else {
			mModeNext.reset();
		}
	}

} } // namespace itp::multitrack

class HelloKinectMultitrackGestureApp : public App {
public:
	void setup() override;
	void update() override;
	void draw() override;
	void cleanup() override;
	void keyDown(ci::app::KeyEvent event) override;

	itp::multitrack::Controller::Ref mController;
};

void HelloKinectMultitrackGestureApp::setup()
{
	try {
		mController = itp::multitrack::Controller::create();
	}
	catch (...) {
		ci::app::console() << "Could not initialize multitrack controller" << std::endl;
		quit();
	}

	setFullScreen(true);
	::ShowCursor(false);
}

void HelloKinectMultitrackGestureApp::update()
{
	mController->update();
}

void HelloKinectMultitrackGestureApp::draw()
{
	mController->draw();
}

void HelloKinectMultitrackGestureApp::cleanup()
{
	mController.reset();
	::ShowCursor(true);
}

void HelloKinectMultitrackGestureApp::keyDown(KeyEvent event)
{
	if (event.getChar() == 'f') {
		if (isFullScreen()) {
			setFullScreen(false);
			::ShowCursor(true);
		}
		else {
			setFullScreen(true);
			::ShowCursor(false);
		}
	}
	else if (event.getChar() == 'q') {
		quit();
	}
}

CINDER_APP(HelloKinectMultitrackGestureApp, RendererGl, [](App::Settings* settings)
{
	settings->prepareWindow(Window::Format().size(1920, 1080).title("ITP Kinect Recording Tools"));
	settings->setFrameRate(60.0f);
})
