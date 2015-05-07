#pragma once

#include "cinder/Surface.h"
#include "cinder/ImageIo.h"
#include "cinder/Utilities.h"

#include "Kinect2.h"

#include <multitrack/Track.h>

namespace itp { namespace multitrack {

	typedef std::shared_ptr<struct PointCloud> PointCloudRef;

	struct PointCloud
	{
		std::deque<ci::vec2> mPoints;

		PointCloud()
		{
			/* no-op */
		}

		PointCloud(const Kinect2::BodyFrame& frame, const Kinect2::DeviceRef& device, bool includeAll = true)
		{
			for (const Kinect2::Body& body : frame.getBodies()) {
				if (body.isTracked()) {
					for (const auto& joint : body.getJointMap()) {
						if (includeAll || joint.second.getTrackingState() == TrackingState::TrackingState_Tracked) {
							mPoints.push_back(device->mapCameraToDepth(joint.second.getPosition()));
						}
					}
				}
			}
		}
	};
	
	template<typename T> inline std::string get_file_extension() { /* no-op */ }
	template<typename T> inline T read_from_file(const ci::fs::path& inputPath) { /* no-op */ }
	template<typename T> inline void write_to_file(const ci::fs::path& outputPath, const T& outputItem) { /* no-op */ }
	
	template<> inline std::string get_file_extension<ci::SurfaceRef>()
	{
		return "png";
	}

	template<> inline ci::SurfaceRef read_from_file<ci::SurfaceRef>(const ci::fs::path& inputPath)
	{
		return ci::Surface::create( ci::loadImage( inputPath ) );
	}
	
	template<> inline void write_to_file<ci::SurfaceRef>(const ci::fs::path& outputPath, const ci::SurfaceRef& outputItem)
	{
		ci::writeImage( outputPath, *outputItem );
	}

	template<> inline std::string get_file_extension<PointCloudRef>()
	{
		return "txt";
	}

	template<> inline PointCloudRef read_from_file<PointCloudRef>(const ci::fs::path& inputPath)
	{
		// Try to open file:
		std::ifstream tFile(inputPath.string());
		// Handle info file:
		if (tFile.is_open()) {
			// Initialize output:
			PointCloudRef tOutput = std::make_shared<PointCloud>();
			// Iterate over each line in info file:
			std::string tTemp;
			while (std::getline(tFile, tTemp)) {
				// Find delimiter:
				std::size_t tFind = tTemp.find_first_of(' ');
				// Handle frame:
				if (tFind != std::string::npos) {
					tOutput->mPoints.push_back(ci::vec2(atof(tTemp.substr(0, tFind).c_str()), atof(tTemp.substr(tFind + 1).c_str())));
				}
				// Handle error:
				else {
					throw std::runtime_error("Could not read file: \'" + inputPath.string() + "\'");
				}
			} 
			// Close info file:
			tFile.close();
			// Return point cloud:
			return tOutput;
		}
		// Handle file-open error:
		throw std::runtime_error("Could not open file: \'" + inputPath.string() + "\'");
	}

	template<> inline void write_to_file<PointCloudRef>(const ci::fs::path& outputPath, const PointCloudRef& outputItem)
	{
		std::ofstream tFile;
		tFile.open(outputPath.string());
		for (const auto& pt : outputItem->mPoints) {
			tFile << pt.x << ' ' << pt.y << std::endl;
		}
		tFile.close();
	}

	/** @brief templated track type */
	template <typename T> class TrackT : public Track {
	public:
		
		typedef std::shared_ptr<TrackT>				Ref;
		typedef std::deque<Ref>						RefDeque;

		typedef std::function<T(void)>				RecorderCallback;
		typedef std::function<void(const T&)>		PlayerCallback;

		/** @brief track player mediator */
		class Player : public TrackBase {
		public:

			typedef std::shared_ptr<Player>			Ref;
			
			typedef std::pair<double,std::string>	FrameInfo;
			typedef std::vector<FrameInfo>			FrameInfoVec;
			
			typedef FrameInfoVec::iterator			FrameInfoIter;
			typedef FrameInfoVec::const_iterator	FrameInfoConstIter;

		private:

			typename TrackT::Ref	mTrack;
			FrameInfoVec			mInfoVec;
			FrameInfoIter			mInfoIterator;
			PlayerCallback			mPlayerCb;
			double					mKeyTimeCurr;
			double					mKeyTimeNext;
			bool					mInitialized;
			
			/** @brief basic constructor */
			Player(typename TrackT::Ref iTrack, PlayerCallback iPlayerCallback) :
				mTrack( iTrack ),
				mPlayerCb( iPlayerCallback ),
				mInfoVec( FrameInfoVec() ),
				mKeyTimeCurr( 0.0 ),
				mKeyTimeNext( 0.0 ),
				mInitialized( false )
			{
				/* no-op */
			}

		public:

			/** @brief static creational method */
			template <typename ... Args> static typename Player::Ref create(Args&& ... args)
			{
				return Player::Ref(new Player(std::forward<Args>(args)...));
			}

			/** @brief update method */
			void update()
			{
				// Handle empty track:
				if (mInfoVec.empty()) {
					mInitialized = false;
					return;
				}
				// Get track endpoints:
				double tBegin    = mInfoVec.front().first;
				double tEnd      = mInfoVec.back().first;
				// Get playhead:
				const double& tPlayhead = mTrack->getTimer()->getPlayhead();
				// Handle playhead out-of-range:
				if (tPlayhead < tBegin || tPlayhead > tEnd) {
					mInitialized = false;
					return;
				}
				// Activate, if necessary:
				if (!mInitialized) {
					mInitialized = true;
					mInfoIterator = mInfoVec.begin();
					mKeyTimeCurr = mInfoVec.front().first;
					mKeyTimeNext = ((mInfoVec.size() > 1) ? (mInfoIterator + 1)->first : mKeyTimeCurr);
				}
				// Update iterator:
				while (tPlayhead >= mKeyTimeNext && mInfoIterator != mInfoVec.end()) {
					mKeyTimeCurr = mInfoIterator->first;
					mKeyTimeNext = ((++mInfoIterator == mInfoVec.end()) ? mKeyTimeCurr : mInfoIterator->first);
				}
			}

			/** @brief draw method */
			void draw()
			{
				if (!mPlayerCb || !mInitialized) return;
				mPlayerCb(read_from_file<T>(mTrack->getDirectory() / mInfoIterator->second));
			}

			/** @brief start method */
			void start()
			{
				// Clear info container:
				mInfoVec.clear();
				mInitialized = false;
				// Try to open info file:
				std::ifstream tInfoFile(mTrack->getInfoPath().string());
				// Handle info file:
				if (tInfoFile.is_open()) {
					std::string tTemp;
					// Iterate over each line in info file:
					while (std::getline(tInfoFile, tTemp)) {
						// Find delimiter:
						std::size_t tFind = tTemp.find_first_of(' ');
						// Handle frame:
						if (tFind != std::string::npos) {
							mInfoVec.push_back(FrameInfo(atof(tTemp.substr(0, tFind).c_str()), tTemp.substr(tFind + 1)));
						}
						// Handle error:
						else {
							throw std::runtime_error("Player could not read file: \'" + mTrack->getInfoPath().string() + "\'");
						}
					}
					// Close info file:
					tInfoFile.close();
				}
				// Handle file-open error:
				else {
					throw std::runtime_error("Player could not open file: \'" + mTrack->getInfoPath().string() + "\'");
				}
			}

			/** @brief stop method */
			void stop()
			{ 
				/* no-op */
			}
		};

		/** @brief track recorder mediator */
		class Recorder : public TrackBase {
		public:

			typedef std::shared_ptr<Recorder> Ref;

		private:

			typename TrackT::Ref	mTrack;			//!< mediator's owner
			T						mBuffer;		//!< current frame

			RecorderCallback		mRecorderCb;	//!< recorder callback function
			PlayerCallback			mPlayerCb;		//!< player callback function

			bool					mActive;		//!< flags whether recorder is active
			std::ofstream			mInfoFile;		//!< info-file output stream

			Recorder(typename TrackT::Ref iTrack, RecorderCallback iRecorderCallback, PlayerCallback iPlayerCallback) :
				mTrack(iTrack),
				mRecorderCb(iRecorderCallback),
				mPlayerCb(iPlayerCallback),
				mActive(false)
			{ 
				/* no-op */
			}

		public:
			
			~Recorder()
			{
				stop_info_file();
			}

			/** @brief static creational method */
			template <typename ... Args> static typename Recorder::Ref create(Args&& ... args)
			{
				return Recorder::Ref(new Recorder(std::forward<Args>(args)...));
			}

			RecorderCallback getRecorderCallbackFn() const
			{
				return mRecorderCb;
			}

			PlayerCallback getPlayerCallbackFn() const
			{
				return mPlayerCb;
			}

			const bool& isActive() const
			{
				return mActive;
			}

			/** @brief update method */
			void update()
			{
				if ( !mRecorderCb ) return;
				// Get current frame:
				T tCurr = mRecorderCb();
				// Check frame validity:
				if( tCurr ) {
					// Set buffer:
					mBuffer = tCurr;
					// Handle active mode:
					if (mActive) {
						// Compose frame filename:
						std::string tFilename = ("frame_" + std::to_string(mTrack->mFrameCount) + "." + get_file_extension<T>());
						// Write frame to info file:
						mInfoFile << mTrack->getTimer()->getPlayhead() << ' ' << tFilename << std::endl;
						// Write frame contents to file:
						write_to_file<T>(mTrack->getDirectory() / tFilename, mBuffer);
						// Increment frame count:
						mTrack->mFrameCount++;
					}
				}
			}

			/** @brief draw method */
			void draw()
			{
				if( !mPlayerCb ) return;
				mPlayerCb( mBuffer );
			}

			/** @brief start method */
			void start()
			{
				start_info_file();
				mActive = true;
				mTrack->mFrameCount = 0;
			}

			/** @brief stop method */
			void stop()
			{
				stop_info_file();
				mActive = false;
			}
			
			void start_info_file()
			{
				// Stop previous, if applicable:
				stop_info_file();
				// Check if directory already exists:
				if (ci::fs::exists(mTrack->getDirectory())) {
					// If path exists but is not a directory, throw:
					if (!fs::is_directory(mTrack->getDirectory())) {
						throw std::runtime_error("Could not open \'" + mTrack->getDirectory().string() + "\' as a directory");
					}
				}
				// Create directory:
				else if (!boost::filesystem::create_directory(mTrack->getDirectory())) {
					throw std::runtime_error("Could not create \'" + mTrack->getDirectory().string() + "\' as a directory");
				}
				// Try to open info file:
				mInfoFile.open( mTrack->getInfoPath().string() );
				// Handle info file not found error:
				if( ! mInfoFile.is_open() )
					throw std::runtime_error( "Recorder could not open file: \'" + mTrack->getInfoPath().string() + "\'" );
			}
			
			void stop_info_file()
			{
				if( mInfoFile.is_open() ) {
					mInfoFile.close();
				}
			}
		};
		
	private:

		TrackBase::Ref	mMediator;		//!< shared_ptr to track mediator
		ci::fs::path	mDirectory;		//!< track's base directory
		std::string		mName;			//!< track's base filename
		size_t			mFrameCount;	//!< number of frames recorded
		
		/** @brief default constructor */
		TrackT(const ci::fs::path& iDirectory, const std::string& iName, Timer::Ref iTimer)
			: Track(iTimer), mDirectory(iDirectory), mName(iName), mFrameCount(0) { /* no-op */ }
		
		/** @brief parented constructor */
		TrackT(const ci::fs::path& iDirectory, const std::string& iName, Track::Ref iParent)
			: Track(iParent), mDirectory(iDirectory), mName(iName), mFrameCount(0) { /* no-op */ }
		
	public:
		
		/** @brief static creational method */
		template <typename ... Args> static typename TrackT::Ref create(Args&& ... args)
		{
			return TrackT::Ref( new TrackT( std::forward<Args>( args )... ) );
		}
		
		ci::fs::path getInfoPath()  const { return mDirectory / ( mName + "_info.txt" ); }
		ci::fs::path getDirectory() const { return mDirectory / mName; }
		
		void update() { if( mMediator ) mMediator->update(); }
		void draw() { if( mMediator ) mMediator->draw(); }
		void start() { if (mMediator) mMediator->start(); }
		void stop() { if( mMediator ) mMediator->stop(); }

		void gotoPlayMode()
		{
			// Stop mediator:
			if (mMediator) mMediator->stop();
			// Cast mediator to recorder:
			typename Recorder::Ref tRecorderCast = std::dynamic_pointer_cast<Recorder>(mMediator);
			// Return on cast error:
			if (!tRecorderCast) { return; }
			// Create player mediator:
			mMediator = Player::create(getRef<TrackT>(), tRecorderCast->getPlayerCallbackFn());
		}

		void gotoRecordMode(RecorderCallback iRecorderCallback, PlayerCallback iPlayerCallback)
		{
			// Stop mediator:
			if( mMediator ) mMediator->stop();
			// Create recorder mediator:
			mMediator = Recorder::create(getRef<TrackT>(), iRecorderCallback, iPlayerCallback);
		}

		/** @brief returns the track's frame-count */
		size_t getFrameCount() const
		{
			return mFrameCount;
		}
	};

} } // namespace itp::multitrack
