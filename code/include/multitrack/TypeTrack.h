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

		/** @brief track player */
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
			PlayerCallback			mPlayerCallback;
			double					mKeyTimeCurr;
			double					mKeyTimeNext;
			
			Player(typename TrackT::Ref iTrack, PlayerCallback iPlayerCallback) :
				mTrack( iTrack ),
				mPlayerCallback( iPlayerCallback ),
				mInfoVec( FrameInfoVec() ),
				mInfoIterator( mInfoVec.end() ),
				mKeyTimeCurr( 0.0 ),
				mKeyTimeNext( 0.0 )
			{
				/* no-op */
			}

		public:

			/** @brief static creational method */
			template <typename ... Args> static typename Player::Ref create(Args&& ... args)
			{
				return Player::Ref(new Player(std::forward<Args>(args)...));
			}

			void update()
			{
				// Handle empty track:
				if( mInfoVec.empty() ) {
					mInfoIterator = mInfoVec.end();
					return;
				}
				// Compute local playhead:
				double tLocalPlayhead = mTrack->getTimer()->getPlayhead() - mTrack->getOffset();
				// Get duration:
				double tDuration = mInfoVec.back().first;
				// Handle playhead out-of-range:
				if( tLocalPlayhead < 0.0 || tLocalPlayhead > tDuration ) {
					mInfoIterator = mInfoVec.end();
					return;
				}
				// Activate, if necessary:
				if( mInfoIterator == mInfoVec.end() ) {
					mInfoIterator = mInfoVec.begin();
					mKeyTimeCurr  = mInfoVec.front().first;
					mKeyTimeNext  = ( ( mInfoVec.size() > 1 ) ? ( mInfoIterator + 1 )->first : mKeyTimeCurr );
				}
				// Update iterator:
				while( tLocalPlayhead >= mKeyTimeNext && mInfoIterator != mInfoVec.end() ) {
					mKeyTimeCurr = mInfoIterator->first;
					mKeyTimeNext = ( ( ++mInfoIterator == mInfoVec.end() ) ? mKeyTimeCurr : mInfoIterator->first );
				}
			}

			void draw()
			{
				if( !mPlayerCallback || mInfoIterator == mInfoVec.end() ) return;
				mPlayerCallback( read_from_file<T>( mTrack->getDirectory() / mInfoIterator->second ) );
			}
			
			void start()
			{
				// Clear info container:
				mInfoVec.clear();
				mInfoIterator = mInfoVec.end();
				// Try to open info file:
				std::ifstream tInfoFile( mTrack->getInfoPath().string() );
				// Handle info file:
				if( tInfoFile.is_open() ) {
					std::string tTemp;
					// Iterate over each line in info file:
					while( std::getline( tInfoFile, tTemp ) ) {
						// Find delimiter:
						std::size_t tFind = tTemp.find_first_of( ' ' );
						// Handle frame:
						if( tFind != std::string::npos ) {
							mInfoVec.push_back( FrameInfo( atof( tTemp.substr( 0, tFind ).c_str() ), tTemp.substr( tFind + 1 ) ) );
						}
						// Handle error:
						else {
							throw std::runtime_error( "Player could not read file: \'" + mTrack->getInfoPath().string() + "\'" );
						}
					}
					// Close info file:
					tInfoFile.close();
				}
				// Handle file-open error:
				else {
					throw std::runtime_error( "Player could not open file: \'" + mTrack->getInfoPath().string() + "\'" );
				}
			}
		};

		/** @brief track recorder */
		class Recorder : public TrackBase {
		public:

			typedef std::shared_ptr<Recorder> Ref;

		private:

			typename TrackT::Ref	mTrack;
			T						mBuffer;
			
			RecorderCallback		mRecorderCallback;
			PlayerCallback			mPlayerCallback;
			
			double					mStart;  //!< local start time (in seconds)
			bool					mActive;
			size_t					mFrameCount;
			std::ofstream			mInfoFile;

			Recorder(typename TrackT::Ref iTrack, RecorderCallback iRecorderCallback, PlayerCallback iPlayerCallback) :
				mTrack(iTrack),
				mRecorderCallback(iRecorderCallback),
				mPlayerCallback(iPlayerCallback),
				mStart(0.0),
				mActive(false),
				mFrameCount(0)
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
				return mRecorderCallback;
			}

			PlayerCallback getPlayerCallbackFn() const
			{
				return mPlayerCallback;
			}

			void update()
			{
				if (!mActive || !mRecorderCallback) return;
				// Get current time:
				double tNow = ci::app::getElapsedSeconds() - mStart;
				// Get current frame:
				T tCurr = mRecorderCallback();
				// Check frame validity:
				if( tCurr ) {
					// Set buffer:
					mBuffer = tCurr;
					// Compose frame filename:
					std::string tFilename = ("frame_" + std::to_string(mFrameCount) + "." + get_file_extension<T>());
					// Write frame to info file:
					mInfoFile << tNow << ' ' << tFilename << std::endl;
					// Write frame contents to file:
					write_to_file<T>( mTrack->getDirectory() / tFilename, mBuffer );
					// Increment frame count:
					mFrameCount++;
				}
			}

			void draw()
			{
				if( !mActive || !mPlayerCallback ) return;
				mPlayerCallback( mBuffer );
			}

			void start()
			{
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
				// Start info file:
				start_info_file();
				// Start recording:
				mActive = true;
				mFrameCount = 0;
				mTrack->setLocalOffsetToCurrent();
				mStart = ci::app::getElapsedSeconds();
			}

			void stop()
			{
				stop_info_file();
				mActive = false;
			}
			
			void start_info_file()
			{
				stop_info_file();
				mInfoFile.open( mTrack->getInfoPath().string() );
				
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

		TrackBase::Ref	mMediator;	//!< shared_ptr to track mediator
		ci::fs::path	mDirectory;	//!< track's base directory
		std::string		mName;		//!< track's base filename
		
		/** @brief default constructor */
		TrackT(const ci::fs::path& iDirectory, const std::string& iName, Timer::Ref iTimer)
		: Track( iTimer ), mDirectory( iDirectory ), mName( iName ) { /* no-op */ }
		
		/** @brief parented constructor */
		TrackT(const ci::fs::path& iDirectory, const std::string& iName, Track::Ref iParent)
		: Track( iParent ), mDirectory( iDirectory ), mName( iName ) { /* no-op */ }
		
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
		void start() { if( mMediator ) mMediator->start(); }
		void stop() { if( mMediator ) mMediator->stop(); }
		
		void gotoIdleMode()
		{
			if( mMediator ) mMediator->stop();
			mMediator.reset();
		}
		
		void gotoPlayMode()
		{
			// Stop mediator:
			if (mMediator) mMediator->stop();
			// Cast mediator to recorder:
			typename Recorder::Ref tRecorderCast = std::dynamic_pointer_cast<Recorder>(mMediator);
			// Return on cast error:
			if (!tRecorderCast) { return; }
			mMediator = Player::create(getRef<TrackT>(), tRecorderCast->getPlayerCallbackFn());
			mMediator->start();
		}
		
		void gotoRecordMode(RecorderCallback iRecorderCallback, PlayerCallback iPlayerCallback)
		{
			if( mMediator ) mMediator->stop();
			mMediator = Recorder::create(getRef<TrackT>(), iRecorderCallback, iPlayerCallback);
			mMediator->start();
		}
	};

} } // namespace itp::multitrack
