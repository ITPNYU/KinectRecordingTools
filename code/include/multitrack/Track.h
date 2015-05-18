#pragma once

#include <multitrack/TrackType.h>
#include <multitrack/Mediator.h>

namespace itp { namespace multitrack {

	/** @brief abstract base class for track types */
	class Track : public std::enable_shared_from_this<Track> {
	public:

		typedef std::shared_ptr<Track>				Ref;
		typedef std::shared_ptr<const Track>		ConstRef;

		typedef std::deque<Track::Ref>				RefDeque;

		/** @brief track group wrapper */
		class Group {
		public:

			typedef std::shared_ptr<Group>			Ref;
			typedef std::deque<Group::Ref>			RefDeque;

		private:

			std::string		mName;		//!< group name
			Track::RefDeque	mTracks;	//!< track vector

			/** @brief basic constructor */
			Group(const std::string& name) :
				mName(name)
			{ /* no-op */
			}

		public:

			/** @brief static creational method */
			template <typename ... Args> static Group::Ref create(Args&& ... args)
			{
				return Group::Ref(new Group(std::forward<Args>(args)...));
			}

			const std::string& getName() const
			{
				return mName;
			}

			/** @brief adds track to bottom of group */
			void pushBottom(const Track::Ref& track)
			{
				mTracks.push_front(track);
			}

			/** @brief adds track to top of group */
			void pushTop(const Track::Ref& track)
			{
				mTracks.push_back(track);
			}

			/** @brief remove bottom track */
			void popBottom()
			{
				mTracks.pop_front();
			}

			/** @brief remove top track */
			void popTop()
			{
				mTracks.pop_back();
			}

			/** @brief update method */
			void update()
			{
				for (auto& tTrack : mTracks) {
					tTrack->update();
				}
			}

			/** @brief draw method */
			void draw()
			{
				for (auto& tTrack : mTracks) {
					tTrack->draw();
				}
			}

			/** @brief start method */
			void start()
			{
				for (auto& tTrack : mTracks) {
					tTrack->start();
				}
			}

			/** @brief stop method */
			void stop()
			{
				for (auto& tTrack : mTracks) {
					tTrack->stop();
				}
			}

			/** @brief returns the maximum framecount within group */
			size_t getFrameCount() const
			{
				size_t count = 0;
				for (auto& tTrack : mTracks) {
					count = std::max(count, tTrack->getFrameCount());
				}
				return count;
			}

			/** @brief record-mode method */
			void gotoRecordMode()
			{
				for (auto& tTrack : mTracks) {
					tTrack->gotoRecordMode();
				}
			}

			/** @brief play-mode method */
			void gotoPlayMode()
			{
				for (auto& tTrack : mTracks) {
					tTrack->gotoPlayMode();
				}
			}
		};

	protected:

		Timer::Ref mTimer;  //!< sequence timer

		/** @brief default constructor */
		Track(Timer::Ref iTimer) :
			mTimer(iTimer)
		{ /* no-op */
		}

	public:

		/** @brief virtual destructor */
		virtual ~Track()
		{
			/* no-op */
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

		/** @brief pure virtual start method */
		virtual void start() = 0;

		/** @brief pure virtual stop method */
		virtual void stop() = 0;

		/** @brief pure virtual frame-count getter method */
		virtual size_t getFrameCount() const = 0;

		/** @brief pure virtual record-mode method */
		virtual void gotoRecordMode() = 0;

		/** @brief pure virtual play-mode method */
		virtual void gotoPlayMode() = 0;
	};

	/** @brief templated track type */
	template <typename T> class TrackT : public Track {
	public:
		
		typedef std::shared_ptr<TrackT>				Ref;
		typedef std::deque<Ref>						RefDeque;

		typedef std::function<T(void)>				RecorderCallback;
		typedef std::function<void(const T&)>		PlayerCallback;

		/** @brief track player mediator */
		class Player : public Mediator {
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
				const double& tPlayhead = mTrack->mTimer->getPlayhead();
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
				/* no-op */
			}

			/** @brief stop method */
			void stop()
			{ 
				/* no-op */
			}
		};

		/** @brief track recorder mediator */
		class Recorder : public Mediator {
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
						mInfoFile << mTrack->mTimer->getPlayhead() << ' ' << tFilename << std::endl;
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

		Mediator::Ref		mMediator;		//!< shared_ptr to track mediator
		ci::fs::path		mDirectory;		//!< track's base directory
		std::string			mName;			//!< track's base filename
		size_t				mFrameCount;	//!< number of frames recorded
		RecorderCallback	mRecorderCb;	//!< recorder callback function
		PlayerCallback		mPlayerCb;		//!< player callback function
		
		/** @brief default constructor */
		TrackT(const ci::fs::path& dir, const std::string& name, Timer::Ref timer, RecorderCallback recorderCb, PlayerCallback playerCb, size_t frameCount = 0) :
			Track(timer), 
			mDirectory(dir),
			mName(name),
			mRecorderCb(recorderCb),
			mPlayerCb(playerCb),
			mFrameCount(frameCount)
		{ /* no-op */ }
		
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
			// Create player mediator:
			mMediator = Player::create(getRef<TrackT>(), mPlayerCb);
		}

		void gotoRecordMode()
		{
			// Stop mediator:
			if( mMediator ) mMediator->stop();
			// Create recorder mediator:
			mMediator = Recorder::create(getRef<TrackT>(), mRecorderCb, mPlayerCb);
		}

		/** @brief returns the track's frame-count */
		size_t getFrameCount() const
		{
			return mFrameCount;
		}
	};

} } // namespace itp::multitrack
