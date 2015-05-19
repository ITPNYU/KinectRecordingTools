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
			Group(const std::string& name) : mName(name) { /* no-op */ }

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

			/** @brief adds track to top of group */
			void push(const Track::Ref& track)
			{
				mTracks.push_back(track);
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

			/** @brief record-mode method */
			void gotoRecordMode(bool active)
			{
				for (auto& tTrack : mTracks) {
					tTrack->gotoRecordMode(active);
				}
			}

			/** @brief play-mode method */
			void gotoPlayMode()
			{
				for (auto& tTrack : mTracks) {
					tTrack->gotoPlayMode();
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
		};

	protected:

		Timer::Ref mTimer;  //!< sequence timer

		/** @brief default constructor */
		Track(Timer::Ref timer) : mTimer(timer) { /* no-op */ }

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

		/** @brief pure virtual record-mode method */
		virtual void gotoRecordMode(bool active) = 0;

		/** @brief pure virtual play-mode method */
		virtual void gotoPlayMode() = 0;

		/** @brief pure virtual frame-count getter method */
		virtual size_t getFrameCount() const = 0;
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
			bool					mInitialized;
			PlayerCallback			mPlayerCb;
			
			/** @brief basic constructor */
			Player(typename TrackT::Ref track, PlayerCallback playerCb) :
				mTrack(track),
				mPlayerCb(playerCb),
				mInfoVec( FrameInfoVec() ),
				mInitialized( false )
			{
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
				// Get iterator:
				mInfoIterator = mInfoVec.begin();
				// Iterate over frames:
				while (mInfoIterator != mInfoVec.end()) {
					FrameInfoIter iterNext = mInfoIterator + 1;
					if (iterNext != mInfoVec.end() && mTrack->mTimer->getPlayhead() < iterNext->first) {
						break;
					}
					mInfoIterator = iterNext;
				}
				// Set initialization flag:
				mInitialized = (mInfoIterator != mInfoVec.end());
			}

			/** @brief draw method */
			void draw()
			{
				if (!mPlayerCb || !mInitialized) return;
				mPlayerCb(read_from_file<T>(mTrack->getDirectory() / mInfoIterator->second));
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

			Recorder(typename TrackT::Ref track, RecorderCallback recorderCb, PlayerCallback playerCb, bool active) :
				mTrack(track),
				mRecorderCb(recorderCb),
				mPlayerCb(playerCb),
				mActive(active)
			{ 
				// Start active recorder, if applicable:
				if (mActive) {
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
					mInfoFile.open(mTrack->getInfoPath().string());
					// Handle info file not found error:
					if (!mInfoFile.is_open()) {
						throw std::runtime_error("Recorder could not open file: \'" + mTrack->getInfoPath().string() + "\'");
					}
					// Reset track's framecount:
					mTrack->mFrameCount = 0;
				}
			}

		public:
			
			~Recorder()
			{
				// Stop active recorder, if applicable:
				if (mActive) {
					if (mInfoFile.is_open()) {
						mInfoFile.close();
					}
				}
			}

			/** @brief static creational method */
			template <typename ... Args> static typename Recorder::Ref create(Args&& ... args)
			{
				return Recorder::Ref(new Recorder(std::forward<Args>(args)...));
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

		void gotoPlayMode()
		{
			// Stop mediator:
			if (mMediator) mMediator.reset();
			// Create player mediator:
			mMediator = Player::create(getRef<TrackT>(), mPlayerCb);
		}

		void gotoRecordMode(bool active)
		{
			// Stop mediator:
			if (mMediator) mMediator.reset();
			// Create recorder mediator:
			mMediator = Recorder::create(getRef<TrackT>(), mRecorderCb, mPlayerCb, active);
		}

		/** @brief returns the track's frame-count */
		size_t getFrameCount() const
		{
			return mFrameCount;
		}
	};

} } // namespace itp::multitrack
