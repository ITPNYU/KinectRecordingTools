#pragma once

#include <multitrack/Track.h>

namespace itp { namespace multitrack {

	/** @brief templated track type */
	template <typename T> class TrackT : public Track {
	public:
		
		typedef std::shared_ptr<TrackT>				Ref;
		typedef std::deque<Ref>						RefDeque;
		
		typedef std::pair<double,T>					Frame;
		typedef std::vector<Frame>					FrameVec;
		
		typedef typename FrameVec::iterator			Iter;
		typedef typename FrameVec::const_iterator	ConstIter;
		
		typedef std::function<T(void)>				RecorderCallback;
		typedef std::function<void(const T&)>		PlayerCallback;

		/** @brief track player */
		class Player : public TrackBase {
		public:

			typedef std::shared_ptr<Player>	Ref;

		private:

			typename TrackT::Ref	mTrack;
			PlayerCallback			mPlayerCallback;
			Iter					mIterator;
			double					mKeyTimeCurr;
			double					mKeyTimeNext;

			Player(typename TrackT::Ref iTrack, PlayerCallback iPlayerCallback) :
				mTrack(iTrack),
				mPlayerCallback(iPlayerCallback),
				mIterator(mTrack->end()),
				mKeyTimeCurr(0.0),
				mKeyTimeNext(0.0)
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
				if (mTrack->empty()) {
					mIterator = mTrack->end();
					return;
				}
				// Compute local playhead:
				double tLocalPlayhead = mTrack->getTimer()->getPlayhead() - mTrack->getOffset();
				// Get duration:
				double tDuration = mTrack->back().first;
				// Handle playhead out-of-range:
				if (tLocalPlayhead < 0.0 || tLocalPlayhead > tDuration) {
					mIterator = mTrack->end();
					return;
				}
				// Activate, if necessary:
				if (mIterator == mTrack->end()) {
					mIterator = mTrack->begin();
					mKeyTimeCurr = mTrack->front().first;
					mKeyTimeNext = ((mTrack->size() > 1) ? (mIterator + 1)->first : mKeyTimeCurr);
				}
				// Update iterator:
				while (tLocalPlayhead >= mKeyTimeNext && mIterator != mTrack->end()) {
					mKeyTimeCurr = (*mIterator).first;
					mKeyTimeNext = ((++mIterator == mTrack->end()) ? mKeyTimeCurr : (*mIterator).first);
				}
			}

			void draw()
			{
				if (!mPlayerCallback || mIterator == mTrack->end()) return;
				mPlayerCallback((*mIterator).second);
			}
		};

		/** @brief track recorder */
		class Recorder : public TrackBase {
		public:

			typedef std::shared_ptr<Recorder> Ref;

		private:

			typename TrackT::Ref	mTrack;
			RecorderCallback		mRecorderCallback;
			PlayerCallback			mPlayerCallback;
			double					mStart;  //!< local start time (in seconds)
			bool					mActive;

			Recorder(typename TrackT::Ref iTrack, RecorderCallback iRecorderCallback, PlayerCallback iPlayerCallback) :
				mTrack(iTrack),
				mRecorderCallback(iRecorderCallback),
				mPlayerCallback(iPlayerCallback),
				mStart(0.0),
				mActive(false)
			{ 
				/* no-op */
			}

		public:

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
				mTrack->push_back(ci::app::getElapsedSeconds() - mStart, mRecorderCallback());
			}

			void draw()
			{
				if (!mActive || !mPlayerCallback || mTrack->empty()) return;
				mPlayerCallback(mTrack->back().second);
			}

			void start()
			{
				mActive = true;
				mTrack->clear();
				mTrack->setLocalOffset(mTrack->getTimer()->getPlayhead());
				mStart = ci::app::getElapsedSeconds();
			}

			void stop()
			{
				mActive = false;
			}
		};
		
	private:
		
		FrameVec		mFrames;	//!< track's frame container
		TrackBase::Ref	mMediator;	//!< shared_ptr to track mediator
		
		/** @brief default constructor */
		TrackT(Timer::Ref iTimer) : Track( iTimer ) { /* no-op */ }
		
		/** @brief parented constructor */
		TrackT(Track::Ref iParent) : Track( iParent ) { /* no-op */ }
		
	public:
		
		/** @brief static creational method */
		template <typename ... Args> static typename TrackT::Ref create(Args&& ... args)
		{
			return TrackT::Ref( new TrackT( std::forward<Args>( args )... ) );
		}
		
		bool empty() const { return mFrames.empty(); }
		size_t size() const { return mFrames.size(); }
		void clear() { mFrames.clear(); }
		
		void push_back(double iTime, const T& iValue) { if( iValue ) mFrames.push_back( Frame( iTime, iValue ) ); }
		void pop_back() { mFrames.pop_back(); }
		
		ConstIter begin() const { return mFrames.cbegin(); }
		Iter begin() { return mFrames.begin(); }
		
		ConstIter end() const { return mFrames.cend(); }
		Iter end() { return mFrames.end(); }
		
		const Frame& front() const { return mFrames.front(); }
		Frame& front() { return mFrames.front(); }
		
		const Frame& back() const { return mFrames.back(); }
		Frame& back() { return mFrames.back(); }
		
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
			Recorder::Ref tRecorderCast = std::dynamic_pointer_cast<Recorder>(mMediator);
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
