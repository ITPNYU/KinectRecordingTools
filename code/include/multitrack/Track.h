#pragma once

#include <multitrack/Timer.h>

namespace itp { namespace multitrack {
	
	/** @brief abstract base class for track and track-mediator types */
	class TrackBase : public std::enable_shared_from_this<TrackBase> {
	public:
		
		typedef std::shared_ptr<TrackBase> Ref;
		
	protected:
		/** @brief default constructor */
		TrackBase() { /* no-op */ }
		
	public:
		
		/** @brief virtual destructor */
		virtual ~TrackBase()
		{
			/* no-op */
		}
		
		/** @brief dynamically casts and returns this entity as const shared_ptr to templated type */
		template <typename T> std::shared_ptr<const T> getRef() const
		{
			return std::dynamic_pointer_cast<T>( shared_from_this() );
		}
		
		/** @brief dynamically casts and returns this entity as shared_ptr to templated type */
		template <typename T> std::shared_ptr<T> getRef()
		{
			return std::dynamic_pointer_cast<T>( shared_from_this() );
		}
		
		/** @brief pure virtual update method */
		virtual void update() = 0;
		
		/** @brief pure virtual draw method */
		virtual void draw() = 0;
		
		/** @brief pure virtual start method */
		virtual void start() = 0;
		
		/** @brief pure virtual stop method */
		virtual void stop() = 0;
	};
	
	/** @brief abstract base class for track types */
	class Track : public TrackBase {
	public:
		
		typedef std::shared_ptr<Track>				Ref;
		typedef std::shared_ptr<const Track>		ConstRef;
		typedef std::weak_ptr<Track>				WeakRef;
		
		typedef std::deque<Ref>						RefDeque;
		
	protected:
		
		Track::WeakRef	mParent; //!< track's parent
		Timer::Ref		mTimer;  //!< sequence timer
		
		/** @brief default constructor */
		Track(Timer::Ref iTimer) :
		mTimer( iTimer )
		{ /* no-op */ }
		
		/** @brief parented constructor */
		Track(Track::Ref iParent) :
		mParent( Track::WeakRef( iParent ) ),
		mTimer( iParent->getTimer() )
		{ /* no-op */ }
		
	public:
		
		/** @brief virtual destructor */
		virtual ~Track()
		{
			/* no-op */
		}
		
		/** @brief returns const shared_ptr to sequence timer */
		Timer::ConstRef getTimer() const
		{
			return mTimer;
		}

		/** @brief returns shared_ptr to sequence timer */
		Timer::Ref getTimer()
		{
			return mTimer;
		}
		
		/** @brief returns true if track has a parent */
		bool hasParent() const
		{
			return ( mParent.lock().get() != NULL );
		}
		
		/** @brief returns const shared_ptr to parent */
		ConstRef getParent() const
		{
			return mParent.lock();
		}
		
		/** @brief returns shared_ptr to parent */
		Ref getParent()
		{
			return mParent.lock();
		}
		
		/** @brief pure virtual play-mode method */
		virtual void gotoPlayMode() = 0;

		/** @brief pure virtual frame-count getter method */
		virtual size_t getFrameCount() const = 0;
	};

} } // namespace itp::multitrack
