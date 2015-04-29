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
		
		/** @brief overloadable update method */
		virtual void update() { /* no-op */ }
		
		/** @brief overloadable draw method */
		virtual void draw() { /* no-op */ }
		
		/** @brief overloadable start method */
		virtual void start() { /* no-op */ }
		
		/** @brief overloadable stop method */
		virtual void stop() { /* no-op */ }
	};
	
	/** @brief abstract base class for track types */
	class Track : public TrackBase {
	public:
		
		typedef std::shared_ptr<Track>				Ref;
		typedef std::shared_ptr<const Track>		ConstRef;
		typedef std::weak_ptr<Track>				WeakRef;
		
		typedef std::deque<Ref>						RefDeque;
		
	protected:
		
		double			mOffset; //!< track's offset from parent start-time (in seconds)
		Track::WeakRef	mParent; //!< track's parent
		Timer::Ref		mTimer;  //!< sequence timer
		
		/** @brief default constructor */
		Track(Timer::Ref iTimer) :
		mTimer( iTimer ),
		mOffset( 0.0 )
		{ /* no-op */ }
		
		/** @brief parented constructor */
		Track(Track::Ref iParent) :
		mParent( Track::WeakRef( iParent ) ),
		mTimer( iParent->getTimer() ),
		mOffset( 0.0 )
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
		
		/** @brief sets track's local offset (in seconds) */
		void setLocalOffset(double iOffset)
		{
			mOffset = iOffset;
		}
		
		/** @brief sets track's local offset to current time (in seconds) */
		void setLocalOffsetToCurrent()
		{
			mOffset = mTimer->getPlayhead();
		}

		/** @brief returns track's local offset (in seconds) */
		double getLocalOffset() const
		{
			return mOffset;
		}
		
		/** @brief returns track's parent's global offset (in seconds) */
		double getParentOffset() const
		{
			return ( hasParent() ? getParent()->getOffset() : 0.0 );
		}
		
		/** @brief returns track's global offset (in seconds) */
		double getOffset() const
		{
			return getParentOffset() + getLocalOffset();
		}
		
		/** @brief overloadable idle-mode method */
		virtual void gotoIdleMode()
		{
			/* no-op */
		}
		
		/** @brief overloadable play-mode method */
		virtual void gotoPlayMode()
		{
			/* no-op */
		}
	};

} } // namespace itp::multitrack
