#pragma once

#include <stdexcept>
#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include <memory>
#include <functional>

#include "cinder/gl/gl.h"

namespace itp { namespace multitrack {
	
	class Timer {
	public:
		
		typedef std::shared_ptr<Timer>			Ref;
		typedef std::shared_ptr<const Timer>	ConstRef;

		typedef std::function<void(void)>		CallbackFn;
		
	private:
		
		bool		mActive;		//!< activity flag
		double		mStart;			//!< local start time (in seconds)
		double		mPlayhead;		//!< playhead time (in seconds)
		double		mLoopMarker;	//!< loop marker (in seconds)
		CallbackFn	mLoopCallback;	//!< loop callback
		
		/** @brief default constructor */
		Timer() :
		mActive( false ),
		mStart( 0.0 ),
		mPlayhead( 0.0 ),
		mLoopMarker( -1.0 )
		{ /* no-op */ }
		
	public:
		
		/** @brief static creational method */
		template <typename ... Args> static Timer::Ref create(Args&& ... args)
		{
			return Timer::Ref( new Timer( std::forward<Args>( args )... ) );
		}

		/** @brief sets loop callback function */
		void setLoopCallback(CallbackFn cb)
		{
			mLoopCallback = cb;
		}

		/** @brief returns true if looping is enabled */
		bool isLoopMarkerEnabled() const
		{
			return ( mLoopMarker > 0.0 );
		}

		/** @brief disables looping */
		void disableLoopMarker()
		{
			mLoopMarker = -1.0;
		}

		/** @brief loop marker setter method */
		void setLoopMarker(double marker)
		{
			mLoopMarker = marker;
		}

		/** @brief loop marker getter method */
		const double& getLoopMarker() const
		{
			return mLoopMarker;
		}
		
		/** @brief playhead getter method */
		const double& getPlayhead() const
		{
			return mPlayhead;
		}
		
		/** @brief timer update method */
		void update()
		{
			if( ! mActive ) return;
			mPlayhead = ci::app::getElapsedSeconds() - mStart;
			if (isLoopMarkerEnabled() && mPlayhead >= mLoopMarker) {
				mStart = ci::app::getElapsedSeconds();
				mPlayhead = 0.0f;
				if (mLoopCallback) {
					mLoopCallback();
				}
			}
		}
		
		/** @brief timer start method */
		void start()
		{
			mActive   = true;
			mStart    = ci::app::getElapsedSeconds();
			mPlayhead = 0.0f;
		}

		/** @brief timer start-at-time method */
		void startAt(double playhead)
		{
			mActive = true;
			mStart = ci::app::getElapsedSeconds() - playhead;
			mPlayhead = playhead;
		}
		
		/** @brief timer stop method */
		void pause()
		{
			mActive = false;
		}

		/** @brief timer reset method */
		void stop()
		{
			mActive   = false;
			mStart    = 0.0;
			mPlayhead = -1.0;
		}
	};
	
} } // namespace itp::multitrack
