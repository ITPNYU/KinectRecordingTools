#pragma once

#include <memory>
#include <iostream>
#include <sstream>
#include <string>

#include "cinder/gl/gl.h"

namespace sequence {
	
	class Timer {
	public:
		
		typedef std::shared_ptr<Timer>			Ref;
		typedef std::shared_ptr<const Timer>	ConstRef;
		
	private:
		
		bool	mActive;	//!< activity flag
		double	mStart;		//!< local start time (in seconds)
		double	mPlayhead;	//!< playhead time (in seconds)
		
		/** @brief default constructor */
		Timer() :
		mActive( false ),
		mStart( 0.0 ),
		mPlayhead( 0.0 )
		{ /* no-op */ }
		
	public:
		
		/** @brief static creational method */
		template <typename ... Args> static Timer::Ref create(Args&& ... args)
		{
			return Timer::Ref( new Timer( std::forward<Args>( args )... ) );
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
		}
		
		/** @brief timer start method */
		void start()
		{
			mActive = true;
			mStart  = ci::app::getElapsedSeconds();
		}
		
		/** @brief timer stop method */
		void stop()
		{
			mActive = false;
		}
	};
	
} // namespace sequence
