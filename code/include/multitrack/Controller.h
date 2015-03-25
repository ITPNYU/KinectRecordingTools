#pragma once

#include "TrackGroup.h"

#include <multitrack/Track.h>
#include <multitrack/TypeTrack.h>

namespace itp { namespace multitrack {
	
	class Controller : public TrackBase {
	public:
		
		typedef std::shared_ptr<Controller>			Ref;
		typedef std::shared_ptr<const Controller>	ConstRef;
		
	private:
		
		Timer::Ref		mTimer;
		Track::Ref		mCursor; // TODO: mRecordingDevices?
		TrackGroup::Ref	mSequence;
		
		/** @brief default constructor */
		Controller() :
		mTimer( Timer::create() ),
		mSequence( TrackGroup::create( mTimer ) )
		{ /* no-op */ }
		
	public:
		
		/** @brief static creational method */
		template <typename ... Args> static Controller::Ref create(Args&& ... args)
		{
			return Controller::Ref( new Controller( std::forward<Args>( args )... ) );
		}
		
		void update()
		{
			mTimer->update();
			mSequence->update();
		}
		
		void draw()
		{
			mSequence->draw();
		}
		
		void start()
		{
			mTimer->start();
			mSequence->start();
		}
		
		void stop()
		{
			mTimer->stop();
			mSequence->stop();
		}

		void cancelRecorder()
		{
			if( mCursor ) {
				mCursor->stop();
				mSequence->removeTrack( mCursor );
				mCursor.reset();
			}
		}
		
		void completeRecorder()
		{
			if( mCursor ) {
				mCursor->gotoPlayMode();
				mCursor.reset();
			}
		}
		
		template <typename T> void addRecorder(std::function<T(void)> iRecorderCallbackFn, std::function<void(const T&)> iPlayerCallbackFn)
		{
			mCursor = mSequence->addTrackRecorder<T>(iRecorderCallbackFn, iPlayerCallbackFn);
		}
	};
	
} } // namespace itp::multitrack
