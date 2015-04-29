#pragma once

#include <multitrack/Track.h>
#include <multitrack/TypeTrack.h>
#include <multitrack/TrackGroup.h>

namespace itp { namespace multitrack {
	
	class Controller : public TrackBase {
	public:
		
		typedef std::shared_ptr<Controller>			Ref;
		typedef std::shared_ptr<const Controller>	ConstRef;
		
	private:
		
		Timer::Ref		mTimer;
		TrackGroup::Ref	mSequence;
		Track::RefDeque	mRecordingDevices;
		ci::fs::path	mDirectory;
		size_t			mUidGenerator;
		
		/** @brief default constructor */
		Controller(const ci::fs::path& iDirectory) :
			mTimer( Timer::create() ),
			mSequence( TrackGroup::create( mTimer ) ),
			mDirectory( iDirectory ),
			mUidGenerator( 0 )
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

		void resetTimer()
		{
			mTimer->reset();
		}

		void cancelRecorder()
		{
			for (auto& tDevice : mRecordingDevices) {
				if (tDevice) {
					tDevice->stop();
					mSequence->removeTrack(tDevice);
					tDevice.reset();
				}
			}
			mRecordingDevices.clear();
		}
		
		void completeRecorder()
		{
			for (auto& tDevice : mRecordingDevices) {
				if (tDevice) {
					tDevice->gotoPlayMode();
					tDevice.reset();
				}
			}
			mRecordingDevices.clear();
		}
		
		template <typename T> void addRecorder(std::function<T(void)> iRecorderCallbackFn, std::function<void(const T&)> iPlayerCallbackFn)
		{
			mRecordingDevices.push_back(mSequence->addTrackRecorder<T>( mDirectory, "track_" + std::to_string( mUidGenerator ), iRecorderCallbackFn, iPlayerCallbackFn));
			// Increment uid generator:
			mUidGenerator++;
		}
	};
	
} } // namespace itp::multitrack
