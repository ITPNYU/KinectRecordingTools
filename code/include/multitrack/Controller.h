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

		void resetAll()
		{
			mTimer->reset();
			mSequence->removeAllTracks();
			mUidGenerator = 0;
		}

		void resetTimer()
		{
			mTimer->reset();
		}

		Timer::Ref getTimer() const
		{
			return mTimer;
		}

		void cancelRecorder()
		{
			// Iterate over recording devices:
			for (auto& tDevice : mRecordingDevices) {
				// Check current device validity:
				if (tDevice) {
					// Remove track:
					mSequence->removeTrack(tDevice);
				}
			}
			// Clear recording devices:
			mRecordingDevices.clear();
		}
		
		void completeRecorder()
		{
			// Iterate over recording devices:
			for (auto& tDevice : mRecordingDevices) {
				// Check current device validity:
				if (tDevice) {
					// Convert non-empty track to player:
					if ( tDevice->getFrameCount() > 0 ) {
						tDevice->gotoPlayMode();
					}
					// Remove empty track:
					else {
						mSequence->removeTrack(tDevice);
					}
				}
			}
			// Clear recording devices:
			mRecordingDevices.clear();
		}
		
		template <typename T> void addRecorder(std::function<T(void)> recorderCb, std::function<void(const T&)> playerCb)
		{
			mRecordingDevices.push_back(mSequence->addTrack<T>(mDirectory, "track_" + std::to_string(mUidGenerator), recorderCb, playerCb));
			// Increment uid generator:
			mUidGenerator++;
		}
	};
	
} } // namespace itp::multitrack
