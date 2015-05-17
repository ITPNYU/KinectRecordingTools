#pragma once

#include <multitrack/Track.h>
#include <multitrack/TypeTrack.h>

namespace itp { namespace multitrack {

	class Controller {
	public:
		
		typedef std::shared_ptr<Controller>			Ref;
		typedef std::shared_ptr<const Controller>	ConstRef;
		
		struct Group 
		{
			std::string		mName;		//!< group name
			Track::RefDeque	mTracks;	//!< track vector
		};

	private:
		
		Timer::Ref		mTimer;

		Group			mSequence;
		
		Track::RefDeque	mRecordingDevices;
		ci::fs::path	mDirectory;
		size_t			mUidGenerator;
		
		/** @brief default constructor */
		Controller(const ci::fs::path& iDirectory) :
			mTimer( Timer::create() ),
			mSequence(Group()),
			mDirectory( iDirectory ),
			mUidGenerator( 0 )
		{ /* no-op */ }
		
	public:
		
		/** @brief static creational method */
		template <typename ... Args> static Controller::Ref create(Args&& ... args)
		{
			return Controller::Ref( new Controller( std::forward<Args>( args )... ) );
		}

		const size_t& getCurrentId() const
		{
			return mUidGenerator;
		}
		
		void update()
		{
			// Update timer:
			mTimer->update();
			// Update sequence:
			for (auto& tTrack : mSequence.mTracks) {
				tTrack->update();
			}
		}
		
		void draw()
		{
			ci::gl::color(1.0, 1.0, 1.0, 1.0);
			for (auto& tTrack : mSequence.mTracks) {
				tTrack->draw();
			}
		}
		
		void start()
		{
			for (auto& tTrack : mSequence.mTracks) {
				tTrack->start();
			}
		}
		
		void stop()
		{
			for (auto& tTrack : mSequence.mTracks) {
				tTrack->stop();
			}
		}

		void resetSequence()
		{
			mSequence.mTracks.clear();
			mUidGenerator = 0;
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
					for (Track::RefDeque::iterator it = mSequence.mTracks.begin(); it != mSequence.mTracks.end(); it++) {
						if (tDevice.get() == (*it).get()) {
							(*it)->stop();
							mSequence.mTracks.erase(it);
							break;
						}
					}
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
					// Get frame count:
					size_t tFrameCount = 0;
					for (const auto &tTrack : mSequence.mTracks) {
						tFrameCount = std::max(tFrameCount, tTrack->getFrameCount());
					}
					// Convert non-empty track to player:
					if (tFrameCount > 0) {
						tDevice->gotoPlayMode();
					}
					// Remove empty track:
					else {
						// Remove track:
						for (Track::RefDeque::iterator it = mSequence.mTracks.begin(); it != mSequence.mTracks.end(); it++) {
							if (tDevice.get() == (*it).get()) {
								(*it)->stop();
								mSequence.mTracks.erase(it);
								break;
							}
						}
					}
				}
			}
			// Clear recording devices:
			mRecordingDevices.clear();
		}
		
		template <typename T> void addRecorder(std::function<T(void)> recorderCb, std::function<void(const T&)> playerCb)
		{
			// Create typed track:
			typename TrackT<T>::Ref tTrack = TrackT<T>::create(mDirectory, "track_" + std::to_string(mUidGenerator), mTimer);
			// Add track to controller:
			mSequence.mTracks.push_back(tTrack);
			// Initialize recorder:
			tTrack->gotoRecordMode(recorderCb, playerCb);
			// TODO
			mRecordingDevices.push_back(tTrack);
			// Increment uid generator:
			mUidGenerator++;
		}

		template <typename T> void addPlayer(std::function<void(const T&)> playerCb)
		{
			// Create typed track:
			typename TrackT<T>::Ref tTrack = TrackT<T>::create(mDirectory, "track_" + std::to_string(mUidGenerator), mTimer);
			// Add track to controller:
			mSequence.mTracks.push_back(tTrack);
			// Initialize recorder:
			tTrack->gotoPlayMode(playerCb);
			// Increment uid generator:
			mUidGenerator++;
		}
	};
	
} } // namespace itp::multitrack
