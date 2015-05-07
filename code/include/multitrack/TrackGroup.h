#pragma once

#include <multitrack/Track.h>
#include <multitrack/TypeTrack.h>

namespace itp { namespace multitrack {

	class TrackGroup : public Track {
	public:
		
		typedef std::shared_ptr<TrackGroup>			Ref;
		typedef std::shared_ptr<const TrackGroup>	ConstRef;
		
	private:
		
		Track::RefDeque	mTracks; //!< track vector
		
		/** @brief default constructor */
		TrackGroup(Timer::Ref iTimer)
		: Track( iTimer ) { /* no-op */ }
		
		/** @brief parented constructor */
		TrackGroup(Track::Ref iParent)
		: Track( iParent ) { /* no-op */ }
		
	public:
		
		/** @brief static creational method */
		template <typename ... Args> static TrackGroup::Ref create(Args&& ... args)
		{
			return TrackGroup::Ref( new TrackGroup( std::forward<Args>( args )... ) );
		}
		
		/** @brief update method */
		void update()
		{
			for( auto &tTrack : mTracks ) {
				tTrack->update();
			}
		}
		
		/** @brief draw method */
		void draw()
		{
			ci::gl::color(1.0, 1.0, 1.0, 1.0);
			for( auto &tTrack : mTracks ) {
				tTrack->draw();
			}
		}

		/** @brief start method */
		void start()
		{
			for (auto &tTrack : mTracks) {
				tTrack->start();
			}
		}

		/** @brief stop method */
		void stop()
		{
			for (auto &tTrack : mTracks) {
				tTrack->stop();
			}
		}
		
		void removeTrack(Track::Ref iTrack)
		{
			for(Track::RefDeque::iterator it = mTracks.begin(); it != mTracks.end(); it++) {
				if( iTrack.get() == (*it).get() ) {
					(*it)->stop();
					mTracks.erase( it );
					return;
				}
			}
		}
		
		template <typename T> 
		Track::Ref addTrack(const ci::fs::path& dir, const std::string& name, std::function<T(void)> recorderCb, std::function<void(const T&)> playerCb)
		{
			// Create typed track:
			typename TrackT<T>::Ref tTrack = TrackT<T>::create(dir, name, getRef<TrackGroup>());
			// Add track to controller:
			mTracks.push_back(tTrack);
			// Initialize recorder:
			tTrack->gotoRecordMode(recorderCb, playerCb);
			// Return track:
			return tTrack;
		}

		/** @brief  play-mode method */
		void gotoPlayMode()
		{
			for (auto &tTrack : mTracks) {
				tTrack->gotoPlayMode();
			}
		}

		/** @brief returns the track group's frame-count */
		size_t getFrameCount() const
		{
			size_t tCount = 0;
			for (const auto &tTrack : mTracks) {
				tCount = std::max(tCount, tTrack->getFrameCount());
			}
			return tCount;
		}
	};

} } // namespace itp::multitrack
