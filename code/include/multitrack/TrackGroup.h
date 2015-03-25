#pragma once

#include "Track.h"

namespace sequence {

	class TrackGroup : public Track {
	public:
		
		typedef std::shared_ptr<TrackGroup>		Ref;
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
		
		void update()
		{
			// Update tracks:
			for( auto &tTrack : mTracks ) {
				tTrack->update();
			}
		}
		
		void draw()
		{
			// Draw tracks:
			for( auto &tTrack : mTracks ) {
				ci::gl::color( 1.0, 1.0, 1.0, 0.5 ); // TODO
				tTrack->draw();
			}
			// Draw info:
			std::stringstream ss;
			ss << "TIME: " << mTimer->getPlayhead();
			ci::gl::drawString( ss.str(), ci::vec2( 25.0 ), ci::Color::white() );
		}
		
		void removeTrack(Track::Ref iTrack)
		{
			for(Track::RefDeque::iterator it = mTracks.begin(); it != mTracks.end(); it++) {
				if( iTrack.get() == (*it).get() ) {
					mTracks.erase( it );
					return;
				}
			}
		}
		
		void addTrack(Track::Ref iTrack)
		{
			mTracks.push_back( iTrack );
		}
		
		template <typename T> Track::Ref addTrackRecorder(std::function<T(void)> iCallbackFn)
		{
			// Create typed track:
			typename TrackT<T>::Ref tTrack = TrackT<T>::create( getRef<TrackGroup>() );
			// Add track to controller:
			addTrack( tTrack );
			// Start recorder:
			tTrack->gotoRecordMode( iCallbackFn );
			// Return track:
			return tTrack;
		}
	};

} // namespace sequence
