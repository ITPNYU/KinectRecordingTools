#pragma once

#include <multitrack/Timer.h>

namespace itp { namespace multitrack {

	/** @brief abstract base class for track-mediator types */
	class Mediator : public std::enable_shared_from_this<Mediator> {
	public:

		typedef std::shared_ptr<Mediator> Ref;

	protected:

		bool mActive; //!< flags whether mediator is active

		/** @brief default constructor */
		Mediator(bool active) : mActive(active) { /* no-op */ }

	public:

		/** @brief virtual destructor */
		virtual ~Mediator()
		{
			/* no-op */
		}

		/** @brief dynamically casts and returns this entity as const shared_ptr to templated type */
		template <typename T> std::shared_ptr<const T> getRef() const
		{
			return std::dynamic_pointer_cast<T>(shared_from_this());
		}

		/** @brief dynamically casts and returns this entity as shared_ptr to templated type */
		template <typename T> std::shared_ptr<T> getRef()
		{
			return std::dynamic_pointer_cast<T>(shared_from_this());
		}

		/** @brief returns mediator's active flag */
		const bool& isActive() const
		{
			return mActive;
		}

		/** @brief sets mediator's active flag */
		void setActiveFlag(bool active)
		{
			mActive = active;
		}

		/** @brief pure virtual update method */
		virtual void update() = 0;

		/** @brief pure virtual draw method */
		virtual void draw() = 0;
	};

} } // namespace itp::multitrack
