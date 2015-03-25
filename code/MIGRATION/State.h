/* ITP Future of Storytelling */

#pragma once

#include <iostream>
#include <functional>
#include <memory>

#include <string>
#include <deque>
#include <map>

/** @brief abstract base class for state entities */
class State : public std::enable_shared_from_this<State> {
public:

	typedef std::shared_ptr<State>				Ref;
	typedef std::shared_ptr<const State>		ConstRef;

	typedef std::map<std::string,State::Ref>	RefMap;
	typedef std::deque<State::Ref>				RefDeque;

protected:

	std::string mName;

	/** @brief protected constructor */
	State(const std::string& iName) : mName(iName) { /* no-op */ }

	/** @brief pure virtual initialization method */
	virtual void initialize() = 0;

public:

	/** @brief virtual destructor */
	virtual ~State() { /* no-op */ }

	/** @brief returns a const shared_ptr to this state object */
	State::ConstRef getBaseRef() const { return shared_from_this(); }

	/** @brief returns a shared_ptr to this state object */
	State::Ref getBaseRef() { return shared_from_this(); }

	/** @brief returns a const ref to this state's name */
	const std::string& getName() const { return mName; }

	/** @brief pure virtual update method */
	virtual void update() = 0;

	/** @brief pure virtual draw method */
	virtual void draw() = 0;
};