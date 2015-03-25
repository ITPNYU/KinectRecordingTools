/* ITP Future of Storytelling */

#pragma once

#include "cinder/Text.h"
#include "cinder/Font.h"
#include "cinder/Rect.h"
#include "cinder/Color.h"
#include "cinder/Vector.h"
#include "cinder/Timeline.h"
#include "cinder/gl/Texture.h"
#include "cinder/app/AppBasic.h"

#include "State.h"

class Directive : public State {
public:

	typedef std::shared_ptr<Directive>				Ref;
	typedef std::shared_ptr<const Directive>		ConstRef;

	typedef std::map<std::string, Directive::Ref>	RefMap;
	typedef std::deque<Directive::Ref>				RefDeque;

private:

	int						mPriority;

	std::string				mLabel;

	ci::Font				mLabelFont;
	ci::gl::TextureRef		mLabelTexture;

	ci::Anim<float>			mLabelOpacity;
	ci::Anim<ci::vec2>		mLabelPosition;

	/** @brief private constructor */
	Directive(const std::string& iName, const std::string& iLabel, const int& iPriority) :
		State(iName),
		mLabel(iLabel),
		mPriority(iPriority),
		mLabelFont(ci::Font("Helvetica", 60)),
		mLabelOpacity(1.0f),
		mLabelPosition(ci::vec2(0.5, 1.0))
	{ /* no-op */ }

	/** @brief initialization method */
	void initialize()
	{
		renderLabel();
	}

	/** @brief label rendering method */
	void renderLabel()
	{
		ci::TextLayout tLayout;
		tLayout.clear(ci::ColorA(0.0f, 0.0f, 0.0f, 0.0f));
		tLayout.setFont(mLabelFont);
		tLayout.setColor(ci::ColorA(1.0f, 1.0f, 0.0f, 1.0f));
		tLayout.addCenteredLine(mLabel);
		tLayout.setBorder(5,5);
		mLabelTexture = ci::gl::Texture::create(tLayout.render(true));
	}

public:

	/** @brief destructor */
	~Directive() { /* no-op */ }

	/** @brief static creational method */
	template <typename ... T> static Directive::Ref create(T&& ... args)
	{
		Directive::Ref t = Directive::Ref(new Directive(std::forward<T>(args)...));
		t->initialize();
		return t;
	}

	/** @brief returns a const shared_ptr to this directive object */
	Directive::ConstRef getRef() const { return std::dynamic_pointer_cast<const Directive>( getBaseRef() ); }

	/** @brief returns a shared_ptr to this directive object */
	Directive::Ref getRef() { return std::dynamic_pointer_cast<Directive>( getBaseRef() ); }

	/** @brief returns a const ref to this directive's priority level */
	const int& getPriority() const { return mPriority; }

	/** @brief returns a const ref to this directive's label */
	const std::string& getLabel() const { return mLabel; }

	/** @brief returns a const ref to this directive's label opacity value */
	const float& getLabelOpacity() const { return mLabelOpacity.value(); }

	/** @brief update method */
	void update() { /* no-op */ }

	/** @brief draw method */
	void draw()
	{
		// Flightcheck routine:
		if (!mLabelTexture || getLabelOpacity() < 1e-3) { return; }
		// Get window dimension:
		ci::vec2 tWindowDim = static_cast<ci::vec2>(ci::app::getWindowSize());
		// Get texture dimension:
		ci::vec2 tLabelDim  = static_cast<ci::vec2>(mLabelTexture->getSize());
		// Compute screen rect:
		ci::vec2  tScreenPos  = (tWindowDim - tLabelDim) * mLabelPosition.value();
		ci::Rectf tScreenRect = ci::Rectf(tScreenPos, tScreenPos + tLabelDim);
		// Draw label: 
		ci::gl::color(1.0f, 1.0f, 1.0f, getLabelOpacity());
		ci::gl::draw( mLabelTexture, tScreenRect );
	}
};
