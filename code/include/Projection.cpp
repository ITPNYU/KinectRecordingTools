/* ITP Future of Storytelling */

#pragma once

#include <iostream>
#include <string>

#include "Projection.h"

namespace itp {
	Projection::Projection()
	{
	}

	Projection::Projection(CameraPersp &_pCam, vec2 _screenSize)
	{
		pCam = _pCam;
		screenSize = _screenSize;
	}
	Projection::Projection(CameraOrtho &_oCam, vec2 _screenSize)
	{
		oCam = _oCam;
		screenSize = _screenSize;
	}

	Projection::~Projection()
	{
	}

	// Use the default VIEW
	vec2 Projection::worldToScreen(const vec3 &worldCoord){
		return pCam.worldToScreen(worldCoord, screenSize.x, screenSize.y);
	}

	// Use a different perspective VIEW for mapping world to screen
	vec2 Projection::worldToScreen(const vec3 &worldCoord, CameraPersp &_pCam, vec2 _screenSize){
		pCam = _pCam;
		screenSize = _screenSize;
		return pCam.worldToScreen(worldCoord, screenSize.x, screenSize.y);
	}

	// Use a different orthogonal VIEW for mapping world to screen
	vec2 Projection::worldToScreen(const vec3 &worldCoord, CameraOrtho &_oCam, vec2 _screenSize){
		oCam = _oCam;
		screenSize = _screenSize;
		return oCam.worldToScreen(worldCoord, screenSize.x, screenSize.y);
	}

}