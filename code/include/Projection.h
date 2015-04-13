#pragma once

#include <iostream>
#include <string>

#include "cinder/gl/gl.h"
#include "cinder/Camera.h"
#include "cinder/gl/GlslProg.h"

using namespace ci;
using namespace std;

namespace itp {
	class Projection
	{
	private:
		// VIEW
		CameraPersp pCam;
		CameraOrtho oCam;
		vec2 screenSize;

	public:
		Projection();
		Projection(CameraPersp &_pCam, vec2 _screenSize);
		Projection(CameraOrtho &_oCam, vec2 _screenSize);
		~Projection();

		//write getter for private VIEW members

		vec2 worldToScreen(const vec3 &worldCoord);
		vec2 worldToScreen(const vec3 &worldCoord, CameraPersp &_pCam, vec2 _screenSize);
		vec2 worldToScreen(const vec3 &worldCoord, CameraOrtho &_oCam, vec2 _screenSize);
	};

}