/* ITP Future of Storytelling */

#pragma once

#include <iostream>
#include <string>

#include "cinder/gl/GlslProg.h"

#define STRINGIFY(x) #x

namespace itp {
	
	static const std::string kGlslDefaultVert =
	STRINGIFY(
			  uniform mat4	ciModelViewProjection;
			  
			  in  vec4			ciPosition;
			  in  vec2			ciTexCoord0;
			  out vec2			vTexCoord0;
			  
			  void main( void )
			  {
				  vTexCoord0  = ciTexCoord0;
				  gl_Position = ciModelViewProjection * ciPosition;
			  }
	);
	
	static const std::string kGlslKinectAlignSilhouetteFrag =
	STRINGIFY(
			  // CONFIG:
			  
			  uniform bool		uSilhouette;
			  
			  // USER TEXTURES:
			  
			  uniform sampler2D	uTextureColor;
			  uniform sampler2D	uTextureLookup;
			  uniform sampler2D	uTextureBody;
			  
			  // SHADER VARS:
			  
			  in  vec2			vTexCoord0;
			  out vec4 			fragColor;
			  
			  // MAIN:
			  
			  void main( void )
			  {
				  // Get body alpha mask:
				  float tBodyMask = 1.0 - texture( uTextureBody, vTexCoord0 ).r;
				  // Set to silhouette, if desired:
				  if( uSilhouette ) {
					  fragColor = vec4( 1.0, 1.0, 1.0, tBodyMask );
				  }
				  else {
					  // Get depth-to-color lookup coordinate:
					  vec2 tCoordAdj = texture( uTextureLookup, vTexCoord0 ).rg;
					  // Set final color from  masked-user color pixel:
					  fragColor = vec4(texture(uTextureColor, tCoordAdj).rgb, tBodyMask);
				  }
		
				  // For debug only:
				  //fragColor = texture( uTextureBody, vTexCoord0 );
			  }
	);
	
	static inline ci::gl::GlslProgRef createKinectAlignSilhouetteShader()
	{
		ci::gl::GlslProg::Format tFormat;
		tFormat.version( 150 );
		tFormat.vertex( kGlslDefaultVert );
		tFormat.fragment( kGlslKinectAlignSilhouetteFrag );
		return ci::gl::GlslProg::create( tFormat );
	}

} // namespace itp
