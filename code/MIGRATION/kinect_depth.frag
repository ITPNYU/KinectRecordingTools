#version 150

// CONFIG:

uniform bool		uGrayscale;
uniform bool		uSilhouette;

// USER TEXTURES:

uniform sampler2D	uTextureColor;
uniform sampler2D	uTextureDepth;
uniform sampler2D	uTextureLookup;
uniform sampler2D   uTextureBody;

// SHADER VARS:

in vec2				vTexCoord0;
out vec4 			fragColor;

// HELPERS:

vec4 color_to_grayscale(vec4 val)
{
	return vec4( vec3( dot( val.rgb, vec3( 0.299, 0.587, 0.114 ) ) ), val.a );
}

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
		// Get masked-user color pixel:
		vec4 tUserColor = vec4( texture( uTextureColor, tCoordAdj ).rgb, tBodyMask );
		// Set to grayscale, if desired:
		if( uGrayscale ) {
			tUserColor = color_to_grayscale( tUserColor );
		}
		// Set final color:
		fragColor = tUserColor;
	}
	

	// TODO - for debug only:
	//fragColor = texture( uTextureBody, vTexCoord0 );
}
 