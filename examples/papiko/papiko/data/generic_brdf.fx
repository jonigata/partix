////////////////////////////////////////////////////////////////
// vertex shader global variables
float4x4 local_to_projection;
float4x4 local_to_world;
//float4x4 world_to_local;

float4 eye_position;
float4 light0_position;

float PI = 3.14159265;
float HALF_PI = 1.570796325;

#define ACOS_BIASED_TABLE

////////////////////////////////////////////////////////////////
// vertex shader
struct VS_INPUT {
	float4	position	: POSITION;
	float3	normal		: NORMAL;
	float4  tangent		: TANGENT;
	float2  uv			: TEXCOORD0;
};

struct VS_OUTPUT {
    	float4 position	: POSITION;
		float2 uv		: TEXCOORD0;
		float3 h		: TEXCOORD1;	// half vector
		float3 i		: TEXCOORD2;	// input(light) vector
		float3 o		: TEXCOORD3;	// output(eye) vector
		float3 normal	: TEXCOORD4;
};

VS_OUTPUT generic_brdf_vertex_shader( VS_INPUT input )
{
	VS_OUTPUT output = (VS_OUTPUT)0;

	output.position	= mul( input.position, local_to_projection );
	output.normal   = input.normal;
	output.uv	= input.uv;

	float4 world_position = mul( input.position, local_to_world );

	float3 light = light0_position.xyz;
	float3 eye   = eye_position.xyz;

	float3 i = normalize( light - world_position.xyz );
	float3 o = normalize( eye   - world_position.xyz );
	float3 h = normalize( i + o );

	output.h = h;
	output.i = i;
	output.o = o;

	return output;
}

////////////////////////////////////////////////////////////////
// pixel shader

sampler s_decal:	register(s0);
sampler s_brdf:		register(s1);
sampler s_smap:		register(s2);

struct PS_INPUT {
	float2 uv	: TEXCOORD0;
	float3 h	: TEXCOORD1;
	float3 i	: TEXCOORD2;
	float3 o	: TEXCOORD3;
	float3 normal	: TEXCOORD4;
};

struct PS_OUTPUT {
	float4 color	: COLOR;
};

float4 xyz1( float4 v )
{
	return float4( v.xyz, 1 );
}

float4 xyz0( float4 v )
{
	return float4( v.xyz, 0 );
}

PS_OUTPUT generic_brdf_pixel_shader( PS_INPUT input )
{
	PS_OUTPUT o = (PS_OUTPUT)0;

	// BRDF
	float3 n = normalize( input.normal );

#ifdef ACOS_BIASED_TABLE
	float x_i = dot( n, normalize( input.i ) );
	float x_o = dot( n, normalize( input.o ) );
	float x_h = dot( n, normalize( input.h ) );
#else 
	float x_i = acos( dot( n, normalize( input.i ) ) ) / HALF_PI;
	float x_o = acos( dot( n, normalize( input.o ) ) ) / HALF_PI;
	float x_h = acos( dot( n, normalize( input.h ) ) ) / HALF_PI;
#endif

	float4 s = xyz1( tex2D( s_smap, input.uv ) );
	float4 t = xyz1( tex2D( s_decal, input.uv ) );

	float4 diffuse  = xyz1( tex2D( s_brdf, float2( x_i, 0.125 ) ) );
	float4 specular = xyz0( tex2D( s_brdf, float2( x_h, 0.375 ) ) * 2 ) ;
	float4 fresnel  = xyz0( tex2D( s_brdf, float2( x_o, 0.625 ) ) );

	//specular += glow_factor * (1-x_i) * (1-x_i);

	o.color = saturate( t * diffuse + s * specular + fresnel );
	//o.color = fresnel;
	//o.color = diffuse;
	//o.color = diffuse;
	//o.color = float4( 1, 1, 1, 1 );

	return o;
}

// -------------------------------------------------------------
// テクニック
// -------------------------------------------------------------
technique TShader
{
	pass P0
	{
	        VertexShader = compile vs_2_0 generic_brdf_vertex_shader();
	        PixelShader  = compile ps_2_0 generic_brdf_pixel_shader();
	}
}
