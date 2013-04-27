////////////////////////////////////////////////////////////////
// vertex shader global variables
float4x4 local_to_projection;
float4x4 local_to_world;
//float4x4 world_to_local;

float4 eye_position;
float4 light0_position;

////////////////////////////////////////////////////////////////
// vertex shader
struct VS_INPUT {
	float4	position : POSITION;
	float3	normal	 : NORMAL;
	float4	tangent	 : TANGENT;
	float2  uv	 : TEXCOORD0;
};

struct VS_OUTPUT {
    	float4 position	: POSITION;
	float2 uv	: TEXCOORD0;
	float3 normal	: TEXCOORD1;
	float3 eye	: TEXCOORD2;
	float3 light    : TEXCOORD3;
};

VS_OUTPUT vs_main( const VS_INPUT i )
{
    	VS_OUTPUT o = (VS_OUTPUT)0;        
	o.position	= mul( i.position, local_to_projection );
	o.uv		= i.uv;
	o.normal	= normalize( mul( i.normal, local_to_world ) );
	
	float4 vertex_world = normalize( mul( i.position, local_to_world ) );

	o.eye		= normalize( eye_position.xyz	- vertex_world.xyz );
	o.light		= normalize( light0_position.xyz - vertex_world.xyz ); 
	
	return o;
}

////////////////////////////////////////////////////////////////
// pixel shader

struct PS_INPUT {
	float2 uv	: TEXCOORD0;
	float3 normal   : TEXCOORD1;
	float3 eye	: TEXCOORD2;
	float3 light    : TEXCOORD3;
};

sampler s_decal:	register(s0);

float4	ps_main(const PS_INPUT i) : COLOR
{
	//return tex2D( s_decal, i.uv ) * ( dot( normalize( i.light ), normalize( i.normal ) ) + 0.5 ) ;
	//return float4(1,1,1,1) * ( dot( normalize( i.light ), normalize( i.normal ) ) ) ;
	return float4(1,1,1,1) * ( dot( normalize( i.light ), normalize( i.normal ) ) ) ;
}

// -------------------------------------------------------------
// テクニック
// -------------------------------------------------------------
technique TShader
{
	pass P0
	{
	        VertexShader = compile vs_1_1 vs_main();
	        PixelShader  = compile ps_2_0 ps_main();
	}
}
