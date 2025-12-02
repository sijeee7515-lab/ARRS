Shader "Custom/PointCloudRGB"
{
    Properties
    {
        _PointSize("Point Size", Float) = 2.0
    }

    SubShader
    {
        Tags { "RenderType"="Opaque" "Queue"="Geometry" }
        LOD 100

        Pass
        {
            CGPROGRAM
            #pragma vertex vert
            #pragma fragment frag
            #pragma multi_compile_fog

            #include "UnityCG.cginc"

            uniform float _PointSize;

            struct appdata
            {
                float4 vertex : POSITION;
                fixed4 color  : COLOR;
            };

            struct v2f
            {
                float4 pos : SV_POSITION;
                fixed4 color : COLOR;
                UNITY_FOG_COORDS(1)
            };

            v2f vert (appdata v)
            {
                v2f o;
                o.pos = UnityObjectToClipPos(v.vertex);

                // Set POINT SIZE (only works on platforms that support PSIZE)
                #if defined(SHADER_API_D3D11) || defined(SHADER_API_GLCORE) || defined(SHADER_API_METAL)
                o.pos.w = max(o.pos.w, 0.0001);
                o.pos.xy += float2(0.0, 0.0);
                #endif

                UNITY_TRANSFER_FOG(o,o.pos);

                o.color = v.color;
                return o;
            }

            fixed4 frag (v2f i) : SV_Target
            {
                UNITY_APPLY_FOG(i.fogCoord, i.color);
                return i.color;
            }
            ENDCG
        }
    }
}
