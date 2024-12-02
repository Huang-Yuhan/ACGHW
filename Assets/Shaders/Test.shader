Shader "Custom/Test"
{
    SubShader
    {
        
        Pass
        {
            
            
            CGPROGRAM
            #pragma vertex vert
            #pragma fragment frag

            #include "UnityCG.cginc"
            #define UNITY_INDIRECT_DRAW_ARGS IndirectDrawIndexedArgs
            #include "UnityIndirect.cginc"

            struct v2f
            {
                float4 pos : SV_POSITION;
                float4 color : COLOR0;
            };

            struct PositionData
            {
                float3 position;
                float4 color;
                float3 velocity;
            };

            StructuredBuffer<PositionData> _PositionBuffer;
            
            v2f vert(appdata_base v, uint svInstanceID : SV_InstanceID)
            {
                InitIndirectDrawArgs(0);
                v2f o;
                uint cmdID = GetCommandID(0);
                uint instanceID = GetIndirectInstanceID(svInstanceID);
                float3 center = _PositionBuffer[instanceID].position;
                float4 wpos = float4(v.vertex.xyz + center, 1);
                o.pos = mul(UNITY_MATRIX_VP, wpos);
                o.color = _PositionBuffer[instanceID].color;
                return o;
            }

            float4 frag(v2f i) : SV_Target
            {
                return i.color;
            }
            ENDCG
        }
    }
}
