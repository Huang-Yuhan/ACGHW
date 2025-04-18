Shader "Custom/ParticleInstanceShader"
{
    SubShader
    {
        
        Pass
        {
            
            
            CGPROGRAM
            #pragma enable_d3d11_debug_symbols
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



            StructuredBuffer<float3> _PositionBuffer;
            float _ParticleRadius;
            float4x4 _ObjectToWorld;
            
            v2f vert(appdata_base v, uint svInstanceID : SV_InstanceID)
            {
                InitIndirectDrawArgs(0);
                v2f o;
                uint cmdID = GetCommandID(0);
                uint instanceID = GetIndirectInstanceID(svInstanceID);
                float3 center = _PositionBuffer[instanceID];
                //将center从局部坐标转换到世界坐标
                center= mul(_ObjectToWorld,float4(center,1)).xyz;
                float4 wpos = float4(v.vertex.xyz*_ParticleRadius*2 + center, 1);
                o.pos = UnityObjectToClipPos(wpos);
                o.color = float4(1, 1, 1, 1);
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
