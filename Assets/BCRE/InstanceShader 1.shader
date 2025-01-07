Shader "Custom/InstanceShader"
{
    Properties
    {
        _Diffuse ("Diffuse", Color) = (1,1,1,1)
        _Specular ("Specular",Color) = (1,1,1,1)
        _Gloss ("Gloss", Range(8.0,256)) = 20
        _Scale("Scale",Range(0.1,10)) = 1
    }
    SubShader
    {
        
        Pass
        {
            Tags { "LightMode"="ForwardBase"}

            
            CGPROGRAM
            #pragma vertex vert
            #pragma fragment frag

            #include "UnityCG.cginc"
            #define UNITY_INDIRECT_DRAW_ARGS IndirectDrawIndexedArgs
            #include "Lighting.cginc"
            #include "UnityIndirect.cginc"

            struct a2v{
                float4 vertex : POSITION;
                float3 normal : NORMAL;
            };
            struct v2f{
                float4 pos:SV_POSITION;
                fixed3 worldNormal:NORMAL;
                fixed3 worldPos:TEXCOORD1;
            };

            
            //ComputeShader中的粒子位置
            StructuredBuffer<float3> _ParticlePositionBuffer;
            StructuredBuffer<uint> _GranuleStateBuffer;
            int _BufferBeginIndex;

            //渲染相关参数
            fixed4 _Diffuse;
            fixed4 _Specular;
            float _Gloss;
            float _Radius;
            
            v2f vert(a2v v, uint svInstanceID : SV_InstanceID)
            {
                InitIndirectDrawArgs(0);
                v2f o;
                uint cmdID = GetCommandID(0);
                uint instanceID = GetIndirectInstanceID(svInstanceID);

                uint granuleIndex = instanceID / 4;
                uint state = _GranuleStateBuffer[granuleIndex];
                if (state == 0)
                {
                    o.pos = float4(999999, 9999999, 999999, 1);
                    o.worldNormal = float3(0, 0, 0);
                    o.worldPos = float3(0, 0, 0);
                    return o;
                }
                
                float3 center = _ParticlePositionBuffer[instanceID + _BufferBeginIndex];
                float4 vertexWolrdPosition = float4(v.vertex.xyz * _Radius*2 + center, 1);               //*2的原因是Mesh球体的半径是0.5，所以要乘以2
                o.pos = mul(UNITY_MATRIX_VP, vertexWolrdPosition);
                o.worldNormal = v.normal;
                o.worldPos = vertexWolrdPosition.xyz;
                return o;
            }

            fixed4 frag(v2f i) : SV_Target
            {
                fixed3 ambient = UNITY_LIGHTMODEL_AMBIENT.xyz;
                fixed3 worldNormal = normalize(i.worldNormal);
                fixed3 worldLightDir = normalize(_WorldSpaceLightPos0.xyz);
                fixed3 diffuse = _LightColor0.rgb * _Diffuse.rgb * saturate(dot(worldNormal,worldLightDir));
                //高光反射部分
                fixed3 viewDir = normalize(_WorldSpaceCameraPos.xyz - i.worldPos.xyz);
                fixed3 halfDir = normalize(worldLightDir + viewDir);
                fixed3 specular = _LightColor0.rgb * _Specular.rgb * pow(max(0,dot(worldNormal, halfDir)),_Gloss);
                return fixed4(ambient+diffuse+specular,1.0);
            }
            ENDCG
        }
    }
    Fallback "Specular"
}
