Shader "PointCloud/PointCloud"
{
    Properties
    {
        [Header(Point Settings)]
        _PointSize("Point Size", Range(1, 100)) = 10.0
        _MinPointSize("Min Point Size", Range(0.5, 10)) = 1.0
        _MaxPointSize("Max Point Size", Range(10, 200)) = 60.0

        [Header(Color Settings)]
        _BaseColor("Base Color", Color) = (0.8, 0.85, 1.0, 1.0)
        _TopColor("Top Color", Color) = (1.0, 0.9, 0.7, 1.0)
        _BottomColor("Bottom Color", Color) = (0.2, 0.35, 0.6, 1.0)
        _HeightMin("Height Min", Float) = -1.0
        _HeightMax("Height Max", Float) = 1.0

        [Header(Shading)]
        [Toggle] _UseHeightGradient("Use Height Gradient", Float) = 1
        _AmbientLight("Ambient Light", Range(0, 1)) = 0.4
    }

    SubShader
    {
        Tags
        {
            "RenderType" = "Opaque"
            "RenderPipeline" = "UniversalPipeline"
            "Queue" = "Geometry"
        }

        Pass
        {
            Name "PointCloud"
            Cull Off

            HLSLPROGRAM
            #pragma vertex vert
            #pragma fragment frag

            #include "Packages/com.unity.render-pipelines.universal/ShaderLibrary/Core.hlsl"

            CBUFFER_START(UnityPerMaterial)
                float  _PointSize;
                float  _MinPointSize;
                float  _MaxPointSize;
                float4 _BaseColor;
                float4 _TopColor;
                float4 _BottomColor;
                float  _HeightMin;
                float  _HeightMax;
                float  _UseHeightGradient;
                float  _AmbientLight;
            CBUFFER_END

            struct Attributes
            {
                float4 positionOS : POSITION;
            };

            struct Varyings
            {
                float4 positionCS : SV_POSITION;
                float4 color      : COLOR;
                float  psize      : PSIZE;  // 点大小（DirectX / OpenGL ES）
            };

            Varyings vert(Attributes v)
            {
                Varyings o;

                float3 worldPos = TransformObjectToWorld(v.positionOS.xyz);
                o.positionCS = TransformWorldToHClip(worldPos);

                // 距离自适应点大小
                float dist = length(GetCameraPositionWS() - worldPos);
                o.psize = clamp(_PointSize * 5.0 / max(dist, 0.001), _MinPointSize, _MaxPointSize);

                // 高度渐变色
                if (_UseHeightGradient > 0.5)
                {
                    float h = saturate((worldPos.y - _HeightMin) / max(_HeightMax - _HeightMin, 0.001));
                    o.color = lerp(_BottomColor, _TopColor, h);
                }
                else
                {
                    o.color = _BaseColor;
                }

                // 简单环境光衰减（模拟朝相机方向变亮）
                o.color.rgb *= _AmbientLight + (1.0 - _AmbientLight);

                return o;
            }

            float4 frag(Varyings i) : SV_Target
            {
                return i.color;
            }
            ENDHLSL
        }
    }

    FallBack "Hidden/Universal Render Pipeline/FallbackError"
}