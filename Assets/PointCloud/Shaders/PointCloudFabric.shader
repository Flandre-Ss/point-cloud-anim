Shader "PointCloud/Fabric"
{
    Properties
    {
        [Header(Point Settings)]
        _PointSize    ("Point Size",     Range(1, 100))  = 12.0
        _MinPointSize ("Min Point Size", Range(0.5, 10)) = 1.0
        _MaxPointSize ("Max Point Size", Range(10, 200)) = 80.0

        [Header(Fabric Color)]
        _FabricColor  ("Fabric Color",  Color) = (0.82, 0.78, 0.74, 1.0)
        _SheenColor   ("Sheen Color",   Color) = (1.0,  0.97, 0.93, 1.0)

        [Header(Fabric BRDF)]
        // SheenRoughness: 小→丝绸锐利高光  大→棉布/毛料漫反射高光
        _SheenRoughness ("Sheen Roughness", Range(0.05, 1.0)) = 0.35
        _SheenStrength  ("Sheen Strength",  Range(0.0,  1.0)) = 0.55
        // WrapAmount: 控制暗部补光（模拟薄布料透光）0=标准  0.5=半透薄纱
        _WrapAmount     ("Wrap Lighting",   Range(0.0,  0.6)) = 0.25

        [Header(Normal Reconstruction)]
        // BodyAxis: 角色中轴的世界坐标 XZ（填入 GameObject 的 X/Z 位置）
        // 每个点的法线 = 从该轴向外辐射，产生每点不同的光照
        _BodyAxisCenter ("Body Axis Center (XZ)", Vector) = (0.0, 0.0, 0.0, 0.0)
        // BodyAxisBlend: 0=纯圆柱法线  1=完全朝摄像机（billboard）
        _BodyAxisBlend  ("Camera Blend",  Range(0.0, 1.0)) = 0.15
        // NormalNoise: 给法线叠加位置噪声，模拟布料凹凸微扰
        _NormalNoise    ("Normal Noise",  Range(0.0, 0.4)) = 0.08

        [Header(Lighting)]
        _Ambient        ("Ambient",          Range(0.0, 1.0)) = 0.28
        // 当场景无主光时的 fallback 方向（归一化 XYZ）
        _FallbackLightDir ("Fallback Light Dir", Vector) = (0.5, 0.8, 0.3, 0)
    }

    SubShader
    {
        Tags
        {
            "RenderType"     = "Opaque"
            "RenderPipeline" = "UniversalPipeline"
            "Queue"          = "Geometry"
        }

        Pass
        {
            Name "FabricPointCloud"
            Cull Off

            HLSLPROGRAM
            #pragma vertex   vert
            #pragma fragment frag

            // 启用 URP 主光源访问
            #pragma multi_compile _ _MAIN_LIGHT_SHADOWS
            #pragma multi_compile _ _ADDITIONAL_LIGHTS_VERTEX _ADDITIONAL_LIGHTS

            #include "Packages/com.unity.render-pipelines.universal/ShaderLibrary/Core.hlsl"
            #include "Packages/com.unity.render-pipelines.universal/ShaderLibrary/Lighting.hlsl"

            CBUFFER_START(UnityPerMaterial)
                float  _PointSize;
                float  _MinPointSize;
                float  _MaxPointSize;
                float4 _FabricColor;
                float4 _SheenColor;
                float  _SheenRoughness;
                float  _SheenStrength;
                float  _WrapAmount;
                float4 _BodyAxisCenter;
                float  _BodyAxisBlend;
                float  _NormalNoise;
                float  _Ambient;
                float4 _FallbackLightDir;
            CBUFFER_END

            // ─────────────────────────────────────────
            // Charlie Sheen BRDF（布料工业标准模型）
            // 来源：Estevez & Kulla 2017 "Production Friendly Microfacet Sheen BRDF"
            // ─────────────────────────────────────────

            // Charlie 分布函数 D：sin²θ 的幂次分布，粗糙度越大越漫
            float CharlieD(float roughness, float NdotH)
            {
                float invR  = 1.0 / max(roughness, 1e-4);
                float sin2h = max(1.0 - NdotH * NdotH, 0.0078125);
                return (2.0 + invR) * pow(sin2h, invR * 0.5) / (2.0 * PI);
            }

            // Charlie 几何遮蔽 V（近似解析式，避免数值积分）
            float CharlieV(float NdotL, float NdotV)
            {
                return 1.0 / (4.0 * (NdotL + NdotV - NdotL * NdotV) + 1e-4);
            }

            // 完整布料 BRDF：Lambert 漫反射 + Charlie Sheen 高光
            float3 FabricBRDF(float3 N, float3 V, float3 L,
                              float3 baseColor, float3 sheenColor,
                              float  sheenRoughness, float sheenStrength)
            {
                float3 H    = normalize(V + L);
                float NdotL = saturate(dot(N, L));
                float NdotV = abs(dot(N, V)) + 1e-5;
                float NdotH = saturate(dot(N, H));

                // Lambertian 漫反射
                float3 diffuse = baseColor * (1.0 / PI);

                // Charlie Sheen 高光
                float  D    = CharlieD(sheenRoughness, NdotH);
                float  Vis  = CharlieV(NdotL, NdotV);
                float3 spec = sheenColor * (D * Vis) * sheenStrength;

                return (diffuse + spec) * NdotL;
            }

            // ─────────────────────────────────────────
            // 顶点/片元结构
            // ─────────────────────────────────────────

            struct Attributes
            {
                float4 positionOS : POSITION;
            };

            struct Varyings
            {
                float4 positionCS : SV_POSITION;
                float3 worldPos   : TEXCOORD0;
                float  psize      : PSIZE;
            };

            Varyings vert(Attributes v)
            {
                Varyings o;

                float3 worldPos   = TransformObjectToWorld(v.positionOS.xyz);
                o.positionCS      = TransformWorldToHClip(worldPos);
                o.worldPos        = worldPos;

                // 距离自适应点大小
                float dist = length(GetCameraPositionWS() - worldPos);
                o.psize = clamp(_PointSize * 5.0 / max(dist, 0.001),
                                _MinPointSize, _MaxPointSize);

                return o;
            }

            // ─────────────────────────────────────────
            // 位置哈希：基于世界坐标生成伪随机扰动，模拟布料微观凹凸
            // ─────────────────────────────────────────
            float3 PositionHash(float3 p)
            {
                p = frac(p * float3(127.1, 311.7, 74.7));
                p += dot(p, p.yzx + 19.19);
                return frac((p.xxy + p.yxx) * p.zyx) * 2.0 - 1.0;
            }

            float4 frag(Varyings i) : SV_Target
            {
                float3 worldPos = i.worldPos;
                float3 camPos   = GetCameraPositionWS();
                float3 V        = normalize(camPos - worldPos);

                // ── 圆柱法线重建 ──────────────────────────
                // 从角色中轴向每个点辐射，不同点得到不同法线方向
                float2 radial2D = worldPos.xz - _BodyAxisCenter.xz;
                // 避免点恰好落在轴上（极少数情况）
                float radialLen = max(length(radial2D), 0.001);
                // 圆柱面法线：水平辐射方向，Y 分量为 0
                float3 N = float3(radial2D.x / radialLen, 0.0, radial2D.y / radialLen);

                // 叠加位置噪声，模拟布料褶皱/纤维起伏
                N = normalize(N + PositionHash(worldPos * 8.0) * _NormalNoise);

                // 少量偏向摄像机，确保朝相机侧的点法线不会完全"侧过去"
                N = normalize(lerp(N, V, _BodyAxisBlend));

                // ── 获取主光源 ────────────────────────────
                Light mainLight = GetMainLight();
                float3 L = normalize(mainLight.direction);
                // 主光强度为零时（无主光）使用 fallback 方向
                float lightIntensity = dot(mainLight.color, float3(0.299, 0.587, 0.114));
                if (lightIntensity < 0.001)
                    L = normalize(_FallbackLightDir.xyz);

                float3 lightColor = mainLight.color + float3(0.001, 0.001, 0.001);

                // ── Wrap Lighting（布料透光近似）────────────
                // 将 NdotL 从 [-W, 1] 重映射到 [0, 1]，暗部不全黑
                float NdotL_raw  = dot(N, L);
                float NdotL_wrap = saturate((NdotL_raw + _WrapAmount) / (1.0 + _WrapAmount));

                // ── 布料 BRDF ────────────────────────────
                // 用 wrap 后的 L' 方向参与 BRDF（保持能量守恒近似）
                float3 L_wrap = normalize(L + N * _WrapAmount);
                float3 brdf   = FabricBRDF(N, V, L_wrap,
                                           _FabricColor.rgb, _SheenColor.rgb,
                                           _SheenRoughness, _SheenStrength);

                float3 color = brdf * lightColor * (1.0 + _WrapAmount);

                // ── 环境光 ───────────────────────────────
                color += _FabricColor.rgb * _Ambient;

                // ── Fresnel 边缘辉光（菲涅尔 Sheen）─────────
                // 布料在掠射角呈现"绒面散射辉光"
                float NdotV    = saturate(dot(N, V));
                float fresnel  = pow(1.0 - NdotV, 3.0);
                color += _SheenColor.rgb * fresnel * _SheenStrength * 0.5;

                return float4(color, 1.0);
            }
            ENDHLSL
        }
    }

    FallBack "Hidden/Universal Render Pipeline/FallbackError"
}