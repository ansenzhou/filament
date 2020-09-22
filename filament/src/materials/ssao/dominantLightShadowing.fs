/*
 * Dominant Light Shadowing
 * see: "Lighting Technology of The Last of Us Part II" by Hawar Doghramachi, Naughty Dog, LLC
 */

#include "ssaoUtils.fs"

struct ConeTraceSetup {
    // runtime parameters
    vec2 startPosSS;
    vec3 startPosVS;
    vec3 normalVS;
    mat4 viewToScreenMat;
    vec2 jitterOffset;          // (x = direction offset, y = step offset)
    float depthParams;
    vec3 coneTraceDirVS;

    // artistic/quality parameters
    vec4 coneTraceParams;       // (x = slope, y = start trace distance, z = inverse max contact distance, w = intensity)
    float invZoom;
    float depthBias;
    float slopeScaledDepthBias;
    uint sampleCount;
};

float GetInvW(vec3 startPosVS, mat4 viewToScreenMat) {
    return 1.0 / (viewToScreenMat * vec4(startPosVS, 1.0)).w;
}

vec2 GetExtScreenTC(vec3 endPosVS, mat4 viewToScreenMat, out float endInvW) {
    vec4 p = viewToScreenMat * vec4(endPosVS, 1.0);
    endInvW = 1.0 / p.w;
    return p.xy * endInvW;
}

float coneTraceOcclusion(in ConeTraceSetup setup, const sampler2D depthTexture) {
    // skip fragments that are back-facing trace direction
    // (avoid overshadowing of translucent surfaces)
    float nDotD = dot(setup.normalVS, setup.coneTraceDirVS);
    if (nDotD < 0.0f) {
        return 0.0f;
    }

    // start position of cone trace
    vec2 startPosSS = setup.startPosSS;
    vec3 startPosVS = setup.startPosVS;
    float startInvW = GetInvW(startPosVS, setup.viewToScreenMat);

    // end position of cone trace
    vec3 endPosVS = setup.coneTraceDirVS * 0.1f + startPosVS;
    float endInvW;
    vec2 endPosSS = GetExtScreenTC(endPosVS, setup.viewToScreenMat, endInvW);

    // cone trace direction in screen-space
    vec2 coneDir = endPosSS - startPosSS;
    float invConeDirLength = 1.0f / length(coneDir);
    coneDir *= invConeDirLength;

    // direction perpendicular to cone trace direction
    vec2 perpConeDir = float2(coneDir.y, -coneDir.x);

    // avoid self-occlusion and reduce banding artifacts by normal variation
    vec3 viewVecVS = normalize(startPosVS);
    float minTraceDistance = (1.0f - abs(dot(setup.normalVS, viewVecVS))) * 0.005f;

    // init trace distance and sample radius
    float invLinearDepth = 1.0f / -setup.startPosVS.z;
    float traceDistance = max(setup.coneTraceParams.y, minTraceDistance) * invLinearDepth;
    float sampleRadiusSS = setup.coneTraceParams.x * traceDistance;
    float endRadiusSS = setup.coneTraceParams.x / invConeDirLength;
    float endRadiusVS = endRadiusSS * setup.invZoom * invLinearDepth / endInvW;

    // calculate depth bias
    float depthBias = saturate(1.0f - nDotD) * setup.slopeScaledDepthBias + setup.depthBias;

    float occlusion = 0.0f;
    for (uint i = 0; i < setup.sampleCount; i++) {
        // step along cone in screen space
        float newSampleRadiusSS = sampleRadiusSS * (sampleRadiusSS + traceDistance) / (traceDistance - sampleRadiusSS);
        float stepDistance = sampleRadiusSS + newSampleRadiusSS;
        sampleRadiusSS = newSampleRadiusSS;

        // apply jitter offset
        float jitterDistance = stepDistance * setup.jitterOffset.y;
        float jitteredTracedDistance = traceDistance + jitterDistance;
        float jitteredSampleRadius = setup.jitterOffset.x * sampleRadiusSS * jitterDistance / stepDistance;

        // sample depth buffer
        vec2 samplePosSS = perpConeDir * jitteredSampleRadius + coneDir * jitteredTracedDistance + startPosSS;
        float bufferDepthValue = -sampleDepthLinear(depthTexture, samplePosSS, 0.0, setup.depthParams);

        // calculate depth of cone center
        float ratio = jitteredTracedDistance * invConeDirLength;
        float lerpedW = 1.0f / mix(startInvW, endInvW, ratio);
        float sampleDepth = lerpedW;

        // calculate depth range of cone slice
        float sampleRadiusVS = endRadiusVS * ratio * lerpedW;
        float jitteredSampleRadiusVS = sampleRadiusVS * setup.jitterOffset.x;
        float sliceHalfRangeVS = sqrt(sampleRadiusVS * sampleRadiusVS + jitteredSampleRadiusVS * jitteredSampleRadiusVS);
        float sampleDepthMin = sampleDepth - sliceHalfRangeVS;
        float sampleDepthMax = sampleDepth + sliceHalfRangeVS;

        // calculate overlap of depth buffer height-field with trace cone
        float depthDifference = sampleDepthMax - bufferDepthValue;
        float overlap = saturate((depthDifference - depthBias) / (sampleDepthMax - sampleDepthMin));

        // attenuate by distance to avoid false occlusion
        float attenuation = saturate(1.0f - (depthDifference * setup.coneTraceParams.z));
        occlusion = max(occlusion, overlap * attenuation);

        if (occlusion >= 1.0f) {
            break;
        }

        traceDistance += stepDistance;
    }

    return occlusion * setup.coneTraceParams.w;
}
