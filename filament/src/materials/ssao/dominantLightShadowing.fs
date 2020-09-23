/*
 * Largely based on "Dominant Light Shadowing"
 * "Lighting Technology of The Last of Us Part II" by Hawar Doghramachi, Naughty Dog, LLC
 */

#include "ssaoUtils.fs"

struct ConeTraceSetup {
    // runtime parameters
    highp vec2 ssStartPos;
    highp vec3 vsStartPos;
    vec3 vsNormal;
    highp mat4 viewToScreenMat;
    vec2 jitterOffset;          // (x = direction offset, y = step offset)
    highp float depthParams;
    vec3 vsConeDirection;

    // artistic/quality parameters
    vec4 coneTraceParams;       // (x = slope, y = start trace distance, z = inverse max contact distance, w = intensity)
    float invZoom;
    float depthBias;
    float slopeScaledDepthBias;
    uint sampleCount;
};

float coneTraceOcclusion(in ConeTraceSetup setup, const sampler2D depthTexture) {
    // skip fragments that are back-facing trace direction
    // (avoid overshadowing of translucent surfaces)
    float NoL = dot(setup.vsNormal, setup.vsConeDirection);
    if (NoL < 0.0) {
        return 0.0;
    }

    // start position of cone trace
    highp vec2 ssStartPos = setup.ssStartPos;
    highp vec3 vsStartPos = setup.vsStartPos;
    highp float ssStartInvW = 1.0 / (setup.viewToScreenMat * vec4(vsStartPos, 1.0)).w;

    // end position of cone trace
    highp vec3 vsEndPos = setup.vsConeDirection * 0.1 + vsStartPos;
    highp vec4 ssEndPos = setup.viewToScreenMat * vec4(vsEndPos, 1.0);
    highp float ssEndInvW = 1.0 / ssEndPos.w;
    ssEndPos.xy *= ssEndInvW;

    // cone trace direction in screen-space
    float ssConeLength = length(ssEndPos.xy - ssStartPos);
    float ssInvConeLength = 1.0 / ssConeLength;
    vec2 ssConeDirection = (ssEndPos.xy - ssStartPos) * ssInvConeLength;

    // direction perpendicular to cone trace direction
    vec2 perpConeDir = vec2(ssConeDirection.y, -ssConeDirection.x);

    // avoid self-occlusion and reduce banding artifacts by normal variation
    vec3 vsViewVector = normalize(vsStartPos);
    float minTraceDistance = (1.0 - abs(dot(setup.vsNormal, vsViewVector))) * 0.005;

    // init trace distance and sample radius
    highp float invLinearDepth = 1.0 / -setup.vsStartPos.z;
    float ssTraceDistance = max(setup.coneTraceParams.y, minTraceDistance) * invLinearDepth;

    float ssSampleRadius = setup.coneTraceParams.x * ssTraceDistance;
    float ssEndRadius    = setup.coneTraceParams.x * ssConeLength;
    float vsEndRadius    = ssEndRadius * setup.invZoom * invLinearDepth * ssEndPos.w;

    // calculate depth bias
    float vsDepthBias = saturate(1.0 - NoL) * setup.slopeScaledDepthBias + setup.depthBias;

    float occlusion = 0.0;
    for (uint i = 0u; i < setup.sampleCount; i++) {
        // step along cone in screen space
        float ssNewSampleRadius = ssSampleRadius * (ssSampleRadius + ssTraceDistance) / (ssTraceDistance - ssSampleRadius);
        float ssStepDistance    = ssSampleRadius + ssNewSampleRadius;
        ssSampleRadius = ssNewSampleRadius;

        // apply jitter offset
        float ssJitterDistance = ssStepDistance * setup.jitterOffset.y;
        float ssJitteredTracedDistance = ssTraceDistance + ssJitterDistance;
        float ssJitteredSampleRadius = setup.jitterOffset.x * ssSampleRadius * ssJitterDistance / ssStepDistance;

        // sample depth buffer
        highp vec2 ssSamplePos = perpConeDir * ssJitteredSampleRadius + ssConeDirection * ssJitteredTracedDistance + ssStartPos;
        highp float vsLinearDepth = -sampleDepthLinear(depthTexture, ssSamplePos, 0.0, setup.depthParams);

        // calculate depth of cone center
        float ratio = ssJitteredTracedDistance * ssInvConeLength;
        highp float vsLinearSampleDepth = 1.0 / mix(ssStartInvW, ssEndInvW, ratio);

        // calculate depth range of cone slice
        float vsSampleRadius = vsEndRadius * ratio * vsLinearSampleDepth;
        float vsJitteredSampleRadius = vsSampleRadius * setup.jitterOffset.x;
        float vsSliceHalfRange = sqrt(vsSampleRadius * vsSampleRadius + vsJitteredSampleRadius * vsJitteredSampleRadius);
        float vsSampleDepthMin = vsLinearSampleDepth - vsSliceHalfRange;
        float vsSampleDepthMax = vsLinearSampleDepth + vsSliceHalfRange;

        // calculate overlap of depth buffer height-field with trace cone
        float vsDepthDifference = vsSampleDepthMax - vsLinearDepth;
        float overlap = saturate((vsDepthDifference - vsDepthBias) / (vsSampleDepthMax - vsSampleDepthMin));

        // attenuate by distance to avoid false occlusion
        float attenuation = saturate(1.0 - (vsDepthDifference * setup.coneTraceParams.z));
        occlusion = max(occlusion, overlap * attenuation);

        if (occlusion >= 1.0) {
            break;
        }

        ssTraceDistance += ssStepDistance;
    }

    return occlusion * setup.coneTraceParams.w;
}
