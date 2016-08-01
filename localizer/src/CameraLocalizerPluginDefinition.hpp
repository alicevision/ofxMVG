#pragma once
#include <vector>
#include <string>

/**
 * Plugin Parameters definition
 */

//Clip
#define kClip(I) std::to_string(I + 1)


//Main Parameters
#define kParamGroupMain "groupMain"

#define kParamOutputIndex "outputIndex"
#define kParamFeaturesType "featuresType"
#define kParamFeaturesPreset "featuresPreset"
#define kParamReconstructionFile "reconstructionFile"
#define kParamDescriptorsFolder "descriptorsFolder"
#define kParamVoctreeFile "voctreeFile"
#define kParamRigMode "rigMode"
#define kParamRigCalibrationFile "rigCalibrationFile"


//Input Parameters
#define kParamGroupInput(I) "groupInput_" + std::to_string(I)

#define kParamInputIsGrayscale(I) "inputIsGrayscale_" + std::to_string(I)
#define kParamInputLensCalibrationFile(I) "inputLensCalibrationFile_" + std::to_string(I)

#define kParamInputGroupLensCalibration(I) "groupInputLensCalibration_" + std::to_string(I)

#define kParamInputSensorWidth(I) "inputSensorWidth_" + std::to_string(I)
#define kParamInputOpticalCenter(I) "inputOpticalCenter_" + std::to_string(I)
#define kParamInputFocalLengthMode(I) "inputFocalLengthMode_" + std::to_string(I)
#define kParamInputFocalLength(I) "inputFocalLength_" + std::to_string(I)
#define kParamInputFocalLengthVarying(I) "inputFocalLengthVarying_" + std::to_string(I)
#define kParamInputDistortion(I) "inputDistortion_" + std::to_string(I)
#define kParamInputDistortionMode(I) "inputDistortionMode_" + std::to_string(I)
#define kParamInputDistortionCoef1(I) "inputDistortionCoef1_" + std::to_string(I)
#define kParamInputDistortionCoef2(I) "inputDistortionCoef2_" + std::to_string(I)
#define kParamInputDistortionCoef3(I) "inputDistortionCoef3_" + std::to_string(I)
#define kParamInputDistortionCoef4(I) "inputDistortionCoef4_" + std::to_string(I)

#define kParamInputGroupRelativePose(I) "groupInputRelativePose_" + std::to_string(I)

#define kParamInputRelativePoseRotateM1(I) "inputRelativePoseRotateM1_" + std::to_string(I)
#define kParamInputRelativePoseRotateM2(I) "inputRelativePoseRotateM2_" + std::to_string(I)
#define kParamInputRelativePoseRotateM3(I) "inputRelativePoseRotateM3_" + std::to_string(I)
#define kParamInputRelativePoseCenter(I) "inputRelativePoseCenter_" + std::to_string(I)


//Advanced Parameters
#define kParamGroupAdvanced "groupAdvanced"

#define kParamOverlay "overlay"
#define kParamAdvancedAlgorithm "advancedAlgorithm"
#define kParamAdvancedReprojectionError "advancedReprojectionError"
#define kParamAdvancedNbImageMatch "advancedNbImageMatch"
#define kParamAdvancedMaxResults "advancedMaxResults"
#define kParamAdvancedVoctreeWeights "advancedVoctreeWeights"
#define kParamAdvancedMatchingError "advancedMatchingError"
#define kParamAdvancedCctagNbNearestKeyFrames "advancedCctagNbNearestKeyFrames"
#define kParamAdvancedBaMinPointVisibility "advancedBaMinPointVisibility"
#define kParamAdvancedDistanceRatio "advancedDistanceRatio"
#define kParamAdvancedUseGuidedMatching "advancedUseGuidedMatching"
#define kParamAdvancedDebugFolder "advancedDebugFolder"
#define kParamAdvancedDebugAlwaysComputeFrame "advancedDebugAlwaysComputeFrame"

//Tracking Parameters
#define kParamGroupTracking "groupTracking"

#define kParamTrackingTrack "trackingTrack"
#define kParamTrackingRangeMode "trackingRangeMode"
#define kParamTrackingRangeMin "trackingRangeMin"
#define kParamTrackingRangeMax "trackingRangeMax"


//Output Parameters
#define kParamGroupOutput "groupOutput"

#define kParamGroupOutputCamera(I) "groupOutputCamera_" + std::to_string(I)

#define kParamOutputTranslate(I) "outputTranslate_" + std::to_string(I)
#define kParamOutputRotate(I) "outputRotate_" + std::to_string(I)
#define kParamOutputScale(I) "outputScale_" + std::to_string(I)
#define kParamOutputOpticalCenter(I) "outputOpticalCenter_" + std::to_string(I)
#define kParamOutputFocalLength(I) "outputFocalLength_" + std::to_string(I)
#define kParamOutputNear(I) "outputNear_" + std::to_string(I)
#define kParamOutputFar(I) "outputFar_" + std::to_string(I)
#define kParamOutputDistortionCoef1(I) "outputDistortionCoef1_" + std::to_string(I)
#define kParamOutputDistortionCoef2(I) "outputDistortionCoef2_" + std::to_string(I)
#define kParamOutputDistortionCoef3(I) "outputDistortionCoef3_" + std::to_string(I)
#define kParamOutputDistortionCoef4(I) "outputDistortionCoef4_" + std::to_string(I)
#define kParamOutputCreateCamera(I) "outputCreateCamera_" + std::to_string(I)

#define kParamOutputStatGroup(I) "groupOutputStat_" + std::to_string(I)

#define kParamOutputStatErrorMean(I) "outputStatErrorMean_" + std::to_string(I)
#define kParamOutputStatErrorMin(I) "outputStatErrorMin_" + std::to_string(I)
#define kParamOutputStatErrorMax(I) "outputStatErrorMax_" + std::to_string(I)
#define kParamOutputStatNbMatchedImages(I) "outputStatNbMatchedImages_" + std::to_string(I)
#define kParamOutputStatNbDetectedFeatures(I) "outputStatNbDetectedFeatures_" + std::to_string(I)
#define kParamOutputStatNbMatchedFeatures(I) "outputStatNbMatchedFeatures_" + std::to_string(I)
#define kParamOutputStatNbInlierFeatures(I) "outputStatNbInlierFeatures_" + std::to_string(I)

//Cache Parameters
#define kParamOutputClear "outputClear"
#define kParamOutputClearCurrentFrame "outputClearCurrentFrame"

//Invalidation Parameters
#define kParamForceInvalidation "forceInvalidation"
#define kParamForceInvalidationAtTime "forceInvalidationAtTime"


/**
 * Choice Parameter option definition
 */

namespace openMVG_ofx {
namespace Localizer {

//kParamFeaturesType options
enum EParamFeaturesType
{
    eParamFeaturesTypeSIFT = 0,
    eParamFeaturesTypeCCTag,
    eParamFeaturesTypeSIFTAndCCTag
};

static const std::vector< std::pair<std::string, std::string> > kStringParamFeaturesType = {
  {"SIFT", "SIFT descriptors"}
#if HAVE_CCTAG
  ,{"CCTag", "CCTag descriptors"}
  ,{"SIFT and CCTag", "SIFT and CCTag descriptors"} 
#endif
};


//kParamFeaturesPreset options
enum EParamFeaturesPreset
{
    eParamFeaturesPresetLow = 0,
    eParamFeaturesPresetMedium,
    eParamFeaturesPresetNormal,
    eParamFeaturesPresetHigh,
    eParamFeaturesPresetUltra
};

static const std::vector< std::pair<std::string, std::string> > kStringParamFeaturesPreset = { 
  {"Low", ""},
  {"Medium", ""},
  {"Normal", ""},
  {"High", ""},
  {"Ultra" , ""}
};


//kParamRigMode options
enum EParamRigMode
{
    eParamRigModeKnown = 0,
    eParamRigModeUnKnown
};

static const std::vector< std::pair<std::string, std::string> > kStringParamRigMode = { 
  {"Known", ""},
  {"Unknown", ""}
};


//kParamInputDistortion options
enum EParamLensDistortion
{
    eParamLensDistortionKnown = 0,
    eParamLensDistortionApproximate,
    eParamLensDistortionUnKnown
};

static const std::vector< std::pair<std::string, std::string> > kStringParamLensDistortion = { 
  {"Known", ""},
  {"Approximate", ""},
  {"Unknown", ""} 
};


//kParamInputDistortionMode options
enum EParamLensDistortionMode
{
    eParamLensDistortionModeNone = 0,
    eParamLensDistortionModeRadial1,
    eParamLensDistortionModeRadial3,
    eParamLensDistortionModeBrown,
    eParamLensDistortionModeFisheye1,
    eParamLensDistortionModeFisheye4
};

static const std::vector< std::pair<std::string, std::string> > kStringParamLensDistortionMode = { 
  {"Pinhole", "No distortion"},
  {"Radial1", "Pinhole camera with 1 radial distortion coefficient: x_d = x_u (1 + K_1 r^2)\n"},
  {"Radial3", "Pinhole camera with 1 radial distortion coefficient: x_d = x_u (1 + K_1 r^2 + K_2 r^4 + K_3 r^6)\n"},
  {"Brown", "Pinhole camera with a 3 radial distortion coefficients and 2 tangential distortion coefficients: x_d = x_u (1 + K_1 r^2 + K_2 r^4 + K_3 r^6) + (T_2 (r^2 + 2 x_u^2) + 2 T_1 x_u y_u)\n"},
  {"Fisheye1", "Fish-eye camera model with only one parameter: x_d = atan(2.0 * r * tan(0.5 * k1)) / k1) / r\n"},
  {"Fisheye4", "Fish-eye camera model with only 3 parameters.\n"}
};


//kParamInputFocalLengthMode options
enum EParamFocalLengthMode
{
    eParamFocalLengthModeKnown = 0,
    eParamFocalLengthModeApproximate,
    eParamFocalLengthModeUnKnown
};

static const std::vector< std::pair<std::string, std::string> > kStringParamFocalLengthMode = { 
  {"Known", ""},
  {"Approximate", ""},
  {"Unknown", ""} 
};


//kParamAdvancedAlgorithm options
enum EParamAlgorithm
{
    eParamAlgorithmFirstBest = 0,
    eParamAlgorithmBestResult,
    eParamAlgorithmAllResults,
    eParamAlgorithmCluster
};

static const std::vector< std::pair<std::string, std::string> > kStringParamAlgorithm = {
  {"First Best", ""},
  {"Best Result", ""},
  {"All Results", ""},
  {"Cluster", ""}
};

//kParamTrackingRangeMode options
enum EParamTrackingRangeMode
{
    eParamRangeFromInputs = 0,
    eParamRangeCustom
};

static const std::vector< std::pair<std::string, std::string> > kStringParamTrackingRangeMode = { 
  {"From Inputs", ""},
  {"Custom", ""}
};

} //namespace Localizer
} //namespace openMVG_ofx