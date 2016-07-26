#pragma once
#include <vector>
#include <string>

/**
 * Plugin Parameters definition
 */

//Clip
#define kClip(I) std::to_string(I + 1)

//Global Parameters
#define kParamOutputIndex "outputIndex"
#define kParamFeaturesType "featuresType"
#define kParamFeaturesPreset "featuresPreset"
#define kParamReconstructionFile "reconstructionFile"
#define kParamDescriptorsFolder "descriptorsFolder"
#define kParamVoctreeFile "voctreeFile"
#define kParamRigMode "rigMode"
#define kParamRigCalibrationFile "rigCalibrationFile"


//Input Parameters
#define kParamGroupInput(I) "groupInput" + std::to_string(I)

#define kParamInputGroupLensCalibration(I) "groupInputLensCalibration" + std::to_string(I)
#define kParamInputGroupRelativePose(I) "groupInputRelativePose" + std::to_string(I)

#define kParamInputLensCalibrationFile(I) "inputLensCalibrationFile" + std::to_string(I)
#define kParamInputDistortion(I) "inputDistortion" + std::to_string(I)
#define kParamInputDistortionMode(I) "inputDistortionMode" + std::to_string(I)
#define kParamInputDistortionCoef1(I) "inputDistortionCoef1" + std::to_string(I)
#define kParamInputDistortionCoef2(I) "inputDistortionCoef2" + std::to_string(I)
#define kParamInputDistortionCoef3(I) "inputDistortionCoef3" + std::to_string(I)
#define kParamInputDistortionCoef4(I) "inputDistortionCoef4" + std::to_string(I)
#define kParamInputOpticalCenter(I) "inputOpticalCenter" + std::to_string(I)
#define kParamInputFocalLengthMode(I) "inputFocalLengthMode" + std::to_string(I)
#define kParamInputFocalLength(I) "inputFocalLength" + std::to_string(I)
#define kParamInputFocalLengthVarying(I) "inputFocalLengthVarying" + std::to_string(I)
#define kParamInputSensorWidth(I) "inputSensorWidth" + std::to_string(I)
#define kParamInputRelativePoseTranslate(I) "inputRelativePoseTranslate" + std::to_string(I)
#define kParamInputRelativePoseRotate(I) "inputRelativePoseRotate" + std::to_string(I)
#define kParamInputRelativePoseScale(I) "inputRelativePoseScale" + std::to_string(I)


//Advanced Parameters
#define kParamGroupAdvanced "groupAdvanced"

#define kParamAdvancedAlgorithm "advancedAlgorithm"
#define kParamAdvancedReprojectionError "advancedReprojectionError"
#define kParamAdvancedNbImageMatch "advancedNbImageMatch"
#define kParamAdvancedMaxResults "advancedMaxResults"
#define kParamAdvancedVoctreeWeights "advancedVoctreeWeights"
#define kParamAdvancedMatchingError "advancedMatchingError"
#define kParamAdvancedCctagNbNearestKeyFrames "advancedCctagNbNearestKeyFrames"
#define kParamAdvancedBaMinPointVisibility "advancedBaMinPointVisibility"
#define kParamAdvancedDebugFolder "advancedDebugFolder"

//Tracking
#define kParamTrackingTrack "trackingTrack"
#define kParamTrackingRangeMode "trackingRangeMode"
#define kParamTrackingRangeMin "trackingRangeMin"
#define kParamTrackingRangeMax "trackingRangeMax"

//Output Parameters
#define kParamGroupOutput "groupOutput"

#define kParamOutputTranslate "outputTranslate"
#define kParamOutputRotate "outputRotate"
#define kParamOutputScale "outputScale"
#define kParamOutputDistortionCoef1 "outputDistortionCoef1"
#define kParamOutputDistortionCoef2 "outputDistortionCoef2"
#define kParamOutputDistortionCoef3 "outputDistortionCoef3"
#define kParamOutputDistortionCoef4 "outputDistortionCoef4"
#define kParamOutputOpticalCenter "outputOpticalCenter"
#define kParamOutputFocalLength "outputFocalLength"
#define kParamOutputNear "outputNear"
#define kParamOutputFar "outputFar"
#define kParamOutputCreateCamera "outputCreateCamera"

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
  ,{"CCTag", "CCTag descriptors"},
  {"SIFT and CCTag", "SIFT and CCTag descriptors"} 
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