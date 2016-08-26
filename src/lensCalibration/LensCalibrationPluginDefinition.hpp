#pragma once

#include <vector>
#include <string>

/**
 * Plugin Parameters definition
 */

//Calibration parameters
#define kParamGroupCalibration "groupCalibration"

#define kParamNbCheckersDetected "NbCheckersDetected"
#define kParamIsCalibrated "IsCalibrated"
#define kParamInputImageIsGray "inputImageIsGray"
#define kParamImageSize "imageSize"
#define kParamPatternType "patternType"
#define kParamPatternSize "patternSize"
#define kParamSquareSize "SquareSize"
#define kParamNbRadialCoef "nbRadialCoef"

#define kParamGroupCalibrationAdvanced "groupAdvanced"

#define kParamMaxCalibFrames "maxCalibFrames"
#define kParamCalibGridSize "calibGridSize"
#define kParamMinInputFrames "minInputFrames"
#define kParamMaxTotalAvgErr "maxTotalAvgErr"
#define kParamCalibrate "outputCalibrate"

//Debug parameters
#define kParamGroupDebug "groupDebug"

#define kParamDebugEnable "debugEnable"
#define kParamDebugRejectedImgFolder "debugRejectedImgFolder"
#define kParamDebugSelectedImgFolder "debugSelectedImgFolder"

//Output parameters
#define kParamGroupOutput "groupOutput"

#define kParamOutputAvgReprojErr "outputAvgReprojErr"
#define kParamOutputClearCalibration "outputClearCalibration"
#define kParamOutputClearAll "outputClearAll"

#define kParamOutputCameraGroup "groupCamera"

#define kParamOutputFocalLenght "outputFocalLenght"
#define kParamOutputPrincipalPointOffset "outputPrincipalPointOffset"

#define kParamOutputLensDistortionGroup "groupLensDistortion"

#define kParamOutputRadialCoef1 "outputRadialCoef1"
#define kParamOutputRadialCoef2 "outputRadialCoef2"
#define kParamOutputRadialCoef3 "outputRadialCoef3"
#define kParamOutputTangentialCoef1 "outputTangentialCoef1"
#define kParamOutputTangentialCoef2 "outputTangentialCoef2"

/**
 * Choice Parameter option definition
 */

namespace openMVG_ofx {
namespace LensCalibration {

//EParamPatternType options
enum EParamPatternType
{
  eParamPatternTypeChessboard = 0,
  eParamPatternTypeCirclesGrid,
  eParamPatternTypeAsymmetricCirclesGrid
#ifdef HAVE_CCTAG
  , eParamPatternTypeCCTagGrid
#endif
};

static const std::vector< std::pair<std::string, std::string> > kStringParamPatternType = {
  {"Chessboard", ""},
  {"Circles grid", ""},
  {"Asymmetric circles grid", ""},
  {"CCTag", ""}
};

} //namespace LensCalibration
} //namespace openMVG_ofx
