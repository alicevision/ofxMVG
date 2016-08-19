#pragma once
#include "LensCalibrationPluginDefinition.hpp"

#include "../common/Image.hpp"

#include <openMVG/calibration/patternDetect.hpp>
#include <openMVG/image/image.hpp>

#include <opencv2/core/mat.hpp>

namespace openMVG_ofx {
namespace LensCalibration {

/**
 * @brief convert a rgb OFX image to a rgb MVG image
 * @param[in] inputImageOFX
 * @param[out] outputImageMVG
 */
void convertRGBImage(const Common::Image<float>& inputImageOFX, openMVG::image::Image<openMVG::image::RGBfColor>& outputImageMVG);

/**
 * @brief convert a rgb MVG image to a rgb OFX image
 * @param[in] inputImageMVG
 * @param[out] outputImageOFX
 */
void convertRGBImage(const openMVG::image::Image<openMVG::image::RGBfColor>& inputImageMVG, Common::Image<float>& outputImageOFX);

/**
   * @brief convert a (matrix) 32 bits rgb image to a gray (unsigned char) 8 bits image
   * @param[in] inputImage
   * @param[out] outputImage
   */
void convertRGB32ToGRAY8(const Common::Image<float>& inputImage, cv::Mat& outputImage);

/**
   * @brief convert a (matrix) 32 bits ggg image to a gray (unsigned char) 8 bits image
   * @param[in] inputImage
   * @param[out] outputImage
   */
void convertGGG32ToGRAY8(const Common::Image<float>& inputImage, cv::Mat& outputImage);

/**
 * @brief get openMVG pattern type enum from Plugin display choice enum
 * @param[in] pattern
 * @return 
 */
openMVG::calibration::Pattern getPatternType(EParamPatternType pattern);

/**
 * @brief Set intrinsic values and distortion coefficients into OFX parameters 
 * @param[in] _outputIsCalibrated
 * @param[in] _outputAvgReprojErr
 * @param[in] _outputCameraFocalLenght
 * @param[in] _outputCameraPrincipalPointOffset
 * @param[in] _outputLensDistortionRadialCoef1
 * @param[in] _outputLensDistortionRadialCoef2
 * @param[in] _outputLensDistortionRadialCoef3
 * @param[in] _outputLensDistortionTangentialCoef1
 * @param[in] _outputLensDistortionTangentialCoef2
 * @param[in] isCalibrated
 * @param[in] totalAvgErr
 * @param[in] cameraMatrix
 * @param[in] distCoeffs
 */
void setOutputParams(OFX::BooleanParam *_outputIsCalibrated,
                     OFX::DoubleParam *_outputAvgReprojErr,
                     OFX::DoubleParam *_outputCameraFocalLenght,
                     OFX::Double2DParam *_outputCameraPrincipalPointOffset,
                     OFX::DoubleParam *_outputLensDistortionRadialCoef1,
                     OFX::DoubleParam *_outputLensDistortionRadialCoef2,
                     OFX::DoubleParam *_outputLensDistortionRadialCoef3,
                     OFX::DoubleParam *_outputLensDistortionTangentialCoef1,
                     OFX::DoubleParam *_outputLensDistortionTangentialCoef2,
                     bool isCalibrated,
                     double totalAvgErr,
                     cv::Mat cameraMatrix,
                     cv::Mat distCoeffs);
  
} //namespace LensCalibration
} //namespace openMVG_ofx

