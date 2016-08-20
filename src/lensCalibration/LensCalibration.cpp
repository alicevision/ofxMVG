#include "LensCalibration.hpp"

#include <openMVG/image/image_converter.hpp>

#include <opencv2/opencv.hpp>

namespace openMVG_ofx {
namespace LensCalibration {


void convertRGBImage(const Common::Image<float>& inputImageOFX, openMVG::image::Image<openMVG::image::RGBfColor>& outputImageMVG)
{
  assert(outputImageMVG.Height() == inputImageOFX.getHeight());
  assert(outputImageMVG.Width() == inputImageOFX.getWidth());
  for(unsigned int y = 0; y < inputImageOFX.getHeight(); ++y)
  {
    for(unsigned int x = 0; x < inputImageOFX.getWidth(); ++x)
    {
      const float* rgbPtr = inputImageOFX.getPixel(x, y);
      outputImageMVG(y, x) = openMVG::image::RGBfColor(rgbPtr[0], rgbPtr[1], rgbPtr[2]);
    }
  }
}

void convertRGBImage(const openMVG::image::Image<openMVG::image::RGBfColor>& inputImageMVG, Common::Image<float>& outputImageOFX)
{
  assert(inputImageMVG.Height() == outputImageOFX.getHeight());
  assert(inputImageMVG.Width() == outputImageOFX.getWidth());
  for(unsigned int y = 0; y < outputImageOFX.getHeight(); ++y)
  {
    for(unsigned int x = 0; x < outputImageOFX.getWidth(); ++x)
    {
      const openMVG::image::RGBfColor& color = inputImageMVG(y, x);
      float* rgbPtr = outputImageOFX.getPixel(x, y);
      rgbPtr[0] = color.r();
      rgbPtr[1] = color.g();
      rgbPtr[2] = color.b();
      rgbPtr[3] = 1.0;
    }
  }
}

void convertRGB32ToGRAY8(const Common::Image<float>& inputImage, cv::Mat& outputImage)
{
  assert(inputImage.getHeight() == outputImage.rows);
  assert(inputImage.getWidth() == outputImage.cols);
  for(unsigned int y = 0; y < inputImage.getHeight(); ++y)
  {
    for(unsigned int x = 0; x < inputImage.getWidth(); ++x)
    {
      const float* rgbPtr = inputImage.getPixel(x, y);
      const float gray = openMVG::image::Rgb2Gray(rgbPtr[0], rgbPtr[1], rgbPtr[2]);
      outputImage.at<unsigned char>(y, x) = (unsigned char)(gray * 255.f);
    }
  }
}

void convertGGG32ToGRAY8(const Common::Image<float>& inputImage, cv::Mat& outputImage)
{
  assert(inputImage.getHeight() == outputImage.rows);
  assert(inputImage.getWidth() == outputImage.cols);
  for(unsigned int y = 0; y < inputImage.getHeight(); ++y)
  {
    for(unsigned int x = 0; x < inputImage.getWidth(); ++x)
    {
      const float* rgbPtr = inputImage.getPixel(x, y);
      const float gray = rgbPtr[0];
      outputImage.at<unsigned char>(y, x) = (unsigned char)(gray * 255.f);
    }
  }
}

openMVG::calibration::Pattern getPatternType(EParamPatternType pattern)
{
  switch(pattern)
  {
    case eParamPatternTypeChessboard : return openMVG::calibration::CHESSBOARD; break;
    case eParamPatternTypeCirclesGrid : return openMVG::calibration::CIRCLES_GRID; break;
    case eParamPatternTypeAsymmetricCirclesGrid : return openMVG::calibration::ASYMMETRIC_CIRCLES_GRID; break;
#ifdef HAVE_CCTAG
    case eParamPatternTypeCCTagGrid : return openMVG::calibration::CCTAG_GRID; break;
#endif
    default : throw std::invalid_argument("Unrecognized Pattern Type : " + std::to_string(pattern));
  }
}

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
                     cv::Mat distCoeffs)
{
  _outputIsCalibrated->setValue(isCalibrated);
  _outputAvgReprojErr->setValue(totalAvgErr);
  _outputCameraFocalLenght->setValue(cameraMatrix.at<double>(0,0));
  _outputCameraPrincipalPointOffset->setValue(cameraMatrix.at<double>(0,2), cameraMatrix.at<double>(1,2));
  _outputLensDistortionRadialCoef1->setValue(distCoeffs.at<double>(0));
  _outputLensDistortionRadialCoef2->setValue(distCoeffs.at<double>(1));
  _outputLensDistortionRadialCoef3->setValue(distCoeffs.at<double>(4));
  _outputLensDistortionTangentialCoef1->setValue(distCoeffs.at<double>(2));
  _outputLensDistortionTangentialCoef2->setValue(distCoeffs.at<double>(3));  
}

} //namespace LensCalibration
} //namespace openMVG_ofx
