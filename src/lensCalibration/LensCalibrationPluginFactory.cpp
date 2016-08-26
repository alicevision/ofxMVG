#pragma once
#include "LensCalibrationPluginFactory.hpp"
#include "LensCalibrationPluginDefinition.hpp"
#include "LensCalibrationInteract.hpp"

namespace openMVG_ofx {
namespace LensCalibration {

void LensCalibrationPluginFactory::describe(OFX::ImageEffectDescriptor& desc)
{
  //Plugin Labels
  desc.setLabels(
    "LensCalibration",
    "LensCalibration",
    "openMVG LensCalibration");

  //Plugin grouping
  desc.setPluginGrouping("openMVG");

  //Plugin description
  desc.setPluginDescription(
    "LensCalibration estimates the best distortion parameters "
    "according to the couple camera/optics of a dataset."
    "\n"
    "The plugin supports video file & folder containing images or "
    "image sequence."
  );

  //Supported contexts
  desc.addSupportedContext(OFX::eContextFilter);
  desc.addSupportedContext(OFX::eContextGeneral);
  desc.addSupportedContext(OFX::eContextPaint);

  //Supported pixel depths
  desc.addSupportedBitDepth(OFX::eBitDepthUByte);
  desc.addSupportedBitDepth(OFX::eBitDepthUShort);
  desc.addSupportedBitDepth(OFX::eBitDepthFloat);

  //Flags
  desc.setSingleInstance(false);
  desc.setHostFrameThreading(false);
  desc.setSupportsMultiResolution(false);
  desc.setSupportsTiles(false);
  desc.setTemporalClipAccess(false);
  desc.setRenderTwiceAlways(false);
  desc.setSupportsMultipleClipPARs(false);

  desc.setOverlayInteractDescriptor( new LensCalibrationOverlayDescriptor);
}

void LensCalibrationPluginFactory::describeInContext(OFX::ImageEffectDescriptor& desc, OFX::ContextEnum context)
{
  //Input Clip
  OFX::ClipDescriptor *srcClip = desc.defineClip(kOfxImageEffectSimpleSourceClipName);
  srcClip->addSupportedComponent(OFX::ePixelComponentRGBA);
  srcClip->setTemporalClipAccess(false);
  srcClip->setSupportsTiles(false);
  srcClip->setIsMask(false);
  srcClip->setOptional(false);
  
  //Output clip
  OFX::ClipDescriptor *dstClip = desc.defineClip(kOfxImageEffectOutputClipName);
  dstClip->addSupportedComponent(OFX::ePixelComponentRGBA);
  dstClip->setSupportsTiles(false);
  
  //Calibration Group
  {
    OFX::GroupParamDescriptor *groupCalibration = desc.defineGroupParam(kParamGroupCalibration);
    groupCalibration->setLabel("Calibration");
    groupCalibration->setAsTab();
    
    {
      OFX::IntParamDescriptor *param = desc.defineIntParam(kParamNbCheckersDetected);
      param->setLabel("Nb Checkers Detected");
      param->setHint("Numbers of checkers detected");
      param->setDisplayRange(0, 100);
      param->setDefault(0);
      param->setAnimates(false);
      param->setEvaluateOnChange(false);
      param->setEnabled(false);
      param->setParent(*groupCalibration);
      param->setLayoutHint(OFX::eLayoutHintDivider);
    }
    
    {
      OFX::BooleanParamDescriptor *param = desc.defineBooleanParam(kParamIsCalibrated);
      param->setLabel("Is calibrated");
      param->setHint("Is calibrated");
      param->setEvaluateOnChange(true);
      param->setEnabled(false);
      param->setAnimates(false);
      param->setParent(*groupCalibration);
    }

    {
      OFX::BooleanParamDescriptor *param = desc.defineBooleanParam(kParamInputImageIsGray);
      param->setLabel("Input image is gray");
      param->setHint("Input image is gray");
      param->setParent(*groupCalibration);
    }
    
    {
      OFX::Int2DParamDescriptor *param = desc.defineInt2DParam(kParamImageSize);
      param->setLabel("Image Size");
      param->setHint("Input image size used to calibrate the optics. Obviously, all images should have the same size.");
      param->setDefault(0, 0);
      param->setDisplayRange(0, 0, 10000, 10000);
      param->setAnimates(false);
      param->setParent(*groupCalibration);
      param->setEnabled(false); // should not be edited by the user
    }

    {
      OFX::ChoiceParamDescriptor *param = desc.defineChoiceParam(kParamPatternType);
      param->setLabel("Pattern Type");
      param->setHint("Type of pattern to detect");
      param->appendOptions(kStringParamPatternType);
      param->setDefault(eParamPatternTypeChessboard);
      param->setAnimates(false);
      param->setParent(*groupCalibration);
    }

    {
      OFX::Int2DParamDescriptor *param = desc.defineInt2DParam(kParamPatternSize);
      param->setLabel("Pattern Size");
      param->setHint("Number of inner corners per one of board dimension Width Height");
      param->setDefault(10, 7);
      param->setRange(2, 2, kOfxFlagInfiniteMax, kOfxFlagInfiniteMax);
      param->setDisplayRange(2, 2, 15, 15);
      param->setAnimates(false);
      param->setParent(*groupCalibration);
    }
    
    {
      OFX::DoubleParamDescriptor *param = desc.defineDoubleParam(kParamSquareSize);
      param->setLabel("Square Size");
      param->setHint("Define the size of the grid's square cells (mm)");
      param->setDisplayRange(0, 100);
      param->setDefault(1);
      param->setAnimates(false);
      param->setParent(*groupCalibration);
      param->setLayoutHint(OFX::eLayoutHintDivider);
    }

    {
      OFX::GroupParamDescriptor *groupAdvanced = desc.defineGroupParam(kParamGroupCalibrationAdvanced);
      groupAdvanced->setLabel("Advanced Parameters");
      groupAdvanced->setParent(*groupCalibration);
      groupAdvanced->setOpen(false);
      
      {
        OFX::IntParamDescriptor *param = desc.defineIntParam(kParamNbRadialCoef);
        param->setLabel("Nb Radial Coef");
        param->setHint("Number of radial coefficient.");
        param->setRange(0, 6);
        param->setDisplayRange(0, 6);
        param->setDefault(3);
        param->setAnimates(false);
        param->setParent(*groupAdvanced);
      }

      {
        OFX::IntParamDescriptor *param = desc.defineIntParam(kParamMaxCalibFrames);
        param->setLabel("Max Calibration Frames");
        param->setHint("Maximal number of frames to use to calibrate from the selected frames.");
        param->setRange(0, kOfxFlagInfiniteMax);
        param->setDisplayRange(0, 1000);
        param->setDefault(100);
        param->setAnimates(false);
        param->setParent(*groupAdvanced);
      }

      {
        OFX::IntParamDescriptor *param = desc.defineIntParam(kParamCalibGridSize);
        param->setLabel("Max Calibration Grid Size");
        param->setHint("Define the number of cells per edge.");
        param->setRange(0, kOfxFlagInfiniteMax);
        param->setDisplayRange(0, 100);
        param->setDefault(10);
        param->setAnimates(false);
        param->setParent(*groupAdvanced);
      }

      {
        OFX::IntParamDescriptor *param = desc.defineIntParam(kParamMinInputFrames);
        param->setLabel("Min Input Frames");
        param->setHint("Minimal number of frames to limit the calibration refinement loop.");
        param->setRange(0, kOfxFlagInfiniteMax);
        param->setDisplayRange(0, 1000);
        param->setDefault(10);
        param->setAnimates(false);
        param->setParent(*groupAdvanced);
      }

      {
        OFX::DoubleParamDescriptor *param = desc.defineDoubleParam(kParamMaxTotalAvgErr);
        param->setLabel("Max Total Average Error");
        param->setHint("Maximal limit of the total average error");
        param->setRange(0, 1);
        param->setDisplayRange(0, 1);
        param->setDefault(0.1);
        param->setAnimates(false);
        param->setParent(*groupAdvanced);
        param->setLayoutHint(OFX::eLayoutHintDivider);
      }
    }
    
    {
      OFX::PushButtonParamDescriptor *param = desc.definePushButtonParam(kParamCalibrate);
      param->setLabel("Calibrate");
      param->setHint("calibrate");
      param->setParent(*groupCalibration);
    }

  }

  //Output Group
  {
    OFX::GroupParamDescriptor *groupOutput = desc.defineGroupParam(kParamGroupOutput);
    groupOutput->setLabel("Output");
    groupOutput->setAsTab();
    
    {
      OFX::DoubleParamDescriptor *param = desc.defineDoubleParam(kParamOutputAvgReprojErr);
      param->setLabel("Average Reprojection Error");
      param->setDisplayRange(0, 10);
      param->setEvaluateOnChange(false);
      param->setEnabled(false);
      param->setAnimates(false);
      param->setParent(*groupOutput);
      param->setLayoutHint(OFX::eLayoutHintDivider);
    }
    
    {
      OFX::GroupParamDescriptor *groupCamera = desc.defineGroupParam(kParamOutputCameraGroup);
      groupCamera->setLabel("Intrinsics Camera Parameters");
      groupCamera->setParent(*groupOutput);
      groupCamera->setOpen(true);

      {
        OFX::DoubleParamDescriptor *param = desc.defineDoubleParam(kParamOutputFocalLenght);
        param->setLabel("Focal Length");
        param->setDisplayRange(1, 100);
        param->setEvaluateOnChange(false);
        param->setEnabled(false);
        param->setAnimates(false);
        param->setParent(*groupCamera);
      }

      {
        OFX::Double2DParamDescriptor *param = desc.defineDouble2DParam(kParamOutputPrincipalPointOffset);
        param->setLabel("Principal Point");
        param->setEvaluateOnChange(false);
        param->setEnabled(false);
        param->setAnimates(false);
        param->setParent(*groupCamera);
      }
    }
    
    {    
      OFX::GroupParamDescriptor *groupLensDistortion = desc.defineGroupParam(kParamOutputLensDistortionGroup);
      groupLensDistortion->setLabel("Lens Distortion Coefficients");
      groupLensDistortion->setParent(*groupOutput);
      groupLensDistortion->setOpen(true);
        
      {
        OFX::DoubleParamDescriptor *param = desc.defineDoubleParam(kParamOutputRadialCoef1);
        param->setLabel("Radial Coef1");
        param->setDisplayRange(-2, 2);
        param->setEvaluateOnChange(false);
        param->setEnabled(false);
        param->setAnimates(false);
        param->setParent(*groupLensDistortion);
      }

      {
        OFX::DoubleParamDescriptor *param = desc.defineDoubleParam(kParamOutputRadialCoef2);
        param->setLabel("Radial Coef2");
        param->setDisplayRange(-2, 2);
        param->setEvaluateOnChange(false);
        param->setEnabled(false);
        param->setAnimates(false);
        param->setParent(*groupLensDistortion);
      }

      {
        OFX::DoubleParamDescriptor *param = desc.defineDoubleParam(kParamOutputRadialCoef3);
        param->setLabel("Radial Coef3");
        param->setDisplayRange(-2, 2);
        param->setEvaluateOnChange(false);
        param->setEnabled(false);
        param->setAnimates(false);
        param->setParent(*groupLensDistortion);
        param->setLayoutHint(OFX::eLayoutHintDivider);
      }

      {
        OFX::DoubleParamDescriptor *param = desc.defineDoubleParam(kParamOutputTangentialCoef1);
        param->setLabel("Tangential Coef1");
        param->setDisplayRange(-2, 2);
        param->setEvaluateOnChange(false);
        param->setEnabled(false);
        param->setAnimates(false);
        param->setIsSecret(true);
        param->setParent(*groupLensDistortion);
      }

      {
        OFX::DoubleParamDescriptor *param = desc.defineDoubleParam(kParamOutputTangentialCoef2);
        param->setLabel("Tangential Coef2");
        param->setDisplayRange(-2, 2);
        param->setEvaluateOnChange(false);
        param->setEnabled(false);
        param->setAnimates(false);
        param->setIsSecret(true);
        param->setParent(*groupLensDistortion);
      }
    }
    
    {
      OFX::PushButtonParamDescriptor *param = desc.definePushButtonParam(kParamOutputClearCalibration);
      param->setLabel("Clear Calibration");
      param->setHint("Clear all calibration values");
      param->setEnabled(true);
      param->setParent(*groupOutput);
    }
    
    {
      OFX::PushButtonParamDescriptor *param = desc.definePushButtonParam(kParamOutputClearAll);
      param->setLabel("Clear All");
      param->setHint("Clear all calibration values and detected checkers in cache");
      param->setEnabled(true);
      param->setParent(*groupOutput);
    }
  }
  
  //Debug Group
  {
    OFX::GroupParamDescriptor *groupDebug = desc.defineGroupParam(kParamGroupDebug);
    groupDebug->setLabel("Debug");
    groupDebug->setAsTab();

    {
      OFX::BooleanParamDescriptor *param = desc.defineBooleanParam(kParamDebugEnable);
      param->setLabel("Enable Debug");
      param->setHint("Would you want to export undistorted images?");
      param->setParent(*groupDebug);
    }

    {
      OFX::StringParamDescriptor *param = desc.defineStringParam(kParamDebugRejectedImgFolder);
      param->setLabel("Rejected Frames");
      param->setHint("Folder to export delete images during the calibration refinement loop.");
      param->setStringType(OFX::eStringTypeDirectoryPath);
      param->setFilePathExists(true);
      param->setParent(*groupDebug);
    }

    {
      OFX::StringParamDescriptor *param = desc.defineStringParam(kParamDebugSelectedImgFolder);
      param->setLabel("Selected Frames");
      param->setHint("Folder to export debug images.");
      param->setStringType(OFX::eStringTypeDirectoryPath);
      param->setFilePathExists(true);
      param->setParent(*groupDebug);
    }
  }
}

OFX::ImageEffect* LensCalibrationPluginFactory::createInstance(OfxImageEffectHandle handle, OFX::ContextEnum context)
{
  return new LensCalibrationPlugin(handle);
}

} //namespace LensCalibration
} //namespace openMVG_ofx
