#include "Image.hpp"
#include <cassert>
#include <iostream>

namespace cameraColorCalibration {

template<typename DataType>
Image<DataType>::Image()
{
}

template<typename DataType>
Image<DataType>::Image(std::size_t width, std::size_t height, std::size_t nbChannels)
{
  createInternalBuffer(width, height, nbChannels);
}

template<typename DataType>
Image<DataType>::Image(OFX::Image *imgData)
{
  std::size_t width = imgData->getRegionOfDefinition().x2 - imgData->getRegionOfDefinition().x1;
  std::size_t height = imgData->getRegionOfDefinition().y2 - imgData->getRegionOfDefinition().y1;
  
  //copy the pointer for delete
  _imgPtr = imgData;
 
  setExternalBuffer((DataType*)imgData->getPixelData(), width, height, imgData->getPixelComponentCount(), imgData->getRowBytes() / sizeof(DataType));
}

template<typename DataType>
Image<DataType>::~Image()
{
  clear();
}

template<typename DataType>
void Image<DataType>::createInternalBuffer(std::size_t width, std::size_t height, std::size_t nbChannels)
{
  clear();
  _hasOwnership = 1;
  _nbChannels = nbChannels;
  _data = new DataType[width * height * _nbChannels];
  _width = width;
  _height = height;
  _size = width * height * _nbChannels;
  _nbPixels = width * height;
  _rowBufferSize = width * _nbChannels;
}

template<typename DataType>
void Image<DataType>::setExternalBuffer(DataType *data, std::size_t width, std::size_t height, std::size_t channels, std::size_t rowBufferSize)
{
  clear();
  _data = data;
  _hasOwnership = 0;
  _width = width;
  _height = height;
  _nbChannels = channels;
  _size = width * height * _nbChannels;
  _nbPixels = width * height;
  _rowBufferSize = rowBufferSize;
}

template<typename DataType>
void Image<DataType>::clear()
{
  if(_hasOwnership)
  {
    delete _imgPtr;
    delete _data;
  }
  _data = nullptr;
  _hasOwnership = 0;
  _width = 0;
  _height = 0;
  _size = 0;
  _nbPixels = 0;
}

template<typename DataType>
void Image<DataType>::setZero()
{
  for(std::size_t row = 0; row < getHeight(); ++row)
  {
    for(std::size_t col = 0; col < getWidth(); ++col)
    {
      DataType *ptr = getPixel(col, row);

      for(std::size_t channel = 0; channel < getNbChannels(); ++channel)
      {
        *ptr = 0;
        
        ++ptr;
      }
    }
  }
}

template<typename DataType>
void Image<DataType>::setRed()
{
  for(std::size_t row = 0; row < getHeight(); ++row)
  {
    for(std::size_t col = 0; col < getWidth(); ++col)
    {
      DataType *ptr = getPixel(col, row);
      *ptr = 1;
      ++ptr;
      for(std::size_t channel = 1; channel < getNbChannels(); ++channel)
      {
        *ptr = 0;
        
        ++ptr;
      }
    }
  }
}

template<typename DataType>
void Image<DataType>::multiply(const Image<DataType> &other)
{
  assert(this->getSize() == other.getSize());

  for(std::size_t row = 0; row < getHeight(); ++row)
  {
    for(std::size_t col = 0; col < getWidth(); ++col)
    {
      DataType *ptr = getPixel(col, row);
      const DataType *otherPtr = other.getPixel(col, row);

      for(std::size_t channel = 0; channel < getNbChannels(); ++channel)
      {
        *ptr *= *otherPtr;
        
        ++ptr;
        ++otherPtr;
      }
    }
  }
}

template<typename DataType>
void Image<DataType>::multiply(float coefficient)
{
  for(std::size_t row = 0; row < getHeight(); ++row)
  {
    for(std::size_t col = 0; col < getWidth(); ++col)
    {
      DataType *ptr = getPixel(col, row);

      for(std::size_t channel = 0; channel < getNbChannels(); ++channel)
      {
        *ptr *= coefficient;
        
        ++ptr;
      }
    }
  }
}

template<typename DataType>
void Image<DataType>::divide(const Image &other)
{
  assert(this->getSize() == other.getSize());

  for(std::size_t row = 0; row < getHeight(); ++row)
  {
    for(std::size_t col = 0; col < getWidth(); ++col)
    {
      DataType *ptr = getPixel(col, row);
      const DataType *otherPtr = other.getPixel(col, row);

      for(std::size_t channel = 0; channel < getNbChannels(); ++channel)
      {
        *ptr /= *otherPtr;
        
        ++ptr;
        ++otherPtr;
      }
    }
  }
}

template<typename DataType>
void Image<DataType>::copyFrom(const Image &other)
{
  if(other.hasOwnership())
  {
    for(std::size_t y = 0; y < getHeight(); ++y)
    {
      for(std::size_t x = 0; x < getWidth(); ++x)
      {
        DataType *ptr = getPixel(x, y);
        const DataType *otherPtr = other.getPixel(x, y);

        for(std::size_t channel = 0; channel < getNbChannels(); ++channel)
        {
          *ptr = *otherPtr;

          ++ptr;
          ++otherPtr;
        }
      }
    }
  }
}

template<typename DataType>
void Image<DataType>::checkSameDimensions(const std::vector< Image<DataType> > &images)
{
  if(images.empty())
  {
    throw std::logic_error("Image group is empty");
  }
  for(auto const &image : images)
  {
    if((image.getWidth() != images[0].getWidth())
            || (image.getHeight() != images[0].getHeight()))
    {
      throw std::logic_error("Group images have different sizes");
    }
  }
}

template class Image<float>;

}


