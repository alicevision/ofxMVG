HdrCalib
============

HdrCalib is used to compute the response function of a couple (camera / lens)  from multiple groups of multi-bracketing images. This response function is a requirement in order to do a proper color calibration of an HDR. 

[HdrCalib on ShuttleOFX.](http://shuttleofx.org/plugin/tuttleofx.hdrcalib)

## Compilation

```
git submodule update -i
mkdir build
cd build
cmake .. -DCMAKE_INSTALL_PREFIX=../install
make install
```
