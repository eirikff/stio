# Notes

This document is just notes and useful remarks/observations. 


## Feature detection

Only `fast` and `fast_grad` feature detectors are used. This can be seen in `svo_factory.cpp:85 loadDetectionOtions`. `fast_grad` is used in the case that edgelet features are used.

The relevant code for feature detection seems to be in `feature_detection.cpp` in `svo_direct`. 

For the mono case, it seems like the feature detection happens in the depth filters when a new keyframe is added. Relevant code is `depth_filter.cpp:291 initializeSeed` called from `depth_filter.cpp:88 addKeyFrame` in `svo_direct`. Called from `frame_handler_mono.cpp:217 processFrame`. This is only called if the current frame (`newFrame()`) is selected to be a keyframe. 

In `Frame`'s constructor, the `initFrame` function checks if the encoding is mono or color. If color, convert to gray scale. Then, it creates the image pyramid.

Might be unnecessary to create 8bit pyramid for thermal images, but need a pyramid of histogram equalized images. This needs to be created from one equalized image that is halfsampled into the pyramid levels, so equalize first, then create pyramid. 


### 16 bit detection

In `svo::factory::loadDetectorOptions` we can see that only the types `kFastGrad` and `kFast` are ever used, where the former is used when edgelets are also used, and the latter when only corners are used. Only these needs to have 16 bit counter parts.


### Todo 

**DONE** Make the histogram equalized image what is drawn on and published for the visualization.

Change from preprocessor ifdefs to option/parameter whether or not histogram equalized detection is used.


## 16 bit Direct Matching

In `sparse_img_align.cpp` it seems like the only relevant function to change for 16 bit matching is the `evaluateError` function. Go further to the `sparse_img_align_utils::computeResidualsOfFrame` function and there's a `uint8_t` type that gets assign a pixel value. This is probably where we need to change. 

Just changing this to use a `reinterpret_cast` might be all that it is too it.

The sparse image align class inherits from the vikit solver class. 

The feature alignment step is done in the `projectMapInFrame` function in `processFrame`. The actual alignment is hidden many many layers deep in the function, in the `Reprojector::reprojectFrame` -> `reprojector_utils::matchCandidates` -> `reprojector_utils::matchCandidate` -> `Matcher::findMatchDirect`. 

It looks like the relevant functions to change here are the `feature_alignment::align1D` and `::align2D`. However, we also need to change `patch_utils::createPatchFromPatchWithBorder` to create 16 bit patches.

After changing `Matcher::patch_` and `Matcher::patch_with_border_` to `uint16_t`, there are many red squiggly lines. These needs to be fixed and indicate where it is relevant to change. 

### Todo

Make the switch between 8 bit and 16 bit matching safer using asserts or something. Currently, it will just access incorrect memory which is bad.

Maybe all the overloaded functions can be reverted to using templates? So there isn't so much repeating code with only a couple changed types here and there.


## IMU

Delay is defined as `Camera-IMU delay: delay_imu_cam = cam_timestamp - imu_timestamp [s]` in `estimator_types.hpp:142` -> `struct ImuParameters`.



## GTSAM Backend

Much of the code from the ceres backend can be used as common code. E.g. 
* the interface code will be very similar and can likely be copied and renamed, and change to match gtsam interface;
* the motion detector and outlier rejection modules can probably be reused; 
* the estimator class is probably what needs to re-implemented using gtsam;
*

