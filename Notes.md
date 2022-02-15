# Notes

This document is just notes and useful remarks/observations. 


## Feature detection

Only `fast` and `fast_grad` feature detectors are used. This can be seen in `svo_factory.cpp:85 loadDetectionOtions`. `fast_grad` is used in the case that edgelet features are used.

The relevant code for feature detection seems to be in `feature_detection.cpp` in `svo_direct`. 

For the mono case, it seems like the feature detection happens in the depth filters when a new keyframe is added. Relevant code is `depth_filter.cpp:291 initializeSeed` called from `depth_filter.cpp:88 addKeyFrame` in `svo_direct`. Called from `frame_handler_mono.cpp:217 processFrame`. This is only called if the current frame (`newFrame()`) is selected to be a keyframe. 

In `Frame`'s constructor, the `initFrame` function checks if the encoding is mono or color. If color, convert to gray scale. Then, it creates the image pyramid.

Might be unnecessary to create 8bit pyramid for thermal images, but need a pyramid of histogram equalized images. This needs to be created from one equalized image that is halfsampled into the pyramid levels, so equalize first, then create pyramid. 


### Todo 

Make the histogram equalized image what is drawn on and published for the visualization.

Change from preprocessor ifdefs to option/parameter whether or not histogram equalized detection is used.


## 16 bit Direct Matching

In `sparse_img_align.cpp` it seems like the only relevant function to change for 16 bit matching is the `evaluateError` function. Go further to the `sparse_img_align_utils::computeResidualsOfFrame` function and there's a `uint8_t` type that gets assign a pixel value. This is probably where we need to change. 

The sparse image align class inherits from the vikit solver class. 

