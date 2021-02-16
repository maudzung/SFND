#!/bin/bash

rm -rf build

mkdir -p build && cd build
cmake ..
make

printed_heading=false

# for detectorType in SHITOMASI HARRIS FAST BRISK ORB AKAZE SIFT; do
#     for descriptorType in BRISK BRIEF ORB FREAK AKAZE SIFT; do
#         for binHogDescriptorType in DES_BINARY DES_HOG; do
#             for matcherType in MAT_BF MAT_FLANN; do
#                 for selectorType in SEL_NN SEL_KNN; do
#                     # echo "$detectorType $descriptorType $binHogDescriptorType $matcherType $selectorType $printed_heading"
#                     ./2D_feature_tracking $detectorType $descriptorType $binHogDescriptorType $matcherType $selectorType $printed_heading
#                     printed_heading=true
#                 done
#             done
#         done
#     done
# done


for detectorType in SHITOMASI HARRIS FAST BRISK ORB AKAZE SIFT; do
    for descriptorType in BRISK BRIEF ORB FREAK AKAZE; do
        for binHogDescriptorType in DES_BINARY; do
            for matcherType in MAT_BF; do
                for selectorType in SEL_KNN; do
                    # echo "$detectorType $descriptorType $binHogDescriptorType $matcherType $selectorType $printed_heading"
                    ./2D_feature_tracking $detectorType $descriptorType $binHogDescriptorType $matcherType $selectorType $printed_heading
                    printed_heading=true
                done
            done
        done
    done
done