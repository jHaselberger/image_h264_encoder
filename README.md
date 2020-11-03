# Usage
Example:
```xml
<launch>
  <node name="h264_encoder" pkg="image_h264_encoder" type="image_h264_encoder_node" output="log">
    <param name="fps" value="20" />
    <param name="bitrate" value="10000" />
    <param name="decoderType" value="0" />
    <param name="logFilePath" value="/tmp/rec/h264Encoder.txt" />
    <param name="targetLocation" value="/tmp/rec/h264Encoder10k.mp4" />
  </node>
</launch>
```

# Dependencies
## OpenCV
Make sure that your OpenCV build supports GStreamer.  
Use python to querry the build configuration:
```python
import cv2
print(cv2.getBuildInformation())
```
Check for GStreamer support:
```txt
GStreamer:                   
  base:                      YES (ver 1.8.3)
  video:                     YES (ver 1.8.3)
  app:                       YES (ver 1.8.3)
  riff:                      YES (ver 1.8.3)
  pbutils:                   YES (ver 1.8.3)
```

## NVIDIA H264 encoder
 1. Download the Vision SDK from https://developer.nvidia.com/nvidia-video-codec-sdk/download 
 2. check if current nvidia driver is sufficent with `nvidia-smi`
 3. Install the SDK:
 ```shell
unzip Video_Codec_SDK.zip
cd Video_Codec_SDK

sudo cp include/* /usr/local/cuda/include
sudo cp Lib/linux/stubs/x86_64/* /usr/local/cuda/lib64/stubs
 ```
 
 4. Get GStreamer Bad plugins
 ```shell
 sudo apt-get install gstreamer1.0-plugins-bad
 git clone https://github.com/GStreamer/gst-plugins-bad.git
cd gst-plugins-bad

git checkout $(gst-launch-1.0 --version | grep version | tr -s ' ' '\n' | tail -1)

./autogen.sh --disable-gtk-doc --noconfigure

export NVENCODE_CFLAGS="-I/usr/local/cuda/include"
./configure --with-cuda-prefix="/usr/local/cuda"
 ```

5. Check output to confirm that nvenc, nvdec plugins is going to be build

6. Build and install nvenc
```
cd sys/nvenc
make 
sudo make install
cd ../..
```

7. Build and install nvdec
```
cd sys/nvdec
make 
sudo make install
cd ../..
```

8. Add GST_PLUGIN_PATH to bashrc
```
echo "export GST_PLUGIN_PATH=$GST_PLUGIN_PATH:/usr/local/lib/gstreamer-1.0/" >> ~/.bashrc
source ~/.bashrc
```

# Common issues
## openCV is not found
In some cases catkin searches for `opencv`, a symbolic link fixes the issue: 
```
sudo ln -s /usr/include/opencv4/ /usr/include/opencv
```
If OpenCV is still not found, try to set the environment variable `OpenCV_DIR`.

## OpenCV version specific videowriter
OpenCV changed the behavior of the videwriter class.  
To enable support for OpenCV 4 adjust the `OPENCVVERSION` to 4 in the [encoder source file](https://github.com/gismo07/image_h264_encoder/blob/13539635c424023b531f81006d5609f43a112d2e/src/image_h264_encoder_node.cpp#L1).
