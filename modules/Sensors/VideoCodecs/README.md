# IVideoCodecs implementation
The IVideoCodecs implementation allows encoding and decoding of video streams. It provides a standardise interface for the video encoding and decoding.

### Currently implemented encoders
- JPEG
- cv::Mat
- H.264 (x264)
- HEVC (x265)
### Currently implemented decoders


### Particular implementations details
- The cvMatVideoEncoder does not require any compression quality or resolution. cv::Mat of different resolutions can be encoded in the same stream.
- The JPEGVideoEncoder does not require any resolution. Different frame resolutions can be encoded in the same stream.

### NAL Unit divider
According to standard RFC3984 a video stream should be divided in NAL Units. A NAL unit consists of a one-byte header and the payload byte string.  The header indicates the type of the NAL unit, the (potential) presence of bit errors or syntax violations in the NAL unit payload, and information regarding the relative importance of the NAL unit for the decoding process.

The NAL one-byte header is added automatically by the x264 and x265 libraries. However, for JPEGVideoEncoder and cvMatVideoEncoder, as they are not pure video encoders but single image encoders, the NAL header is added at each frame. The NAL header for JPEGVideoEncoder and cvMatVideoEncoder is always 0x00000001 (4 bytes).