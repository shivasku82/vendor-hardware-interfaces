#define LOG_TAG "RemoteCamUtils@3.4"
// #define LOG_NDEBUG 0
#include <log/log.h>

#include <libyuv.h>
#include <linux/videodev2.h>
#include <sys/mman.h>
#include <cmath>
#include <cstring>
#include <jpeglib.h>
#include <linux/v4l2-mediabus.h>
#include "RemoteCameraUtils.h"
#include "ExternalCameraUtils.h"
#include <aidlcommonsupport/NativeHandle.h>


namespace {
buffer_handle_t sEmptyBuffer = nullptr;
}  // Anonymous namespace

namespace android {
namespace hardware {
namespace camera {
namespace device {
namespace implementation {

Frame::Frame(uint32_t width, uint32_t height, uint32_t fourcc)
    : mWidth(width), mHeight(height), mFourcc(fourcc) {}

V4L2Frame::V4L2Frame(uint32_t w, uint32_t h, uint32_t fourcc, int bufIdx, int fd, uint32_t dataSize,
                     uint64_t offset)
    : Frame(w, h, fourcc) {
    mDataSize= dataSize;
    mBufferIndex = bufIdx;
    mFd = fd;
    mOffset = offset;
}

// int V4L2Frame::setData(uint8_t* data) {
//     mData = data;
//     mMapped = true;

//     return 0;
// }

// V4L2Frame::V4L2Frame(uint32_t w, uint32_t h, uint32_t fourcc) :
//         Frame(w, h, fourcc) {
//             mDataSize = w * (h + 1) * 2;
// }

int V4L2Frame::map(uint8_t** data, size_t* dataSize) {
    if (data == nullptr || dataSize == nullptr) {
        ALOGE("%s: V4L2 buffer map bad argument: data %p, dataSize %p", __FUNCTION__, data,
              dataSize);
        return -EINVAL;
    }

    std::lock_guard<std::mutex> lk(mLock);
    if (!mMapped) {
        void* addr = mmap(NULL, mDataSize, PROT_READ, MAP_SHARED, mFd, mOffset);
        if (addr == MAP_FAILED) {
            ALOGE("%s: V4L2 buffer map failed %s", __FUNCTION__, strerror(errno));
            return -EINVAL;
        }
        mData = static_cast<uint8_t*>(addr);
        mMapped = true;
    }
    *data = mData;
    *dataSize = mDataSize;
    ALOGI("%s: V4L map FD %d, data %p size %zu mOffset %d", __FUNCTION__, mFd, mData, mDataSize,
           (int)mOffset);
    return 0;
}

int V4L2Frame::unmap() {
    std::lock_guard<std::mutex> lk(mLock);
    mMapped = false;
    return 0;
}

V4L2Frame::~V4L2Frame() {
    ALOGI("%s %d", __FUNCTION__, __LINE__);
    if (mData != nullptr) {
        ALOGW("%s: delete v4l2Frame->mData.", __FUNCTION__);
        delete [] mData;
        mData = nullptr;
    }
}

int V4L2Frame::getData(uint8_t** outData, size_t* dataSize) {
    *outData = mData;
    *dataSize = mDataSize;
    return 0;
}


AllocatedFrame::AllocatedFrame(uint32_t w, uint32_t h) : Frame(w, h, V4L2_PIX_FMT_YUV420){};

AllocatedFrame::~AllocatedFrame() {}

int AllocatedFrame::allocate(YCbCrLayout* out) {
    std::lock_guard<std::mutex> lk(mLock);
    if ((mWidth % 2) || (mHeight % 2)) {
        ALOGE("%s: bad dimension %dx%d (not multiple of 2)", __FUNCTION__, mWidth, mHeight);
        return -EINVAL;
    }

    uint32_t dataSize = mWidth * mHeight * 3 / 2;  // YUV420
    if (mData.size() != dataSize) {
        mData.resize(dataSize);
    }

    if (out != nullptr) {
        out->y = mData.data();
        out->yStride = mWidth;
        uint8_t* cbStart = mData.data() + mWidth * mHeight;
        uint8_t* crStart = cbStart + mWidth * mHeight / 4;
        out->cb = cbStart;
        out->cr = crStart;
        out->cStride = mWidth / 2;
        out->chromaStep = 1;
    }
    return 0;
}

int AllocatedFrame::getData(uint8_t** outData, size_t* dataSize) {
    YCbCrLayout layout;
    int ret = allocate(&layout);
    if (ret != 0) {
        return ret;
    }
    *outData = mData.data();
    *dataSize = mBufferSize;
    return 0;
}

int AllocatedFrame::getLayout(YCbCrLayout* out) {
    IMapper::Rect noCrop = {0, 0, static_cast<int32_t>(mWidth), static_cast<int32_t>(mHeight)};
    return getCroppedLayout(noCrop, out);
}

int AllocatedFrame::getCroppedLayout(const IMapper::Rect& rect, YCbCrLayout* out) {
    if (out == nullptr) {
        ALOGE("%s: null out", __FUNCTION__);
        return -1;
    }

    std::lock_guard<std::mutex> lk(mLock);
    if ((rect.left + rect.width) > static_cast<int>(mWidth) ||
        (rect.top + rect.height) > static_cast<int>(mHeight) || (rect.left % 2) || (rect.top % 2) ||
        (rect.width % 2) || (rect.height % 2)) {
        ALOGE("%s: bad rect left %d top %d w %d h %d", __FUNCTION__, rect.left, rect.top,
              rect.width, rect.height);
        return -1;
    }

    out->y = mData.data() + mWidth * rect.top + rect.left;
    out->yStride = mWidth;
    uint8_t* cbStart = mData.data() + mWidth * mHeight;
    uint8_t* crStart = cbStart + mWidth * mHeight / 4;
    out->cb = cbStart + mWidth * rect.top / 4 + rect.left / 2;
    out->cr = crStart + mWidth * rect.top / 4 + rect.left / 2;
    out->cStride = mWidth / 2;
    out->chromaStep = 1;
    return 0;
}

bool isAspectRatioClose(float ar1, float ar2) {
    const float kAspectRatioMatchThres = 0.025f;  // This threshold is good enough to distinguish
                                                  // 4:3/16:9/20:9
                                                  // 1.33 / 1.78 / 2
    return (std::abs(ar1 - ar2) < kAspectRatioMatchThres);
}

/*double SupportedV4L2Format::FrameRate::getDouble() const {
    return durationDenominator / static_cast<double>(durationNumerator);
}*/

aidl::android::hardware::camera::common::Status importBufferImpl(
        /*inout*/ std::map<int, CirculatingBuffers>& circulatingBuffers,
        /*inout*/ HandleImporter& handleImporter, int32_t streamId, uint64_t bufId,
        buffer_handle_t buf,
        /*out*/ buffer_handle_t** outBufPtr, bool allowEmptyBuf) {
    using aidl::android::hardware::camera::common::Status;
    if (buf == nullptr && bufId == BUFFER_ID_NO_BUFFER) {
        if (allowEmptyBuf) {
            *outBufPtr = &sEmptyBuffer;
            return Status::OK;
        } else {
            ALOGE("%s: bufferId %" PRIu64 " has null buffer handle!", __FUNCTION__, bufId);
            return Status::ILLEGAL_ARGUMENT;
        }
    }

    CirculatingBuffers& cbs = circulatingBuffers[streamId];
    if (cbs.count(bufId) == 0) {
        if (buf == nullptr) {
            ALOGE("%s: bufferId %" PRIu64 " has null buffer handle!", __FUNCTION__, bufId);
            return Status::ILLEGAL_ARGUMENT;
        }
        // Register a newly seen buffer
        buffer_handle_t importedBuf = buf;
        handleImporter.importBuffer(importedBuf);
        if (importedBuf == nullptr) {
            ALOGE("%s: output buffer for stream %d is invalid!", __FUNCTION__, streamId);
            return Status::INTERNAL_ERROR;
        } else {
            cbs[bufId] = importedBuf;
        }
    }
    *outBufPtr = &cbs[bufId];
    return Status::OK;
}

uint32_t getFourCcFromLayout(const YCbCrLayout& layout) {
    intptr_t cb = reinterpret_cast<intptr_t>(layout.cb);
    intptr_t cr = reinterpret_cast<intptr_t>(layout.cr);
    if (std::abs(cb - cr) == 1 && layout.chromaStep == 2) {
        // Interleaved format
        if (layout.cb > layout.cr) {
            return V4L2_PIX_FMT_NV21;
        } else {
            return V4L2_PIX_FMT_NV12;
        }
    } else if (layout.chromaStep == 1) {
        // Planar format
        if (layout.cb > layout.cr) {
            return V4L2_PIX_FMT_YVU420;  // YV12
        } else {
            return V4L2_PIX_FMT_YUV420;  // YU12
        }
    } else {
        return FLEX_YUV_GENERIC;
    }
}

int getCropRect(CroppingType ct, const Size& inSize, const Size& outSize, IMapper::Rect* out) {
    if (out == nullptr) {
        ALOGE("%s: out is null", __FUNCTION__);
        return -1;
    }

    uint32_t inW = inSize.width;
    uint32_t inH = inSize.height;
    uint32_t outW = outSize.width;
    uint32_t outH = outSize.height;

    // Handle special case where aspect ratio is close to input but scaled
    // dimension is slightly larger than input
    float arIn = ASPECT_RATIO(inSize);
    float arOut = ASPECT_RATIO(outSize);
    if (isAspectRatioClose(arIn, arOut)) {
        out->left = 0;
        out->top = 0;
        out->width = inW;
        out->height = inH;
        return 0;
    }

    if (ct == VERTICAL) {
        uint64_t scaledOutH = static_cast<uint64_t>(outH) * inW / outW;
        if (scaledOutH > inH) {
            ALOGE("%s: Output size %dx%d cannot be vertically cropped from input size %dx%d",
                  __FUNCTION__, outW, outH, inW, inH);
            return -1;
        }
        scaledOutH = scaledOutH & ~0x1;  // make it multiple of 2

        out->left = 0;
        out->top = ((inH - scaledOutH) / 2) & ~0x1;
        out->width = inW;
        out->height = static_cast<int32_t>(scaledOutH);
        ALOGV("%s: crop %dx%d to %dx%d: top %d, scaledH %d", __FUNCTION__, inW, inH, outW, outH,
              out->top, static_cast<int32_t>(scaledOutH));
    } else {
        uint64_t scaledOutW = static_cast<uint64_t>(outW) * inH / outH;
        if (scaledOutW > inW) {
            ALOGE("%s: Output size %dx%d cannot be horizontally cropped from input size %dx%d",
                  __FUNCTION__, outW, outH, inW, inH);
            return -1;
        }
        scaledOutW = scaledOutW & ~0x1;  // make it multiple of 2

        out->left = ((inW - scaledOutW) / 2) & ~0x1;
        out->top = 0;
        out->width = static_cast<int32_t>(scaledOutW);
        out->height = inH;
        ALOGV("%s: crop %dx%d to %dx%d: top %d, scaledW %d", __FUNCTION__, inW, inH, outW, outH,
              out->top, static_cast<int32_t>(scaledOutW));
    }

    return 0;
}

int formatConvert(const YCbCrLayout& in, const YCbCrLayout& out, Size sz, uint32_t format) {
    int ret = 0;
    switch (format) {
        case V4L2_PIX_FMT_NV21:
            ret = libyuv::I420ToNV21(
                    static_cast<uint8_t*>(in.y), in.yStride, static_cast<uint8_t*>(in.cb),
                    in.cStride, static_cast<uint8_t*>(in.cr), in.cStride,
                    static_cast<uint8_t*>(out.y), out.yStride, static_cast<uint8_t*>(out.cr),
                    out.cStride, sz.width, sz.height);
            if (ret != 0) {
                ALOGE("%s: convert to NV21 buffer failed! ret %d", __FUNCTION__, ret);
                return ret;
            }
            break;
        case V4L2_PIX_FMT_NV12:
            ret = libyuv::I420ToNV12(
                    static_cast<uint8_t*>(in.y), in.yStride, static_cast<uint8_t*>(in.cb),
                    in.cStride, static_cast<uint8_t*>(in.cr), in.cStride,
                    static_cast<uint8_t*>(out.y), out.yStride, static_cast<uint8_t*>(out.cb),
                    out.cStride, sz.width, sz.height);
            if (ret != 0) {
                ALOGE("%s: convert to NV12 buffer failed! ret %d", __FUNCTION__, ret);
                return ret;
            }
            break;
        case V4L2_PIX_FMT_YVU420:  // YV12
        case V4L2_PIX_FMT_YUV420:  // YU12
            // TODO: maybe we can speed up here by somehow save this copy?
            ret = libyuv::I420Copy(
                    static_cast<uint8_t*>(in.y), in.yStride, static_cast<uint8_t*>(in.cb),
                    in.cStride, static_cast<uint8_t*>(in.cr), in.cStride,
                    static_cast<uint8_t*>(out.y), out.yStride, static_cast<uint8_t*>(out.cb),
                    out.cStride, static_cast<uint8_t*>(out.cr), out.cStride, sz.width, sz.height);
            if (ret != 0) {
                ALOGE("%s: copy to YV12 or YU12 buffer failed! ret %d", __FUNCTION__, ret);
                return ret;
            }
            break;
        case FLEX_YUV_GENERIC:
            // TODO: b/72261744 write to arbitrary flexible YUV layout. Slow.
            ALOGE("%s: unsupported flexible yuv layout"
                  " y %p cb %p cr %p y_str %d c_str %d c_step %d",
                  __FUNCTION__, out.y, out.cb, out.cr, out.yStride, out.cStride, out.chromaStep);
            return -1;
        default:
            ALOGE("%s: unknown YUV format 0x%x!", __FUNCTION__, format);
            return -1;
    }
    return 0;
}

int encodeJpegYU12(const Size& inSz, const YCbCrLayout& inLayout, int jpegQuality,
                   const void* app1Buffer, size_t app1Size, void* out, const size_t maxOutSize,
                   size_t& actualCodeSize) {
    /* libjpeg is a C library so we use C-style "inheritance" by
     * putting libjpeg's jpeg_destination_mgr first in our custom
     * struct. This allows us to cast jpeg_destination_mgr* to
     * CustomJpegDestMgr* when we get it passed to us in a callback */
    struct CustomJpegDestMgr {
        struct jpeg_destination_mgr mgr;
        JOCTET* mBuffer;
        size_t mBufferSize;
        size_t mEncodedSize;
        bool mSuccess;
    } dmgr;

    jpeg_compress_struct cinfo = {};
    jpeg_error_mgr jerr;

    /* Initialize error handling with standard callbacks, but
     * then override output_message (to print to ALOG) and
     * error_exit to set a flag and print a message instead
     * of killing the whole process */
    cinfo.err = jpeg_std_error(&jerr);

    cinfo.err->output_message = [](j_common_ptr cinfo) {
        char buffer[JMSG_LENGTH_MAX];

        /* Create the message */
        (*cinfo->err->format_message)(cinfo, buffer);
        ALOGE("libjpeg error: %s", buffer);
    };
    cinfo.err->error_exit = [](j_common_ptr cinfo) {
        (*cinfo->err->output_message)(cinfo);
        if (cinfo->client_data) {
            auto& dmgr = *reinterpret_cast<CustomJpegDestMgr*>(cinfo->client_data);
            dmgr.mSuccess = false;
        }
    };
    /* Now that we initialized some callbacks, let's create our compressor */
    jpeg_create_compress(&cinfo);

    /* Initialize our destination manager */
    dmgr.mBuffer = static_cast<JOCTET*>(out);
    dmgr.mBufferSize = maxOutSize;
    dmgr.mEncodedSize = 0;
    dmgr.mSuccess = true;
    cinfo.client_data = static_cast<void*>(&dmgr);

    /* These lambdas become C-style function pointers and as per C++11 spec
     * may not capture anything */
    dmgr.mgr.init_destination = [](j_compress_ptr cinfo) {
        auto& dmgr = reinterpret_cast<CustomJpegDestMgr&>(*cinfo->dest);
        dmgr.mgr.next_output_byte = dmgr.mBuffer;
        dmgr.mgr.free_in_buffer = dmgr.mBufferSize;
        ALOGV("%s:%d jpeg start: %p [%zu]", __FUNCTION__, __LINE__, dmgr.mBuffer, dmgr.mBufferSize);
    };

    dmgr.mgr.empty_output_buffer = [](j_compress_ptr cinfo __unused) {
        ALOGV("%s:%d Out of buffer", __FUNCTION__, __LINE__);
        return 0;
    };

    dmgr.mgr.term_destination = [](j_compress_ptr cinfo) {
        auto& dmgr = reinterpret_cast<CustomJpegDestMgr&>(*cinfo->dest);
        dmgr.mEncodedSize = dmgr.mBufferSize - dmgr.mgr.free_in_buffer;
        ALOGV("%s:%d Done with jpeg: %zu", __FUNCTION__, __LINE__, dmgr.mEncodedSize);
    };
    cinfo.dest = reinterpret_cast<struct jpeg_destination_mgr*>(&dmgr);

    /* We are going to be using JPEG in raw data mode, so we are passing
     * straight subsampled planar YCbCr and it will not touch our pixel
     * data or do any scaling or anything */
    cinfo.image_width = inSz.width;
    cinfo.image_height = inSz.height;
    cinfo.input_components = 3;
    cinfo.in_color_space = JCS_YCbCr;

    /* Initialize defaults and then override what we want */
    jpeg_set_defaults(&cinfo);

    jpeg_set_quality(&cinfo, jpegQuality, 1);
    jpeg_set_colorspace(&cinfo, JCS_YCbCr);
    cinfo.raw_data_in = 1;
    cinfo.dct_method = JDCT_IFAST;

    /* Configure sampling factors. The sampling factor is JPEG subsampling 420
     * because the source format is YUV420. Note that libjpeg sampling factors
     * are... a little weird. Sampling of Y=2,U=1,V=1 means there is 1 U and
     * 1 V value for each 2 Y values */
    cinfo.comp_info[0].h_samp_factor = 2;
    cinfo.comp_info[0].v_samp_factor = 2;
    cinfo.comp_info[1].h_samp_factor = 1;
    cinfo.comp_info[1].v_samp_factor = 1;
    cinfo.comp_info[2].h_samp_factor = 1;
    cinfo.comp_info[2].v_samp_factor = 1;

    /* Let's not hardcode YUV420 in 6 places... 5 was enough */
    int maxVSampFactor =
            std::max({cinfo.comp_info[0].v_samp_factor, cinfo.comp_info[1].v_samp_factor,
                      cinfo.comp_info[2].v_samp_factor});
    int cVSubSampling = cinfo.comp_info[0].v_samp_factor / cinfo.comp_info[1].v_samp_factor;

    /* Start the compressor */
    jpeg_start_compress(&cinfo, TRUE);

    /* Compute our macroblock height, so we can pad our input to be vertically
     * macroblock aligned.
     * TODO: Does it need to be horizontally MCU aligned too? */

    size_t mcuV = DCTSIZE * maxVSampFactor;
    size_t paddedHeight = mcuV * ((inSz.height + mcuV - 1) / mcuV);

    /* libjpeg uses arrays of row pointers, which makes it really easy to pad
     * data vertically (unfortunately doesn't help horizontally) */
    std::vector<JSAMPROW> yLines(paddedHeight);
    std::vector<JSAMPROW> cbLines(paddedHeight / cVSubSampling);
    std::vector<JSAMPROW> crLines(paddedHeight / cVSubSampling);

    uint8_t* py = static_cast<uint8_t*>(inLayout.y);
    uint8_t* pcr = static_cast<uint8_t*>(inLayout.cr);
    uint8_t* pcb = static_cast<uint8_t*>(inLayout.cb);

    for (uint32_t i = 0; i < paddedHeight; i++) {
        /* Once we are in the padding territory we still point to the last line
         * effectively replicating it several times ~ CLAMP_TO_EDGE */
        int li = -1;
        if(i < inSz.height - 1) {
            li = i;
        }
        else {
            li = inSz.height -1;
        }
        //int li = std::min(i, inSz.height - 1);
        yLines[i] = static_cast<JSAMPROW>(py + li * inLayout.yStride);
        if (i < paddedHeight / cVSubSampling) {
            if(i < ((inSz.height - 1) / cVSubSampling)) {
                li = i;
            }
            else {
                li = (inSz.height - 1) / cVSubSampling;
            }
            //li = std::min(i, (inSz.height - 1) / cVSubSampling);
            crLines[i] = static_cast<JSAMPROW>(pcr + li * inLayout.cStride);
            cbLines[i] = static_cast<JSAMPROW>(pcb + li * inLayout.cStride);
        }
    }

    /* If APP1 data was passed in, use it */
    if (app1Buffer && app1Size) {
        jpeg_write_marker(&cinfo, JPEG_APP0 + 1, static_cast<const JOCTET*>(app1Buffer), app1Size);
    }

    /* While we still have padded height left to go, keep giving it one
     * macroblock at a time. */
    while (cinfo.next_scanline < cinfo.image_height) {
        const uint32_t batchSize = DCTSIZE * maxVSampFactor;
        const uint32_t nl = cinfo.next_scanline;
        JSAMPARRAY planes[3]{&yLines[nl], &cbLines[nl / cVSubSampling],
                             &crLines[nl / cVSubSampling]};

        uint32_t done = jpeg_write_raw_data(&cinfo, planes, batchSize);

        if (done != batchSize) {
            ALOGE("%s: compressed %u lines, expected %u (total %u/%u)", __FUNCTION__, done,
                  batchSize, cinfo.next_scanline, cinfo.image_height);
            return -1;
        }
    }

    /* This will flush everything */
    jpeg_finish_compress(&cinfo);

    /* Grab the actual code size and set it */
    actualCodeSize = dmgr.mEncodedSize;

    return 0;
}

Size getMaxThumbnailResolution(const common::V1_0::helper::CameraMetadata& chars) {
    Size thumbSize{0, 0};
    camera_metadata_ro_entry entry = chars.find(ANDROID_JPEG_AVAILABLE_THUMBNAIL_SIZES);
    for (uint32_t i = 0; i < entry.count; i += 2) {
        Size sz{static_cast<int32_t>(entry.data.i32[i]),
                static_cast<int32_t>(entry.data.i32[i + 1])};
        if (sz.width * sz.height > thumbSize.width * thumbSize.height) {
            thumbSize = sz;
        }
    }

    if (thumbSize.width * thumbSize.height == 0) {
        ALOGW("%s: non-zero thumbnail size not available", __FUNCTION__);
    }

    return thumbSize;
}

void freeReleaseFences(std::vector<CaptureResult>& results) {
    //for (auto& result : results) {
        // if (result.inputBuffer.releaseFence.getNativeHandle() != nullptr) {
        //     native_handle_t* handle =
        //             const_cast<native_handle_t*>(result.inputBuffer.releaseFence.getNativeHandle());
        //     native_handle_close(handle);
        //     native_handle_delete(handle);
        // }
        // for (auto& buf : result.outputBuffers) {
        //     if (buf.releaseFence.getNativeHandle() != nullptr) {
        //         native_handle_t* handle =
        //                 const_cast<native_handle_t*>(buf.releaseFence.getNativeHandle());
        //         native_handle_close(handle);
        //         native_handle_delete(handle);
        //     }
        // }
    //}
    for (auto& result : results) {
        native_handle_t* inputReleaseFence =
                ::android::makeFromAidl(result.inputBuffer.releaseFence);
        if (inputReleaseFence != nullptr) {
            native_handle_close(inputReleaseFence);
            native_handle_delete(inputReleaseFence);
        }
        for (auto& buf : result.outputBuffers) {
            native_handle_t* outReleaseFence = ::android::makeFromAidl(buf.releaseFence);
            if (outReleaseFence != nullptr) {
                native_handle_close(outReleaseFence);
                native_handle_delete(outReleaseFence);
            }
        }
    }
    return;
}

#define ARRAY_SIZE(a) (sizeof(a) / sizeof(a[0]))
#define UPDATE(md, tag, data, size)               \
    do {                                          \
        if ((md).update((tag), (data), (size))) { \
            ALOGE("Update " #tag " failed!");     \
            return BAD_VALUE;                     \
        }                                         \
    } while (0)

status_t fillCaptureResultCommon(common::V1_0::helper::CameraMetadata& md, nsecs_t timestamp,
                                 camera_metadata_ro_entry& activeArraySize) {
    if (activeArraySize.count < 4) {
        ALOGE("%s: cannot find active array size!", __FUNCTION__);
        return -EINVAL;
    }
    // android.control
    // For USB camera, we don't know the AE state. Set the state to converged to
    // indicate the frame should be good to use. Then apps don't have to wait the
    // AE state.
    const uint8_t aeState = ANDROID_CONTROL_AE_STATE_CONVERGED;
    UPDATE(md, ANDROID_CONTROL_AE_STATE, &aeState, 1);

    const uint8_t ae_lock = ANDROID_CONTROL_AE_LOCK_OFF;
    UPDATE(md, ANDROID_CONTROL_AE_LOCK, &ae_lock, 1);

    // Set AWB state to converged to indicate the frame should be good to use.
    const uint8_t awbState = ANDROID_CONTROL_AWB_STATE_CONVERGED;
    UPDATE(md, ANDROID_CONTROL_AWB_STATE, &awbState, 1);

    const uint8_t awbLock = ANDROID_CONTROL_AWB_LOCK_OFF;
    UPDATE(md, ANDROID_CONTROL_AWB_LOCK, &awbLock, 1);

    const uint8_t flashState = ANDROID_FLASH_STATE_UNAVAILABLE;
    UPDATE(md, ANDROID_FLASH_STATE, &flashState, 1);

    // This means pipeline latency of X frame intervals. The maximum number is 4.
    const uint8_t requestPipelineMaxDepth = 4;
    UPDATE(md, ANDROID_REQUEST_PIPELINE_DEPTH, &requestPipelineMaxDepth, 1);

    // android.scaler
    const int32_t crop_region[] = {
            activeArraySize.data.i32[0],
            activeArraySize.data.i32[1],
            activeArraySize.data.i32[2],
            activeArraySize.data.i32[3],
    };
    UPDATE(md, ANDROID_SCALER_CROP_REGION, crop_region, ARRAY_SIZE(crop_region));

    // android.sensor
    UPDATE(md, ANDROID_SENSOR_TIMESTAMP, &timestamp, 1);

    // android.statistics
    const uint8_t lensShadingMapMode = ANDROID_STATISTICS_LENS_SHADING_MAP_MODE_OFF;
    UPDATE(md, ANDROID_STATISTICS_LENS_SHADING_MAP_MODE, &lensShadingMapMode, 1);

    const uint8_t sceneFlicker = ANDROID_STATISTICS_SCENE_FLICKER_NONE;
    UPDATE(md, ANDROID_STATISTICS_SCENE_FLICKER, &sceneFlicker, 1);

    return OK;
}

using namespace tinyxml2;
int RemoteCameraProviderConfig::loadDeviceCfg(XMLElement* deviceCfg) {
    int ret = 0;
    while (deviceCfg != nullptr) {
        RemoteCameraConfig* cfg = new RemoteCameraConfig();
        if (cfg == nullptr) {
            ALOGE("%s: Fatal error, cannot create RemoteCameraConfig.", __FUNCTION__);
            ret = -1;
            break;
        }
        XMLElement* path = deviceCfg->FirstChildElement("Path");
        if (path == nullptr) {
            ALOGE("%s: invalid driver name", __FUNCTION__);
            ret = -1;
            break;
        }
        cfg->path = path->GetText();

        XMLElement* driver = deviceCfg->FirstChildElement("Driver");
        if (driver == nullptr) {
            ALOGE("%s: invalid driver name", __FUNCTION__);
            ret = -1;
            break;
        }
        cfg->driver = driver->GetText();

        XMLElement* numVideoBuf = deviceCfg->FirstChildElement("NumVideoBuffers");
        if (numVideoBuf == nullptr) {
            ALOGI("%s: no num video buffers specified", __FUNCTION__);
        } else {
            cfg->numVideoBuffers = numVideoBuf->UnsignedAttribute("count", 2);
        }
        XMLElement* numStillBuf = deviceCfg->FirstChildElement("NumStillBuffers");
        if (numStillBuf == nullptr) {
            ALOGI("%s: no num still buffers specified", __FUNCTION__);
        } else {
            cfg->numStillBuffers = numStillBuf->UnsignedAttribute("count", /*Default*/ 2);
        }

        XMLElement* settings = deviceCfg->FirstChildElement("Setting");
        if (settings == nullptr) {
            ALOGI("%s: no setting for the device.", __FUNCTION__);
        } else {
            cfg->width = settings->UnsignedAttribute("width", DEFAULT_WIDTH);
            cfg->height = settings->UnsignedAttribute("height", DEFAULT_HEIGHT);
            cfg->fps = settings->UnsignedAttribute("fps", DEFAULT_FPS);
            cfg->format = settings->Attribute("format");
        }

        // Bindings;
        XMLElement* bind = deviceCfg->FirstChildElement("Bind");
        if (bind == nullptr) {
            ALOGE("%s fatal error: no binding defined in config file", __FUNCTION__);
            ret = -1;
            break;
        }
        XMLElement* virtId = bind->FirstChildElement("VirtId");
        while (virtId != nullptr) {
            std::string id = virtId->GetText();
            cfg->virtIds.push_back(id);
            virtId = virtId->NextSiblingElement("VirtId");
        }

        XMLElement* subdev = deviceCfg->FirstChildElement("Subdev");
        if (subdev != nullptr) {
            cfg->subdev = subdev->GetText();
            ALOGI("%s: create subdev %s.", __FUNCTION__, cfg->subdev.c_str());

            const char* irAttribute = subdev->Attribute("IR");
            if (irAttribute == nullptr) {
                cfg->irMode = UNKNOWN_MODE;
            } else {
                std::string m = std::string(irAttribute);
                if (m == "AdaptiveNightMode") {
                    cfg->irMode = ADAPTIVE_NIGHT_MODE;
                } else if (m == "HostControlDayMode") {
                    cfg->irMode = HOSTCONTROL_DAY_MODE;
                } else if (m == "HostControlNightMode") {
                    cfg->irMode = HOSTCONTROL_NIGHT_MODE;
                } else if (m == "AdaptiveDayMode") {
                    cfg->irMode = ADAPTIVE_DAY_MODE;
                } else {
                    cfg->irMode = UNKNOWN_MODE;
                }
            }
        }

        mCameraConfigs.insert(cfg);
        deviceCfg = deviceCfg->NextSiblingElement("Device");
    }

    return ret;
}

int RemoteCameraProviderConfig::loadLoopbackCfg(XMLElement* loopbackCfg) {
    int ret = 0;
    if (loopbackCfg != nullptr) {
        XMLElement* pairId = loopbackCfg->FirstChildElement("PairId");
        if (pairId == nullptr) {
            ALOGI("%s error: Not confiugred loopback, use default id 50", __FUNCTION__);
            mLoopbackPairId = "50";
        } else {
            mLoopbackPairId = pairId->GetText();
            XMLElement* srcCamera = loopbackCfg->FirstChildElement("SrcCamera");
            if (srcCamera == nullptr) {
                ALOGE("%s error: Not confiugred loopback src camera, use default video0",
                      __FUNCTION__);
                mLoopbackSrcCamera = "/dev/video0";
            } else {
                mLoopbackSrcCamera = srcCamera->GetText();
            }
            // loopbackConfig
            XMLElement* driver = loopbackCfg->FirstChildElement("Driver");
            if (driver == nullptr) {
                ALOGE("%s error: Not confiugred loopback, use default driver unknown",
                      __FUNCTION__);
                mLoopbackConfig.driver = "unknown";
            } else {
                mLoopbackConfig.driver = driver->GetText();
            }
            XMLElement* settings = loopbackCfg->FirstChildElement("Setting");
            if (settings == nullptr) {
                ALOGI("%s: no setting for the device.", __FUNCTION__);
                mLoopbackConfig.width = 1920;
                mLoopbackConfig.height = 1080;
                mLoopbackConfig.fps = 30;
                mLoopbackConfig.format = "UYVY";
            } else {
                mLoopbackConfig.width = settings->UnsignedAttribute("width", 1920);
                mLoopbackConfig.height = settings->UnsignedAttribute("height", 1080);
                mLoopbackConfig.fps = settings->UnsignedAttribute("fps", 30);
                mLoopbackConfig.format = settings->Attribute("format");
            }
            XMLElement* irmode = loopbackCfg->FirstChildElement("IRMode");
            if (irmode == nullptr) {
                mLoopbackConfig.irMode = ADAPTIVE_DAY_MODE;
            } else {
                std::string m = irmode->GetText();
                if (m == "AdaptiveDayMode") {
                    mLoopbackConfig.irMode = ADAPTIVE_DAY_MODE;
                } else if (m == "AdaptiveNightMode") {
                    mLoopbackConfig.irMode = ADAPTIVE_NIGHT_MODE;
                } else if (m == "HostControlDayMode") {
                    mLoopbackConfig.irMode = HOSTCONTROL_DAY_MODE;
                } else if (m == "HostControlNightMode") {
                    mLoopbackConfig.irMode = HOSTCONTROL_NIGHT_MODE;
                } else {
                    mLoopbackConfig.irMode = UNKNOWN_MODE;
                }
            }
        }
    }
    return ret;
}

int RemoteCameraProviderConfig::loadMediaCtrl(XMLElement* mediaCtrlCfg)
{
    if (mediaCtrlCfg == nullptr) {
        ALOGE("%s: error of null mediaCtrlCfg", __FUNCTION__);
        return -1;
    }

    //MediaControl format
    mMCFormats.clear();
    XMLElement* fmtCfg = mediaCtrlCfg->FirstChildElement("format");
    while (fmtCfg != nullptr) {
        McFormat fmt = {};
        fmt.entityName = fmtCfg->Attribute("name");
        fmt.pad = fmtCfg->UnsignedAttribute("pad");
        fmt.width = fmtCfg->UnsignedAttribute("width", 1920);
        fmt.height = fmtCfg->UnsignedAttribute("height", 1080);
        std::string pixelFmt = fmtCfg->Attribute("format");

        if (pixelFmt == "V4L2_MBUS_FMT_UYVY8_1X16") 
            fmt.pixelCode = V4L2_MBUS_FMT_UYVY8_1X16;
        else if (pixelFmt == "V4L2_MBUS_FMT_RGB888_1X24")
            fmt.pixelCode = V4L2_MBUS_FMT_RGB888_1X24;
        else {
            ALOGE("%s: no support fmt!", __FUNCTION__);
            return -1;
        }

        ALOGI("%s: entityName=%s", __FUNCTION__, fmt.entityName.c_str());
        mMCFormats.push_back(fmt);
        fmtCfg = fmtCfg->NextSiblingElement("format");
    }
    ALOGI("%s: %d fmtCfgs", __FUNCTION__, (int)mMCFormats.size());

    //MediaControl links
    mMCLinks.clear();
    XMLElement* linkCfg = mediaCtrlCfg->FirstChildElement("link");
    while (linkCfg != nullptr) {
        McLink link = {};
        link.srcEntityName = linkCfg->Attribute("srcName");
        link.srcPad = linkCfg->UnsignedAttribute("srcPad");
        link.sinkEntityName = linkCfg->Attribute("sinkName");
        link.sinkPad = linkCfg->UnsignedAttribute("sinkPad");

        mMCLinks.push_back(link);
        linkCfg = linkCfg->NextSiblingElement("link");
    }

    return 0;
}

void RemoteCameraProviderConfig::loadFromCfg(const char* cfgPath) {
    std::string platform;

    if (cfgPath == nullptr) {
        ALOGE("%s: no available ivi_camera_config.xml.", __FUNCTION__);
        return;
    }

    ALOGI("%s: ivicamera provider camera path %s", __FUNCTION__, cfgPath);
    XMLDocument configXml;
    XMLError err = configXml.LoadFile(cfgPath);
    if (err != XML_SUCCESS) {
        ALOGE("%s: Unable to load external camera config file '%s'. Error: %s", __FUNCTION__,
              cfgPath, XMLDocument::ErrorIDToName(err));
        return;
    }

    XMLElement* iviCam = configXml.FirstChildElement("IVICamera");
    if (iviCam == nullptr) {
        ALOGE("%s: no external camera config specified", __FUNCTION__);
        return;
    }
    platform = iviCam->Attribute("platform");

    XMLElement* mediactrlCfg = iviCam->FirstChildElement("MediaCtlConfig");
    while (mediactrlCfg != nullptr) {
        if (mediactrlCfg->Attribute("platform") != nullptr
        && mediactrlCfg->Attribute("platform") == platform) {
            ALOGI("%s: load mediacontrol for platform %s", __FUNCTION__, platform.c_str());
            if (loadMediaCtrl(mediactrlCfg) != 0) {
                ALOGE("%s: Failed to load mediacontrol config.", __FUNCTION__);
            }
            break;
        }
        mediactrlCfg = mediactrlCfg->NextSiblingElement("MediaCtlConfig");
    }

    XMLElement* providerCfg = iviCam->FirstChildElement("Provider");
    if (providerCfg == nullptr) {
        ALOGI("%s: no external camera provider config specified", __FUNCTION__);
        return;
    }

    XMLElement* offset = providerCfg->FirstChildElement("CameraIdOffset");
    if (offset != nullptr) {
        mCameraIdOffset = std::atoi(offset->GetText());
    }

    XMLElement* ignore = providerCfg->FirstChildElement("ignore");
    if (ignore == nullptr) {
        ALOGI("%s: no internal ignored device specified", __FUNCTION__);
        return;
    }

    XMLElement* id = ignore->FirstChildElement("id");
    while (id != nullptr) {
        const char* text = id->GetText();
        std::string ignoreDevice = "video";
        if (text != nullptr) {
            ignoreDevice += text;
            mIgnoreDevices.insert(ignoreDevice);
            ALOGI("%s: device %s will be ignored by external camera provider, %s ignored.",
                  __FUNCTION__, text, ignoreDevice.c_str());
        }
        id = id->NextSiblingElement("id");
    }

    // Get device config.
    XMLElement* deviceCfg = iviCam->FirstChildElement("Device");
    if (loadDeviceCfg(deviceCfg) != 0) {
        ALOGE("%s: Failed to load device config in xml.", __FUNCTION__);
        return;
    }

    // Get loopback config.
    XMLElement* loopbackCfg = iviCam->FirstChildElement("Loopback");
    if (loadLoopbackCfg(loopbackCfg) != 0) {
        ALOGE("%s: Failed to load loopback config.", __FUNCTION__);
    }

    // Print IVICameraProviderConfig
    printCfg();
    return;
}

void RemoteCameraProviderConfig::printCfg() {
    ALOGD("===========================");
    ALOGD("IVICameraProviderConfig:");
    ALOGD("CameraIdOffset=%d", mCameraIdOffset);
    ALOGD("-------------------------");
    ALOGD("loopbackPairId=%s", mLoopbackPairId.c_str());
    ALOGD("loopbackSrcCamera=%s", mLoopbackSrcCamera.c_str());
    ALOGD("Loopback %s setting: %dx%d@%d with %s", mLoopbackConfig.driver.c_str(),
          mLoopbackConfig.width, mLoopbackConfig.height, mLoopbackConfig.fps,
          mLoopbackConfig.format.c_str());
    ALOGD("===========================");
}

#undef ARRAY_SIZE
#undef UPDATE

}  // namespace implementation
}  // namespace device

}  // namespace camera
}  // namespace hardware
}  // namespace android
