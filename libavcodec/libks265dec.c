/*
 * HEVC video Decoder from qianyi
 *
 * This file is part of FFmpeg.
 *
 * FFmpeg is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * FFmpeg is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with FFmpeg; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

#include "libavutil/attributes.h"
#include "libavutil/common.h"
#include "libavutil/display.h"
#include "libavutil/internal.h"
#include "libavutil/md5.h"
#include "libavutil/opt.h"
#include "libavutil/pixdesc.h"
#include "libavutil/stereo3d.h"

#include "bswapdsp.h"
#include "bytestream.h"
#include "cabac_functions.h"
#include "golomb.h"

#ifndef WIN32
#include <unistd.h>
#include <pthread.h>
#include <semaphore.h>
#include <dlfcn.h>
#endif

#include "hevc.h"


#include "libavutil/buffer.h"

#include "avcodec.h"
#include "bswapdsp.h"
#include "get_bits.h"
#include "internal.h"
#include "thread.h"
#include "videodsp.h"

#include "ks265dec.h"

#include "stdatomic.h"
#include <sched.h>
//#include "qyauth_env.h"

#define MAX_KS265_CORE 8
#define MIN_KS265_CORE 0



typedef struct  KS265Context {

    const AVClass   *c;  // needed by private avoptions

    unsigned        reserved[4];        // reserved fields for kwaiplayer
                                        // [0]: error code of each decode
                                        // [1]: unused
                                        // [2]: unused
                                        // [3]: unused

    void*           m_decoder;
    KS265DecConfig* params;
    char*           ks265Params;
    int             is_nalff;           ///< this flag is != 0 if bitstream is encapsulated
                                        ///< as a format defined in 14496-15
    int             nal_length_size;    ///< Number of bytes used for nal length (1, 2 or 4)
    int             ksdec_threads;      ///< decoder threads
    int             outputPixfmt;
    int             queueDecode;
    int             supportSlices;
    KS265Frame      m_ksDecFrame;
    int             b_use_pthread;
    int             b_alpha_decode;

    enum AVPixelFormat pix_fmt;         // pix_fmt set to avctx

    atomic_bool     b_has_been_freed;
    atomic_bool     decoder_lock;

    int             b_zerobuffer_contains_ref;  // flag for detect zero buffer in sps when stream contains B/P frame
} KS265Context;



#define KSVIDEO_ENABLE_LOGO_FLAG 0x1000
static int detect_start_code(uint8_t* p,int len){
    if(len<=3){
        return 0;
    }
    if((p[0]==0 && p[1] == 0 && p[2]==0 && p[3]==1)
       ||(p[0]==0 && p[1] == 0 && p[2]==1)){
        return 1;
    }
    return 0;
}

static int isSlice(enum NAL_UNIT_TYPE type) {
    return (type >= NAL_UNIT_TYPE_TRAIL_N) &&
           (type <= NAL_UNIT_TYPE_CRA_NUT);
}

static int isIntra(enum NAL_UNIT_TYPE type) {
    return (type >= NAL_UNIT_TYPE_BLA_W_LP) &&
           (type <= NAL_UNIT_TYPE_CRA_NUT);
}
static int isNoRef(enum NAL_UNIT_TYPE type) {
    return (type == NAL_UNIT_TYPE_TRAIL_N) ||
           (type == NAL_UNIT_TYPE_TSA_N)||
           (type == NAL_UNIT_TYPE_STSA_N)||
           (type == NAL_UNIT_TYPE_RADL_N)||
           (type == NAL_UNIT_TYPE_RASL_N);
}
// check this frame needs to skip by its naltype and temporal id
static int checkSkip(AVCodecContext *avctx, enum NAL_UNIT_TYPE nuh_type){
    if (avctx == NULL || nuh_type == 0xFF) {
        return 0;
    }

    if( nuh_type >= NAL_UNIT_TYPE_VPS_NUT &&
        nuh_type <= NAL_UNIT_TYPE_PPS_NUT){
        return 0;
    }
    //uint8_t temporal_id = (pData[1]&0x07) -1;  TODO
    KS265Context *s = avctx->priv_data;
    if ((avctx->skip_frame >= AVDISCARD_NONREF   && isNoRef(nuh_type))||
        (avctx->skip_frame >= AVDISCARD_BIDIR    && isNoRef(nuh_type))||
        (avctx->skip_frame >= AVDISCARD_NONINTRA && (!isIntra(nuh_type) ))||
        (avctx->skip_frame >= AVDISCARD_NONKEY   && (!isIntra(nuh_type) ))||
        (avctx->skip_frame >= AVDISCARD_ALL)) {
        return 1;
    }
    //s->m_iLastTid = UINT8_MAX; TODO
    return 0;
}


static int hevc_decode_frame(AVCodecContext *avctx, void *data, int *got_output,
                             AVPacket *avpkt)
{
    int ret = 0, enableLogo = 0;
    KS265Context *s = avctx->priv_data;

    if (atomic_load(&s->b_has_been_freed)) {
        return 0;
    }

    // get lock first
    while (atomic_exchange(&s->decoder_lock, 1)) {
        sched_yield();
    }

    // if decoder is free, return
    if (atomic_load(&s->b_has_been_freed)) {
        atomic_exchange(&s->decoder_lock, 0);
        return 0;
    }

    AVFrame * pOutFrame = data;
    if ( avpkt && avpkt->flags == AV_PKT_FLAG_CORRUPT ) {
        ret = KS265DecodeFlush( s->m_decoder, 1);
        atomic_exchange(&s->decoder_lock, 0);
        return 0;
    }

    if((avpkt->flags & KSVIDEO_ENABLE_LOGO_FLAG) != 0)
        enableLogo = 1;

    uint8_t * packet_buf= avpkt->data;
    int32_t packet_size = avpkt->size;
    enum NAL_UNIT_TYPE nuh_type = 0xFF;
    while ( packet_size > 0) {
        int32_t i, nal_length = 0;
        enum NAL_UNIT_TYPE nuh_type_tmp;
        if( s->is_nalff ) {
            // parse nal_length at the begin
            for(i = 0; i < s->nal_length_size; i++) {
                nal_length = (nal_length << 8) | packet_buf[i];
            }
            if (nal_length == 1) { // length = [0,0,0,1], should be start code
                s->is_nalff = 0;
                av_log(avctx, AV_LOG_ERROR, "length = 1, try startcode");
                continue;
            }
            if ( nal_length <= 0 ) {
                av_log(avctx, AV_LOG_ERROR, "pkg incomplete");
                atomic_exchange(&s->decoder_lock, 0);
                return -1;
            }
            // replace nal_length with start code 0x00000001
            *(uint32_t*)packet_buf = 0x01000000;

            // skip nal_length
            packet_buf += s->nal_length_size;
            packet_size -= s->nal_length_size;
            if ( packet_size < nal_length ) {
                av_log(avctx, AV_LOG_ERROR, "pkg incomplete");
                break;
            }

            nuh_type_tmp = ((packet_buf[0] & 0x7E) >> 1);
            if (isSlice(nuh_type_tmp))
                nuh_type = nuh_type_tmp;
        }
        else {
            // raw bitstream
            nal_length = packet_size;
        }

        packet_buf += nal_length;
        packet_size -= nal_length;
    }

    if ( avpkt->size > 0 ) {
        int64_t pts = avpkt->pts == AV_NOPTS_VALUE  ? avpkt->dts : avpkt->pts;
        if ( !s->is_nalff || !checkSkip(avctx, nuh_type) ) {
            ret = KS265DecodeFrame( s->m_decoder, avpkt->data, avpkt->size, pts);
            s->reserved[0] = ret;
            if (ret != 0 ){
                avctx->decoder_errors++;
                av_log(avctx, AV_LOG_ERROR, "KS265DecodeFrame decode error (%d) %d\n", ret, avpkt->size);
            }
        }
    }


    if ( avpkt && avpkt->size == 0 ) {
        ret = KS265DecodeFlush( s->m_decoder, 0);
    }


    ret = KS265DecoderGetDecodedFrame( s->m_decoder, &s->m_ksDecFrame, enableLogo);
    if (ret < 0 ){
        av_log(avctx, AV_LOG_ERROR, "get frame err %d\n", ret);
        atomic_exchange(&s->decoder_lock, 0);
        return -1;
    }
    if (s->m_ksDecFrame.bValid == 0) {  // have output frame;
        atomic_exchange(&s->decoder_lock, 0);
        return avpkt->size;
    }

    if (s->m_ksDecFrame.pData[1] == NULL && s->m_ksDecFrame.pData[2] == NULL &&
        s->m_ksDecFrame.iStride[1] == 0 && s->m_ksDecFrame.iStride[2] == 0) {
        s->pix_fmt = AV_PIX_FMT_GRAY8;
    }

    pOutFrame->format = s->pix_fmt;
    pOutFrame->width  = s->m_ksDecFrame.frameinfo.nWidth;
    pOutFrame->height = s->m_ksDecFrame.frameinfo.nHeight;
    pOutFrame->key_frame = s->m_ksDecFrame.frameinfo.key_frame;

    pOutFrame->color_range     = avctx->color_range;
    pOutFrame->colorspace      = avctx->colorspace;
    pOutFrame->color_primaries = avctx->color_primaries;
    pOutFrame->color_trc       = avctx->color_trc;

    avctx->pix_fmt = s->pix_fmt;
    avctx->width  = avctx->coded_width  = s->m_ksDecFrame.frameinfo.nWidth;
    avctx->height = avctx->coded_height = s->m_ksDecFrame.frameinfo.nHeight;

    {
        ret = ff_get_buffer(avctx, pOutFrame, 0);
        if (ret < 0) {
            av_log(avctx, AV_LOG_ERROR, "ff_get_buffer err(%x) in %s\n", ret, __func__);
            atomic_exchange(&s->decoder_lock, 0);
            return ret;
        }

        const int comp = pOutFrame->format == AV_PIX_FMT_GRAY8 ? 1 : 3;
        for( int i = 0; i < comp; i++) {
            int height = s->m_ksDecFrame.frameinfo.nHeight / (i==0?1:2);
            int width  = s->m_ksDecFrame.frameinfo.nWidth  / (i==0?1:2);
            uint8_t *pDst = pOutFrame->data[i];
            uint8_t *pSrc = s->m_ksDecFrame.pData[i];
            int iDstride  = pOutFrame->linesize[i];
            int iSstride  = s->m_ksDecFrame.iStride[i];
            for( int j = 0; j < height; j++ ) {
                memcpy( pDst, pSrc,  width);
                pDst += iDstride;
                pSrc += iSstride;
            }
        }
    }


    pOutFrame->pkt_pts =
    pOutFrame->pts = s->m_ksDecFrame.frameinfo.pts;

    KS265DecoderReturnDecodedFrame(s->m_decoder, &s->m_ksDecFrame);

    *got_output = 1;

    atomic_exchange(&s->decoder_lock, 0);
    return avpkt->size;
}



static av_cold int hevc_decode_free(AVCodecContext *avctx)
{
    KS265Context*   s = avctx->priv_data;

    if (atomic_load(&s->b_has_been_freed)) {
        return 0;
    }

    while (atomic_exchange(&s->decoder_lock, 1)) {
        sched_yield();
    }

    if (atomic_load(&s->b_has_been_freed)) {
        atomic_exchange(&s->decoder_lock, 0);
        return 0;
    }

    if (s->m_decoder) {
        KS265DecoderDestroy(s->m_decoder);
    }

    s->m_decoder = NULL;
    av_free(s->params);
    atomic_exchange(&s->b_has_been_freed, 1);
    atomic_exchange(&s->decoder_lock, 0);

    return 0;
}



static av_cold int ks265_init_context(AVCodecContext *avctx) {

    KS265Context *s = avctx->priv_data;

    s->pix_fmt              =  (s->outputPixfmt == KS265_PIXFMT_YUV_420SP_VU) ? AV_PIX_FMT_NV21 :
                              ((s->outputPixfmt == KS265_PIXFMT_YUV_420SP)    ? AV_PIX_FMT_NV12 : AV_PIX_FMT_YUV420P);
    avctx->color_primaries  = AVCOL_PRI_UNSPECIFIED;
    avctx->color_trc        = AVCOL_TRC_UNSPECIFIED;
    avctx->colorspace       = AVCOL_SPC_UNSPECIFIED;
    avctx->color_range      = AVCOL_RANGE_MPEG;

    avctx->sample_aspect_ratio.num = 1;
    avctx->sample_aspect_ratio.den = 1;

    s->m_ksDecFrame.bValid = 0;
    s->b_has_been_freed = 0;
    s->decoder_lock = 0;

    memset(s->reserved, 0, sizeof(s->reserved));

    return 0;
}





static int ks265_decode_extradata(AVCodecContext *avctx)
{
    KS265Context *s = avctx->priv_data;
    GetByteContext gb;
    int ret;

    bytestream2_init(&gb, avctx->extradata, avctx->extradata_size);

    if (avctx->extradata_size > 3 &&
        (avctx->extradata[0] ||
         avctx->extradata[1] ||
         avctx->extradata[2] > 1)) {
        int i, j, num_arrays, nal_len_size;
        s->is_nalff = 1;

        bytestream2_skip(&gb, 21);
        nal_len_size = (bytestream2_get_byte(&gb) & 3) + 1;
        num_arrays   = bytestream2_get_byte(&gb);

        /* nal units in the hvcC always have length coded with 2 bytes,
         * so put a fake nal_length_size = 2 while parsing them */
        s->nal_length_size = 2;

        /* Decode nal units from hvcC. */
        for (i = 0; i < num_arrays; i++) {
            int type = bytestream2_get_byte(&gb) & 0x3f;
            int cnt  = bytestream2_get_be16(&gb);
            // TODO: SEI message decoding (we temporally disable SEI decoding)
            // We can use an extra NAL（like NAL 31 in our H.264 live streaming）for SEI.
            // SEI can be decoded at the end of current frame.
            if (type != HEVC_NAL_VPS && type != HEVC_NAL_SPS && type != HEVC_NAL_PPS)
                continue;
            for (j = 0; j < cnt; j++) {
                // +2 for the nal size field
                int nalsize = bytestream2_get_be16(&gb);
                if (bytestream2_get_bytes_left(&gb) < nalsize) {
                    av_log(avctx, AV_LOG_ERROR,
                           "Invalid NAL unit size in extradata.\n");
                    return AVERROR_INVALIDDATA;
                }

                // av_log(avctx, AV_LOG_WARNING, "get nalsize = [%d] before KS265DecodeFrame\n", nalsize);
                ret = KS265DecodeFrame( s->m_decoder, (unsigned char*)gb.buffer, nalsize,0);
                if (ret != 0) {
                    av_log(avctx, AV_LOG_ERROR,
                           "Decoding extradata %d %d from hvcC failed %x\n",
                           type, i, ret);
                    return -1;
                }
                bytestream2_skip(&gb, nalsize);
            }
        }
        s->nal_length_size = nal_len_size;
    }
    else if ( avctx->extradata_size > 3 ) {
        s->is_nalff = 0;

        ret = KS265DecodeFrame( s->m_decoder, avctx->extradata, avctx->extradata_size,0);
        if (ret != 0)
            return -1;
    }
    else {
        s->is_nalff = 1;
        s->nal_length_size = 4;
    }
    return 0;
}

static int vui_pixfmt2pixfmt(KS265Context *ctx, int luma_bit_depth, int chroma_fmt_idc)
{
    int pix_fmt;

    switch (luma_bit_depth) {
        case 8:
            if (chroma_fmt_idc == 0) pix_fmt = AV_PIX_FMT_GRAY8;
            if (chroma_fmt_idc == 1)
            {
                if (ctx->outputPixfmt == KS265_PIXFMT_YUV_420SP)
                    pix_fmt = AV_PIX_FMT_NV12;
                else if (ctx->outputPixfmt == KS265_PIXFMT_YUV_420SP_VU)
                    pix_fmt = AV_PIX_FMT_NV21;
                else
                    pix_fmt = AV_PIX_FMT_YUV420P;
            }
            if (chroma_fmt_idc == 2) pix_fmt = AV_PIX_FMT_YUV422P;
            if (chroma_fmt_idc == 3) pix_fmt = AV_PIX_FMT_YUV444P;
           break;
        case 9:
            if (chroma_fmt_idc == 0) pix_fmt = AV_PIX_FMT_GRAY16;
            if (chroma_fmt_idc == 1) pix_fmt = AV_PIX_FMT_YUV420P9;
            if (chroma_fmt_idc == 2) pix_fmt = AV_PIX_FMT_YUV422P9;
            if (chroma_fmt_idc == 3) pix_fmt = AV_PIX_FMT_YUV444P9;
            break;
        case 10:
            if (chroma_fmt_idc == 0) pix_fmt = AV_PIX_FMT_GRAY16;
            if (chroma_fmt_idc == 1) pix_fmt = AV_PIX_FMT_YUV420P10;
            if (chroma_fmt_idc == 2) pix_fmt = AV_PIX_FMT_YUV422P10;
            if (chroma_fmt_idc == 3) pix_fmt = AV_PIX_FMT_YUV444P10;
            break;
        case 12:
            if (chroma_fmt_idc == 0) pix_fmt = AV_PIX_FMT_GRAY16;
            if (chroma_fmt_idc == 1) pix_fmt = AV_PIX_FMT_YUV420P12;
            if (chroma_fmt_idc == 2) pix_fmt = AV_PIX_FMT_YUV422P12;
            if (chroma_fmt_idc == 3) pix_fmt = AV_PIX_FMT_YUV444P12;
            break;
        default:
            return AVERROR_INVALIDDATA;
    }

    return pix_fmt;
}



static int export_stream_params(AVCodecContext *avctx)
{
#if 1
    int ret;

    unsigned int num = 0, den = 0;

    KS265Context *decoder = (KS265Context*)avctx->priv_data;
    {
        vui_parameters vui = { 0 };
        KS265FrameInfo frminf = { 0 };

        ret = KS265DumpVUIParameters(decoder->m_decoder, &vui,&frminf);
        if (ret < 0) {
            //stp_log(avctx, 1, "KS265DumpVUIParameters = %d\n",ret);
            return ret;
        }

        decoder->pix_fmt = vui_pixfmt2pixfmt(decoder, frminf.nLumaBitDepth, frminf.nChromaFormatIdc);
        avctx->pix_fmt = decoder->pix_fmt;                        // sps->pix_fmt;
        avctx->coded_width = frminf.nCodedWidth;                  // sps->width;
        avctx->coded_height = frminf.nCodedHeight;                // sps->height;
        avctx->width = frminf.nWidth;                             // sps->output_width;
        avctx->height = frminf.nHeight;                           // sps->output_height;
        avctx->has_b_frames = frminf.has_b_frames;                //sps->temporal_layer[sps->max_sub_layers - 1].num_reorder_pics;
        avctx->profile = frminf.profile;                          //sps->ptl.general_ptl.profile_idc;
        avctx->level = frminf.level;                              //sps->ptl.general_ptl.level_idc;


        //stp_log(NULL, 1,
        //  "export_stream_params width=%d,height=%d,coded_width=%d,coded_height=%d,pix_fmt=%d\n",
        //  avctx->width, avctx->height, avctx->coded_width, avctx->coded_height,avctx->pix_fmt);


        //avctx->pix_fmt = vui_pixfmt2pixfmt(vui.video_format);     // sps->pix_fmt;

        if (vui.b_valid && vui.aspect_ratio_info_present_flag)
        {
            AVRational sar = {vui.sar_width, vui.sar_height};
            ff_set_sar(avctx, sar);
        }

        if (vui.b_valid && vui.video_full_range_flag)
            avctx->color_range =  AVCOL_RANGE_JPEG;
        else
            avctx->color_range = AVCOL_RANGE_MPEG;

        if (vui.b_valid && vui.colour_description_present_flag) {
            avctx->color_primaries = vui.colour_primaries;
            avctx->color_trc       = vui.transfer_characteristics;
            avctx->colorspace      = vui.matrix_coeffs;
        }
        else {
            avctx->color_primaries = AVCOL_PRI_UNSPECIFIED;
            avctx->color_trc       = AVCOL_TRC_UNSPECIFIED;
            avctx->colorspace      = AVCOL_SPC_UNSPECIFIED;
        }

        if (vui.b_valid && vui.vui_timing_info_present_flag) {

            num = vui.vui_num_units_in_tick;
            den = vui.vui_time_scale;

            //stp_log(NULL, 1,"export_stream_params num=%dd\n", num, den);
        }

        if (num != 0 && den != 0) {

            av_reduce(&avctx->framerate.den, &avctx->framerate.num, num, den, 1 << 30);
        }

    }

    return 0;

#endif //
}



static av_cold int hevc_decode_init(AVCodecContext *avctx) {

    KS265Context *ctx = avctx->priv_data;

    int  ret = ks265_init_context(avctx);
    if (ret < 0)
        return ret;

    ctx->params = (KS265DecConfig*)(av_mallocz(sizeof(KS265DecConfig)));
    if (!ctx->params) {
        av_log(avctx, AV_LOG_ERROR, "Could not allocate ks265 param structure.\n");
        return AVERROR(ENOMEM);
    }


    if (ctx->ksdec_threads < 0) {
        ctx->params->threads = avctx->thread_count; // auto set to core number by decoder
    }
    else {
        ctx->params->threads = ctx->ksdec_threads;
    }

    int cpu_count = av_cpu_count();
    ctx->params->threads = av_clip(ctx->params->threads, MIN_KS265_CORE, FFMIN(cpu_count, MAX_KS265_CORE));

    ctx->params->bEnableOutputRecToFile = 0;
    ctx->params->strRecYuvFileName = NULL;
    ctx->params->logLevel = 0;
    ctx->params->outputPixfmt = ctx->outputPixfmt;
    ctx->params->nal_mode = ctx->queueDecode;
    ctx->params->b_use_pthread = ctx->b_use_pthread;
    // init for support 400
    ctx->params->b_alpha_decode = ctx->b_alpha_decode;
    ctx->params->p_log_func = av_log;
    ctx->params->p_arg = avctx;
    ctx->params->b_zerobuffer_contains_ref = ctx->b_zerobuffer_contains_ref;

    if (ctx->ks265Params) {
        AVDictionary *dict    = NULL;
        AVDictionaryEntry *en = NULL;

        if (!av_dict_parse_string(&dict, ctx->ks265Params, "=", ":", 0)) {
            while ((en = av_dict_get(dict, "", en, AV_DICT_IGNORE_SUFFIX))) {
                if (!strcmp(en->key, "queue_decode")) {
                    av_log(avctx, AV_LOG_INFO, "set queue_decode to %s", en->value);
                    ctx->params->nal_mode = atoi(en->value);
                }
                else if (!strcmp(en->key, "pic_out_pos")) {
                    av_log(avctx, AV_LOG_INFO, "set pic_out_pos to %s", en->value);
                    ctx->params->pic_out_pos = atoi(en->value);
                }
                else if (!strcmp(en->key, "use_pthread")) {
                    av_log(avctx, AV_LOG_INFO, "set use_pthread to %s", en->value);
                    ctx->params->b_use_pthread = atoi(en->value);
                }
                else if (!strcmp(en->key, "alpha_decode")) {
                    av_log(avctx, AV_LOG_INFO, "set alpha_decode to %s", en->value);
                    ctx->params->b_alpha_decode = atoi(en->value);
                }
            }
            av_dict_free(&dict);
        }
    }

#ifndef __arm__
    ctx->params->b_use_pthread = 1;
#endif

    if (ctx->supportSlices) {
        av_log(avctx, AV_LOG_INFO, "kw265: queue decode will be disabled when supporting slices\n");
        ctx->params->nal_mode = 0;
    }

    if (ctx->params->b_use_pthread) {
        av_log(avctx, AV_LOG_INFO, "kw265: queue decode will be disabled when using pthread\n");
        ctx->params->nal_mode = 0;
    }

    av_log(avctx, AV_LOG_INFO, "dec thread: %d [%d %d] cpu_count %d\n",
    ctx->params->threads, ctx->ksdec_threads, avctx->thread_count, cpu_count);

    int errCode = 0;
    ctx->m_decoder = KS265DecoderCreate(ctx->params);
    if (!ctx->m_decoder) {
        av_log(avctx, AV_LOG_ERROR, "Cannot open libks265 decoder.\n");
        hevc_decode_free(avctx);
        return AVERROR_INVALIDDATA;
    }

    ctx->is_nalff = 0;
    ctx->nal_length_size = 4;
    if (avctx->extradata_size > 0 && avctx->extradata) {
        ret = ks265_decode_extradata(avctx);
        if (ret < 0) {
            hevc_decode_free(avctx);
            return ret;
        }

        ret = export_stream_params(avctx);
    }

    //stp_log(avctx, 1, "hevc_decode_init return:export_stream_params = %d\n", ret);

    return ret;
}

static void hevc_decode_flush(AVCodecContext *avctx)
{
    KS265Context *s = avctx->priv_data;

    if (atomic_load(&s->b_has_been_freed)) {
        return;
    }

    while (atomic_exchange(&s->decoder_lock, 1)) {
        sched_yield();
    }

    if (atomic_load(&s->b_has_been_freed)) {
        atomic_exchange(&s->decoder_lock, 0);
        return;
    }

    int decStat_ret = KS265DecodeFlush(s->m_decoder, 1);

    atomic_exchange(&s->decoder_lock, 0);
}


#define OFFSET(x) offsetof(KS265Context, x)
#define PAR (AV_OPT_FLAG_DECODING_PARAM | AV_OPT_FLAG_VIDEO_PARAM)
#define VD AV_OPT_FLAG_VIDEO_PARAM | AV_OPT_FLAG_DECODING_PARAM

static const AVProfile profiles[] = {
    { FF_PROFILE_HEVC_MAIN,                 "Main"                },
    { FF_PROFILE_UNKNOWN },
};


static const AVOption options[] = {
    { "ksdec_threads", "threads pass to decoder(0:auto, should less than cpu core count)", OFFSET(ksdec_threads),
        AV_OPT_TYPE_INT, {.i64 = 0}, 0, 128, PAR },
    { "threads", "threads pass to decoder(0:auto, should less than cpu core count)", OFFSET(ksdec_threads),
            AV_OPT_TYPE_INT, {.i64 = 4}, 0, 8,  PAR,"threads"},
    { "auto", "threads pass to decoder(0:auto, should less than cpu core count)", 0,
            AV_OPT_TYPE_CONST, {.i64 = 0}, 0, 8, PAR,"threads"},  //TODO current default 4
    { "output_pixfmt", "set the output frame pixel format: 0 - yuv420p, 1 - nv12, 2 - nv21", OFFSET(outputPixfmt), AV_OPT_TYPE_INT, {.i64 = 0}, 0, 2, PAR },
    { "queue_decode", "buffer some frames in queue to decode", OFFSET(queueDecode), AV_OPT_TYPE_INT, {.i64 = 1 }, 0, 1, PAR },
    { "support_slices", "support multi-slice/tile bitstream", OFFSET(supportSlices), AV_OPT_TYPE_INT, {.i64 = 0}, 0, 1, PAR },
    { "use_pthread", "use pthread instead of atomic to sync", OFFSET(b_use_pthread), AV_OPT_TYPE_INT, {.i64 = 0}, 0, 1, PAR },
    { "alpha_decode", "open alpha decode to support yuv400", OFFSET(b_alpha_decode), AV_OPT_TYPE_INT, {.i64 = 0}, 0, 1, PAR },
    { "ks265_params", "set extra params to ks265 decoder", OFFSET(ks265Params), AV_OPT_TYPE_STRING, { 0 }, 0, 0, VD },
    { "contains_ref_frame_zerobuffer", "detect zero buffer in sps when contains ref", OFFSET(b_zerobuffer_contains_ref), AV_OPT_TYPE_INT, {.i64 = 0}, 0, 1, PAR },
    { NULL },
};



static const AVClass ks265_decoder_class = {
    .class_name = "ks265 Decoder",
    .item_name  = av_default_item_name,
    .option     = options,
    .version    = LIBAVUTIL_VERSION_INT,
};

AVCodec ff_libks265_decoder = {
    .name                  = "libks265dec",
    .long_name             = NULL_IF_CONFIG_SMALL("hevc(h.265) ks265 decoder"),
    .type                  = AVMEDIA_TYPE_VIDEO,
    .id                    = AV_CODEC_ID_HEVC,
    .priv_data_size        = sizeof(KS265Context),
    .priv_class            = &ks265_decoder_class,
    .init                  = hevc_decode_init,
    .close                 = hevc_decode_free,
    .decode                = hevc_decode_frame,
    .flush                 = hevc_decode_flush,
    .capabilities          = AV_CODEC_CAP_DR1 | AV_CODEC_CAP_DELAY,
//                             AV_CODEC_CAP_SLICE_THREADS | AV_CODEC_CAP_FRAME_THREADS,
    .caps_internal         = FF_CODEC_CAP_INIT_THREADSAFE | FF_CODEC_CAP_EXPORTS_CROPPING |
                             FF_CODEC_CAP_ALLOCATE_PROGRESS | FF_CODEC_CAP_INIT_CLEANUP,

    .profiles              = NULL_IF_CONFIG_SMALL(profiles),
};

