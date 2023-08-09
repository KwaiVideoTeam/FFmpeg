/*
 * utils.c
 *
 * Copyright (c) 2003 Fabrice Bellard
 * Copyright (c) 2013 Zhang Rui <bbcallen@gmail.com>
 *
 * This file is part of ijkPlayer.
 *
 * ijkPlayer is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * ijkPlayer is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with ijkPlayer; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */
#if 1
#include <stdlib.h>
#include "url.h"
#include "avformat.h"


#define KWAI_DUMMY_DEMUXER(x) \
AVInputFormat ff_##x##_demuxer; \
extern int av_register_##x##_demuxer(AVInputFormat *demuxer, int demuxer_size);                 \
int av_register_##x##_demuxer(AVInputFormat *demuxer, int demuxer_size)                         \
{                                                                                               \
    if (demuxer_size != sizeof(AVInputFormat)) {                                                \
        av_log(NULL, AV_LOG_ERROR, "av_register_##x##_demuxer: ABI mismatch.\n");               \
        return -1;                                                                              \
    }                                                                                           \
    memcpy(&ff_##x##_demuxer, demuxer, demuxer_size);                                           \
    return 0;                                                                                   \
} \
\
static const AVClass ff_##x##_demuxer_class = {                    \
    .class_name = #x,                                              \
    .item_name  = av_default_item_name,                            \
    .version    = LIBAVUTIL_VERSION_INT,                           \
    };                                                             \
                                                                   \
AVInputFormat ff_##x##_demuxer = {                                 \
    .name                = #x,                                     \
    .priv_data_size      = 1,                                      \
    .priv_class          = &ff_##x##_demuxer_class,                \
}; \

KWAI_DUMMY_DEMUXER(kwailive);
KWAI_DUMMY_DEMUXER(kphls);
KWAI_DUMMY_DEMUXER(kpwebrtc);
KWAI_DUMMY_DEMUXER(kphls_aemon);
KWAI_DUMMY_DEMUXER(kpwebrtc_aemon);

#endif
