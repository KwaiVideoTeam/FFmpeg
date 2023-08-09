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
#include <stdlib.h>
#include "url.h"
#include "avformat.h"

#define KWAI_FF_PROTOCOL(x)                                                                          \
URLProtocol ff_##x##_protocol;                                                               \
int av_register_##x##_protocol(URLProtocol *protocol, int protocol_size);                        \
int av_register_##x##_protocol(URLProtocol *protocol, int protocol_size)                         \
{                                                                                                   \
    if (protocol_size != sizeof(URLProtocol)) {                                                     \
        av_log(NULL, AV_LOG_ERROR, "av_register_##x##_protocol: ABI mismatch.\n");               \
        return -1;                                                                                  \
    }                                                                                               \
    memcpy(&ff_##x##_protocol, protocol, protocol_size);                                            \
    return 0;                                                                                       \
}
#define KWAI_DUMMY_PROTOCOL(x)                                       \
KWAI_FF_PROTOCOL(x);                                                 \
static const AVClass ff_##x##_context_class = {                    \
    .class_name = #x,                                               \
    .item_name  = av_default_item_name,                             \
    .version    = LIBAVUTIL_VERSION_INT,                            \
    };                                                              \
                                                                    \
URLProtocol ff_##x##_protocol = {                                   \
    .name                = #x,                                      \
    .url_open2           = dummy_open,                           \
    .priv_data_size      = 1,                                       \
    .priv_data_class     = &ff_##x##_context_class,                \
};
static int dummy_open(URLContext *h, const char *arg, int flags, AVDictionary **options)
{
    return -1;
}
KWAI_DUMMY_PROTOCOL(kwai);
KWAI_DUMMY_PROTOCOL(kpasync);
KWAI_DUMMY_PROTOCOL(kphttphook);
KWAI_DUMMY_PROTOCOL(kpinject);
KWAI_DUMMY_PROTOCOL(kplongurl);
KWAI_DUMMY_PROTOCOL(kpmediadatasource);
KWAI_DUMMY_PROTOCOL(kpsegment);
KWAI_DUMMY_PROTOCOL(kptcphook);
KWAI_DUMMY_PROTOCOL(kpindexcontent);
KWAI_DUMMY_PROTOCOL(kpwebrtc);
KWAI_DUMMY_PROTOCOL(kpffmpegdatasource);
KWAI_DUMMY_PROTOCOL(kpasync_aemon);
KWAI_DUMMY_PROTOCOL(kphttphook_aemon);
KWAI_DUMMY_PROTOCOL(kpinject_aemon);
KWAI_DUMMY_PROTOCOL(kplongurl_aemon);
KWAI_DUMMY_PROTOCOL(kpmediadatasource_aemon);
KWAI_DUMMY_PROTOCOL(kpsegment_aemon);
KWAI_DUMMY_PROTOCOL(kptcphook_aemon);
KWAI_DUMMY_PROTOCOL(kpindexcontent_aemon);
KWAI_DUMMY_PROTOCOL(kpwebrtc_aemon);
KWAI_DUMMY_PROTOCOL(kpffmpegdatasource_aemon);


