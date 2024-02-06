tests/data/add_keyframe_index.flv: TAG = GEN
tests/data/add_keyframe_index.flv: ffmpeg$(PROGSSUF)$(EXESUF) | tests/data
	$(M)$(TARGET_EXEC) $(TARGET_PATH)/$< \
		-f lavfi -i "sws_flags=+accurate_rnd+bitexact;testsrc=r=7:n=2:d=20" -sws_flags '+accurate_rnd+bitexact' -metadata "encoder=Lavf" -pix_fmt yuv420p -c:v flv1 -g 7 -f flv -flags +bitexact -fflags +bitexact \
		-flvflags add_keyframe_index -idct simple -dct int -y $(TARGET_PATH)/tests/data/add_keyframe_index.flv 2> /dev/null;

FATE_AFILTER-$(call ALLYES, FLV_MUXER FLV_DEMUXER AVDEVICE TESTSRC_FILTER LAVFI_INDEV FLV_ENCODER) += fate-flv-add_keyframe_index
fate-flv-add_keyframe_index: tests/data/add_keyframe_index.flv
fate-flv-add_keyframe_index: CMD = ffmetadata -flags +bitexact -i $(TARGET_PATH)/tests/data/add_keyframe_index.flv


FATE_ENHANCED_FLVENC_FFMPEG-$(call REMUX, FLV MOV, FLV_DEMUXER HEVC_PARSER) += fate-enhanced-flv-hevc
fate-enhanced-flv-hevc: CMD = transcode mov $(TARGET_SAMPLES)/hevc/dv84.mov\
		flv "-c copy" "-c copy"

FATE_ENHANCED_FLVENC_FFMPEG-$(call REMUX, FLV IVF, FLV_DEMUXER VP9_PARSER) += fate-enhanced-flv-vp9
fate-enhanced-flv-vp9: CMD = transcode ivf $(TARGET_SAMPLES)/vp9-test-vectors/vp90-2-05-resize.ivf\
		flv "-c copy" "-c copy"

FATE_FFMPEG_FFPROBE += $(FATE_FLVENC_FFMPEG_FFPROBE-yes)
FATE_SAMPLES_FFMPEG += $(FATE_ENHANCED_FLVENC_FFMPEG-yes)
fate-flvenc: $(FATE_FLVENC_FFMPEG_FFPROBE-yes) $(FATE_ENHANCED_FLVENC_FFMPEG-yes)
