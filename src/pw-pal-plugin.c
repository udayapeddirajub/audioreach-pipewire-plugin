/* Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <string.h>
#include <stdio.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdlib.h>
#include <signal.h>
#include <limits.h>
#include <math.h>
#include <time.h>
#include <dirent.h>
#include <linux/input.h>
#include <spa/utils/result.h>
#include <spa/utils/string.h>
#include <spa/utils/json.h>
#include <spa/utils/ringbuffer.h>
#include <spa/debug/types.h>
#include <spa/pod/builder.h>
#include <spa/pod/parser.h>
#include <spa/node/node.h>
#include <spa/node/utils.h>
#include <spa/node/keys.h>
#include <spa/param/audio/format-utils.h>
#include <spa/param/audio/raw.h>
#include <spa/param/buffers.h>
#include <pipewire/impl.h>
#include <pipewire/i18n.h>
#include <PalApi.h>
#include <PalDefs.h>
#include <agm/agm_api.h>
#include "SoundTriggerUtils.h"


#define LOG_TAG "pw-pal-plugin"

#define BITS_PER_BYTE 8
#define NUM_BYTES ((SW_MAX + BITS_PER_BYTE) / BITS_PER_BYTE)
#define BIT_VALUE(bit, array) \
    ((array[(bit) / BITS_PER_BYTE] >> ((bit) % BITS_PER_BYTE)) & 1)

PW_LOG_TOPIC_STATIC(log_topic, "log:" LOG_TAG);
#define PW_LOG_TOPIC_DEFAULT log_topic

#define PW_DEFAULT_SAMPLE_FORMAT "S16"
#define PW_DEFAULT_SAMPLE_RATE 48000
#define PW_DEFAULT_SAMPLE_CHANNELS 2
#define PW_DEFAULT_SAMPLE_POSITION "[ FL FR ]"
#define PW_DEFAULT_BUFFER_DURATION_MS 25
#define PW_LOW_LATENCY_BUFFER_DURATION_MS 5
#define PW_DEEP_BUFFER_BUFFER_DURATION_MS 20
#define MAX_NAME_LENGTH 20
#define DEV_INPUT_DIR "/dev/input"
#define FILE_PREFIX "event"
#define MAX_DEVICES 4
#define SVA_SOURCE_BUF_SIZE 512
#define SVA_SOURCE_BUF_COUNT 8
#define SVA_BIT_WIDTH 16
#define SVA_SAMPLE_RATE 16000
#define SVA_LAB_DURATION 2000
#define SVA_FRAME_SIZE_BYTES 2
#define SVA_PHRASE_ID 1
#define SVA_NUM_SOUND_MODELS 3
#define SVA_PDK_CONFIDENCE_LEVEL 40
#define SVA_PDK_CONFIDENCE_LEVEL_DEFAULT 60
#define SVA_HIST_BUFFER_DURATION_MSEC 1750
#define SVA_PREROLL_DURATION_MSEC 250
#define ST_CONFIG_VERSION_V2 0x2


enum sva_state{
        SVA_STATE_IDLE = 0,
        SVA_STATE_LOADED,
        SVA_STATE_ACTIVE,
        SVA_STATE_DETECTED,
};

struct sva_config {
    char model_path[256];       // sva.sound.model.path
    uint8_t *model_data;        // loaded binary (malloc'd)
    uint32_t model_size;        // computed from stat()
    uint32_t confidence_level;  // sva.confidence.level (0-100)
    uint32_t lab_duration;      // sva.lab.duration (ms, default 2000)
    bool lab_enabled;           // sva.lab.enabled (default true)
    char mode[8];               // sva.mode ("LPI" / "NLPI")
    uint32_t sample_rate;       // sva.sample.rate (default 16000)
    uint32_t channels;          // sva.channels (default 1)
    uint32_t bit_width;         // sva.bit.width (default 16)
};

struct sva_runtime {
      bool detection_pending;         // set by PAL callback
      uint32_t detection_confidence;  // from detection event
      uint64_t kwd_start_ts;          // keyword start timestamp
      uint64_t kwd_end_ts;            // keyword end timestamp
      bool lab_draining;              // true while draining LAB buffer post-detection
      uint32_t lab_bytes_read;        // bytes drained so far
      uint32_t lab_bytes_total;       // target bytes = lab_duration worth of PCM
      bool lab_status_pending;
};


struct pw_userdata {
    struct pw_context *context;

    struct pw_properties *props;

    struct pw_impl_module *module;

    struct spa_hook module_listener;

    struct pw_core *core;
    struct spa_hook core_proxy_listener;
    struct spa_hook core_listener;

    struct pw_properties *stream_props;
    struct pw_stream *stream;
    struct spa_hook stream_listener;
    struct spa_audio_info_raw info;
    uint32_t frame_size;
    struct spa_audio_info format;

    unsigned int do_disconnect:1;

    pal_stream_handle_t *stream_handle;
    struct pal_device *pal_device;
    struct pal_stream_attributes *stream_attributes;
    bool isplayback;
    pal_stream_type_t stream_type;
    pal_device_id_t pal_device_id[MAX_DEVICES];
    uint32_t no_of_devices;
    bool is_offload;
    size_t source_buf_size;
    size_t source_buf_count;
    size_t sink_buf_size;
    size_t sink_buf_count;

    struct spa_source *jack_src;
    int jack_fd;
    char jack_name[MAX_NAME_LENGTH];

    bool is_sva;                        
    enum sva_state sva_state;           
    pal_stream_handle_t *sva_pal_handle;
    struct sva_config sva_cfg;
    struct sva_runtime sva_rt;    
    
};

static void sva_set_state(struct pw_userdata *udata, enum sva_state next_state);
static void sva_emit_props_changed(struct pw_userdata *udata);
static int close_pal_stream(struct pw_userdata *udata);
static void sva_schedule_idle_status(struct pw_userdata *udata);

static void pw_pal_destroy_stream(void *d)
{
    struct pw_userdata *udata = d;

    spa_hook_remove(&udata->stream_listener);
    udata->stream = NULL;
}

static int32_t pa_pal_out_cb(pal_stream_handle_t *stream_handle,
                            uint32_t event_id, uint32_t *event_data,
                            uint32_t event_size, uint64_t cookie) {

    return 0;
}
static void pw_pal_set_volume (struct pw_userdata *udata, float gain)
{
    int rc = 0, i;
    uint32_t channel_mask = 1;
    uint32_t no_vol_pair = udata->stream_attributes->out_media_config.ch_info.channels;
    struct pal_volume_data *volume = (struct pal_volume_data *)malloc(sizeof(uint32_t) +
        (sizeof(struct pal_channel_vol_kv) * (no_vol_pair)));
    for (i = 0; i < no_vol_pair; i++)
        channel_mask = (channel_mask | udata->stream_attributes->out_media_config.ch_info.ch_map[i]);
    channel_mask = (channel_mask << 1);
    if (volume) {
        volume->no_of_volpair = no_vol_pair;
        for (i = 0; i < no_vol_pair; i++) {
            volume->volume_pair[i].channel_mask = channel_mask;
            volume->volume_pair[i].vol = gain;
        }
        rc = pal_stream_set_volume(udata->stream_handle, volume);
        free(volume);
    }
}


/* ═══════════════════════════════════════════════════════════════
 * SVA PAL stream callback — receives detection events from DSP
 * ═══════════════════════════════════════════════════════════════ */
struct sva_detect_ctx {
    uint32_t conf;
    uint64_t kw_start_ms;
    uint64_t kw_end_ms;
    bool capture_available;
};

struct sva_lab_status_ctx { char status[16]; uint64_t bytes; };

static int sva_publish_lab_status_on_loop(struct spa_loop *loop, bool async, uint32_t seq, const void *data, size_t size, void *user_data)
{
    struct pw_userdata *udata = user_data;
    const struct sva_lab_status_ctx *ctx = data;
    uint8_t pod_buf[256]; struct spa_pod_builder b = SPA_POD_BUILDER_INIT(pod_buf, sizeof(pod_buf));
    struct spa_pod_frame f[2]; const struct spa_pod *pod; int rc;
    (void)loop; (void)async; (void)seq; (void)size;
    if (!udata || !udata->stream || !ctx) return -EINVAL;
    spa_pod_builder_push_object(&b, &f[0], SPA_TYPE_OBJECT_Props, SPA_PARAM_Props);
    spa_pod_builder_prop(&b, SPA_PROP_params, 0); spa_pod_builder_push_struct(&b, &f[1]);
    spa_pod_builder_string(&b, "sva.lab.status"); spa_pod_builder_string(&b, ctx->status);
    spa_pod_builder_string(&b, "sva.lab.bytes"); spa_pod_builder_long(&b, (int64_t)ctx->bytes);
    spa_pod_builder_pop(&b, &f[1]); pod = spa_pod_builder_pop(&b, &f[0]);
    if (!pod) return -EINVAL;
    rc = pw_stream_update_params(udata->stream, &pod, 1);
    pw_log_warn("[SVA] update_params rc=%d (sva.lab.status=%s)",
            rc, ctx->status);
    udata->sva_rt.lab_status_pending = false;
    if (rc < 0) pw_log_error("[SVA] LAB status publication failed: %s", spa_strerror(rc));
    return rc;
}

static void sva_schedule_lab_status(struct pw_userdata *udata, const char *status)
{
    struct sva_lab_status_ctx ctx;
    if (!udata || !udata->context || !status || udata->sva_rt.lab_status_pending) return;
    memset(&ctx, 0, sizeof(ctx)); snprintf(ctx.status, sizeof(ctx.status), "%s", status);
    ctx.bytes = udata->sva_rt.lab_bytes_read; udata->sva_rt.lab_status_pending = true;
    if (pw_loop_invoke(pw_context_get_main_loop(udata->context), sva_publish_lab_status_on_loop, 0, &ctx, sizeof(ctx), false, udata) < 0) {
        udata->sva_rt.lab_status_pending = false; pw_log_error("[SVA] failed to schedule LAB status=%s", status);
    }
}

/*
 * Publish sva.detection.status=idle and sva.lab.status=idle together.
 * Called after sva.reset/sva.restart so the node's cached Props never
 * leave behind a stale "detected"/"complete" value from the previous
 * cycle for the next subscriber (e.g. a fresh `pw-voiceui listen`) to
 * replay as if it just happened.
 */
static int sva_publish_idle_on_loop(struct spa_loop *loop, bool async,
                                    uint32_t seq, const void *data,
                                    size_t size, void *user_data)
{
    struct pw_userdata *udata = user_data;
    uint8_t pod_buf[256]; struct spa_pod_builder b = SPA_POD_BUILDER_INIT(pod_buf, sizeof(pod_buf));
    struct spa_pod_frame f[2]; const struct spa_pod *pod; int rc;
    (void)loop; (void)async; (void)seq; (void)data; (void)size;
    if (!udata || !udata->stream) return -EINVAL;
    spa_pod_builder_push_object(&b, &f[0], SPA_TYPE_OBJECT_Props, SPA_PARAM_Props);
    spa_pod_builder_prop(&b, SPA_PROP_params, 0); spa_pod_builder_push_struct(&b, &f[1]);
    spa_pod_builder_string(&b, "sva.detection.status"); spa_pod_builder_string(&b, "idle");
    spa_pod_builder_string(&b, "sva.lab.status"); spa_pod_builder_string(&b, "idle");
    spa_pod_builder_pop(&b, &f[1]); pod = spa_pod_builder_pop(&b, &f[0]);
    if (!pod) return -EINVAL;
    rc = pw_stream_update_params(udata->stream, &pod, 1);
    pw_log_warn("[SVA] update_params rc=%d (sva.status=idle)", rc);
    if (rc < 0) pw_log_error("[SVA] idle status publication failed: %s", spa_strerror(rc));
    return rc;
}

static void sva_schedule_idle_status(struct pw_userdata *udata)
{
    if (!udata || !udata->context) return;
    if (pw_loop_invoke(pw_context_get_main_loop(udata->context), sva_publish_idle_on_loop,
                       0, NULL, 0, false, udata) < 0)
        pw_log_error("[SVA] failed to schedule idle status");
}

static int sva_push_detection_on_loop(struct spa_loop *loop, bool async,
                                      uint32_t seq, const void *data,
                                      size_t size, void *user_data);
static int32_t sva_pal_stream_callback(pal_stream_handle_t *stream_handle,
                                       uint32_t event_id, uint32_t *event_data,
                                       uint32_t event_data_size, uint64_t cookie);

static int sva_push_detection_on_loop(struct spa_loop *loop, bool async,
                                      uint32_t seq, const void *data,
                                      size_t size, void *user_data)
{
    struct pw_userdata *udata = user_data;
    const struct sva_detect_ctx *ctx = data;
    uint8_t buf[1024];
    struct spa_pod_builder b = SPA_POD_BUILDER_INIT(buf, sizeof(buf));
    struct spa_pod_frame f[2];
    int rc;

    (void)loop; (void)async; (void)seq; (void)size;
    if (!udata || !udata->stream || !ctx)
        return -EINVAL;

    spa_pod_builder_push_object(&b, &f[0], SPA_TYPE_OBJECT_Props, SPA_PARAM_Props);
    spa_pod_builder_prop(&b, SPA_PROP_params, 0);
    spa_pod_builder_push_struct(&b, &f[1]);
    spa_pod_builder_string(&b, "sva.detection.status");
    spa_pod_builder_string(&b, "detected");
    spa_pod_builder_string(&b, "sva.detection.confidence");
    spa_pod_builder_int(&b, (int32_t)ctx->conf);
    spa_pod_builder_string(&b, "sva.detection.kwd_start_ts");
    spa_pod_builder_long(&b, (int64_t)ctx->kw_start_ms);
    spa_pod_builder_string(&b, "sva.detection.kwd_end_ts");
    spa_pod_builder_long(&b, (int64_t)ctx->kw_end_ms);
    spa_pod_builder_string(&b, "sva.detection.capture_available");
    spa_pod_builder_bool(&b, ctx->capture_available);
    if (ctx->capture_available && udata->sva_cfg.lab_enabled) {
        spa_pod_builder_string(&b, "sva.lab.status");
        spa_pod_builder_string(&b, "started");
        spa_pod_builder_string(&b, "sva.lab.bytes");
        spa_pod_builder_long(&b, 0);
    }
    spa_pod_builder_pop(&b, &f[1]);
    {
        const struct spa_pod *pod = spa_pod_builder_pop(&b, &f[0]);
        if (!pod)
            return -EINVAL;
        rc = pw_stream_update_params(udata->stream, &pod, 1);
    }
    if (rc < 0)
        pw_log_error("[SVA] detection property publication failed: %s", spa_strerror(rc));
    else
        pw_log_warn("[SVA] Detection properties published");
    return rc;
}

static int32_t sva_pal_stream_callback(pal_stream_handle_t *stream_handle,
                                    uint32_t event_id,
                                    uint32_t *event_data,
                                    uint32_t event_data_size,
                                    uint64_t cookie)
{
    (void)stream_handle; (void)event_data_size;
    struct pw_userdata *udata = (struct pw_userdata *)cookie;

    pw_log_warn("[SVA] pal_stream_callback event_id=%u event_data=%p",
                event_id, (void *)event_data);

    if (!udata || !event_data) {
        pw_log_warn("[SVA] pal_stream_callback: NULL udata or event_data");
        return 0;
    }

    struct pal_st_phrase_recognition_event *phrase_event =
        (struct pal_st_phrase_recognition_event *)((void *)event_data);
    struct pal_st_recognition_event *event = &phrase_event->common;

    pw_log_warn("[SVA] ════════════════════════════════════════");
    pw_log_warn("[SVA] *** KEYWORD DETECTED ***");
    pw_log_warn("[SVA]   status              = %d", event->status);
    pw_log_warn("[SVA]   type                = %d", (int)event->type);
    pw_log_warn("[SVA]   capture_available   = %s",
                event->capture_available ? "true" : "false");
    pw_log_warn("[SVA]   capture_delay_ms    = %d", event->capture_delay_ms);
    pw_log_warn("[SVA]   capture_preamble_ms = %d", event->capture_preamble_ms);
    pw_log_warn("[SVA]   data_size           = %u", event->data_size);
    pw_log_warn("[SVA]   media_config.sr     = %u", event->media_config.sample_rate);
    pw_log_warn("[SVA]   media_config.ch     = %u", event->media_config.ch_info.channels);

    /* Extract confidence from phrase_extras[] */
    uint32_t conf = 0;
    uint64_t kw_end_ms = 0, kw_start_ms = 0;

    if (event->type == PAL_SOUND_MODEL_TYPE_KEYPHRASE &&
        phrase_event->num_phrases > 0) {
        conf = phrase_event->phrase_extras[0].confidence_level;
        pw_log_warn("[SVA]   phrase[0].id         = %u",
                    phrase_event->phrase_extras[0].id);
        pw_log_warn("[SVA]   phrase[0].confidence = %u%%", conf);
    }

    /* Derive keyword window from capture timestamps */
    kw_end_ms   = (uint64_t)(event->capture_delay_ms > 0 ? event->capture_delay_ms : 0);
    kw_start_ms = kw_end_ms > 750 ? kw_end_ms - 750 : 0;

    pw_log_warn("[SVA]   kw_start = %llu ms", (unsigned long long)kw_start_ms);
    pw_log_warn("[SVA]   kw_end   = %llu ms", (unsigned long long)kw_end_ms);
    pw_log_warn("[SVA] ════════════════════════════════════════");

    /* Store in runtime state */
    udata->sva_rt.detection_pending    = true;
    udata->sva_rt.detection_confidence = conf;
    udata->sva_rt.kwd_start_ts        = kw_start_ms;
    udata->sva_rt.kwd_end_ts          = kw_end_ms;

    /* Update state machine */
    sva_set_state(udata, SVA_STATE_DETECTED);

    if (udata->sva_cfg.lab_enabled && event->capture_available) {
          uint32_t bytes_per_ms = (udata->sva_cfg.sample_rate * udata->sva_cfg.channels *
                                    (udata->sva_cfg.bit_width / 8)) / 1000;
          udata->sva_rt.lab_bytes_total = bytes_per_ms * udata->sva_cfg.lab_duration;
          udata->sva_rt.lab_bytes_read  = 0;
          udata->sva_rt.lab_draining    = true;
          pw_log_warn("[SVA] LAB drain armed: target=%u bytes (%u ms)",
                      udata->sva_rt.lab_bytes_total, udata->sva_cfg.lab_duration);

    } else {
          udata->sva_rt.lab_draining = false;
    }

    struct sva_detect_ctx ctx = {
        .conf              = conf,
        .kw_start_ms       = kw_start_ms,
        .kw_end_ms         = kw_end_ms,
        .capture_available = event->capture_available,
    };
    pw_loop_invoke(pw_context_get_main_loop(udata->context),
                   sva_push_detection_on_loop, 0, &ctx, sizeof(ctx),
                   false, udata);

    return 0;
}

static int sva_set_recognition_config(struct pw_userdata *udata)
{
    uint32_t num_phrases = 1;
    uint32_t opaque_size = sizeof(struct st_param_header) +
                        sizeof(struct st_confidence_levels_info_v2) +
                        sizeof(struct st_param_header) +
                        sizeof(struct st_hist_buffer_info);
    uint32_t rc_config_size = sizeof(struct pal_st_recognition_config) + opaque_size;

    pal_param_payload *rec_payload = calloc(1, sizeof(pal_param_payload) + rc_config_size);
    if (!rec_payload) return -ENOMEM;

    /* Match the original working PAL implementation: the recognition payload
     * size includes the outer pal_param_payload header. */
    rec_payload->payload_size = sizeof(pal_param_payload) + rc_config_size;
    struct pal_st_recognition_config *rec_cfg =
        (struct pal_st_recognition_config *)rec_payload->payload;

    rec_cfg->capture_handle    = 0;
    rec_cfg->capture_device    = PAL_DEVICE_IN_HANDSET_VA_MIC;
    rec_cfg->capture_requested = udata->sva_cfg.lab_enabled ? 1 : 0;
    rec_cfg->num_phrases       = num_phrases;
    rec_cfg->data_size         = opaque_size;
    rec_cfg->data_offset       = sizeof(struct pal_st_recognition_config);
    rec_cfg->callback          = NULL;
    rec_cfg->cookie            = (uint8_t *)udata;
    rec_cfg->phrases[0].id               = SVA_PHRASE_ID;
    rec_cfg->phrases[0].recognition_modes = PAL_RECOGNITION_MODE_VOICE_TRIGGER;
    rec_cfg->phrases[0].confidence_level  = udata->sva_cfg.confidence_level;
    rec_cfg->phrases[0].num_levels        = 0;

    uint8_t *payload_ptr = (uint8_t *)rec_cfg + rec_cfg->data_offset;
    struct st_param_header *hdr = (struct st_param_header *)payload_ptr;
    hdr->key_id       = ST_PARAM_KEY_CONFIDENCE_LEVELS;
    hdr->payload_size = sizeof(struct st_confidence_levels_info_v2);
    payload_ptr += sizeof(struct st_param_header);

    struct st_confidence_levels_info_v2 *conf_info =
        (struct st_confidence_levels_info_v2 *)payload_ptr;
    conf_info->version          = ST_CONFIG_VERSION_V2;
    conf_info->num_sound_models = SVA_NUM_SOUND_MODELS;
    conf_info->conf_levels[0].sm_id = ST_SM_ID_SVA_F_STAGE_GMM;
    conf_info->conf_levels[0].num_kw_levels = num_phrases;
    conf_info->conf_levels[0].kw_levels[0].kw_level = (int32_t)udata->sva_cfg.confidence_level;
    conf_info->conf_levels[0].kw_levels[0].num_user_levels = 0;
    conf_info->conf_levels[1].sm_id = ST_SM_ID_SVA_S_STAGE_PDK;
    conf_info->conf_levels[1].num_kw_levels = num_phrases;
    conf_info->conf_levels[1].kw_levels[0].kw_level = SVA_PDK_CONFIDENCE_LEVEL;
    conf_info->conf_levels[1].kw_levels[0].num_user_levels = 0;
    conf_info->conf_levels[2].sm_id = ST_SM_ID_SVA_S_STAGE_USER;
    conf_info->conf_levels[2].num_kw_levels = num_phrases;
    conf_info->conf_levels[2].kw_levels[0].kw_level = 0;
    conf_info->conf_levels[2].kw_levels[0].num_user_levels = 0;
    payload_ptr += sizeof(struct st_confidence_levels_info_v2);

    hdr = (struct st_param_header *)payload_ptr;
    hdr->key_id       = ST_PARAM_KEY_HISTORY_BUFFER_CONFIG;
    hdr->payload_size = sizeof(struct st_hist_buffer_info);
    payload_ptr += sizeof(struct st_param_header);
    struct st_hist_buffer_info *hist = (struct st_hist_buffer_info *)payload_ptr;
    hist->version                   = ST_CONFIG_VERSION_V2;
    hist->hist_buffer_duration_msec = SVA_HIST_BUFFER_DURATION_MSEC;
    hist->pre_roll_duration_msec    = SVA_PREROLL_DURATION_MSEC;

    int rc = pal_stream_set_param(udata->sva_pal_handle,
                                PAL_PARAM_ID_RECOGNITION_CONFIG, rec_payload);
    free(rec_payload);
    return rc;
}

/* ═══════════════════════════════════════════════════════════════
 * sva_load_sound_model() — Open PAL + Load Keyphrase Model
 * ═══════════════════════════════════════════════════════════════ */
static int sva_load_sound_model(struct pw_userdata *udata)
{
    int rc = 0;
    struct stat st;
    FILE *fp = NULL;

    /* 1. Read model binary from filesystem */
    if (stat(udata->sva_cfg.model_path, &st) != 0) {
        pw_log_error("SVA: Cannot stat model: %s (%m)", udata->sva_cfg.model_path);
        return -errno;
    }

    udata->sva_cfg.model_size = (uint32_t)st.st_size;

    if (udata->sva_cfg.model_data) {
        free(udata->sva_cfg.model_data);
        udata->sva_cfg.model_data = NULL;
    }

    udata->sva_cfg.model_data = malloc(udata->sva_cfg.model_size);
    if (!udata->sva_cfg.model_data) return -ENOMEM;

    fp = fopen(udata->sva_cfg.model_path, "rb");
    if (!fp) {
        free(udata->sva_cfg.model_data);
        udata->sva_cfg.model_data = NULL;
        return -errno;
    }

    if (fread(udata->sva_cfg.model_data, 1, udata->sva_cfg.model_size, fp)
        != udata->sva_cfg.model_size) {
        pw_log_error("SVA: Short read on model file");
        fclose(fp);
        free(udata->sva_cfg.model_data);
        udata->sva_cfg.model_data = NULL;
        return -EIO;
    }
    fclose(fp);

    /* 2. Open PAL Stream — WITH callback for detection events */

    if (udata->stream_attributes) {
        free(udata->stream_attributes);
        udata->stream_attributes = NULL;
    }
    udata->stream_attributes = calloc(1, sizeof(struct pal_stream_attributes));
    if (udata->stream_attributes == NULL) {
        pw_log_error("SVA: failed to allocate stream attributes: %m");
        return -ENOMEM;
    }
    struct pal_stream_attributes *stream_attr = udata->stream_attributes;
    memset(stream_attr, 0, sizeof(*stream_attr));
    stream_attr->type                                = PAL_STREAM_VOICE_UI;
    stream_attr->info.voice_rec_info.version         = 1;
    stream_attr->flags                               = 0;
    stream_attr->direction                           = PAL_AUDIO_INPUT;
    stream_attr->in_media_config.sample_rate         = udata->sva_cfg.sample_rate ? udata->sva_cfg.sample_rate : SVA_SAMPLE_RATE;
    stream_attr->in_media_config.bit_width           = SVA_BIT_WIDTH;
    stream_attr->in_media_config.aud_fmt_id          = PAL_AUDIO_FMT_PCM_S16_LE;
    stream_attr->in_media_config.ch_info.channels    = udata->sva_cfg.channels ? udata->sva_cfg.channels : 1;
    stream_attr->in_media_config.ch_info.ch_map[0]   = PAL_CHMAP_CHANNEL_FL;

    /* Build pal_device fresh */
    if (udata->pal_device) {
        free(udata->pal_device);
        udata->pal_device = NULL;
    }
    udata->no_of_devices = 1;
    udata->pal_device = calloc(1, sizeof(struct pal_device));
    if (udata->pal_device == NULL) {
        pw_log_error("SVA: failed to allocate PAL device: %m");
        free(udata->stream_attributes);
        udata->stream_attributes = NULL;
        return -ENOMEM;
    }
    memset(udata->pal_device, 0, sizeof(struct pal_device));
    udata->pal_device->id                      = PAL_DEVICE_IN_HANDSET_VA_MIC;
    udata->pal_device->config.sample_rate      = SVA_SAMPLE_RATE;
    udata->pal_device->config.bit_width        = SVA_BIT_WIDTH;
    udata->pal_device->config.ch_info.channels = 1;
    udata->pal_device->config.ch_info.ch_map[0] = PAL_CHMAP_CHANNEL_FL;
    udata->pal_device->config.aud_fmt_id       = PAL_AUDIO_FMT_PCM_S16_LE;

    rc = pal_stream_open(stream_attr, udata->no_of_devices, udata->pal_device,
                        0, NULL,
                        sva_pal_stream_callback, (uint64_t)udata,
                        &udata->sva_pal_handle);
    if (rc != 0 || !udata->sva_pal_handle) {
        pw_log_error("SVA: pal_stream_open FAILED rc=%d", rc);
        return rc;
    }

    /* 3. Build pal_st_phrase_sound_model (KEYPHRASE type) */
    size_t sm_hdr_size = sizeof(struct pal_st_phrase_sound_model);
    pal_param_payload *sound_model_payload = calloc(1,
        sizeof(pal_param_payload) + sm_hdr_size + udata->sva_cfg.model_size);
    if (!sound_model_payload) {
        pal_stream_close(udata->sva_pal_handle);
        udata->sva_pal_handle = NULL;
        return -ENOMEM;
    }
    /* payload_size = content only (not including pal_param_payload header) */
    sound_model_payload->payload_size = (uint32_t)(sm_hdr_size + udata->sva_cfg.model_size);
    struct pal_st_phrase_sound_model *phrase_sm =
        (struct pal_st_phrase_sound_model *)sound_model_payload->payload;
    struct pal_st_sound_model *common_sm = &phrase_sm->common;
    common_sm->type        = PAL_SOUND_MODEL_TYPE_KEYPHRASE;
    common_sm->data_size   = udata->sva_cfg.model_size;
    common_sm->data_offset = (uint32_t)sm_hdr_size;
    /* HeySnapdragon vendor UUID */
    common_sm->vendor_uuid.timeLow          = 0x68ab2d40;
    common_sm->vendor_uuid.timeMid          = 0xe860;
    common_sm->vendor_uuid.timeHiAndVersion = 0x11e3;
    common_sm->vendor_uuid.clockSeq         = 0x95ef;
    common_sm->vendor_uuid.node[0] = 0x00;
    common_sm->vendor_uuid.node[1] = 0x02;
    common_sm->vendor_uuid.node[2] = 0xa5;
    common_sm->vendor_uuid.node[3] = 0xd5;
    common_sm->vendor_uuid.node[4] = 0xc5;
    common_sm->vendor_uuid.node[5] = 0x1b;
    /* phrase[0]: id=1 (PDK6 HeySnapdragon phrase ID is 1, not 0) */
    phrase_sm->num_phrases = 1;
    phrase_sm->phrases[0].id               = SVA_PHRASE_ID;
    phrase_sm->phrases[0].recognition_mode = PAL_RECOGNITION_MODE_VOICE_TRIGGER;
    phrase_sm->phrases[0].num_users        = 0;
    /* Copy .uim binary after header */
    memcpy((uint8_t *)phrase_sm + sm_hdr_size,
        udata->sva_cfg.model_data, udata->sva_cfg.model_size);
    pw_log_warn("SVA: LOAD_SOUND_MODEL payload_size=%u sm_hdr=%zu model=%u phrase_id=1",
                sound_model_payload->payload_size,
                sm_hdr_size, udata->sva_cfg.model_size);
    rc = pal_stream_set_param(udata->sva_pal_handle,
                            PAL_PARAM_ID_LOAD_SOUND_MODEL,
                            (void *)sound_model_payload);
    free(sound_model_payload);
    
    pw_log_warn("SVA: LOAD_SOUND_MODEL rc=%d", rc);
    if (rc != 0) {
        pw_log_error("SVA: LOAD_SOUND_MODEL FAILED rc=%d", rc);
        pal_stream_close(udata->sva_pal_handle);
        udata->sva_pal_handle = NULL;
        return rc;
    }
    pw_log_warn("SVA: Sound model LOADED (KEYPHRASE type, %u bytes)", udata->sva_cfg.model_size);
    return 0;
}


static void sva_emit_props_changed(struct pw_userdata *udata)
{
    const char *state_names[] = { "idle", "loaded", "active", "detected" };
    pw_log_warn("SVA: state=%s model=%s conf=%u mode=%s",
                state_names[udata->sva_state],
                udata->sva_cfg.model_path[0] ? udata->sva_cfg.model_path : "(none)",
                udata->sva_cfg.confidence_level,
                udata->sva_cfg.mode);
}

static void sva_set_state(struct pw_userdata *udata, enum sva_state next_state)
{
    const char *state_names[] = { "IDLE", "LOADED", "ACTIVE", "DETECTED" };
    if (udata->sva_state == next_state && next_state != SVA_STATE_DETECTED)
        return;
    pw_log_warn("SVA: [TRANSITION] %s -> %s", 
                state_names[udata->sva_state], state_names[next_state]);
    udata->sva_state = next_state;
    sva_emit_props_changed(udata);
}

static int close_pal_stream(struct pw_userdata *udata)
{
    int rc = -1;
    if (udata->is_sva) {
        if (udata->sva_pal_handle) {
            pal_stream_stop(udata->sva_pal_handle);
            pal_stream_close(udata->sva_pal_handle);
            udata->sva_pal_handle = NULL;
        }
        udata->sva_rt.lab_draining = false;
        return 0;
    }

    if (udata->stream_handle) {
        rc = pal_stream_stop(udata->stream_handle);
        if (rc) {
            pw_log_error("pal_stream_stop failed for %p error %d", udata->stream_handle, rc);
        }
        rc = pal_stream_close(udata->stream_handle);
        if (rc)
            pw_log_error("could not close sink handle %p, error %d", udata->stream_handle, rc);
        udata->stream_handle = NULL;
    }
    else
        return 0;

    return rc;
}
static void pw_pal_stream_start(struct pw_userdata *udata)
{
    int rc = 0;
    pal_buffer_config_t out_buf_cfg, in_buf_cfg;

    rc = pal_stream_open(udata->stream_attributes, udata->no_of_devices, udata->pal_device,
         0, NULL, pa_pal_out_cb, (uint64_t)udata, &udata->stream_handle);

    if (rc) {
        udata->stream_handle = NULL;
        pw_log_error("Could not open output stream %d", rc);
        goto exit;
    }

    if (udata->isplayback) {
        in_buf_cfg.buf_size = 0;
        in_buf_cfg.buf_count = 0;
        out_buf_cfg.buf_size = udata->sink_buf_size;
        out_buf_cfg.buf_count = udata->sink_buf_count;
    } else {
        out_buf_cfg.buf_size = 0;
        out_buf_cfg.buf_count = 0;
        in_buf_cfg.buf_size = udata->source_buf_size;
        in_buf_cfg.buf_count = udata->source_buf_count;
    }

    rc = pal_stream_set_buffer_size(udata->stream_handle, &in_buf_cfg, &out_buf_cfg);
    if(rc) {
        pw_log_error("pal_stream_set_buffer_size failed\n");
        goto cleanup;
    }
    rc = pal_stream_start(udata->stream_handle);
    if (rc) {
        pw_log_error("pal_stream_start failed, error %d\n", rc);
        goto cleanup;
        }
    if (udata->isplayback) {
        pw_log_error("pal_stream_start set volume, error %d\n", rc);
        pw_pal_set_volume(udata, 1.0);
    }

    return;
cleanup:
    if (close_pal_stream(udata))
        pw_log_error("could not close sink handle %p", udata->stream_handle);
exit:
    return;

}
static void pw_pal_change_stream_state(void *d, enum pw_stream_state old,
        enum pw_stream_state state, const char *error)
{
    struct pw_userdata *udata = d;
    
    switch (state) {
    case PW_STREAM_STATE_ERROR:
    case PW_STREAM_STATE_UNCONNECTED:
        if (!udata->is_sva) {
              pw_impl_module_schedule_destroy(udata->module);
        } else {
              pw_log_error("[SVA] stream unconnected/error — releasing PAL resources");
              close_pal_stream(udata);
              sva_set_state(udata, SVA_STATE_IDLE);
        }
        break;
    case PW_STREAM_STATE_PAUSED:
        if (!udata->is_sva) {
            close_pal_stream(udata);
        }
        break;
    case PW_STREAM_STATE_STREAMING:
        if (!udata->is_sva) {
            pw_pal_stream_start(udata);
        }
        break;
    default:
        break;
    }
}

static void pw_pal_process_stream(void *d)
{
    struct pw_userdata *udata = d;
    struct pw_buffer *buf = pw_stream_dequeue_buffer(udata->stream);
    struct spa_data *bd;
    struct pal_buffer pal_buf = { 0 };
    uint32_t size = 0;
    const char *lab_status = NULL;
    int rc;

    if (!buf) { pw_log_error("out of buffers: %m"); return; }
    if (!buf->buffer || buf->buffer->n_datas == 0) goto queue;
    bd = &buf->buffer->datas[0];
    if (!bd->chunk || !bd->data) goto queue;

    if (udata->isplayback) {
        uint32_t off = SPA_MIN(bd->chunk->offset, bd->maxsize);
        size = SPA_MIN(bd->chunk->size, bd->maxsize - off);
        pal_buf.buffer = SPA_PTROFF(bd->data, off, void);
        pal_buf.size = size;
        if (udata->stream_handle && (rc = pal_stream_write(udata->stream_handle, &pal_buf)) < 0)
            pw_log_error("Could not write data: %d", rc);
    } else {
        size = buf->requested ? SPA_MIN(buf->requested * udata->frame_size, bd->maxsize) : bd->maxsize;
        pal_buf.buffer = bd->data;
        pal_buf.size = size;
        if (udata->is_sva) {
            if (udata->sva_rt.lab_draining && udata->sva_pal_handle) {
                pw_log_warn("[SVA] PipeWire process entered");
                {
                    uint32_t remaining = udata->sva_rt.lab_bytes_total -
                                         udata->sva_rt.lab_bytes_read;
                    pal_buf.size = SPA_MIN(pal_buf.size, remaining);
                }
                rc = pal_stream_read(udata->sva_pal_handle, &pal_buf);
                if (rc < 0) {
                    pw_log_error("[SVA] LAB read failed: %d", rc);
                    size = 0;
                    udata->sva_rt.lab_draining = false;
                    lab_status = "error";
                } else if (rc == 0) {
                    pw_log_warn("[SVA] LAB read returned zero bytes; stopping drain");
                    size = 0;
                    udata->sva_rt.lab_draining = false;
                    lab_status = "error";
                } else {
                    size = (uint32_t)rc;
                    udata->sva_rt.lab_bytes_read += size;
                    if (udata->sva_rt.lab_bytes_read >= udata->sva_rt.lab_bytes_total)
                        udata->sva_rt.lab_draining = false;
                    pw_log_warn("[SVA] LAB progress=%u/%u", udata->sva_rt.lab_bytes_read, udata->sva_rt.lab_bytes_total);
                    if (!udata->sva_rt.lab_draining) {
                        /* Queue this buffer first; status is published afterwards. */
                        lab_status = "complete";
                        pw_log_warn("[SVA] LAB complete=%u", udata->sva_rt.lab_bytes_read);
                    }
                }
            } else size = 0;
        } else if (udata->stream_handle) {
            if ((rc = pal_stream_read(udata->stream_handle, &pal_buf)) < 0) {
                pw_log_error("Could not read data: %d", rc); size = 0;
            } else size = (uint32_t)rc;
        }
        bd->chunk->offset = 0;
        bd->chunk->size = size;
        bd->chunk->stride = udata->frame_size;
        buf->size = udata->frame_size ? size / udata->frame_size : 0;
    }
queue:
    rc = pw_stream_queue_buffer(udata->stream, buf);
    if (rc < 0) {
        pw_log_error("pw_stream_queue_buffer failed: %s", spa_strerror(rc));
    } else if (lab_status) {
        sva_schedule_lab_status(udata, lab_status);
    }
}

static int sva_start_recognition(struct pw_userdata *udata)
{
    int rc = sva_load_sound_model(udata);
    if (rc != 0) return rc;
    sva_set_state(udata, SVA_STATE_LOADED);
    rc = sva_set_recognition_config(udata);
    if (rc != 0) { close_pal_stream(udata); sva_set_state(udata, SVA_STATE_IDLE); return rc; }
    rc = pal_stream_start(udata->sva_pal_handle);
    if (rc != 0) { close_pal_stream(udata); sva_set_state(udata, SVA_STATE_IDLE); return rc; }
    sva_set_state(udata, SVA_STATE_ACTIVE);
    pw_log_warn("[SVA] Recognition STARTED");
    return 0;
}

static void pw_pal_change_stream_param(void *data, uint32_t id, const struct spa_pod *param) {
    struct pw_userdata *udata = data;

    if (param == NULL)
        return;

        if (udata->is_sva && id == SPA_PARAM_Props) {
        struct spa_pod_object *obj = (struct spa_pod_object *)param;
        struct spa_pod_prop *prop;
        bool model_path_updated = false;
        SPA_POD_OBJECT_FOREACH(obj, prop) {
            if (prop->key == SPA_PROP_params) {
                const struct spa_pod *iter;
                const struct spa_pod *params_struct = &prop->value;
                if (!spa_pod_is_struct(params_struct))
                    continue;
                SPA_POD_STRUCT_FOREACH(params_struct, iter) {
                    const char *key;
                    if (spa_pod_get_string(iter, &key) < 0)
                        break;
                    const struct spa_pod *value = spa_pod_next(iter);
                    if (!value)
                        break;
                    const char *str_val;
                    int32_t int_val;
                    if (spa_streq(key, "sva.sound.model.path") || spa_streq(key, "sva.model.path")) {
                        if (spa_pod_get_string(value, &str_val) == 0) {
                            snprintf(udata->sva_cfg.model_path, sizeof(udata->sva_cfg.model_path), "%s", str_val);
                            model_path_updated = true;
                        }
                    } else if (spa_streq(key, "sva.confidence.level")) {
                        if (spa_pod_get_int(value, &int_val) == 0 && int_val >= 0 && int_val <= 100)
                            udata->sva_cfg.confidence_level = (uint32_t)int_val;
                        else if (spa_pod_get_string(value, &str_val) == 0) {
                            char *end; long v = strtol(str_val, &end, 10);
                            if (*str_val && *end == '\0' && v >= 0 && v <= 100)
                                udata->sva_cfg.confidence_level = (uint32_t)v;
                            else { pw_log_error("SVA: invalid confidence level"); return; }
                        }
                    } else if (spa_streq(key, "sva.lab.duration")) {
                        if (spa_pod_get_int(value, &int_val) == 0 && int_val > 0)
                            udata->sva_cfg.lab_duration = (uint32_t)int_val;
                        else { pw_log_error("SVA: invalid LAB duration"); return; }
                    } else if (spa_streq(key, "sva.lab.enabled")) {
                        bool bool_val;
                        if (spa_pod_get_bool(value, &bool_val) == 0)
                            udata->sva_cfg.lab_enabled = bool_val;
                    } else if (spa_streq(key, "sva.mode")) {
                        if (spa_pod_get_string(value, &str_val) == 0)
                            snprintf(udata->sva_cfg.mode, sizeof(udata->sva_cfg.mode), "%s", str_val);
                    } else if (spa_streq(key, "sva.reset")) {
                       bool val = false;
                       if (spa_pod_get_bool(value, &val) == 0 && val) {
                          pw_log_warn("SVA: sva.reset requested — releasing PAL resources");
                          close_pal_stream(udata);
                          udata->sva_cfg.model_path[0] = '\0';
                          sva_set_state(udata, SVA_STATE_IDLE);
                          sva_schedule_idle_status(udata);
                          return;
                        }
                   } else if (spa_streq(key, "sva.restart")) {
                       bool val = false;
                       if (spa_pod_get_bool(value, &val) == 0 && val) {
                          if (udata->sva_pal_handle &&
                              (udata->sva_state == SVA_STATE_DETECTED ||
                               udata->sva_state == SVA_STATE_ACTIVE)) {
                              pw_log_warn("SVA: sva.restart requested — restarting "
                                          "recognition, model stays loaded");
                              int rc = pal_stream_start(udata->sva_pal_handle);
                              if (rc == 0) {
                                  sva_set_state(udata, SVA_STATE_ACTIVE);
                                  sva_schedule_idle_status(udata);
                                  pw_log_warn("SVA: Recognition RESTARTED");
                              } else {
                                  pw_log_error("SVA: sva.restart FAILED rc=%d", rc);
                              }
                          } else {
                              pw_log_warn("SVA: sva.restart requested but no model "
                                          "loaded (state=%d) — ignoring", udata->sva_state);
                          }
                          return;
                        }
                   }
                    iter = value;
                }
            }
        }
        pw_log_warn("SVA: Props received via pw_stream param_changed");
        pw_log_warn("SVA:   model.path = %s", udata->sva_cfg.model_path[0] ? udata->sva_cfg.model_path : "(not set)");
        pw_log_warn("SVA:   confidence = %u", udata->sva_cfg.confidence_level);
        /* Trigger model load if IDLE and path is set; if a new model path
         * arrives while a previous session is LOADED/ACTIVE/DETECTED,
         * release it first instead of silently ignoring the request. */
        if (model_path_updated && udata->sva_cfg.model_path[0] != '\0' &&
            udata->sva_state != SVA_STATE_IDLE) {
            pw_log_warn("SVA: New model path received while active (state=%d) — "
                        "releasing previous session before reload", udata->sva_state);
            close_pal_stream(udata);
            sva_set_state(udata, SVA_STATE_IDLE);
        }
        if (model_path_updated && udata->sva_state == SVA_STATE_IDLE &&
            udata->sva_cfg.model_path[0] != '\0') {
            int rc = sva_start_recognition(udata);
            if (rc != 0)
                pw_log_error("SVA: recognition start failed rc=%d", rc);
        }
        return;
    }
    if (id == SPA_PARAM_Format)
    {
        if (spa_format_parse(param, &udata->format.media_type, &udata->format.media_subtype) < 0)
            return;
        if (udata->format.media_type == SPA_MEDIA_TYPE_audio &&
            udata->format.media_subtype == SPA_MEDIA_SUBTYPE_raw) {
            spa_format_audio_raw_parse(param, &udata->format.info.raw);
        } else {
            pw_log_info("Compressed format detected: subtype=%u", udata->format.media_subtype);
        }
    }    
}

static const struct pw_stream_events pw_pal_stream_events = {
    PW_VERSION_STREAM_EVENTS,
    .destroy = pw_pal_destroy_stream,
    .state_changed = pw_pal_change_stream_state,
    .process = pw_pal_process_stream,
    .param_changed = pw_pal_change_stream_param
};

static int pw_pal_create_stream(struct pw_userdata *udata)
{
    int res;
    uint32_t n_params = 0;
    const struct spa_pod *params[2];
    uint8_t buffer[1024];
    struct spa_pod_builder b;
    spa_pod_builder_init(&b, buffer, sizeof(buffer));
    if (udata->isplayback) {
        udata->stream = pw_stream_new(udata->core, "example sink", udata->stream_props);
        if (udata->is_offload) {
            params[n_params++] = spa_pod_builder_add_object(&b,
                            SPA_TYPE_OBJECT_ParamBuffers, SPA_PARAM_Buffers,
                            SPA_PARAM_BUFFERS_buffers, SPA_POD_Int(udata->sink_buf_count),
                            SPA_PARAM_BUFFERS_blocks,  SPA_POD_Int(0),
                            SPA_PARAM_BUFFERS_size,    SPA_POD_Int(udata->sink_buf_size),
                            SPA_PARAM_BUFFERS_stride,  SPA_POD_Int(udata->frame_size));
        } else {
            params[n_params++] = spa_pod_builder_add_object(&b,
                            SPA_TYPE_OBJECT_ParamBuffers, SPA_PARAM_Buffers,
                            SPA_PARAM_BUFFERS_buffers, SPA_POD_Int(udata->sink_buf_count),
                            SPA_PARAM_BUFFERS_blocks,  0,
                            SPA_PARAM_BUFFERS_size,    SPA_POD_Int(udata->sink_buf_size),
                            SPA_PARAM_BUFFERS_stride,  SPA_POD_Int(udata->frame_size));
        }
    } else {
        udata->stream = pw_stream_new(udata->core, "example source", udata->stream_props);
        params[n_params++] = spa_pod_builder_add_object(&b,
                        SPA_TYPE_OBJECT_ParamBuffers, SPA_PARAM_Buffers,
                        SPA_PARAM_BUFFERS_buffers, SPA_POD_Int(udata->source_buf_count),
                        SPA_PARAM_BUFFERS_blocks,  0,
                        SPA_PARAM_BUFFERS_size,    SPA_POD_Int(udata->source_buf_size),
                        SPA_PARAM_BUFFERS_stride,  SPA_POD_Int(udata->frame_size));
    }

    if (udata->stream == NULL)
        return -errno;

    pw_stream_add_listener(udata->stream,
            &udata->stream_listener,
            &pw_pal_stream_events, udata);

    if (udata->is_offload) {
        params[n_params++] =  spa_pod_builder_add_object(&b,
                        SPA_TYPE_OBJECT_Format, SPA_PARAM_EnumFormat,
                        SPA_FORMAT_mediaType, SPA_POD_Id(SPA_MEDIA_TYPE_audio),
                        SPA_FORMAT_mediaSubtype, SPA_POD_Id(SPA_MEDIA_SUBTYPE_mp3),
                        SPA_FORMAT_AUDIO_format, SPA_POD_Id(SPA_AUDIO_FORMAT_ENCODED),
                        SPA_FORMAT_AUDIO_rate, SPA_POD_Int(44100),
                        SPA_FORMAT_AUDIO_channels, SPA_POD_Int(2));
    } else {
        params[n_params++] = spa_format_audio_raw_build(&b,
                        SPA_PARAM_EnumFormat, &udata->info);
    }

    res = pw_stream_connect(udata->stream,
              udata->isplayback ? PW_DIRECTION_INPUT : PW_DIRECTION_OUTPUT,
              PW_ID_ANY,
              PW_STREAM_FLAG_AUTOCONNECT |
              PW_STREAM_FLAG_NO_CONVERT |
              PW_STREAM_FLAG_MAP_BUFFERS |
              (udata->is_sva ? 0 : PW_STREAM_FLAG_RT_PROCESS),
              params, n_params);
    if (res < 0) {
        pw_log_error("pw_stream_connect failed: %s", spa_strerror(res));
        return res;
    }
    if (udata->is_offload)
        pw_stream_set_active(udata->stream, true);
    return 0;
}

static void pw_pal_core_error(void *data, uint32_t id, int seq, int res, const char *message)
{
    struct pw_userdata *udata = data;

    pw_log_error("error id:%u seq:%d res:%d (%s): %s",
            id, seq, res, spa_strerror(res), message);

    if (id == PW_ID_CORE && res == -EPIPE)
        pw_impl_module_schedule_destroy(udata->module);
}

static const struct pw_core_events pw_pal_events_core = {
    PW_VERSION_CORE_EVENTS,
    .error = pw_pal_core_error,
};

static void pw_pal_core_destroy(void *d)
{
    struct pw_userdata *udata = d; 
    spa_hook_remove(&udata->core_listener);
    udata->core = NULL;
    pw_impl_module_schedule_destroy(udata->module);
}

static const struct pw_proxy_events pw_pal_proxy_events_core = {
    .destroy = pw_pal_core_destroy,
};

static void pw_pal_userdata_destroy(struct pw_userdata *udata)
{
    close_pal_stream(udata);
    if (udata->sva_cfg.model_data) {
        free(udata->sva_cfg.model_data);
        udata->sva_cfg.model_data = NULL;
    }
    
    if (udata->stream)
        pw_stream_destroy(udata->stream);
    if (udata->pal_device) {
        free(udata->pal_device);
        udata->pal_device = NULL;
    }
    if (udata->stream_attributes) {
        free(udata->stream_attributes);
        udata->stream_attributes = NULL;
    }
    if (fcntl(udata->jack_fd, F_GETFD) != -1 || errno != EBADF)
        close(udata->jack_fd);
    if (udata->core && udata->do_disconnect)
        pw_core_disconnect(udata->core);
    pw_properties_free(udata->stream_props);
    pw_properties_free(udata->props);
    free(udata);
}

static void pw_pal_module_destroy(void *data)
{
    struct pw_userdata *udata = data;
    spa_hook_remove(&udata->module_listener);
    pw_pal_userdata_destroy(udata);
}

static const struct pw_impl_module_events pw_pal_events_module = {
    PW_VERSION_IMPL_MODULE_EVENTS,
    .destroy = pw_pal_module_destroy,
};

static inline uint32_t format_from_name(const char *name, size_t len)
{
    int i;
    for (i = 0; spa_type_audio_format[i].name; i++) {
        if (strncmp(name, spa_debug_type_short_name(spa_type_audio_format[i].name), len) == 0)
            return spa_type_audio_format[i].type;
    }
    return SPA_AUDIO_FORMAT_UNKNOWN;
}

static uint32_t pw_pal_get_channel(const char *name)
{
    int i;
    for (i = 0; spa_type_audio_channel[i].name; i++) {
        if (spa_streq(name, spa_debug_type_short_name(spa_type_audio_channel[i].name)))
            return spa_type_audio_channel[i].type;
    }
    return SPA_AUDIO_CHANNEL_UNKNOWN;
}

static void pw_pal_get_parse_position(struct spa_audio_info_raw *info, const char *val, size_t len)
{
    struct spa_json it[2];
    char v[256];

    spa_json_init(&it[0], val, len);
        if (spa_json_enter_array(&it[0], &it[1]) <= 0)
                spa_json_init(&it[1], val, len);

    info->channels = 0;
    while (spa_json_get_string(&it[1], v, sizeof(v)) > 0 &&
        info->channels < SPA_AUDIO_MAX_CHANNELS) {
        info->position[info->channels++] = pw_pal_get_channel(v);
    }
}

static void pw_pal_fetch_audio_info(const struct pw_properties *props, struct spa_audio_info_raw *info)
{
    const char *str;

    spa_zero(*info);

    if ((str = pw_properties_get(props, PW_KEY_AUDIO_FORMAT)) == NULL)
        str = PW_DEFAULT_SAMPLE_FORMAT;
    info->format = format_from_name(str, strlen(str));

    info->rate = pw_properties_get_uint32(props, PW_KEY_AUDIO_RATE, info->rate);
    if (info->rate == 0)
        info->rate = PW_DEFAULT_SAMPLE_RATE;

    info->channels = pw_properties_get_uint32(props, PW_KEY_AUDIO_CHANNELS, info->channels);
    info->channels = SPA_MIN(info->channels, SPA_AUDIO_MAX_CHANNELS);
    if ((str = pw_properties_get(props, SPA_KEY_AUDIO_POSITION)) != NULL)
        pw_pal_get_parse_position(info, str, strlen(str));
    if (info->channels == 0)
        pw_pal_get_parse_position(info, PW_DEFAULT_SAMPLE_POSITION, strlen(PW_DEFAULT_SAMPLE_POSITION));
}

static int pw_pal_get_frame_size(const struct spa_audio_info_raw *audio_info)
{
    int res = audio_info->channels;
    switch (audio_info->format) {
    case SPA_AUDIO_FORMAT_U8:
    case SPA_AUDIO_FORMAT_S8:
    case SPA_AUDIO_FORMAT_ALAW:
    case SPA_AUDIO_FORMAT_ULAW:
        return res;
    case SPA_AUDIO_FORMAT_S16:
    case SPA_AUDIO_FORMAT_S16_OE:
    case SPA_AUDIO_FORMAT_U16:
        return res * 2;
    case SPA_AUDIO_FORMAT_S24:
    case SPA_AUDIO_FORMAT_S24_OE:
    case SPA_AUDIO_FORMAT_U24:
        return res * 3;
    case SPA_AUDIO_FORMAT_S24_32:
    case SPA_AUDIO_FORMAT_S24_32_OE:
    case SPA_AUDIO_FORMAT_S32:
    case SPA_AUDIO_FORMAT_S32_OE:
    case SPA_AUDIO_FORMAT_U32:
    case SPA_AUDIO_FORMAT_U32_OE:
    case SPA_AUDIO_FORMAT_F32:
    case SPA_AUDIO_FORMAT_F32_OE:
        return res * 4;
    case SPA_AUDIO_FORMAT_F64:
    case SPA_AUDIO_FORMAT_F64_OE:
        return res * 8;
    default:
        return 0;
    }
}

static void pw_pal_set_props(struct pw_userdata *udata, struct pw_properties *props, const char *key)
{
    const char *str;
    if ((str = pw_properties_get(props, key)) != NULL) {
        if (pw_properties_get(udata->stream_props, key) == NULL)
            pw_properties_set(udata->stream_props, key, str);
    }
}

static size_t pw_stream_get_buffer_size(struct pw_userdata *udata, struct pal_media_config spec, pal_stream_type_t type)
{
        uint32_t buffer_duration = PW_DEFAULT_BUFFER_DURATION_MS;
        size_t length = 0, frames = 0;
        switch (type) {
        case PAL_STREAM_DEEP_BUFFER:
            buffer_duration = PW_DEEP_BUFFER_BUFFER_DURATION_MS;
            break;
        case PAL_STREAM_LOW_LATENCY:
            buffer_duration = PW_LOW_LATENCY_BUFFER_DURATION_MS;
        default:
            break;
        }

        frames = spec.sample_rate * buffer_duration;
        length = ((frames * udata->frame_size) / 1000);

        return (length/udata->frame_size) * udata->frame_size;
}
static int pw_pal_fill_stream_info(struct pw_userdata *udata)
{
    udata->stream_attributes = calloc(1, sizeof(struct pal_stream_attributes));
    if (!udata->stream_attributes) { pw_log_error("failed to allocate PAL stream attributes"); return -ENOMEM; }
    udata->stream_attributes->type  = udata->stream_type;
    udata->stream_attributes->flags = 0;
    if (!udata->is_sva) {
        udata->stream_attributes->info.opt_stream_info.version      = 1;
        udata->stream_attributes->info.opt_stream_info.duration_us  = -1;
        udata->stream_attributes->info.opt_stream_info.has_video    = false;
        udata->stream_attributes->info.opt_stream_info.is_streaming = false;
    } else {
        udata->stream_attributes->info.voice_rec_info.version = 1;
    }

    if (udata->isplayback) {
        udata->stream_attributes->direction = PAL_AUDIO_OUTPUT;
        udata->stream_attributes->out_media_config.bit_width = 16;
        udata->stream_attributes->out_media_config.ch_info.ch_map[0] = PAL_CHMAP_CHANNEL_FL;
        udata->stream_attributes->out_media_config.ch_info.ch_map[1] = PAL_CHMAP_CHANNEL_FR;
        udata->sink_buf_count = 4;
        if(!(udata->is_offload)) {
            udata->stream_attributes->out_media_config.sample_rate = udata->info.rate;
            switch (udata->stream_attributes->out_media_config.bit_width) {
                case 32: udata->stream_attributes->out_media_config.aud_fmt_id = PAL_AUDIO_FMT_PCM_S32_LE; break;
                case 24: udata->stream_attributes->out_media_config.aud_fmt_id = PAL_AUDIO_FMT_PCM_S24_3LE; break;
                default: udata->stream_attributes->out_media_config.aud_fmt_id = PAL_AUDIO_FMT_DEFAULT_PCM; break;
            }
            udata->stream_attributes->out_media_config.ch_info.channels = udata->info.channels;
            udata->sink_buf_size = pw_stream_get_buffer_size(udata, udata->stream_attributes->out_media_config, udata->stream_type);
        } else {
            udata->stream_attributes->flags  = PAL_STREAM_FLAG_NON_BLOCKING_MASK;
            udata->stream_attributes->out_media_config.sample_rate = 44100 ;
            udata->stream_attributes->out_media_config.ch_info.channels = 2;
            udata->stream_attributes->out_media_config.aud_fmt_id = PAL_AUDIO_FMT_DEFAULT_COMPRESSED;
            udata->sink_buf_size = 16484;
        }
    } else {
        udata->stream_attributes->direction = PAL_AUDIO_INPUT;
        if (!udata->is_sva) {
            udata->stream_attributes->in_media_config.sample_rate = udata->info.rate;
            udata->stream_attributes->in_media_config.bit_width = 16;
            switch (udata->stream_attributes->in_media_config.bit_width) {
                case 32: udata->stream_attributes->in_media_config.aud_fmt_id = PAL_AUDIO_FMT_PCM_S32_LE; break;
                case 24: udata->stream_attributes->in_media_config.aud_fmt_id = PAL_AUDIO_FMT_PCM_S24_3LE; break;
                default: udata->stream_attributes->in_media_config.aud_fmt_id = PAL_AUDIO_FMT_DEFAULT_PCM; break;
            }
            udata->stream_attributes->in_media_config.ch_info.channels = udata->info.channels;
            udata->stream_attributes->in_media_config.ch_info.ch_map[0] = PAL_CHMAP_CHANNEL_FL;
            udata->stream_attributes->in_media_config.ch_info.ch_map[1] = PAL_CHMAP_CHANNEL_FR;
            if (udata->info.channels >= 3)
                udata->stream_attributes->in_media_config.ch_info.ch_map[2] = PAL_CHMAP_CHANNEL_C;
            if (udata->info.channels >= 4) {
                udata->stream_attributes->in_media_config.ch_info.ch_map[2] = PAL_CHMAP_CHANNEL_LB;
                udata->stream_attributes->in_media_config.ch_info.ch_map[3] = PAL_CHMAP_CHANNEL_RB;
            }

            udata->source_buf_size  = 512;
            udata->source_buf_count = 8;
        } else {
            udata->stream_attributes->in_media_config.sample_rate  = udata->sva_cfg.sample_rate;
            udata->stream_attributes->in_media_config.bit_width    = SVA_BIT_WIDTH;
            udata->stream_attributes->in_media_config.aud_fmt_id   = PAL_AUDIO_FMT_PCM_S16_LE;
            udata->stream_attributes->in_media_config.ch_info.channels   = udata->sva_cfg.channels;
            udata->stream_attributes->in_media_config.ch_info.ch_map[0]  = PAL_CHMAP_CHANNEL_FL;
            udata->source_buf_size  = SVA_SOURCE_BUF_SIZE;
            udata->source_buf_count = SVA_SOURCE_BUF_COUNT;
        }
    }

    if (udata->pal_device) free(udata->pal_device);
    udata->pal_device = calloc(udata->no_of_devices, sizeof(struct pal_device));
    if (!udata->pal_device) {
    pw_log_error("failed to allocate PAL devices");
    free(udata->stream_attributes);
    udata->stream_attributes = NULL;
    return -ENOMEM;
}

    for (int i = 0; i < udata->no_of_devices; i++) {
        udata->pal_device[i].id = udata->pal_device_id[i];
        
        if (!udata->is_sva) {
            udata->pal_device[i].config.sample_rate = 48000;
            udata->pal_device[i].config.bit_width = 16;
            udata->pal_device[i].config.ch_info.channels = 2;
            udata->pal_device[i].config.ch_info.ch_map[0] = PAL_CHMAP_CHANNEL_FL;
            udata->pal_device[i].config.ch_info.ch_map[1] = PAL_CHMAP_CHANNEL_FR;
            udata->pal_device[i].config.aud_fmt_id = PAL_AUDIO_FMT_PCM_S16_LE;
        } else {
            udata->pal_device[i].config.sample_rate = SVA_SAMPLE_RATE;
            udata->pal_device[i].config.bit_width = SVA_BIT_WIDTH;
            udata->pal_device[i].config.ch_info.channels = 1;
            udata->pal_device[i].config.ch_info.ch_map[0] = PAL_CHMAP_CHANNEL_FL;
            udata->pal_device[i].config.aud_fmt_id = PAL_AUDIO_FMT_PCM_S16_LE;
        }
    } return 0;
}

static inline bool pw_stream_is_running(struct pw_userdata *udata)
{
    if (udata == NULL || udata->stream == NULL)
        return false;

    const char *err = NULL;
    enum pw_stream_state st = pw_stream_get_state(udata->stream, &err);

    if (st == PW_STREAM_STATE_ERROR) {
        pw_log_error("%s: stream is in ERROR state%s%s", __func__, err ? ": " : "", err ? err : "");
        return false;
    }

    return st == PW_STREAM_STATE_STREAMING;
}

static int handle_device_connection(struct pw_userdata *udata, bool state)
{
    int ret = 0;
    struct pal_device dev;
    if (!udata) return -EINVAL;

    if (udata->no_of_devices != 1) {
        pw_log_info("%s: combined playback selected, skip routing for jack '%s'",
            __func__, udata->jack_name);
        return 0;
    }

    pw_log_info("%s: processing device connection for jack '%s'", __func__, udata->jack_name);

    if (strstr(udata->jack_name, "DP")) {
        pal_param_device_connection_t *device_connection = (pal_param_device_connection_t *)
            calloc(1, sizeof(pal_param_device_connection_t));
        device_connection->connection_state = state;
        device_connection->id = udata->pal_device_id[0];
        ret = pal_set_param (PAL_PARAM_ID_DEVICE_CONNECTION, device_connection,
                sizeof(pal_param_device_connection_t));
        free(device_connection);
        return ret;
    }
    else if (strstr(udata->jack_name, "Headset")) {

        if (!pw_stream_is_running(udata) || !udata->stream_handle) {
            pw_log_error("%s: stream not streaming; skip headset routing", __func__);
            return 0;
        }

        const pal_device_id_t target = udata->isplayback
            ? (state ? PAL_DEVICE_OUT_WIRED_HEADSET : PAL_DEVICE_OUT_SPEAKER)
            : (state ? PAL_DEVICE_IN_WIRED_HEADSET  : PAL_DEVICE_IN_SPEAKER_MIC);

        memset(&dev, 0, sizeof(dev));
        dev.id = target;

        ret = pal_stream_set_device(udata->stream_handle, 1, &dev);
        if (ret) {
            pw_log_error("%s: pal_stream_set_device(%d) failed: %d", __func__, target, ret);
            return ret;
        }
        return 0;
    }
    return 0;
}


static void handle_jack_boot_event(struct pw_userdata *udata)
{
    int fd = udata->jack_fd;
    int connected = 0;
    uint8_t sw_bitmask[NUM_BYTES];

    memset(sw_bitmask, 0, sizeof(sw_bitmask));
    // Query current switch state
    if (ioctl(fd, EVIOCGSW(sizeof(sw_bitmask)), sw_bitmask) >= 0) {
        // Check for HDMI/DP jack state
        if (BIT_VALUE(SW_LINEOUT_INSERT, sw_bitmask))
            connected = 1;

        if (connected) {
            pw_log_info("%s: Connected (boot time)", udata->jack_name);
            if(handle_device_connection(udata, true))
                pw_log_error("Failed to handle device connection");
        }
    } else {
        pw_log_error("Failed to query initial jack state");
    }
}

static void on_jack_event(void *userdata, int fd, uint32_t mask)
{
    struct pw_userdata *udata = userdata;
    char name[256] = {0,};
    struct input_event ev;
    bool connected;
    int rc;

    ioctl(fd, EVIOCGNAME(sizeof(name)), name);
    if (!strstr(name, udata->jack_name))
        return;

    if (mask & (SPA_IO_ERR | SPA_IO_HUP)) {
        pw_log_error("error or hang-up on the hdmi/dp fd");
        pw_loop_destroy_source(pw_context_get_main_loop(udata->context), udata->jack_src);
        udata->jack_src = NULL;

        if (udata->jack_fd >= 0) {
            close(udata->jack_fd);
            udata->jack_fd = -1;
        }
        return;
    }

    ssize_t ret = read(fd, &ev, sizeof(ev));

    if (ret == sizeof(ev)) {
        if (ev.type != EV_SW)
            return;

        if (ev.code == SW_LINEOUT_INSERT || ev.code == SW_HEADPHONE_INSERT) {
            const char *state = ev.value ? "Connected" : "Disconnected";
            pw_log_info("Jack (%s): %s", udata->jack_name, state);

            if (handle_device_connection(udata, ev.value ? true : false))
                pw_log_error("Failed to handle %s device connection",udata->jack_name);
        }
    } else if (ret < 0) {
        pw_log_error("Error reading event: %s", strerror(errno));
    } else {
        pw_log_error("Short read: got %zd bytes", ret);
    }
}

static int jack_open_fd(struct pw_userdata *udata)
{
    int ret = 0;
    DIR *d = opendir(DEV_INPUT_DIR);
    if (d == NULL) {
        pw_log_error("opendir() failed");
        ret = -ENOENT;
        goto exit;
    }

    struct dirent *dir;
    while ((dir = readdir(d)) != NULL) {
        /* Filter out non block devices and files that don't have
           the right prefix. */
        if (dir->d_type != DT_CHR ||
                strncmp(FILE_PREFIX, dir->d_name,
                    strlen(FILE_PREFIX)) != 0)
            continue;

        char filepath[512];
        snprintf(filepath, sizeof(filepath), "%s/%s",
            DEV_INPUT_DIR, dir->d_name);
        int fd = open(filepath, O_RDONLY);
        if (fd < 0) {
            pw_log_error("open() failed %s", filepath);
            continue;
        }

        char name[256] = {0,};
        ioctl(fd, EVIOCGNAME(sizeof(name)), name);

        /* Search for the keyword in the input's name. */
        if (strstr(name, udata->jack_name) != NULL) {
            closedir(d);
            return fd;
        }

        close(fd);
    }

    closedir(d);
    ret = -EINVAL;
exit:
    pw_log_error("No jack input device found in %s", DEV_INPUT_DIR);
    return ret;
}

static int jack_register(struct pw_userdata *udata)
{
    int ret = 0;
    udata->jack_fd = jack_open_fd(udata);
    if (udata->jack_fd < 0) {
        ret = -EBADFD;
        goto exit;
    }

    ret = fcntl(udata->jack_fd, F_SETFL, O_NONBLOCK);
    if (ret < 0) {
        pw_log_error("fcntl() failed");
        goto exit;
    }

    udata->jack_src = pw_loop_add_io(pw_context_get_main_loop(udata->context), udata->jack_fd,
            SPA_IO_IN | SPA_IO_ERR | SPA_IO_HUP, true, on_jack_event, udata);

    if (udata->jack_src == NULL) {
        pw_log_error("pw_loop_add_io failed for jack_fd=%d", udata->jack_fd);
        ret = -EIO;
    }

exit:
    return ret;
}


SPA_EXPORT
int pipewire__module_init(struct pw_impl_module *module, const char *args)
{
    struct pw_context *context = pw_impl_module_get_context(module);
    struct pw_properties *props = NULL;
    const char *offload = NULL;
    uint32_t id = pw_global_get_id(pw_impl_module_get_global(module));
    uint32_t pid = getpid();
    struct pw_userdata *udata;
    const char *str, *value, *role;
    int res = 0;

    PW_LOG_TOPIC_INIT(log_topic);

    udata = calloc(1, sizeof(struct pw_userdata));
    if (udata == NULL)
        return -errno;
    udata->jack_fd = -1;
    if (args == NULL)
        args = "";

    props = pw_properties_new_string(args);
    if (props == NULL) {
        res = -errno;
        pw_log_error( "can't create properties: %m");
        goto error;
    }
    udata->props = props;

    udata->stream_props = pw_properties_new(NULL, NULL);
    if (udata->stream_props == NULL) {
        res = -errno;
        pw_log_error( "can't create properties: %m");
        goto error;
    }

    udata->module = module;
    udata->context = context;
    res = agm_init();
    if (res) {
        pw_log_error("%s: agm init failed\n", __func__);
        goto error;
    }
    res = pal_init();
    if (res) {
        pw_log_error("%s: pal init failed\n", __func__);
        goto error;
    }
    if (pw_properties_get(props, PW_KEY_NODE_VIRTUAL) == NULL)
        pw_properties_set(props, PW_KEY_NODE_VIRTUAL, "true");

    if (pw_properties_get(props, PW_KEY_MEDIA_CLASS) == NULL)
        pw_properties_set(props, PW_KEY_MEDIA_CLASS, "Audio/Sink");

    value = pw_properties_get(props, PW_KEY_MEDIA_CLASS);
    if (value) {
        if (strstr(value, "Sink")) {
            udata->isplayback = true;
            udata->pal_device_id[0] = PAL_DEVICE_OUT_SPEAKER;
        }
        else {
            udata->isplayback = false;
            if (!udata->is_sva) {
                udata->pal_device_id[0] = PAL_DEVICE_IN_SPEAKER_MIC;
            }
        }
    }

    udata->no_of_devices = 1;
    value = pw_properties_get(props, PW_KEY_NODE_NAME);
    if (value) {
        if (strstr(value, "pal_sink_speaker"))
            udata->pal_device_id[0] = PAL_DEVICE_OUT_SPEAKER;
        else if (strstr(value, "pal_sink_headset"))
            udata->pal_device_id[0] = PAL_DEVICE_OUT_WIRED_HEADSET;
        else if (strstr(value, "pal_source_speaker_mic"))
            udata->pal_device_id[0] = PAL_DEVICE_IN_SPEAKER_MIC;
        else if (strstr(value, "pal_source_headset_mic"))
            udata->pal_device_id[0] = PAL_DEVICE_IN_WIRED_HEADSET;
        else if (strstr(value, "pal_sink_dp_out"))
            udata->pal_device_id[0] = PAL_DEVICE_OUT_AUX_DIGITAL;
        else if (strstr(value, "pal_sink_hdmi_out"))
            udata->pal_device_id[0] = PAL_DEVICE_OUT_HDMI;
        else if (strstr(value, "pal_sink_combined")) {
            udata->pal_device_id[0] = PAL_DEVICE_OUT_WIRED_HEADSET;
            udata->pal_device_id[1] = PAL_DEVICE_OUT_SPEAKER;
            udata->no_of_devices = 2;
        }
        else if (strstr(value, "pal_source_voice_ui")) {
            udata->pal_device_id[0] = PAL_DEVICE_IN_HANDSET_VA_MIC;
            udata->is_sva = true;  
        }
    }

    if (pw_properties_get(props, PW_KEY_MEDIA_ROLE) == NULL) {
        if (!udata->is_sva)
            pw_properties_set(props, PW_KEY_MEDIA_ROLE, "notification");
        else
            pw_properties_set(props, PW_KEY_MEDIA_ROLE, "VoiceUI");
    }

    role = pw_properties_get(props, PW_KEY_MEDIA_ROLE);

    if (udata->is_sva || (role && strstr(role, "VoiceUI"))) {
        udata->stream_type = PAL_STREAM_VOICE_UI;
        udata->is_sva = true;
    } else if (role && udata->isplayback) { 
            if (strstr(role,"music"))
                udata->stream_type = PAL_STREAM_DEEP_BUFFER;
            else
                udata->stream_type = PAL_STREAM_LOW_LATENCY;
    } else {
        udata->stream_type = PAL_STREAM_DEEP_BUFFER;
    }

    if (pw_properties_get(props, PW_KEY_NODE_NAME) == NULL)
        pw_properties_setf(props, PW_KEY_NODE_NAME, "example-sink-%u-%u", pid, id);

    if (pw_properties_get(props, PW_KEY_NODE_DESCRIPTION) == NULL)
        pw_properties_set(props, PW_KEY_NODE_DESCRIPTION,
        pw_properties_get(props, PW_KEY_NODE_NAME));

    if ((str = pw_properties_get(props, "stream.props")) != NULL)
        pw_properties_update_string(udata->stream_props, str, strlen(str));

    offload = pw_properties_get(udata->stream_props, "compress.offload");
    udata->is_offload = offload && strcmp(offload, "true") == 0;

    if ((str = pw_properties_get(props, "jack-name")) != NULL)
        snprintf(udata->jack_name, sizeof(udata->jack_name), "%s", str);

    pw_pal_set_props(udata, props, PW_KEY_AUDIO_RATE);
    pw_pal_set_props(udata, props, PW_KEY_AUDIO_CHANNELS);
    pw_pal_set_props(udata, props, SPA_KEY_AUDIO_POSITION);
    pw_pal_set_props(udata, props, PW_KEY_NODE_NAME);
    pw_pal_set_props(udata, props, PW_KEY_NODE_DESCRIPTION);
    pw_pal_set_props(udata, props, PW_KEY_NODE_GROUP);
    pw_pal_set_props(udata, props, PW_KEY_NODE_LATENCY);
    pw_pal_set_props(udata, props, PW_KEY_NODE_VIRTUAL);
    pw_pal_set_props(udata, props, PW_KEY_MEDIA_CLASS);

    pw_pal_fetch_audio_info(udata->stream_props, &udata->info);
    if (udata->is_sva) {
        
        udata->frame_size    = SVA_FRAME_SIZE_BYTES; 
    } else if (!udata->is_offload) {
        udata->frame_size = pw_pal_get_frame_size(&udata->info);
        if (udata->frame_size == 0) {
            res = -EINVAL;
            pw_log_error("can't parse audio format");
            goto error;
        }
    } else {
        udata->stream_type = PAL_STREAM_COMPRESSED;
        udata->frame_size = 16;
        pw_properties_set(udata->stream_props, PW_KEY_MEDIA_CLASS, "Audio/Sink");
        pw_properties_set(udata->stream_props, PW_KEY_AUDIO_FORMAT, "encoded");
        pw_properties_set(udata->stream_props, "audio.coding.format", "mp3");
    }
    udata->core = pw_context_get_object(udata->context, PW_TYPE_INTERFACE_Core);
    if (udata->core == NULL) {
        str = pw_properties_get(props, PW_KEY_REMOTE_NAME);
        udata->core = pw_context_connect(udata->context,
                pw_properties_new(
                    PW_KEY_REMOTE_NAME, str,
                    NULL),
                0);
        udata->do_disconnect = true;
    }

    if (udata->core == NULL) {
        res = -errno;
        pw_log_error("can't connect: %m");
        goto error;
    }

    pw_proxy_add_listener((struct pw_proxy*)udata->core,
            &udata->core_proxy_listener,
            &pw_pal_proxy_events_core, udata);
    pw_core_add_listener(udata->core,
            &udata->core_listener,
            &pw_pal_events_core, udata);

    if (udata->is_sva) {
        pw_log_info("SVA: VoiceUI node initialized");
        udata->sva_state = SVA_STATE_IDLE;
        /* Parse SVA config from module args */
        if ((str = pw_properties_get(props, "sva.sound.model.path")) != NULL)
            snprintf(udata->sva_cfg.model_path, sizeof(udata->sva_cfg.model_path), "%s", str);
        if ((str = pw_properties_get(props, "sva.confidence.level")) != NULL)
            udata->sva_cfg.confidence_level = atoi(str);
        else
            udata->sva_cfg.confidence_level = SVA_PDK_CONFIDENCE_LEVEL_DEFAULT;
        if ((str = pw_properties_get(props, "sva.lab.duration")) != NULL)
            udata->sva_cfg.lab_duration = atoi(str);
        else
            udata->sva_cfg.lab_duration = SVA_LAB_DURATION;
        if ((str = pw_properties_get(props, "sva.lab.enabled")) != NULL)
            udata->sva_cfg.lab_enabled = spa_streq(str, "true");
        else
            udata->sva_cfg.lab_enabled = true;
        if ((str = pw_properties_get(props, "sva.mode")) != NULL)
            snprintf(udata->sva_cfg.mode, sizeof(udata->sva_cfg.mode), "%s", str);
        else
            snprintf(udata->sva_cfg.mode, sizeof(udata->sva_cfg.mode), "NLPI");
        if ((str = pw_properties_get(props, "sva.sample.rate")) != NULL)
            udata->sva_cfg.sample_rate = SVA_SAMPLE_RATE;
        else
            udata->sva_cfg.sample_rate = SVA_SAMPLE_RATE;
        if ((str = pw_properties_get(props, "sva.channels")) != NULL)
            udata->sva_cfg.channels = 1;
        else
            udata->sva_cfg.channels = 1;
        if ((str = pw_properties_get(props, "sva.bit.width")) != NULL)
            udata->sva_cfg.bit_width = SVA_BIT_WIDTH;
        else
            udata->sva_cfg.bit_width = SVA_BIT_WIDTH;
        pw_log_info("SVA config: model=%s confidence=%u mode=%s sr=%u ch=%u",
                    udata->sva_cfg.model_path,
                    udata->sva_cfg.confidence_level,
                    udata->sva_cfg.mode,
                    udata->sva_cfg.sample_rate,
                    udata->sva_cfg.channels);
        /* Create the SVA PipeWire stream with the fixed V1 format. */
        udata->info.format = SPA_AUDIO_FORMAT_S16_LE;
        udata->info.rate = SVA_SAMPLE_RATE;
        udata->info.channels = 1;
        udata->info.position[0] = SPA_AUDIO_CHANNEL_MONO;
        udata->frame_size = SVA_FRAME_SIZE_BYTES;
        if ((res = pw_pal_fill_stream_info(udata)) < 0)
            goto error;
        if ((res = pw_pal_create_stream(udata)) < 0)
            goto error;
        pw_impl_module_add_listener(module, &udata->module_listener,
                                    &pw_pal_events_module, udata);
        /* Auto-load model if path provided in module args */
        if (udata->sva_state == SVA_STATE_IDLE && udata->sva_cfg.model_path[0] != '\0') {
            int rc = sva_start_recognition(udata);
            if (rc != 0)
                pw_log_error("SVA: auto recognition start failed rc=%d", rc);
        } else {
            pw_log_warn("SVA: No model path — waiting for runtime trigger via pw-voiceui");
        }
        return 0;   
    }

    if ((res = pw_pal_fill_stream_info(udata)) < 0)
        goto error;
    if ((res = pw_pal_create_stream(udata)) < 0)
        goto error;
    pw_impl_module_add_listener(module, &udata->module_listener, &pw_pal_events_module, udata);
    if (udata->jack_name && udata->jack_name[0] != '\0') {
        if(jack_register(udata))
            pw_log_error("failed to register jack event for %s", udata->jack_name);
        else {
            handle_jack_boot_event(udata);
        }
    }
    return 0;

error:
    pw_pal_userdata_destroy(udata);
    return res;
}
