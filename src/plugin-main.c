/*
 * Mic Reverb + Echo: нативный аудиофильтр для OBS Studio.
 *
 * Реверберация: Freeverb (8 comb-фильтров + 4 all-pass на канал).
 * Эхо: линия задержки с обратной связью.
 * Сухой сигнал всегда идёт с полной громкостью, эффекты добавляются сверху,
 * поэтому уровни "Реверб" и "Эхо" работают как регуляторы wet-микса.
 */

#include <obs-module.h>
#include <media-io/audio-io.h>
#include <util/bmem.h>
#include <math.h>
#include <string.h>

OBS_DECLARE_MODULE()
OBS_MODULE_USE_DEFAULT_LOCALE("mic-reverb-echo", "en-US")

#define S_REVERB_MIX "reverb_mix"
#define S_ROOM_SIZE "room_size"
#define S_DAMPING "damping"
#define S_ECHO_MIX "echo_mix"
#define S_ECHO_DELAY "echo_delay_ms"
#define S_ECHO_FEEDBACK "echo_feedback"

#define T_REVERB_MIX obs_module_text("ReverbMix")
#define T_ROOM_SIZE obs_module_text("RoomSize")
#define T_DAMPING obs_module_text("Damping")
#define T_ECHO_MIX obs_module_text("EchoMix")
#define T_ECHO_DELAY obs_module_text("EchoDelay")
#define T_ECHO_FEEDBACK obs_module_text("EchoFeedback")

#define NUM_COMBS 8
#define NUM_ALLPASS 4
#define STEREO_SPREAD 23
#define MAX_ECHO_SECONDS 2

static const int comb_tuning[NUM_COMBS] = {1116, 1188, 1277, 1356,
					   1422, 1491, 1557, 1617};
static const int allpass_tuning[NUM_ALLPASS] = {556, 441, 341, 225};

struct comb {
	float *buf;
	int size;
	int idx;
	float store;
};

struct allpass {
	float *buf;
	int size;
	int idx;
};

struct channel_state {
	struct comb combs[NUM_COMBS];
	struct allpass aps[NUM_ALLPASS];
	float *echo_buf;
	int echo_size;
	int echo_idx;
};

struct reverb_echo {
	obs_source_t *context;
	size_t channels;
	uint32_t sample_rate;
	struct channel_state ch[MAX_AUDIO_CHANNELS];
	uint64_t next_ts;

	/* параметры (пишутся из UI-потока, читаются из аудио-потока) */
	float reverb_mix;
	float feedback;
	float damp1;
	float damp2;
	float echo_mix;
	float echo_fb;
	int echo_delay;
};

static inline float flush_denormal(float v)
{
	return fabsf(v) < 1e-20f ? 0.0f : v;
}

static int scaled_size(int tuning, uint32_t sample_rate)
{
	int size = (int)((double)tuning * (double)sample_rate / 44100.0);
	return size < 1 ? 1 : size;
}

static void alloc_state(struct reverb_echo *f)
{
	for (size_t c = 0; c < f->channels; c++) {
		struct channel_state *st = &f->ch[c];

		for (int i = 0; i < NUM_COMBS; i++) {
			int size = scaled_size(comb_tuning[i] + (int)c * STEREO_SPREAD,
					       f->sample_rate);
			st->combs[i].size = size;
			st->combs[i].buf = bzalloc(sizeof(float) * (size_t)size);
		}
		for (int i = 0; i < NUM_ALLPASS; i++) {
			int size = scaled_size(allpass_tuning[i] + (int)c * STEREO_SPREAD,
					       f->sample_rate);
			st->aps[i].size = size;
			st->aps[i].buf = bzalloc(sizeof(float) * (size_t)size);
		}
		st->echo_size = (int)(f->sample_rate * MAX_ECHO_SECONDS) + 1;
		st->echo_buf = bzalloc(sizeof(float) * (size_t)st->echo_size);
	}
}

static void free_state(struct reverb_echo *f)
{
	for (size_t c = 0; c < f->channels; c++) {
		struct channel_state *st = &f->ch[c];
		for (int i = 0; i < NUM_COMBS; i++)
			bfree(st->combs[i].buf);
		for (int i = 0; i < NUM_ALLPASS; i++)
			bfree(st->aps[i].buf);
		bfree(st->echo_buf);
	}
}

static void clear_state(struct reverb_echo *f)
{
	for (size_t c = 0; c < f->channels; c++) {
		struct channel_state *st = &f->ch[c];
		for (int i = 0; i < NUM_COMBS; i++) {
			memset(st->combs[i].buf, 0, sizeof(float) * (size_t)st->combs[i].size);
			st->combs[i].store = 0.0f;
		}
		for (int i = 0; i < NUM_ALLPASS; i++)
			memset(st->aps[i].buf, 0, sizeof(float) * (size_t)st->aps[i].size);
		memset(st->echo_buf, 0, sizeof(float) * (size_t)st->echo_size);
	}
}

/* Горячая клавиша: включить/выключить фильтр (как глазок в окне фильтров). */
static void re_toggle_hotkey(void *data, obs_hotkey_id id, obs_hotkey_t *hotkey, bool pressed)
{
	UNUSED_PARAMETER(id);
	UNUSED_PARAMETER(hotkey);
	struct reverb_echo *f = data;
	if (!pressed)
		return;
	obs_source_set_enabled(f->context, !obs_source_enabled(f->context));
}

static const char *re_get_name(void *unused)
{
	UNUSED_PARAMETER(unused);
	return obs_module_text("MicReverbEcho");
}

static void re_update(void *data, obs_data_t *settings)
{
	struct reverb_echo *f = data;

	f->reverb_mix = (float)obs_data_get_double(settings, S_REVERB_MIX) / 100.0f;

	float room = (float)obs_data_get_double(settings, S_ROOM_SIZE) / 100.0f;
	f->feedback = room * 0.28f + 0.7f;

	float damp = (float)obs_data_get_double(settings, S_DAMPING) / 100.0f * 0.4f;
	f->damp1 = damp;
	f->damp2 = 1.0f - damp;

	f->echo_mix = (float)obs_data_get_double(settings, S_ECHO_MIX) / 100.0f;
	f->echo_fb = (float)obs_data_get_double(settings, S_ECHO_FEEDBACK) / 100.0f;

	double ms = obs_data_get_double(settings, S_ECHO_DELAY);
	int delay = (int)(ms * (double)f->sample_rate / 1000.0);
	int max_delay = f->sample_rate * MAX_ECHO_SECONDS;
	if (delay < 1)
		delay = 1;
	if (delay > max_delay)
		delay = max_delay;
	f->echo_delay = delay;
}

static void *re_create(obs_data_t *settings, obs_source_t *source)
{
	struct reverb_echo *f = bzalloc(sizeof(struct reverb_echo));
	f->context = source;

	audio_t *audio = obs_get_audio();
	f->sample_rate = audio_output_get_sample_rate(audio);
	f->channels = audio_output_get_channels(audio);
	if (f->sample_rate == 0)
		f->sample_rate = 48000;
	if (f->channels > MAX_AUDIO_CHANNELS)
		f->channels = MAX_AUDIO_CHANNELS;

	alloc_state(f);
	re_update(f, settings);

	obs_hotkey_register_source(source, "mic_reverb_echo.toggle",
				   obs_module_text("ToggleHotkey"), re_toggle_hotkey, f);
	return f;
}

static void re_destroy(void *data)
{
	struct reverb_echo *f = data;
	free_state(f);
	bfree(f);
}

static void re_defaults(obs_data_t *settings)
{
	obs_data_set_default_double(settings, S_REVERB_MIX, 25.0);
	obs_data_set_default_double(settings, S_ROOM_SIZE, 60.0);
	obs_data_set_default_double(settings, S_DAMPING, 50.0);
	obs_data_set_default_double(settings, S_ECHO_MIX, 0.0);
	obs_data_set_default_double(settings, S_ECHO_DELAY, 300.0);
	obs_data_set_default_double(settings, S_ECHO_FEEDBACK, 35.0);
}

static obs_properties_t *re_properties(void *unused)
{
	UNUSED_PARAMETER(unused);
	obs_properties_t *p = obs_properties_create();
	obs_property_t *prop;

	prop = obs_properties_add_float_slider(p, S_REVERB_MIX, T_REVERB_MIX, 0.0, 100.0, 1.0);
	obs_property_float_set_suffix(prop, " %");
	prop = obs_properties_add_float_slider(p, S_ROOM_SIZE, T_ROOM_SIZE, 0.0, 100.0, 1.0);
	obs_property_float_set_suffix(prop, " %");
	prop = obs_properties_add_float_slider(p, S_DAMPING, T_DAMPING, 0.0, 100.0, 1.0);
	obs_property_float_set_suffix(prop, " %");

	prop = obs_properties_add_float_slider(p, S_ECHO_MIX, T_ECHO_MIX, 0.0, 100.0, 1.0);
	obs_property_float_set_suffix(prop, " %");
	prop = obs_properties_add_float_slider(p, S_ECHO_DELAY, T_ECHO_DELAY, 20.0, 2000.0, 5.0);
	obs_property_float_set_suffix(prop, " ms");
	prop = obs_properties_add_float_slider(p, S_ECHO_FEEDBACK, T_ECHO_FEEDBACK, 0.0, 90.0, 1.0);
	obs_property_float_set_suffix(prop, " %");

	return p;
}

static struct obs_audio_data *re_filter_audio(void *data, struct obs_audio_data *audio)
{
	struct reverb_echo *f = data;
	const size_t frames = audio->frames;

	/* После паузы (фильтр был выключен) очищаем хвосты, чтобы не звучал старый звук. */
	if (f->next_ts == 0 || audio->timestamp > f->next_ts + 50000000ULL)
		clear_state(f);
	f->next_ts = audio->timestamp + (uint64_t)frames * 1000000000ULL / f->sample_rate;

	for (size_t c = 0; c < f->channels; c++) {
		float *x = (float *)audio->data[c];
		if (!x)
			continue;

		struct channel_state *st = &f->ch[c];

		for (size_t n = 0; n < frames; n++) {
			const float dry = x[n];

			/* --- реверб (Freeverb) --- */
			const float in = dry * 0.03f;
			float rev = 0.0f;

			for (int i = 0; i < NUM_COMBS; i++) {
				struct comb *cb = &st->combs[i];
				float out = cb->buf[cb->idx];
				cb->store = flush_denormal(out * f->damp2 + cb->store * f->damp1);
				cb->buf[cb->idx] = in + cb->store * f->feedback;
				if (++cb->idx >= cb->size)
					cb->idx = 0;
				rev += out;
			}

			for (int i = 0; i < NUM_ALLPASS; i++) {
				struct allpass *ap = &st->aps[i];
				float bufout = ap->buf[ap->idx];
				float out = -rev + bufout;
				ap->buf[ap->idx] = rev + bufout * 0.5f;
				if (++ap->idx >= ap->size)
					ap->idx = 0;
				rev = out;
			}

			/* --- эхо --- */
			int r = st->echo_idx - f->echo_delay;
			if (r < 0)
				r += st->echo_size;
			const float echo = st->echo_buf[r];
			st->echo_buf[st->echo_idx] = flush_denormal(dry + echo * f->echo_fb);
			if (++st->echo_idx >= st->echo_size)
				st->echo_idx = 0;

			x[n] = dry + rev * f->reverb_mix * 3.0f + echo * f->echo_mix;
		}
	}

	return audio;
}

static struct obs_source_info mic_reverb_echo_filter = {
	.id = "mic_reverb_echo_filter",
	.type = OBS_SOURCE_TYPE_FILTER,
	.output_flags = OBS_SOURCE_AUDIO,
	.get_name = re_get_name,
	.create = re_create,
	.destroy = re_destroy,
	.update = re_update,
	.get_defaults = re_defaults,
	.get_properties = re_properties,
	.filter_audio = re_filter_audio,
};

bool obs_module_load(void)
{
	obs_register_source(&mic_reverb_echo_filter);
	blog(LOG_INFO, "[mic-reverb-echo] loaded");
	return true;
}

void obs_module_unload(void)
{
}
