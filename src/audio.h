// audio.h -- the game's sound, over SDL_mixer:
//
//   one-shots  a track per fire (capped, reaped when done) -- the
//              decoupler pop
//   the loop   one persistent track, infinite loops -- the engine hum;
//              gain tracks the throttle
//   the music  one persistent track, infinite loops -- ambient
//
// Everything plays at FULL level (no positional audio): in a tracking
// camera the ship sits at a fixed distance, so attenuation would be a
// constant, and there is no air in space to carry a sound by anyway.
// listenerRelative() below is kept as the pure-math utility for if we
// ever want positional playback (the mixer's OpenAL-style listener frame,
// +x right / +y up / -z forward) -- the game does not use it today.
//
// Everything degrades to SILENCE: no audio device (headless, the e2e
// battery under Xvfb) or a missing file just means that sound never
// happens -- init() / loadAudio() report it once and every call after
// is a no-op, so the game runs exactly as it did before audio existed.
#pragma once

#include <SDL3/SDL.h>
#include <SDL3_mixer/SDL_mixer.h>

#include <map>
#include <string>
#include <vector>

#include <glm/glm.hpp>

/* A source's WORLD position converted to the mixer's listener frame
   (the listener at the origin, looking down -z). The camera supplies
   the listener's forward (view direction) and screen up; the world is
   y-up. Pure math -- unit-testable without a mixer (test_audio). */
inline glm::dvec3 listenerRelative(const glm::dvec3 &listener,
                                   const glm::dvec3 &listenerUp,
                                   const glm::dvec3 &listenerFwd,
                                   const glm::dvec3 &source)
{
    const glm::dvec3 f = glm::normalize(listenerFwd);
    glm::dvec3 r = glm::cross(f, listenerUp);
    if(glm::length(r) < 1e-9) { r = glm::dvec3(1.0, 0.0, 0.0); }   // up || forward: any right axis works
    r = glm::normalize(r);
    const glm::dvec3 u = glm::cross(r, f);
    const glm::dvec3 rel = source - listener;
    return glm::dvec3(glm::dot(rel, r), glm::dot(rel, u), -glm::dot(rel, f));
}

class Audio {
public:
    /* MIX_Init + open the default playback device. false (no device)
       -> the object stays silent forever; every method below is a no-op. */
    bool init();
    void shutdown();

    bool enabled() const { return mixer_ != nullptr; }

    /* The device sample rate (0 when disabled). Backends negotiate
       differently -- Pulse/PipeWire usually 48 kHz, ALSA the hardware's
       44.1 kHz -- so callers can pick a matching asset. */
    int deviceRate() const;

    /* One-shot SFX (the decoupler pop). Capped at MAX_ONESHOTS concurrent;
       finished tracks are reaped on update(). `balance` (default 1.0) is a
       per-sound level trim relative to the SFX master: a full-scale transient
       (peak 0 dB) reads far louder than a steady loop at the same gain, so a
       hot one-shot is pulled down here (the decoupler uses 0.4). */
    void playOnce(const std::string &path, float balance = 1.0f);

    /* The looping SFX (the engine hum). active starts/stops it (a 150 ms
       fade on the stop so the cutoff doesn't clip); gain in [0,1] is the
       throttle. Full level -- the ship's own engine, no distance falloff
       (there is no air to carry it in space). */
    void setLoop(const std::string &path, bool active, float gain);

    /* Ambient music: load once (decoded on playback, not pre-expanded),
       loop forever. A missing file is a logged no-op. */
    void setMusic(const std::string &path);
    void stopMusic();

    /* Master levels in [0,1] (the Settings window drives these). */
    void setSfxVolume(float v);
    void setMusicVolume(float v);

    /* Per frame: reaps finished one-shots and completes any engine
       stop-fade. No-op when disabled/idle. */
    void update();

private:
    static const size_t MAX_ONESHOTS = 8;

    /* Cached per path. `predecode` = decode the whole file into PCM at
       load time (the real-time callback then only copies samples). The
       SFX are tiny so that is free; the music is a 10-minute OGG whose
       full decode costs ~1 s of CPU + ~200 MB of RAM at boot -- so it
       streams instead (see audio.cpp). */
    MIX_Audio *loadAudio(const std::string &path, bool predecode);   // cached per path; null on miss

    /* Grow the track's internal mix buffers on the MAIN thread (a brief
       play/stop) so the real-time callback never hits its first
       SDL_realloc. A track created right before its first play allocates
       those buffers inside the audio callback -- on a thread that must
       never block -- which underruns on ALSA (the "crack" a tap produced).
       A reused track is already warm and needs no priming. */
    void primeTrack(MIX_Track *t);

    struct Shot {
        MIX_Track *t;
        Uint32 born_ms = 0;   // for the debug log: how long the pop actually lived
    };

    bool dbg_ = false;   // AUDIO_DEBUG=1: trace every audio event (headless debugging)

    MIX_Mixer *mixer_ = nullptr;

    float sfxVol_ = 1.0f;     // master for the one-shots + the loop
    float musicVol_ = 0.5f;   // ambient music (background, not the star)

    std::map<std::string, MIX_Audio *> audios_;      // loaded once per path

    std::vector<Shot> oneShots_;                     // capped; reaped in update()

    MIX_Track *loop_ = nullptr;
    std::string loopPath_;
    bool loopActive_ = false;
    float loopGain_ = 0.0f;   // the throttle; the gain applied is gain*sfxVol_
    // A stop-fade is in flight: MIX_StopTrack(fade) leaves the track "playing"
    // until the fade drains, so this flag arms the fade exactly once (a
    // re-armed fade never completed -- the old "sticky engine"); update()
    // reaps the track when the fade finishes.
    bool loopStopping_ = false;

    MIX_Track *music_ = nullptr;
    std::string musicPath_;
};
