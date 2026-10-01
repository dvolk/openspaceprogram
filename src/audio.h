// audio.h -- the game's sound over SDL_mixer: one-shots (decoupler pop),
// an engine loop (gain = throttle), and ambient music.
// Full level (no positional audio -- no air in space). Everything degrades
// to silence: no device or missing file means that sound never happens.
#pragma once

#include <SDL3/SDL.h>
#include <SDL3_mixer/SDL_mixer.h>

#include <map>
#include <string>
#include <vector>

#include <glm/glm.hpp>

/* A source's world position in the mixer's listener frame (+x right,
   +y up, -z forward). Pure math, unit-testable without a mixer. */
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
    // MIX_Init + open the default playback device. false -> silent forever.
    bool init();
    void shutdown();

    bool enabled() const { return mixer_ != nullptr; }

    // Device sample rate (0 when disabled); backends negotiate differently.
    int deviceRate() const;

    // One-shot SFX. `balance` trims a hot one-shot relative to the SFX master.
    void playOnce(const std::string &path, float balance = 1.0f);

    // Looping SFX (engine hum). `path` is a C string to avoid per-frame
    // temporaries (called every frame in a pilot scene).
    void setLoop(const char *path, bool active, float gain);

    // Ambient music: streams (a full predecode of a long OGG stalls boot).
    void setMusic(const std::string &path);
    void stopMusic();

    // Master levels in [0,1] (the Settings window drives these).
    void setSfxVolume(float v);
    void setMusicVolume(float v);

    // Per frame: reap finished one-shots, complete any engine stop-fade.
    void update();

private:
    static const size_t MAX_ONESHOTS = 8;

    // Cached per path. `predecode`: SFX are tiny (free); music streams.
    MIX_Audio *loadAudio(const std::string &path, bool predecode);

    // Grow the track's mix buffers on the MAIN thread so the real-time
    // callback never hits its first SDL_realloc (the "crack on a tap").
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
    bool loopStopping_ = false;   // arms the stop-fade exactly once (a re-armed fade never completes)

    MIX_Track *music_ = nullptr;
    std::string musicPath_;
};
