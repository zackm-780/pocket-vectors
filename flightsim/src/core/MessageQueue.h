#pragma once
// Additive core contract (P2): short on-screen feedback messages ("FLAPS 15", "GEAR DOWN").
// Producers (input/PilotControls, later others) post; the HUD (H2) reads the newest ones and
// fades them by age. Fixed capacity ring, no heap allocation, not thread safe (main thread only).

#include <cstdarg>
#include <cstdio>

namespace fsim {

struct Message {
    char text[32] = {0};
    double timeS = 0;  // MessageQueue::nowS() when posted
};

class MessageQueue {
public:
    static constexpr int kCapacity = 8;

    // Advances the queue clock (wall seconds). Call once per frame.
    void tick(double dtS) { nowS_ += dtS; }
    double nowS() const { return nowS_; }

    void post(const char* text) { postf("%s", text); }

#if defined(__GNUC__) || defined(__clang__)
    __attribute__((format(printf, 2, 3)))
#endif
    void
    postf(const char* fmt, ...) {
        Message& m = ring_[(head_ + size_) % kCapacity];
        if (size_ == kCapacity) {
            head_ = (head_ + 1) % kCapacity;  // overwrite the oldest
        } else {
            ++size_;
        }
        va_list ap;
        va_start(ap, fmt);
        std::vsnprintf(m.text, sizeof m.text, fmt, ap);
        va_end(ap);
        m.timeS = nowS_;
        ++postedCount_;
    }

    int size() const { return size_; }
    // Total messages ever posted (lets a consumer notice new ones without comparing text).
    unsigned long totalPosted() const { return postedCount_; }
    // i = 0 is the newest message, size()-1 the oldest still held.
    const Message& newest(int i = 0) const { return ring_[(head_ + size_ - 1 - i) % kCapacity]; }
    // Age of the message in seconds; the HUD hides messages older than a few seconds.
    double ageS(const Message& m) const { return nowS_ - m.timeS; }
    void clear() { size_ = head_ = 0; }

private:
    Message ring_[kCapacity];
    int head_ = 0, size_ = 0;
    double nowS_ = 0;
    unsigned long postedCount_ = 0;
};

}  // namespace fsim
