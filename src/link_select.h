#pragma once
// Which links feed the scene (fusion), chosen with memory so the choice does not
// jump between ticks. Pure C++ (no IDF): runs in firmware and in host tests.
//
// Each link's quality (link_quality_db, dsp.h) is smoothed by an EMA (tau_s).
// Up to max_n links are selected, best first, from those that have been valid
// for settle_s without a break (a bursty transmitter, e.g. a router's data, is
// never picked up for a few seconds and dropped again). A selected link stays
// selected for at least dwell_s; an unselected one replaces the worst selected
// only after beating it by margin_db for hold_s without a break. A link that
// stops being valid (too slow, window not full) or disappears is dropped at
// once: its CSI cannot be used anyway.
#include <stdint.h>
#include <string.h>
#include "fixedvec.h"

struct LinkKey {               // transmitter + PHY format + receiving board
    uint8_t mac[6];
    uint8_t fmt;
    uint8_t peer;              // 0 = this board, 1 = the other board
    bool operator==(const LinkKey& o) const { return !memcmp(this, &o, sizeof(*this)); }
};
static_assert(sizeof(LinkKey) == 8, "LinkKey is compared bytewise");

struct LinkInput {
    LinkKey key;
    float   quality_db;        // instantaneous link_quality_db()
    bool    valid;             // its CSI feeds the DSP (enough frames, window full)
};

struct SelectParams {
    float tau_s     = 5.0f;    // quality EMA time constant
    float settle_s  = 10.0f;   // continuously valid this long before it can be selected
    float dwell_s   = 20.0f;   // minimum time a link stays selected
    float margin_db = 3.0f;    // a challenger must beat the worst selected by this...
    float hold_s    = 5.0f;    // ...continuously for this long
};

template <int N>
class LinkSelector {
public:
    // Advance by dt seconds; selected[i] says whether in[i] feeds the scene.
    void update(float dt, const LinkInput* in, int n, int max_n, bool* selected, const SelectParams& p = SelectParams()) {
        // Track every input; forget links that are gone.
        for (auto& e : t_) e.seen = false;
        const float a = dt / p.tau_s < 1.0f ? dt / p.tau_s : 1.0f;
        for (int i = 0; i < n; i++) {
            Entry* e = find(in[i].key);
            if (!e) {
                Entry ne{}; ne.key = in[i].key; ne.ema = in[i].quality_db;
                if (!t_.push_back(ne)) continue;
                e = &t_[(int)t_.size() - 1];
            }
            e->seen = true;
            e->valid = in[i].valid;
            e->valid_s = e->valid ? e->valid_s + dt : 0;
            e->ema += a * (in[i].quality_db - e->ema);
            if (e->sel) e->sel_s += dt;
        }
        compact();

        // Unusable links leave at once; a lowered max_n drops the worst.
        for (auto& e : t_) if (e.sel && !e.valid) e.sel = false;
        while (count() > max_n) worst_selected()->sel = false;

        // Free places go to the best settled links.
        while (count() < max_n) {
            Entry* c = best_unselected(p.settle_s);
            if (!c) break;
            select(c);
        }

        // Full: the best challenger replaces the worst selected link only with
        // a sustained margin, and never before that link's dwell time.
        Entry* c = best_unselected(p.settle_s);
        Entry* w = worst_selected();
        for (auto& e : t_) if (&e != c) e.better_s = 0;
        if (c && w && c->ema > w->ema + p.margin_db) {
            c->better_s += dt;
            if (c->better_s >= p.hold_s && w->sel_s >= p.dwell_s) { w->sel = false; select(c); }
        } else if (c) {
            c->better_s = 0;
        }

        for (int i = 0; i < n; i++) { const Entry* e = find(in[i].key); selected[i] = e && e->sel; }
    }

    // Smoothed quality of a link (for display); 0 if unknown.
    float quality(const LinkKey& k) const { for (const auto& e : t_) if (e.key == k) return e.ema; return 0; }

private:
    struct Entry { LinkKey key; float ema, sel_s, better_s, valid_s; bool sel, valid, seen; };
    FixedVec<Entry, N> t_;

    Entry* find(const LinkKey& k) { for (auto& e : t_) if (e.key == k) return &e; return nullptr; }
    const Entry* find(const LinkKey& k) const { for (const auto& e : t_) if (e.key == k) return &e; return nullptr; }
    void compact() {
        int j = 0;
        for (int i = 0; i < (int)t_.size(); i++) if (t_[i].seen) t_[j++] = t_[i];
        t_.truncate((size_t)j);
    }
    int count() const { int c = 0; for (const auto& e : t_) c += e.sel; return c; }
    void select(Entry* e) { e->sel = true; e->sel_s = 0; e->better_s = 0; }
    Entry* best_unselected(float settle_s) {
        Entry* b = nullptr;
        for (auto& e : t_) if (!e.sel && e.valid && e.valid_s >= settle_s && (!b || e.ema > b->ema)) b = &e;
        return b;
    }
    Entry* worst_selected() {
        Entry* w = nullptr;
        for (auto& e : t_) if (e.sel && (!w || e.ema < w->ema)) w = &e;
        return w;
    }
};
