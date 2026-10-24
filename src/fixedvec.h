#pragma once
#include <cstdint>
#include <cstddef>
// Fixed-capacity vector-like container (no heap). Drop-in for the subset of
// std::vector used here: clear/push_back/size/empty/operator[]/begin/end/back.
template <typename T, int N>
struct FixedVec {
    T data_[N]{}; uint16_t n_ = 0;
    void clear() { n_ = 0; }
    bool push_back(const T& v) { if (n_ < N) { data_[n_++] = v; return true; } return false; }
    void truncate(size_t n) { if (n < n_) n_ = (uint16_t)n; }
    size_t size() const { return n_; }
    static constexpr size_t capacity() { return N; }
    bool empty() const { return n_ == 0; }
    T& operator[](int i) { return data_[i]; }
    const T& operator[](int i) const { return data_[i]; }
    T* begin() { return data_; }  T* end() { return data_ + n_; }
    const T* begin() const { return data_; }  const T* end() const { return data_ + n_; }
    T& back() { return data_[n_ - 1]; }
    const T& back() const { return data_[n_ - 1]; }
};
// Fixed ring that keeps the most recent N items, iterated oldest..newest.
template <typename T, int N>
struct FixedRing {
    T data_[N]{}; uint16_t n_ = 0, head_ = 0;
    void clear() { n_ = 0; head_ = 0; }
    void push(const T& v) { if (n_ < N) { data_[(head_ + n_) % N] = v; n_++; } else { data_[head_] = v; head_ = (head_ + 1) % N; } }
    size_t size() const { return n_; }
    bool empty() const { return n_ == 0; }
    const T& operator[](int i) const { return data_[(head_ + i) % N]; }  // 0 = oldest
};
