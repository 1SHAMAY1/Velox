#pragma once

#include <cstdint>
#include <cstddef>
#include <cstdlib>
#include <cstring>
#include <cassert>
#include <utility>
#include <new>
#include <immintrin.h>

namespace Velox {

// ============================================================================
// FixedArray: Stack/Inline Fixed Capacity Buffer (0 Dynamic Allocations)
// ============================================================================
template <typename T, size_t Capacity>
class FixedArray {
public:
    FixedArray() : m_size(0) {}

    void PushBack(const T& item) {
        assert(m_size < Capacity && "FixedArray capacity exceeded!");
        m_data[m_size++] = item;
    }

    void PushBack(T&& item) {
        assert(m_size < Capacity && "FixedArray capacity exceeded!");
        m_data[m_size++] = std::move(item);
    }

    void PopBack() {
        if (m_size > 0) {
            --m_size;
        }
    }

    void Clear() { m_size = 0; }

    T& operator[](size_t index) {
        assert(index < m_size);
        return m_data[index];
    }

    const T& operator[](size_t index) const {
        assert(index < m_size);
        return m_data[index];
    }

    T& Back() {
        assert(m_size > 0);
        return m_data[m_size - 1];
    }

    const T& Back() const {
        assert(m_size > 0);
        return m_data[m_size - 1];
    }

    T* Data() { return m_data; }
    const T* Data() const { return m_data; }

    size_t Size() const { return m_size; }
    bool IsEmpty() const { return m_size == 0; }
    bool IsFull() const { return m_size == Capacity; }
    static constexpr size_t MaxCapacity() { return Capacity; }

    T* begin() { return m_data; }
    T* end() { return m_data + m_size; }
    const T* begin() const { return m_data; }
    const T* end() const { return m_data + m_size; }

private:
    T m_data[Capacity];
    size_t m_size;
};

// ============================================================================
// PodVector: Contiguous Lightweight POD Buffer (Zero-STL Minimal Growth)
// ============================================================================
template <typename T>
class PodVector {
public:
    PodVector() : m_data(nullptr), m_size(0), m_capacity(0) {}
    explicit PodVector(size_t initialCapacity) : m_data(nullptr), m_size(0), m_capacity(0) {
        Reserve(initialCapacity);
    }

    ~PodVector() {
        if (m_data) {
            std::free(m_data);
            m_data = nullptr;
        }
        m_size = 0;
        m_capacity = 0;
    }

    PodVector(const PodVector& other) : m_data(nullptr), m_size(0), m_capacity(0) {
        if (other.m_size > 0) {
            Reserve(other.m_size);
            std::memcpy(m_data, other.m_data, other.m_size * sizeof(T));
            m_size = other.m_size;
        }
    }

    PodVector& operator=(const PodVector& other) {
        if (this != &other) {
            Clear();
            if (other.m_size > 0) {
                Reserve(other.m_size);
                std::memcpy(m_data, other.m_data, other.m_size * sizeof(T));
                m_size = other.m_size;
            }
        }
        return *this;
    }

    PodVector(PodVector&& other) noexcept 
        : m_data(other.m_data), m_size(other.m_size), m_capacity(other.m_capacity) {
        other.m_data = nullptr;
        other.m_size = 0;
        other.m_capacity = 0;
    }

    PodVector& operator=(PodVector&& other) noexcept {
        if (this != &other) {
            if (m_data) std::free(m_data);
            m_data = other.m_data;
            m_size = other.m_size;
            m_capacity = other.m_capacity;
            other.m_data = nullptr;
            other.m_size = 0;
            other.m_capacity = 0;
        }
        return *this;
    }

    void Reserve(size_t newCapacity) {
        if (newCapacity <= m_capacity) return;
        T* newData = static_cast<T*>(std::realloc(m_data, newCapacity * sizeof(T)));
        if (!newData && newCapacity > 0) {
            assert(false && "PodVector allocation failed!");
        }
        m_data = newData;
        m_capacity = newCapacity;
    }

    void Resize(size_t newSize) {
        if (newSize > m_capacity) {
            Reserve(newSize < m_capacity * 2 ? m_capacity * 2 : newSize);
        }
        if (newSize > m_size) {
            std::memset(m_data + m_size, 0, (newSize - m_size) * sizeof(T));
        }
        m_size = newSize;
    }

    void PushBack(const T& val) {
        if (m_size >= m_capacity) {
            Reserve(m_capacity == 0 ? 16 : m_capacity * 2);
        }
        m_data[m_size++] = val;
    }

    void PopBack() {
        if (m_size > 0) --m_size;
    }

    void SwapAndPop(size_t index) {
        assert(index < m_size);
        if (index != m_size - 1) {
            m_data[index] = m_data[m_size - 1];
        }
        --m_size;
    }

    void Clear() { m_size = 0; }

    T& operator[](size_t index) {
        assert(index < m_size);
        return m_data[index];
    }

    const T& operator[](size_t index) const {
        assert(index < m_size);
        return m_data[index];
    }

    T* Data() { return m_data; }
    const T* Data() const { return m_data; }

    size_t Size() const { return m_size; }
    size_t Capacity() const { return m_capacity; }
    bool IsEmpty() const { return m_size == 0; }

    T* begin() { return m_data; }
    T* end() { return m_data + m_size; }
    const T* begin() const { return m_data; }
    const T* end() const { return m_data + m_size; }

private:
    T* m_data;
    size_t m_size;
    size_t m_capacity;
};

// ============================================================================
// Bitset: High-Performance 1-Bit SIMD Tag Mask
// ============================================================================
class Bitset {
public:
    Bitset() : m_words(nullptr), m_wordCount(0), m_bitCount(0) {}
    explicit Bitset(size_t bitCount) : m_words(nullptr), m_wordCount(0), m_bitCount(0) {
        Resize(bitCount);
    }

    ~Bitset() {
        if (m_words) {
            std::free(m_words);
            m_words = nullptr;
        }
        m_wordCount = 0;
        m_bitCount = 0;
    }

    Bitset(const Bitset& other) : m_words(nullptr), m_wordCount(0), m_bitCount(0) {
        if (other.m_wordCount > 0) {
            Resize(other.m_bitCount);
            std::memcpy(m_words, other.m_words, m_wordCount * sizeof(uint64_t));
        }
    }

    Bitset& operator=(const Bitset& other) {
        if (this != &other) {
            Resize(other.m_bitCount);
            if (other.m_wordCount > 0) {
                std::memcpy(m_words, other.m_words, m_wordCount * sizeof(uint64_t));
            }
        }
        return *this;
    }

    void Resize(size_t bitCount) {
        m_bitCount = bitCount;
        size_t newWordCount = (bitCount + 63) / 64;
        if (newWordCount > m_wordCount) {
            // 32-byte aligned for AVX2
            uint64_t* newWords = static_cast<uint64_t*>(_aligned_malloc(newWordCount * sizeof(uint64_t), 32));
            assert(newWords && "Bitset allocation failed!");
            if (m_words) {
                std::memcpy(newWords, m_words, m_wordCount * sizeof(uint64_t));
                _aligned_free(m_words);
            }
            // Zero initialize newly expanded words
            std::memset(newWords + m_wordCount, 0, (newWordCount - m_wordCount) * sizeof(uint64_t));
            m_words = newWords;
            m_wordCount = newWordCount;
        }
    }

    void Set(size_t index) {
        assert(index < m_bitCount);
        m_words[index / 64] |= (1ULL << (index % 64));
    }

    void Clear(size_t index) {
        assert(index < m_bitCount);
        m_words[index / 64] &= ~(1ULL << (index % 64));
    }

    void Toggle(size_t index) {
        assert(index < m_bitCount);
        m_words[index / 64] ^= (1ULL << (index % 64));
    }

    bool Test(size_t index) const {
        if (index >= m_bitCount) return false;
        return (m_words[index / 64] & (1ULL << (index % 64))) != 0;
    }

    void ResetAll() {
        if (m_words && m_wordCount > 0) {
            std::memset(m_words, 0, m_wordCount * sizeof(uint64_t));
        }
    }

    void SetAll() {
        if (m_words && m_wordCount > 0) {
            std::memset(m_words, 0xFF, m_wordCount * sizeof(uint64_t));
        }
    }

    size_t BitCount() const { return m_bitCount; }
    size_t WordCount() const { return m_wordCount; }
    uint64_t* Words() { return m_words; }
    const uint64_t* Words() const { return m_words; }

private:
    uint64_t* m_words;
    size_t m_wordCount;
    size_t m_bitCount;
};

// ============================================================================
// LinearArena: High-Speed Bump Allocator (O(1) Per-Frame Alloc / Reset)
// ============================================================================
class LinearArena {
public:
    explicit LinearArena(size_t capacityBytes = 4 * 1024 * 1024) // 4 MB default
        : m_capacity(capacityBytes), m_offset(0) {
        m_buffer = static_cast<uint8_t*>(_aligned_malloc(m_capacity, 64)); // 64-byte cache line align
        assert(m_buffer && "LinearArena allocation failed!");
    }

    ~LinearArena() {
        if (m_buffer) {
            _aligned_free(m_buffer);
            m_buffer = nullptr;
        }
        m_offset = 0;
    }

    void* Allocate(size_t sizeBytes, size_t alignment = 16) {
        size_t currentAddr = reinterpret_cast<size_t>(m_buffer + m_offset);
        size_t alignedAddr = (currentAddr + (alignment - 1)) & ~(alignment - 1);
        size_t newOffset = (alignedAddr - reinterpret_cast<size_t>(m_buffer)) + sizeBytes;

        if (newOffset > m_capacity) {
            assert(false && "LinearArena out of memory! Consider increasing capacity.");
            return nullptr;
        }

        m_offset = newOffset;
        return reinterpret_cast<void*>(alignedAddr);
    }

    template <typename T, typename... Args>
    T* New(Args&&... args) {
        void* ptr = Allocate(sizeof(T), alignof(T));
        if (!ptr) return nullptr;
        return new (ptr) T(std::forward<Args>(args)...);
    }

    template <typename T>
    T* AllocateArray(size_t count) {
        return static_cast<T*>(Allocate(sizeof(T) * count, alignof(T)));
    }

    void Reset() {
        m_offset = 0; // O(1) instantaneous clearing of all per-frame allocations
    }

    size_t UsedBytes() const { return m_offset; }
    size_t Capacity() const { return m_capacity; }

private:
    uint8_t* m_buffer;
    size_t m_capacity;
    size_t m_offset;
};

} // namespace Velox
