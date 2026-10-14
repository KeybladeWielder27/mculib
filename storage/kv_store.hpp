// kv_store.hpp - power-fail-safe key/value settings store on raw flash.
//
// Keys are 16-bit numbers (0..0xFFFE), values are byte strings (up to 32767
// bytes, in practice a few bytes to a few hundred). Header-only, C++11, no
// heap, no exceptions, no RTTI, no platform headers; flash access goes through
// the small mcu::Flash interface.
//
//   mcu::KvStore<> kv(flash);                 // uses every sector of `flash`
//   kv.begin();                               // mount (formats a blank flash)
//   kv.put_u32(kBaud, 115200);                // write
//   uint32_t baud = kv.get_u32_or(kBaud, 9600);
//   kv.remove(kBaud);
//
// How it works
//   * Log-structured. Each put appends a record (key, length, value, CRC-16)
//     to the active sector. The newest record for a key wins; remove() appends
//     a tombstone. Nothing is ever rewritten in place, so a power cut can only
//     leave a torn record at the end of the log, which the CRC rejects. After
//     a power cut every key has either the value it had before the interrupted
//     call or the new one, never a mixture.
//   * Wear levelling. When the active sector is full the store moves on to the
//     next erased one, going round all sectors in turn, so erases are spread
//     evenly. When only one erased sector is left, the oldest sector is
//     compacted: its live records are copied into a fresh sector, which is then
//     marked "complete", and only then is the old sector erased. A power cut
//     at any point either leaves the old sector untouched (the half-built copy
//     is discarded at the next mount) or leaves a complete copy (the old
//     sector is erased at the next mount), even if the erase itself was torn.
//   * put() writes nothing when the stored value is already identical, so
//     saving settings at every boot does not wear the flash.
//   * Each program unit is written once after an erase, so it also works on
//     flash with ECC (program_unit 4, 8 or 16).
//
// Costs and limits
//   * RAM: about 9 bytes per sector (MaxSectors) plus a few dozen bytes, and
//     about 40 bytes of stack inside calls. No index is kept in RAM, so get()
//     reads through the log: use it for settings, not for high-rate data.
//   * Capacity: roughly (sectors - 1) x payload; fragmentation matters when
//     values are big compared with a sector. A put that cannot fit returns
//     kFull after compacting what it can.
//   * After kIoError (flash reported a failure) the store is not ready: call
//     begin() again, which re-mounts and repairs.
//   * Bit rot inside a record makes the CRC reject it; the store then falls
//     back to the previous value of that key (if still present) and ignores
//     the rest of that sector's later records. Store a version/CRC of the whole
//     settings block if that matters to you.
//   * Not interrupt safe; one context at a time.
//
// Sector layout (all integers little-endian):
//   header  16 bytes: magic "KVS1", sequence number, sequence of the sector
//           being compacted (compaction target only), flags, CRC-16
//   done    one program unit, left erased; a compaction target gets zeros here
//           once all live records of its source have been copied into it
//   records [key:2][info:2 (bit 15 = tombstone, bits 0..14 = length)]
//           [value][crc:2], padded with 0xFF to a program unit
//
// Header-only; depends on hal/flash.hpp and checksum/crc.hpp.

#ifndef MCU_KV_STORE_HPP
#define MCU_KV_STORE_HPP

#include <stddef.h>
#include <stdint.h>

#include "checksum/crc.hpp"
#include "hal/flash.hpp"

namespace mcu {

enum class KvResult : uint8_t {
  kOk,
  kNotFound,        // no such key
  kFull,            // no room even after compaction
  kTooBig,          // a single record would not fit in a sector
  kBadArg,          // bad key, length, null pointer, or unusable flash geometry
  kBufferTooSmall,  // get(): the value is longer than the buffer (length still reported)
  kNotReady,        // begin() not called, or an earlier I/O error
  kIoError,         // the flash reported a failure
  kCorrupt,         // the flash content is unusable (call format())
};

struct KvStats {
  uint32_t records_written;  // records appended since begin()
  uint32_t skipped_writes;   // puts that matched the stored value and wrote nothing
  uint32_t sector_opens;     // sectors started
  uint32_t compactions;      // oldest-sector compactions
};

template <size_t MaxSectors = 16>
class KvStore {
  static_assert(MaxSectors >= 2 && MaxSectors <= 255, "MaxSectors must be 2..255");

 public:
  static constexpr uint16_t kInvalidKey = 0xFFFF;
  static constexpr size_t kMaxValue = 0x7FFF;

  // Uses `sector_count` sectors starting at `first_sector` (0 = all that remain).
  explicit KvStore(Flash& flash, size_t first_sector = 0, size_t sector_count = 0)
      : fl_(flash),
        first_(first_sector),
        want_(sector_count),
        n_(0),
        ssz_(0),
        unit_(1),
        data_start_(0),
        used_n_(0),
        active_(0),
        tail_(0),
        closed_(false),
        ready_(false),
        io_err_(false),
        verify_(true),
        full_need_(0) {
    stats_ = KvStats{0, 0, 0, 0};
    for (size_t i = 0; i < MaxSectors; ++i) {
      st_[i] = kFree;
      seq_[i] = 0;
      order_[i] = 0;
    }
  }

  // Mounts the store: scans the sectors, repairs an interrupted write or
  // compaction, and formats a blank flash. Safe to call again at any time.
  KvResult begin() {
    ready_ = false;
    io_err_ = false;
    KvResult r = geometry();
    if (r != KvResult::kOk) return r;
    used_n_ = 0;
    closed_ = false;
    full_need_ = 0;

    // 1. classify every sector; clear out anything that is not clean
    Hdr hd[MaxSectors];
    for (size_t i = 0; i < n_; ++i) {
      st_[i] = classify(i, hd[i]);
      if (io_err_) return fail_io();
    }
    for (size_t i = 0; i < n_; ++i) {
      if (st_[i] == kDirty) {
        r = release(i);
        if (r != KvResult::kOk) return r;
      }
    }
    // 2. order the live sectors oldest to newest
    for (size_t i = 0; i < n_; ++i) {
      if (st_[i] != kUsed) continue;
      seq_[i] = hd[i].seq;
      size_t p = used_n_++;
      while (p > 0 && seq_diff(seq_[order_[p - 1]], seq_[i]) > 0) {
        order_[p] = order_[p - 1];
        --p;
      }
      order_[p] = (uint8_t)i;
    }
    for (size_t p = 1; p < used_n_; ++p)
      if (seq_[order_[p]] == seq_[order_[p - 1]]) return KvResult::kCorrupt;

    // 3. finish or undo an interrupted compaction
    //    * newest sector is a target whose copy is not complete: discard it (its
    //      source is still intact, because it is erased only after "done")
    //    * a complete target exists and its source is still around (maybe half
    //      erased): erase the source
    for (;;) {
      bool changed = false;
      if (used_n_ > 0) {
        const size_t newest = order_[used_n_ - 1];
        if (hd[newest].gc && !hd[newest].done && find_seq(hd[newest].src) < n_) {
          r = release(newest);
          if (r != KvResult::kOk) return r;
          changed = true;
        }
      }
      for (size_t p = 0; p < used_n_ && !changed; ++p) {
        const size_t d = order_[p];
        if (!(hd[d].gc && hd[d].done)) continue;
        const size_t src = find_seq(hd[d].src);
        if (src < n_) {
          r = release(src);
          if (r != KvResult::kOk) return r;
          changed = true;
        }
      }
      if (!changed) break;
    }

    if (used_n_ == 0) {
      r = open_sector(false, 0);
      if (r != KvResult::kOk) return r;
    } else {
      active_ = order_[used_n_ - 1];
      if (count_free() == 0) return KvResult::kCorrupt;
      // find where the active sector ends
      size_t off = data_start_;
      Rec rec;
      for (;;) {
        const Scan s = scan(active_, off, rec);
        if (s == kRec) continue;
        closed_ = (s == kBad);
        break;
      }
      if (io_err_) return fail_io();
      tail_ = closed_ ? ssz_ : off;
    }
    ready_ = true;
    return KvResult::kOk;
  }

  // Erases every sector and starts a fresh store. All keys are lost.
  KvResult format() {
    ready_ = false;
    io_err_ = false;
    KvResult r = geometry();
    if (r != KvResult::kOk) return r;
    for (size_t i = 0; i < n_; ++i) {
      if (!fl_.erase(first_ + i)) return fail_io();
      st_[i] = kFree;
    }
    used_n_ = 0;
    full_need_ = 0;
    r = open_sector(false, 0);
    if (r != KvResult::kOk) return r;
    ready_ = true;
    return KvResult::kOk;
  }

  bool ready() const { return ready_; }
  const KvStats& stats() const { return stats_; }
  // Largest value one put can hold.
  size_t max_value_size() const {
    const size_t room = (ssz_ - data_start_) / unit_ * unit_;  // whole units
    const size_t slack = rec_size(0);                          // a put keeps room for a tombstone
    return room > slack + 6 ? room - slack - 6 : 0;
  }
  void set_verify(bool on) { verify_ = on; }  // read each record back after writing (default on)

  // ---- writing ----

  KvResult put(uint16_t key, const void* value, size_t len) {
    if (!ready_) return KvResult::kNotReady;
    if (key == kInvalidKey || len > kMaxValue || (len != 0 && value == 0)) return KvResult::kBadArg;
    if (rec_size(len) + rec_size(0) > ssz_ - data_start_) return KvResult::kTooBig;
    const uint8_t* v = static_cast<const uint8_t*>(value);
    Loc loc = {0, 0, 0, false};
    const bool found = find(key, loc);
    if (io_err_) return fail_io();
    if (found && !loc.deleted && loc.len == len) {
      bool same = true;
      uint8_t tmp[16];
      for (size_t o = 0; o < len && same; o += sizeof tmp) {
        const size_t k = len - o < sizeof tmp ? len - o : sizeof tmp;
        if (!rd(loc.sector, loc.off + 4 + o, tmp, k)) return fail_io();
        for (size_t i = 0; i < k; ++i) same = same && tmp[i] == v[o + i];
      }
      if (same) {
        ++stats_.skipped_writes;
        return KvResult::kOk;
      }
    }
    return append(key, false, v, len, 0, 0);
  }

  // Removes a key (kNotFound if it does not exist).
  KvResult remove(uint16_t key) {
    if (!ready_) return KvResult::kNotReady;
    if (key == kInvalidKey) return KvResult::kBadArg;
    Loc loc = {0, 0, 0, false};
    const bool found = find(key, loc);
    if (io_err_) return fail_io();
    if (!found || loc.deleted) return KvResult::kNotFound;
    return append(key, true, 0, 0, 0, 0);
  }

  // ---- reading ----

  // Copies the value into `buf`. `*out_len` (optional) receives the value's
  // length; with a short buffer nothing is copied and kBufferTooSmall returned.
  KvResult get(uint16_t key, void* buf, size_t cap, size_t* out_len = 0) {
    if (!ready_) return KvResult::kNotReady;
    if (key == kInvalidKey) return KvResult::kBadArg;
    Loc loc = {0, 0, 0, false};
    const bool found = find(key, loc);
    if (io_err_) return fail_io();
    if (!found || loc.deleted) return KvResult::kNotFound;
    if (out_len) *out_len = loc.len;
    if (loc.len > cap) return KvResult::kBufferTooSmall;
    if (loc.len != 0) {
      if (buf == 0) return KvResult::kBadArg;
      if (!rd(loc.sector, loc.off + 4, buf, loc.len)) return fail_io();
    }
    return KvResult::kOk;
  }

  // Length of a stored value without reading it.
  KvResult length(uint16_t key, size_t* out_len) {
    if (!ready_) return KvResult::kNotReady;
    if (key == kInvalidKey || out_len == 0) return KvResult::kBadArg;
    Loc loc = {0, 0, 0, false};
    const bool found = find(key, loc);
    if (io_err_) return fail_io();
    if (!found || loc.deleted) return KvResult::kNotFound;
    *out_len = loc.len;
    return KvResult::kOk;
  }

  bool contains(uint16_t key) {
    size_t n;
    return length(key, &n) == KvResult::kOk;
  }

  // Walks the stored keys in ascending order: first_key() then next_key().
  KvResult first_key(uint16_t& key) { return key_after(-1, key); }
  KvResult next_key(uint16_t& key) { return key_after((int32_t)key, key); }

  // ---- typed helpers (fixed-size integers are stored little-endian) ----

  KvResult put_u8(uint16_t key, uint8_t v) { return put(key, &v, 1); }
  KvResult put_u16(uint16_t key, uint16_t v) {
    const uint8_t b[2] = {(uint8_t)v, (uint8_t)(v >> 8)};
    return put(key, b, 2);
  }
  KvResult put_u32(uint16_t key, uint32_t v) {
    const uint8_t b[4] = {(uint8_t)v, (uint8_t)(v >> 8), (uint8_t)(v >> 16), (uint8_t)(v >> 24)};
    return put(key, b, 4);
  }
  KvResult put_i32(uint16_t key, int32_t v) { return put_u32(key, (uint32_t)v); }
  KvResult put_str(uint16_t key, const char* s) {
    if (s == 0) return KvResult::kBadArg;
    size_t n = 0;
    while (s[n]) ++n;
    return put(key, s, n);  // stored without the terminator
  }

  KvResult get_u8(uint16_t key, uint8_t& v) { return get_fixed(key, &v, 1); }
  KvResult get_u16(uint16_t key, uint16_t& v) {
    uint8_t b[2];
    const KvResult r = get_fixed(key, b, 2);
    if (r == KvResult::kOk) v = (uint16_t)(b[0] | (b[1] << 8));
    return r;
  }
  KvResult get_u32(uint16_t key, uint32_t& v) {
    uint8_t b[4];
    const KvResult r = get_fixed(key, b, 4);
    if (r == KvResult::kOk)
      v = (uint32_t)b[0] | ((uint32_t)b[1] << 8) | ((uint32_t)b[2] << 16) | ((uint32_t)b[3] << 24);
    return r;
  }
  KvResult get_i32(uint16_t key, int32_t& v) {
    uint32_t u;
    const KvResult r = get_u32(key, u);
    if (r == KvResult::kOk) v = (int32_t)u;
    return r;
  }
  // The stored value, or `def` if the key is missing, has another size, or cannot be read.
  uint8_t get_u8_or(uint16_t key, uint8_t def) {
    uint8_t v;
    return get_u8(key, v) == KvResult::kOk ? v : def;
  }
  uint16_t get_u16_or(uint16_t key, uint16_t def) {
    uint16_t v;
    return get_u16(key, v) == KvResult::kOk ? v : def;
  }
  uint32_t get_u32_or(uint16_t key, uint32_t def) {
    uint32_t v;
    return get_u32(key, v) == KvResult::kOk ? v : def;
  }
  int32_t get_i32_or(uint16_t key, int32_t def) {
    int32_t v;
    return get_i32(key, v) == KvResult::kOk ? v : def;
  }
  // Copies a string value and adds the terminator. kBufferTooSmall if it does not fit.
  KvResult get_str(uint16_t key, char* buf, size_t cap) {
    if (buf == 0 || cap == 0) return KvResult::kBadArg;
    size_t n = 0;
    const KvResult r = get(key, buf, cap - 1, &n);
    if (r == KvResult::kOk) buf[n] = 0;
    return r;
  }

 private:
  enum SectorState : uint8_t { kFree, kUsed, kDirty };
  enum Scan : uint8_t { kRec, kEnd, kBad };

  struct Hdr {
    bool valid;
    uint32_t seq;
    uint32_t src;
    bool gc;
    bool done;  // compaction target whose copy is complete
  };
  struct Rec {
    size_t off;
    uint16_t key;
    uint16_t len;
    bool deleted;
    size_t size;
  };
  struct Loc {
    size_t sector;
    size_t off;
    size_t len;
    bool deleted;
  };
  typedef Crc16CcittFalse Crc;

  static constexpr uint32_t kMagic = 0x3153564Bu;  // "KVS1"
  static constexpr size_t kHdrBytes = 16;
  static constexpr size_t kChunk = 32;

  // ---- geometry ----
  static size_t align_up(size_t v, size_t a) { return (v + a - 1) / a * a; }
  size_t rec_size(size_t len) const { return align_up(6 + len, unit_); }
  size_t done_off() const { return align_up(kHdrBytes, unit_); }
  static int32_t seq_diff(uint32_t a, uint32_t b) { return (int32_t)(a - b); }

  KvResult geometry() {
    unit_ = fl_.program_unit();
    ssz_ = fl_.sector_size();
    if (!(unit_ == 1 || unit_ == 2 || unit_ == 4 || unit_ == 8 || unit_ == 16)) return KvResult::kBadArg;
    const size_t total = fl_.sector_count();
    if (first_ > total) return KvResult::kBadArg;
    n_ = want_ ? want_ : total - first_;
    if (first_ + n_ > total || n_ < 2 || n_ > MaxSectors) return KvResult::kBadArg;
    data_start_ = done_off() + unit_;
    if (ssz_ % unit_ != 0 || ssz_ < data_start_ + rec_size(16)) return KvResult::kBadArg;
    return KvResult::kOk;
  }

  KvResult fail_io() {
    ready_ = false;
    return KvResult::kIoError;
  }

  // ---- low-level flash access (remember failures) ----
  bool rd(size_t sector, size_t off, void* dst, size_t n) {
    if (!fl_.read(first_ + sector, off, dst, n)) {
      io_err_ = true;
      return false;
    }
    return true;
  }
  bool pr(size_t sector, size_t off, const void* src, size_t n) {
    if (!fl_.program(first_ + sector, off, src, n)) {
      io_err_ = true;
      return false;
    }
    return true;
  }
  bool er(size_t sector) {
    if (!fl_.erase(first_ + sector)) {
      io_err_ = true;
      return false;
    }
    return true;
  }

  static uint16_t rd16(const uint8_t* p) { return (uint16_t)(p[0] | (p[1] << 8)); }
  static uint32_t rd32(const uint8_t* p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
  }
  static void wr32(uint8_t* p, uint32_t v) {
    p[0] = (uint8_t)v;
    p[1] = (uint8_t)(v >> 8);
    p[2] = (uint8_t)(v >> 16);
    p[3] = (uint8_t)(v >> 24);
  }

  // True if sector bytes [from, end) are all 0xFF.
  bool blank_from(size_t sector, size_t from) {
    uint8_t tmp[kChunk];
    for (size_t o = from; o < ssz_; o += sizeof tmp) {
      const size_t k = ssz_ - o < sizeof tmp ? ssz_ - o : sizeof tmp;
      if (!rd(sector, o, tmp, k)) return false;
      for (size_t i = 0; i < k; ++i)
        if (tmp[i] != 0xFF) return false;
    }
    return true;
  }

  // ---- sector headers ----
  void encode_hdr(uint8_t* b, uint32_t seq, bool gc, uint32_t src) const {
    wr32(b, kMagic);
    wr32(b + 4, seq);
    wr32(b + 8, src);
    b[12] = gc ? 1 : 0;
    b[13] = 0;
    const uint16_t c = Crc::compute(b, 14);
    b[14] = (uint8_t)c;
    b[15] = (uint8_t)(c >> 8);
  }

  SectorState classify(size_t i, Hdr& h) {
    h.valid = false;
    h.seq = h.src = 0;
    h.gc = false;
    h.done = false;
    uint8_t b[kHdrBytes];
    if (!rd(i, 0, b, sizeof b)) return kDirty;
    if (rd32(b) == kMagic && Crc::compute(b, 14) == rd16(b + 14)) {
      uint8_t mark[16];
      if (!rd(i, done_off(), mark, unit_)) return kDirty;
      bool done = false;
      for (size_t k = 0; k < unit_; ++k) done = done || mark[k] != 0xFF;
      h.valid = true;
      h.seq = rd32(b + 4);
      h.src = rd32(b + 8);
      h.gc = (b[12] & 1) != 0;
      h.done = done;
      return kUsed;
    }
    return blank_from(i, 0) && !io_err_ ? kFree : kDirty;
  }

  // Index of the used sector with this sequence number, or n_ if none.
  size_t find_seq(uint32_t seq) const {
    for (size_t p = 0; p < used_n_; ++p)
      if (seq_[order_[p]] == seq) return order_[p];
    return n_;
  }

  size_t count_free() const {
    size_t c = 0;
    for (size_t i = 0; i < n_; ++i) c += st_[i] == kFree ? 1 : 0;
    return c;
  }

  // Erases a sector and forgets it.
  KvResult release(size_t i) {
    if (!er(i)) return fail_io();
    st_[i] = kFree;
    size_t p = 0;
    while (p < used_n_ && order_[p] != i) ++p;
    if (p < used_n_) {
      for (; p + 1 < used_n_; ++p) order_[p] = order_[p + 1];
      --used_n_;
    }
    if (used_n_ > 0) active_ = order_[used_n_ - 1];
    return KvResult::kOk;
  }

  // Starts a new active sector in the next erased sector after the current one.
  KvResult open_sector(bool gc, uint32_t src_seq) {
    size_t cand = n_;
    const size_t start = used_n_ ? (active_ + 1) % n_ : 0;
    for (size_t k = 0; k < n_ && cand == n_; ++k) {
      const size_t i = (start + k) % n_;
      if (st_[i] == kFree) cand = i;
    }
    if (cand == n_) return KvResult::kCorrupt;
    const uint32_t seq = used_n_ ? seq_[order_[used_n_ - 1]] + 1 : 1;
    uint8_t b[kHdrBytes];
    encode_hdr(b, seq, gc, src_seq);
    uint8_t chk[kHdrBytes];
    if (!pr(cand, 0, b, sizeof b) || !rd(cand, 0, chk, sizeof chk)) {
      st_[cand] = kDirty;
      return fail_io();
    }
    for (size_t k = 0; k < sizeof b; ++k) {
      if (b[k] != chk[k]) {
        st_[cand] = kDirty;
        return fail_io();
      }
    }
    st_[cand] = kUsed;
    seq_[cand] = seq;
    order_[used_n_++] = (uint8_t)cand;
    active_ = cand;
    tail_ = data_start_;
    closed_ = false;
    ++stats_.sector_opens;
    return KvResult::kOk;
  }

  // ---- records ----

  // Reads the record at *off in `sector`. kRec: valid record (r filled, *off
  // advanced). kEnd: clean end of the log (*off = first free byte). kBad: a
  // torn or damaged record starts at *off.
  Scan scan(size_t sector, size_t& off, Rec& r) {
    if (off + rec_size(0) > ssz_) return kEnd;
    uint8_t h[4];
    if (!rd(sector, off, h, 4)) return kBad;
    const uint16_t key = rd16(h);
    const uint16_t info = rd16(h + 2);
    if (key == kInvalidKey) {
      if (info == 0xFFFF && blank_from(sector, off) && !io_err_) return kEnd;
      return kBad;
    }
    const size_t len = info & 0x7FFFu;
    const size_t size = rec_size(len);
    if (off + size > ssz_) return kBad;
    Crc crc;
    crc.update(h, 4);
    uint8_t tmp[kChunk];
    for (size_t o = 0; o < len; o += sizeof tmp) {
      const size_t k = len - o < sizeof tmp ? len - o : sizeof tmp;
      if (!rd(sector, off + 4 + o, tmp, k)) return kBad;
      crc.update(tmp, k);
    }
    uint8_t c[2];
    if (!rd(sector, off + 4 + len, c, 2)) return kBad;
    if (crc.value() != rd16(c)) return kBad;
    r.off = off;
    r.key = key;
    r.len = (uint16_t)len;
    r.deleted = (info & 0x8000u) != 0;
    r.size = size;
    off += size;
    return kRec;
  }

  // Newest record (live or tombstone) for `key` across the whole log.
  bool find(uint16_t key, Loc& loc) {
    bool found = false;
    for (size_t p = 0; p < used_n_; ++p) {
      const size_t s = order_[p];
      size_t off = data_start_;
      Rec r;
      while (scan(s, off, r) == kRec) {
        if (r.key == key) {
          loc.sector = s;
          loc.off = r.off;
          loc.len = r.len;
          loc.deleted = r.deleted;
          found = true;
        }
      }
    }
    return found;
  }

  // True if a record for `key` exists after (sector at order position `p`, offset `after`).
  bool superseded(size_t p, size_t after, uint16_t key) {
    for (; p < used_n_; ++p) {
      const size_t s = order_[p];
      size_t off = after;
      Rec r;
      while (scan(s, off, r) == kRec)
        if (r.key == key) return true;
      after = data_start_;
    }
    return false;
  }

  KvResult key_after(int32_t prev, uint16_t& out) {
    if (!ready_) return KvResult::kNotReady;
    for (;;) {
      int32_t best = -1;
      for (size_t p = 0; p < used_n_; ++p) {
        const size_t s = order_[p];
        size_t off = data_start_;
        Rec r;
        while (scan(s, off, r) == kRec)
          if ((int32_t)r.key > prev && (best < 0 || (int32_t)r.key < best)) best = (int32_t)r.key;
      }
      if (io_err_) return fail_io();
      if (best < 0) return KvResult::kNotFound;
      Loc loc = {0, 0, 0, false};
      find((uint16_t)best, loc);
      if (io_err_) return fail_io();
      if (!loc.deleted) {
        out = (uint16_t)best;
        return KvResult::kOk;
      }
      prev = best;
    }
  }

  KvResult get_fixed(uint16_t key, void* dst, size_t want) {
    uint8_t tmp[8];
    size_t n = 0;
    const KvResult r = get(key, tmp, want, &n);
    if (r != KvResult::kOk) return r == KvResult::kBufferTooSmall ? KvResult::kBadArg : r;
    if (n != want) return KvResult::kBadArg;
    for (size_t i = 0; i < want; ++i) static_cast<uint8_t*>(dst)[i] = tmp[i];
    return KvResult::kOk;
  }

  // Writes one record at the tail of the active sector. The value comes from
  // `mem`, or (when mem is null and len > 0) from flash at (src_sector, src_off).
  KvResult write_record(uint16_t key, bool deleted, const uint8_t* mem, size_t len,
                        size_t src_sector, size_t src_off) {
    const size_t size = rec_size(len);
    uint8_t buf[kChunk];
    size_t n = 0;
    size_t at = tail_;
    Crc crc;
    uint8_t h[4];
    h[0] = (uint8_t)key;
    h[1] = (uint8_t)(key >> 8);
    const uint16_t info = (uint16_t)(len | (deleted ? 0x8000u : 0u));
    h[2] = (uint8_t)info;
    h[3] = (uint8_t)(info >> 8);
    crc.update(h, 4);
    for (size_t i = 0; i < 4; ++i) buf[n++] = h[i];
    size_t done = 0;
    while (done < len) {
      uint8_t piece[kChunk];
      const size_t k = len - done < sizeof piece ? len - done : sizeof piece;
      if (mem) {
        for (size_t i = 0; i < k; ++i) piece[i] = mem[done + i];
      } else if (!rd(src_sector, src_off + 4 + done, piece, k)) {
        return fail_io();
      }
      crc.update(piece, k);
      for (size_t i = 0; i < k; ++i) {
        buf[n++] = piece[i];
        if (n == sizeof buf) {
          if (!pr(active_, at, buf, n)) return fail_io();
          at += n;
          n = 0;
        }
      }
      done += k;
    }
    const uint16_t c = crc.value();
    const uint8_t cb[2] = {(uint8_t)c, (uint8_t)(c >> 8)};
    for (size_t i = 0; i < 2; ++i) {
      buf[n++] = cb[i];
      if (n == sizeof buf) {
        if (!pr(active_, at, buf, n)) return fail_io();
        at += n;
        n = 0;
      }
    }
    while (at + n < tail_ + size) {  // pad to a whole program unit
      buf[n++] = 0xFF;
      if (n == sizeof buf) {
        if (!pr(active_, at, buf, n)) return fail_io();
        at += n;
        n = 0;
      }
    }
    if (n != 0) {
      if (!pr(active_, at, buf, n)) return fail_io();
      at += n;
    }
    if (verify_) {
      size_t off = tail_;
      Rec r;
      if (scan(active_, off, r) != kRec || r.size != size || r.key != key || io_err_) {
        return fail_io();
      }
    }
    tail_ += size;
    ++stats_.records_written;
    return KvResult::kOk;
  }

  // Appends a record, making room first if needed.
  KvResult append(uint16_t key, bool deleted, const uint8_t* mem, size_t len, size_t src_sector,
                  size_t src_off) {
    const size_t need = rec_size(len);
    // A put leaves room for one tombstone, so that a full store can still free itself by remove().
    KvResult r = ensure_space(need, deleted ? 0 : rec_size(0));
    if (r != KvResult::kOk) return r;
    r = write_record(key, deleted, mem, len, src_sector, src_off);
    if (r == KvResult::kOk) full_need_ = 0;
    return r;
  }

  KvResult ensure_space(size_t record, size_t slack) {
    const size_t need = record + slack;
    if (full_need_ != 0 && need >= full_need_) return KvResult::kFull;
    for (size_t iter = 0; iter < 2 * n_ + 2; ++iter) {
      if (!closed_ && tail_ + need <= ssz_) return KvResult::kOk;
      const size_t freec = count_free();
      KvResult r;
      if (freec >= 2) r = open_sector(false, 0);
      else if (freec == 1) r = compact_oldest();
      else {
        ready_ = false;
        return KvResult::kCorrupt;
      }
      if (r != KvResult::kOk) return r;
    }
    if (full_need_ == 0 || need < full_need_) full_need_ = need;
    return KvResult::kFull;
  }

  // Copies the live records of the oldest sector into a fresh sector, then erases it.
  KvResult compact_oldest() {
    if (used_n_ == 0) return KvResult::kCorrupt;
    const size_t src = order_[0];
    const uint32_t src_seq = seq_[src];
    KvResult r = open_sector(true, src_seq);
    if (r != KvResult::kOk) return r;
    size_t off = data_start_;
    Rec rec;
    while (scan(src, off, rec) == kRec) {
      if (rec.deleted) continue;
      if (superseded_by_later(src, rec)) continue;
      r = write_record(rec.key, false, 0, rec.len, src, rec.off);
      if (r != KvResult::kOk) return r;
    }
    if (io_err_) return fail_io();
    // The copy is complete: say so in the target, and only then erase the source.
    const uint8_t zeros[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    if (!pr(active_, done_off(), zeros, unit_)) return fail_io();
    r = release(src);
    if (r != KvResult::kOk) return r;
    ++stats_.compactions;
    return KvResult::kOk;
  }

  // Is there a newer record for rec.key after `rec` in sector `src` or in any later sector?
  bool superseded_by_later(size_t src, const Rec& rec) {
    size_t p = 0;
    while (p < used_n_ && order_[p] != src) ++p;
    return superseded(p, rec.off + rec.size, rec.key);
  }

  Flash& fl_;
  size_t first_, want_, n_, ssz_, unit_, data_start_;
  size_t used_n_, active_, tail_;
  bool closed_, ready_, io_err_, verify_;
  size_t full_need_;
  SectorState st_[MaxSectors];
  uint32_t seq_[MaxSectors];
  uint8_t order_[MaxSectors];  // used sectors, oldest first
  KvStats stats_;
};

}  // namespace mcu

#endif  // MCU_KV_STORE_HPP
