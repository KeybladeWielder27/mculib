// Host-side tests for the cli library. Build and run from this folder:
//   g++ -std=c++11 -Wall -Wextra -Werror -I.. test_cli.cpp -o test_cli && ./test_cli
// (-I.. puts the root of the collection on the include path.)

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "cli/console.hpp"
#include "cli/line_editor.hpp"
#include "cli/output.hpp"
#include "cli/parse.hpp"
#include "cli/tokenizer.hpp"
#include "hal/loopback_stream.hpp"

static int g_checks = 0;
static int g_failures = 0;

#define CHECK(cond)                                          \
  do {                                                       \
    ++g_checks;                                              \
    if (!(cond)) {                                           \
      ++g_failures;                                          \
      printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
    }                                                        \
  } while (0)

static void print_escaped(const char* s) {
  for (; *s; ++s) {
    unsigned char c = (unsigned char)*s;
    if (c == '\r') printf("\\r");
    else if (c == '\n') printf("\\n");
    else if (c == '\t') printf("\\t");
    else if (c == '\b') printf("\\b");
    else if (c == '\a') printf("\\a");
    else if (c < 0x20 || c >= 0x7F) printf("\\x%02X", c);
    else putchar(c);
  }
}

#define CHECK_STR(actual, expected)                                  \
  do {                                                               \
    ++g_checks;                                                      \
    const char* a_ = (actual);                                       \
    const char* e_ = (expected);                                     \
    if (strcmp(a_, e_) != 0) {                                       \
      ++g_failures;                                                  \
      printf("FAIL %s:%d\n   got: \"", __FILE__, __LINE__);          \
      print_escaped(a_);                                             \
      printf("\"\n  want: \"");                                      \
      print_escaped(e_);                                             \
      printf("\"\n");                                                \
    }                                                                \
  } while (0)

static uint32_t g_rng = 424242;
static uint8_t rnd8() {
  g_rng = g_rng * 1664525u + 1013904223u;
  return (uint8_t)(g_rng >> 24);
}

// ---------------------------------------------------------------------------
// tokenizer
// ---------------------------------------------------------------------------
static size_t tok(const char* in, size_t max_args, char* joined, mcu::TokenizeStatus& st) {
  char line[256];
  char* argv[17];
  strcpy(line, in);
  size_t argc = 99;
  st = mcu::tokenize(line, argv, max_args, argc);
  joined[0] = '\0';
  if (st == mcu::TokenizeStatus::kOk) {
    CHECK(argv[argc] == nullptr);
    for (size_t i = 0; i < argc; ++i) {
      if (i) strcat(joined, "|");
      strcat(joined, argv[i]);
    }
  }
  return argc;
}

static void expect_tokens(const char* in, size_t want_argc, const char* want_joined) {
  char joined[256];
  mcu::TokenizeStatus st;
  size_t argc = tok(in, 16, joined, st);
  CHECK(st == mcu::TokenizeStatus::kOk);
  CHECK(argc == want_argc);
  CHECK_STR(joined, want_joined);
}

static void test_tokenizer() {
  expect_tokens("", 0, "");
  expect_tokens("   \t ", 0, "");
  expect_tokens("a", 1, "a");
  expect_tokens("  a   b\tc  ", 3, "a|b|c");
  expect_tokens("set name \"hello world\" 42", 4, "set|name|hello world|42");
  expect_tokens("x \"\" y", 3, "x||y");
  expect_tokens("x '' y", 3, "x||y");
  expect_tokens("\"a\\\"b\"", 1, "a\"b");
  expect_tokens("\"tab\\there\"", 1, "tab\there");
  expect_tokens("\"back\\\\slash\"", 1, "back\\slash");
  expect_tokens("'no \\n escape'", 1, "no \\n escape");
  expect_tokens("a\\ b c", 2, "a b|c");
  expect_tokens("ab\"c d\"e", 1, "abc de");
  expect_tokens("'a b'c", 1, "a bc");
  expect_tokens("it\\'s", 1, "it's");
  expect_tokens("trailing\\", 1, "trailing\\");
  expect_tokens("\"unix\\qescape\"", 1, "unixqescape");
  expect_tokens("a \"b c\" 'd e' f", 4, "a|b c|d e|f");

  char joined[256];
  mcu::TokenizeStatus st;
  tok("a \"b c", 16, joined, st);
  CHECK(st == mcu::TokenizeStatus::kUnterminatedQuote);
  tok("a 'b", 16, joined, st);
  CHECK(st == mcu::TokenizeStatus::kUnterminatedQuote);

  size_t argc = tok("a b c", 3, joined, st);
  CHECK(st == mcu::TokenizeStatus::kOk && argc == 3);
  tok("a b c d", 3, joined, st);
  CHECK(st == mcu::TokenizeStatus::kTooManyArgs);
  argc = tok("", 0, joined, st);
  CHECK(st == mcu::TokenizeStatus::kOk && argc == 0);
  tok("a", 0, joined, st);
  CHECK(st == mcu::TokenizeStatus::kTooManyArgs);

  // The result lives inside the original buffer.
  char line[] = "foo \"bar baz\" qux";
  char* argv[5];
  size_t n;
  CHECK(mcu::tokenize(line, argv, 4, n) == mcu::TokenizeStatus::kOk && n == 3);
  CHECK(argv[0] >= line && argv[2] < line + sizeof line);
}

// Random garbage must never read or write outside the buffer (run under a sanitizer).
static void test_tokenizer_fuzz() {
  static const char alphabet[] = {' ', '\t', '"', '\'', '\\', 'a', 'b', 'n', 'c'};
  for (int iter = 0; iter < 20000; ++iter) {
    size_t len = rnd8() % 40;
    char* buf = (char*)malloc(len + 1);
    for (size_t i = 0; i < len; ++i) buf[i] = alphabet[rnd8() % sizeof alphabet];
    buf[len] = '\0';
    char* argv[9];
    size_t argc;
    mcu::TokenizeStatus st = mcu::tokenize(buf, argv, 8, argc);
    if (st == mcu::TokenizeStatus::kOk) {
      CHECK(argc <= 8 && argv[argc] == nullptr);
      for (size_t i = 0; i < argc; ++i) CHECK(argv[i] >= buf && argv[i] <= buf + len);
    }
    free(buf);
  }
}

// ---------------------------------------------------------------------------
// parse
// ---------------------------------------------------------------------------
static void test_parse_uint() {
  uint32_t v = 123;
  CHECK(mcu::parse_uint("0", v) && v == 0);
  CHECK(mcu::parse_uint("42", v) && v == 42);
  CHECK(mcu::parse_uint("007", v) && v == 7);
  CHECK(mcu::parse_uint("0x1F40", v) && v == 8000);
  CHECK(mcu::parse_uint("0Xff", v) && v == 255);
  CHECK(mcu::parse_uint("0b1010", v) && v == 10);
  CHECK(mcu::parse_uint("4294967295", v) && v == 4294967295u);
  CHECK(mcu::parse_uint("0xFFFFFFFF", v) && v == 0xFFFFFFFFu);

  v = 77;
  CHECK(!mcu::parse_uint("4294967296", v) && v == 77);  // overflow, output untouched
  CHECK(!mcu::parse_uint("99999999999", v));
  CHECK(!mcu::parse_uint("0x100000000", v));
  CHECK(!mcu::parse_uint("", v));
  CHECK(!mcu::parse_uint(nullptr, v));
  CHECK(!mcu::parse_uint("0x", v));
  CHECK(!mcu::parse_uint("0b", v));
  CHECK(!mcu::parse_uint("12a", v));
  CHECK(!mcu::parse_uint("1 2", v));
  CHECK(!mcu::parse_uint("-1", v));
  CHECK(!mcu::parse_uint("+1", v));
  CHECK(!mcu::parse_uint("0b102", v));
  CHECK(!mcu::parse_uint("0xG", v));
  CHECK(!mcu::parse_uint("1.5", v));
  CHECK(v == 77);

  CHECK(mcu::parse_uint("255", v, 255) && v == 255);
  CHECK(!mcu::parse_uint("256", v, 255));
}

static void test_parse_int() {
  int32_t v = 5;
  CHECK(mcu::parse_int("0", v) && v == 0);
  CHECK(mcu::parse_int("-0", v) && v == 0);
  CHECK(mcu::parse_int("123", v) && v == 123);
  CHECK(mcu::parse_int("+123", v) && v == 123);
  CHECK(mcu::parse_int("-123", v) && v == -123);
  CHECK(mcu::parse_int("-0x10", v) && v == -16);
  CHECK(mcu::parse_int("2147483647", v) && v == mcu::kInt32Max);
  CHECK(mcu::parse_int("-2147483648", v) && v == mcu::kInt32Min);
  v = 9;
  CHECK(!mcu::parse_int("2147483648", v) && v == 9);
  CHECK(!mcu::parse_int("-2147483649", v) && v == 9);
  CHECK(!mcu::parse_int("--1", v));
  CHECK(!mcu::parse_int("-", v));
  CHECK(!mcu::parse_int("", v));
  CHECK(!mcu::parse_int(nullptr, v));
  CHECK(!mcu::parse_int("1e3", v));

  CHECK(mcu::parse_int("-100", v, -100, 100) && v == -100);
  CHECK(!mcu::parse_int("-101", v, -100, 100));
  CHECK(!mcu::parse_int("101", v, -100, 100));
}

static void test_parse_bool() {
  bool b = false;
  const char* yes[] = {"1", "on", "ON", "True", "yes", "Y"};
  const char* no[] = {"0", "off", "Off", "FALSE", "no", "n"};
  for (size_t i = 0; i < 6; ++i) {
    b = false;
    CHECK(mcu::parse_bool(yes[i], b) && b);
    b = true;
    CHECK(mcu::parse_bool(no[i], b) && !b);
  }
  b = true;
  CHECK(!mcu::parse_bool("maybe", b) && b);
  CHECK(!mcu::parse_bool("", b));
  CHECK(!mcu::parse_bool("onn", b));
  CHECK(!mcu::parse_bool("2", b));
  CHECK(!mcu::parse_bool(nullptr, b));
}

static void test_parse_scaled() {
  int32_t v = 1;
  CHECK(mcu::parse_scaled("3.14", v, 2) && v == 314);
  CHECK(mcu::parse_scaled("3.1", v, 2) && v == 310);
  CHECK(mcu::parse_scaled("5", v, 2) && v == 500);
  CHECK(mcu::parse_scaled(".25", v, 2) && v == 25);
  CHECK(mcu::parse_scaled("5.", v, 2) && v == 500);
  CHECK(mcu::parse_scaled("-0.5", v, 1) && v == -5);
  CHECK(mcu::parse_scaled("-.5", v, 1) && v == -5);
  CHECK(mcu::parse_scaled("+2.5", v, 1) && v == 25);
  CHECK(mcu::parse_scaled("0", v, 3) && v == 0);
  CHECK(mcu::parse_scaled("-0", v, 3) && v == 0);
  CHECK(mcu::parse_scaled("12", v, 0) && v == 12);
  CHECK(mcu::parse_scaled("3.300", v, 3) && v == 3300);
  CHECK(mcu::parse_scaled("0.000000001", v, 9) && v == 1);
  CHECK(mcu::parse_scaled("21474836.47", v, 2) && v == 2147483647);
  CHECK(mcu::parse_scaled("214748364.7", v, 1) && v == 2147483647);
  CHECK(mcu::parse_scaled("-21474836.48", v, 2) && v == mcu::kInt32Min);

  v = 33;
  CHECK(!mcu::parse_scaled("1.234", v, 2) && v == 33);  // too many decimals: rejected
  CHECK(!mcu::parse_scaled("1.5", v, 0));
  CHECK(!mcu::parse_scaled("21474836.48", v, 2));       // overflow
  CHECK(!mcu::parse_scaled("-21474836.49", v, 2));
  CHECK(!mcu::parse_scaled("214748364.8", v, 1));
  CHECK(!mcu::parse_scaled("2147483648", v, 0));
  CHECK(!mcu::parse_scaled("21474837", v, 2));          // 21474837.00 overflows after padding
  CHECK(!mcu::parse_scaled("", v, 2));
  CHECK(!mcu::parse_scaled(".", v, 2));
  CHECK(!mcu::parse_scaled("-", v, 2));
  CHECK(!mcu::parse_scaled("1.2.3", v, 2));
  CHECK(!mcu::parse_scaled("1,5", v, 2));
  CHECK(!mcu::parse_scaled("abc", v, 2));
  CHECK(!mcu::parse_scaled("1", v, 10));
  CHECK(!mcu::parse_scaled(nullptr, v, 2));
  CHECK(v == 33);
}

static void test_parse_hex_bytes() {
  uint8_t buf[4];
  size_t n = 0;
  CHECK(mcu::parse_hex_bytes("00fF10", buf, sizeof buf, n) && n == 3);
  CHECK(buf[0] == 0x00 && buf[1] == 0xFF && buf[2] == 0x10);
  CHECK(mcu::parse_hex_bytes("DEADBEEF", buf, sizeof buf, n) && n == 4);
  CHECK(buf[0] == 0xDE && buf[3] == 0xEF);
  CHECK(!mcu::parse_hex_bytes("DEADBEEF00", buf, sizeof buf, n));  // does not fit
  CHECK(!mcu::parse_hex_bytes("ABC", buf, sizeof buf, n));         // odd length
  CHECK(!mcu::parse_hex_bytes("", buf, sizeof buf, n));
  CHECK(!mcu::parse_hex_bytes("GG", buf, sizeof buf, n));
  CHECK(!mcu::parse_hex_bytes("0x12", buf, sizeof buf, n));
  CHECK(!mcu::parse_hex_bytes(nullptr, buf, sizeof buf, n));
}

// ---------------------------------------------------------------------------
// output helpers
// ---------------------------------------------------------------------------
static void test_output() {
  mcu::BufferOutput<64> o;
  o.print_uint(0);
  CHECK_STR(o.c_str(), "0");
  o.clear();
  o.print_uint(4294967295u);
  CHECK_STR(o.c_str(), "4294967295");
  o.clear();
  o.print_uint(255, 16);
  CHECK_STR(o.c_str(), "FF");
  o.clear();
  o.print_uint(5, 2);
  CHECK_STR(o.c_str(), "101");
  o.clear();
  o.print_uint(10, 16, 4);
  CHECK_STR(o.c_str(), "000A");
  o.clear();
  o.print_uint(0xFFFFFFFFu, 2);
  CHECK(o.size() == 32);
  o.clear();
  o.print_uint(7, 99);  // bad base falls back to decimal
  CHECK_STR(o.c_str(), "7");
  o.clear();
  o.print_int(-1);
  CHECK_STR(o.c_str(), "-1");
  o.clear();
  o.print_int(mcu::kInt32Min);
  CHECK_STR(o.c_str(), "-2147483648");
  o.clear();
  o.print_int(mcu::kInt32Max);
  CHECK_STR(o.c_str(), "2147483647");
  o.clear();
  o.print_hex(0xAB, 4);
  CHECK_STR(o.c_str(), "00AB");
  o.clear();
  o.print_fixed(-5, 2);
  CHECK_STR(o.c_str(), "-0.05");
  o.clear();
  o.print_fixed(31415, 3);
  CHECK_STR(o.c_str(), "31.415");
  o.clear();
  o.print_fixed(0, 2);
  CHECK_STR(o.c_str(), "0.00");
  o.clear();
  o.print_fixed(5, 0);
  CHECK_STR(o.c_str(), "5");
  o.clear();
  o.print_fixed(mcu::kInt32Min, 2);
  CHECK_STR(o.c_str(), "-21474836.48");
  o.clear();
  const uint8_t d[3] = {0xDE, 0xAD, 0x01};
  o.print_bytes_hex(d, 3);
  CHECK_STR(o.c_str(), "DEAD01");
  o.clear();
  o.println("hi");
  CHECK_STR(o.c_str(), "hi\r\n");
  o.clear();
  o.print(nullptr);
  o.println();
  CHECK_STR(o.c_str(), "\r\n");

  mcu::BufferOutput<4> small;  // holds 3 characters
  small.print("abcdef");
  CHECK_STR(small.c_str(), "abc");
  CHECK(small.overflowed());

  // print_fixed and parse_scaled are inverses.
  for (int i = 0; i < 2000; ++i) {
    int32_t val = (int32_t)(((uint32_t)rnd8() << 24 | (uint32_t)rnd8() << 16 | (uint32_t)rnd8() << 8 | rnd8()));
    unsigned dec = rnd8() % 10;
    mcu::BufferOutput<32> b;
    b.print_fixed(val, dec);
    int32_t back = 0;
    CHECK(mcu::parse_scaled(b.c_str(), back, dec));
    CHECK(back == val);
  }
}

// ---------------------------------------------------------------------------
// line editor
// ---------------------------------------------------------------------------
struct Feed {
  mcu::LineEvent last;
  int lines;
  int cancels;
  int toolong;
};

template <typename Ed>
static Feed feed_str(Ed& ed, const char* s, char* captured = nullptr) {
  Feed f = {mcu::LineEvent::kNone, 0, 0, 0};
  for (; *s; ++s) {
    f.last = ed.feed((uint8_t)*s);
    if (f.last == mcu::LineEvent::kLine) {
      ++f.lines;
      if (captured) strcpy(captured, ed.line());
    } else if (f.last == mcu::LineEvent::kCancelled) {
      ++f.cancels;
    } else if (f.last == mcu::LineEvent::kTooLong) {
      ++f.toolong;
    }
  }
  return f;
}

static void test_line_editor() {
  char got[64];
  {
    mcu::BufferOutput<64> echo;
    mcu::LineEditor<32> ed(&echo);
    Feed f = feed_str(ed, "hello\r", got);
    CHECK(f.lines == 1 && f.last == mcu::LineEvent::kLine);
    CHECK_STR(got, "hello");
    CHECK_STR(echo.c_str(), "hello\r\n");
  }
  {  // CR+LF counts once; bare LF also ends a line
    mcu::LineEditor<32> ed(nullptr);
    Feed f = feed_str(ed, "one\r\ntwo\r\nthree\n", got);
    CHECK(f.lines == 3);
    CHECK_STR(got, "three");
    f = feed_str(ed, "\n\r\n", got);  // LF, CR+LF: empty lines
    CHECK(f.lines == 2);
    CHECK_STR(got, "");
  }
  {  // LF after CR is swallowed even across calls
    mcu::LineEditor<32> ed(nullptr);
    CHECK(ed.feed('a') == mcu::LineEvent::kNone);
    CHECK(ed.feed('\r') == mcu::LineEvent::kLine);
    CHECK(ed.feed('\n') == mcu::LineEvent::kNone);
    CHECK(ed.feed('b') == mcu::LineEvent::kNone);
    CHECK(ed.feed('\r') == mcu::LineEvent::kLine);
    CHECK_STR(ed.line(), "b");
  }
  {  // backspace and DEL
    mcu::BufferOutput<64> echo;
    mcu::LineEditor<32> ed(&echo);
    feed_str(ed, "ab\b\bc\r", got);
    CHECK_STR(got, "c");
    CHECK_STR(echo.c_str(), "ab\b \b\b \bc\r\n");
    echo.clear();
    feed_str(ed, "xy\x7F" "z\r", got);
    CHECK_STR(got, "xz");
    echo.clear();
    feed_str(ed, "\b\b\b" "q\r", got);  // backspace on an empty line: nothing, no echo
    CHECK_STR(got, "q");
    CHECK_STR(echo.c_str(), "q\r\n");
  }
  {  // Ctrl-U kills the line
    mcu::BufferOutput<64> echo;
    mcu::LineEditor<32> ed(&echo);
    feed_str(ed, "abc\x15" "d\r", got);
    CHECK_STR(got, "d");
    CHECK_STR(echo.c_str(), "abc\b \b\b \b\b \bd\r\n");
  }
  {  // Ctrl-C cancels
    mcu::BufferOutput<64> echo;
    mcu::LineEditor<32> ed(&echo);
    Feed f = feed_str(ed, "abc\x03");
    CHECK(f.cancels == 1 && f.lines == 0);
    CHECK_STR(echo.c_str(), "abc^C\r\n");
    f = feed_str(ed, "x\r", got);
    CHECK(f.lines == 1);
    CHECK_STR(got, "x");
  }
  {  // escape sequences do not insert characters
    mcu::LineEditor<32> ed(nullptr);
    feed_str(ed, "a\x1b[Ab\x1b[1;5Cc\x1bOP" "d\x1b[3~" "e\r", got);
    CHECK_STR(got, "abcde");
    feed_str(ed, "a\x1b\r", got);  // ESC then Enter: ESC alone, Enter still works
    CHECK_STR(got, "a");
    feed_str(ed, "a\x1b[1;\r", got);  // truncated sequence interrupted by Enter
    CHECK_STR(got, "a");
  }
  {  // control characters ignored, tab and high bytes kept
    mcu::LineEditor<32> ed(nullptr);
    feed_str(ed, "a\x01\x02" "b\r", got);
    CHECK_STR(got, "ab");
    feed_str(ed, "a\tb\r", got);
    CHECK_STR(got, "a\tb");
    feed_str(ed, "\xC3\xA9\r", got);
    CHECK_STR(got, "\xC3\xA9");
  }
  {  // line too long: refused, reported, never run truncated
    mcu::BufferOutput<64> echo;
    mcu::LineEditor<8> ed(&echo);  // up to 7 characters
    Feed f = feed_str(ed, "1234567\r", got);
    CHECK(f.lines == 1);
    CHECK_STR(got, "1234567");
    echo.clear();
    f = feed_str(ed, "12345678\r");
    CHECK(f.toolong == 1 && f.lines == 0);
    CHECK_STR(echo.c_str(), "1234567\a\r\n");
    f = feed_str(ed, "ok\r", got);
    CHECK(f.lines == 1);
    CHECK_STR(got, "ok");
    // backspace takes back refused characters first
    feed_str(ed, "12345678\b\r", got);
    CHECK_STR(got, "1234567");
    feed_str(ed, "123456789\b\b\b\r", got);
    CHECK_STR(got, "123456");
    f = feed_str(ed, "123456789\b\b\r");  // 2 refused, 2 taken back: the line is valid again
    CHECK(f.lines == 1 && f.toolong == 0);
    f = feed_str(ed, "1234567890\b\b\r");  // 3 refused, only 2 taken back: still too long
    CHECK(f.toolong == 1 && f.lines == 0);
    // Ctrl-U also clears the refused count
    f = feed_str(ed, "123456789\x15" "z\r", got);
    CHECK(f.lines == 1);
    CHECK_STR(got, "z");
  }
  {  // set_echo and reset
    mcu::BufferOutput<64> echo;
    mcu::LineEditor<16> ed;
    feed_str(ed, "ab\r", got);
    CHECK(echo.size() == 0);
    ed.set_echo(&echo);
    feed_str(ed, "cd\r", got);
    CHECK_STR(echo.c_str(), "cd\r\n");
    feed_str(ed, "partial");
    ed.reset();
    feed_str(ed, "fresh\r", got);
    CHECK_STR(got, "fresh");
  }
  {  // random bytes never break it (run under a sanitizer)
    mcu::BufferOutput<256> echo;
    mcu::LineEditor<12> ed(&echo);
    for (int i = 0; i < 100000; ++i) {
      ed.feed(rnd8());
      if (echo.size() > 200) echo.clear();
      CHECK(ed.length() < 12);
    }
  }
}

// ---------------------------------------------------------------------------
// console
// ---------------------------------------------------------------------------
static int g_count = 0;

static int cmd_add(mcu::CommandContext& c) {
  int32_t a, b;
  if (!c.get_int(1, a) || !c.get_int(2, b)) return mcu::kCmdReported;
  c.out.print_int(a + b);
  c.out.println();
  return mcu::kCmdOk;
}
static int cmd_echo(mcu::CommandContext& c) {
  for (size_t i = 1; i < c.argc; ++i) {
    if (i > 1) c.out.put(' ');
    c.out.print(c.argv[i]);
  }
  c.out.println();
  return mcu::kCmdOk;
}
static int cmd_led(mcu::CommandContext& c) {
  bool on;
  if (!c.get_bool(1, on)) return mcu::kCmdReported;
  *static_cast<bool*>(c.user) = on;
  return mcu::kCmdOk;
}
static int cmd_fail(mcu::CommandContext&) { return 5; }
static int cmd_usage(mcu::CommandContext&) { return mcu::kCmdUsage; }
static int cmd_quiet(mcu::CommandContext&) { return mcu::kCmdReported; }
static int cmd_flood(mcu::CommandContext& c) {
  for (int i = 0; i < 40; ++i) c.out.print("0123456789");
  c.out.println();
  return mcu::kCmdOk;
}
static int cmd_count(mcu::CommandContext&) {
  ++g_count;
  return mcu::kCmdOk;
}
static int cmd_scaled(mcu::CommandContext& c) {
  int32_t v;
  if (!c.get_scaled(1, v, 2)) return mcu::kCmdReported;
  c.out.print_fixed(v * 2, 2);
  c.out.println();
  return mcu::kCmdOk;
}

static const mcu::Command kCmds[] = {
    {"add", cmd_add, "add two numbers", "<a> <b>", 2, 2},
    {"echo", cmd_echo, "print arguments", "[text...]", 0, mcu::kAnyArgs},
    {"led", cmd_led, "switch the LED", "<on|off>", 1, 1},
    {"fail", cmd_fail, "always fails", nullptr, 0, 0},
    {"usage", cmd_usage, "asks for usage", "<x>", 0, 0},
    {"quiet", cmd_quiet, "fails silently", nullptr, 0, 0},
    {"flood", cmd_flood, "prints a lot", nullptr, 0, 0},
    {"count", cmd_count, "counts calls", nullptr, 0, 0},
    {"scaled", cmd_scaled, "doubles a 2-decimal number", "<x>", 1, 1},
};
static const size_t kNumCmds = sizeof kCmds / sizeof kCmds[0];

// Runs the console against `input` and returns everything it printed.
template <typename Con, typename Wire>
static const char* exec(Con& con, Wire& wire, const char* input) {
  static char out[4096];
  size_t n = 0;
  wire.inject(reinterpret_cast<const uint8_t*>(input), strlen(input));
  for (int i = 0; i < 64; ++i) {
    con.poll();
    n += wire.take(reinterpret_cast<uint8_t*>(out) + n, sizeof out - 1 - n);
  }
  out[n] = '\0';
  return out;
}

static void test_console_basics() {
  bool led = false;
  mcu::LoopbackStream<256, 512> wire;
  mcu::Console<64, 8, 512> con(wire, kCmds, kNumCmds, &led);

  con.begin();
  uint8_t tmp[16];
  size_t n = wire.take(tmp, sizeof tmp);
  CHECK(n == 2 && tmp[0] == '>' && tmp[1] == ' ');

  CHECK_STR(exec(con, wire, "add 2 3\r"), "add 2 3\r\n5\r\n> ");
  CHECK_STR(exec(con, wire, "add -10 0x10\r"), "add -10 0x10\r\n6\r\n> ");
  CHECK_STR(exec(con, wire, "\r"), "\r\n> ");
  CHECK_STR(exec(con, wire, "   \r"), "   \r\n> ");
  CHECK_STR(exec(con, wire, "bogus\r"), "bogus\r\nunknown command: bogus (try 'help')\r\n> ");
  CHECK_STR(exec(con, wire, "add 1\r"), "add 1\r\nusage: add <a> <b>\r\n> ");
  CHECK_STR(exec(con, wire, "add 1 2 3\r"), "add 1 2 3\r\nusage: add <a> <b>\r\n> ");
  CHECK_STR(exec(con, wire, "add 1 x\r"), "add 1 x\r\nerror: bad argument 2: x\r\n> ");
  CHECK_STR(exec(con, wire, "fail\r"), "fail\r\nerror 5\r\n> ");
  CHECK_STR(exec(con, wire, "usage\r"), "usage\r\nusage: usage <x>\r\n> ");
  CHECK_STR(exec(con, wire, "quiet\r"), "quiet\r\n> ");
  CHECK_STR(exec(con, wire, "scaled 1.25\r"), "scaled 1.25\r\n2.50\r\n> ");
  CHECK_STR(exec(con, wire, "scaled 1.255\r"), "scaled 1.255\r\nerror: bad argument 1: 1.255\r\n> ");

  // quoting
  CHECK_STR(exec(con, wire, "echo \"a  b\" 'c d' e\\ f\r"),
            "echo \"a  b\" 'c d' e\\ f\r\na  b c d e f\r\n> ");
  CHECK_STR(exec(con, wire, "echo\r"), "echo\r\n\r\n> ");
  CHECK_STR(exec(con, wire, "echo \"abc\r"), "echo \"abc\r\nerror: unterminated quote\r\n> ");

  // the user pointer reaches the handler
  CHECK(!led);
  exec(con, wire, "led on\r");
  CHECK(led);
  exec(con, wire, "led OFF\r");
  CHECK(!led);
  CHECK_STR(exec(con, wire, "led maybe\r"), "led maybe\r\nerror: bad argument 1: maybe\r\n> ");

  // Ctrl-C and editing
  CHECK_STR(exec(con, wire, "abc\x03"), "abc^C\r\n> ");
  CHECK_STR(exec(con, wire, "ecXo\b\bho hi\r"), "ecXo\b \b\b \bho hi\r\nhi\r\n> ");

  CHECK(con.dropped_output() == 0);
}

static void test_console_help() {
  mcu::LoopbackStream<256, 1024> wire;
  mcu::Console<64, 8, 1024> con(wire, kCmds, kNumCmds);

  const char* out = exec(con, wire, "help\r");
  CHECK(strstr(out, "  add     add two numbers\r\n") != nullptr);
  CHECK(strstr(out, "  scaled  doubles a 2-decimal number\r\n") != nullptr);
  CHECK(strstr(out, "  help    list commands") != nullptr);
  CHECK(strncmp(out, "help\r\n", 6) == 0);

  CHECK_STR(exec(con, wire, "help add\r"), "help add\r\nadd <a> <b>\r\n  add two numbers\r\n> ");
  CHECK_STR(exec(con, wire, "help fail\r"), "help fail\r\nfail\r\n  always fails\r\n> ");
  CHECK_STR(exec(con, wire, "help nosuch\r"), "help nosuch\r\nunknown command: nosuch\r\n> ");
}

static void test_console_options() {
  bool led = false;
  mcu::LoopbackStream<256, 512> wire;
  mcu::Console<64, 8, 512> con(wire, kCmds, kNumCmds, &led);

  CHECK_STR(exec(con, wire, "ECHO hi\r"), "ECHO hi\r\nunknown command: ECHO (try 'help')\r\n> ");
  con.set_ignore_case(true);
  CHECK_STR(exec(con, wire, "ECHO hi\r"), "ECHO hi\r\nhi\r\n> ");
  CHECK_STR(exec(con, wire, "HELP echo\r"), "HELP echo\r\necho [text...]\r\n  print arguments\r\n> ");

  con.set_prompt("cli> ");
  CHECK_STR(exec(con, wire, "echo x\r"), "echo x\r\nx\r\ncli> ");

  con.set_echo(false);
  CHECK_STR(exec(con, wire, "echo quiet\r"), "quiet\r\ncli> ");
  con.set_echo(true);

  // out() mixes banner text in with command output
  con.out().println("hello from firmware");
  CHECK_STR(exec(con, wire, ""), "hello from firmware\r\n");
}

static void test_console_limits() {
  {  // too many arguments (MaxArgs counts the command name)
    mcu::LoopbackStream<256, 512> wire;
    mcu::Console<64, 4, 512> con(wire, kCmds, kNumCmds);
    CHECK_STR(exec(con, wire, "echo a b c\r"), "echo a b c\r\na b c\r\n> ");
    CHECK_STR(exec(con, wire, "echo a b c d\r"),
              "echo a b c d\r\nerror: too many arguments\r\n> ");
  }
  {  // line too long
    mcu::LoopbackStream<256, 512> wire;
    mcu::Console<16, 8, 512> con(wire, kCmds, kNumCmds);
    const char* out = exec(con, wire, "echo 0123456789012345678901234567890\r");
    CHECK(strstr(out, "error: line too long\r\n> ") != nullptr);
    CHECK(strstr(out, "unknown command") == nullptr);
    // and the console is fine afterwards
    CHECK_STR(exec(con, wire, "add 1 2\r"), "add 1 2\r\n3\r\n> ");
  }
}

static void test_console_one_command_per_poll() {
  mcu::LoopbackStream<256, 512> wire;
  mcu::Console<64, 8, 512> con(wire, kCmds, kNumCmds);
  g_count = 0;
  const char* in = "count\rcount\rcount\r";
  wire.inject(reinterpret_cast<const uint8_t*>(in), strlen(in));
  con.poll();
  CHECK(g_count == 1);
  con.poll();
  CHECK(g_count == 2);
  con.poll();
  CHECK(g_count == 3);
  con.poll();
  CHECK(g_count == 3);
}

static void test_console_backpressure() {
  mcu::LoopbackStream<64, 16> wire;  // transmit side holds only 16 bytes
  mcu::Console<64, 8, 512> con(wire, kCmds, kNumCmds);

  const char* a = "help\r";
  wire.inject(reinterpret_cast<const uint8_t*>(a), strlen(a));
  con.poll();
  CHECK(wire.tx_size() == 16);  // the stream is full; the rest waits in the queue

  // The next command arrives while the reply is still stuck.
  const char* b = "echo hi\r";
  wire.inject(reinterpret_cast<const uint8_t*>(b), strlen(b));
  for (int i = 0; i < 10; ++i) con.poll();
  CHECK(wire.tx_size() == 16);
  CHECK(wire.available() == strlen(b));  // not read: the console waits for the stream

  // Drain slowly; everything arrives complete and in order.
  static char out[4096];
  size_t n = 0;
  for (int i = 0; i < 2000; ++i) {
    con.poll();
    n += wire.take(reinterpret_cast<uint8_t*>(out) + n, 5);
  }
  out[n] = '\0';
  CHECK(strncmp(out, "help\r\n", 6) == 0);
  CHECK(strstr(out, "  add     add two numbers\r\n") != nullptr);
  const char* tail = "echo hi\r\nhi\r\n> ";
  CHECK(n >= strlen(tail) && strcmp(out + n - strlen(tail), tail) == 0);
  CHECK(con.dropped_output() == 0);
}

static void test_console_drops_when_flooded() {
  mcu::LoopbackStream<64, 16> wire;
  mcu::Console<64, 8, 16> con(wire, kCmds, kNumCmds);  // 16-byte queue, 16-byte stream
  const char* a = "flood\r";
  wire.inject(reinterpret_cast<const uint8_t*>(a), strlen(a));
  con.poll();
  CHECK(con.dropped_output() > 0);  // lost, but counted and nothing crashed

  // After draining, the console still works.
  static char out[1024];
  size_t n = 0;
  for (int i = 0; i < 50; ++i) {
    con.poll();
    n += wire.take(reinterpret_cast<uint8_t*>(out) + n, 16);
  }
  const char* b = "echo ok\r";
  wire.inject(reinterpret_cast<const uint8_t*>(b), strlen(b));
  n = 0;
  for (int i = 0; i < 50; ++i) {
    con.poll();
    n += wire.take(reinterpret_cast<uint8_t*>(out) + n, 16);
  }
  out[n] = '\0';
  CHECK(strstr(out, "ok\r\n") != nullptr);
}

// A stream that accepts less than write_space() promised.
class LyingStream final : public mcu::Stream {
 public:
  size_t read(uint8_t*, size_t) override { return 0; }
  size_t write(const uint8_t*, size_t len) override { return len > 1 ? len - 1 : len; }
  size_t available() const override { return 0; }
  size_t write_space() const override { return 100; }
};

static void test_console_lying_stream() {
  LyingStream s;
  mcu::Console<32, 4, 64> con(s, kCmds, kNumCmds);
  con.out().println("some text to send");
  con.poll();
  CHECK(con.dropped_output() >= 1);  // short writes are counted as drops, never looped on
}

static void test_console_fuzz() {
  bool led = false;
  mcu::LoopbackStream<256, 64> wire;
  mcu::Console<24, 5, 128> con(wire, kCmds, kNumCmds, &led);
  static const char* words[] = {"add ", "echo ", "led ", "help ", "scaled ", "1.5", "-7", "\"", "'",
                                "\\", "\r", "\n", "\x03", "\x15", "\b", "\x1b[A", " ", "on", "flood\r"};
  uint8_t sink[64];
  for (int iter = 0; iter < 20000; ++iter) {
    const char* w = words[rnd8() % (sizeof words / sizeof words[0])];
    wire.inject(reinterpret_cast<const uint8_t*>(w), strlen(w));
    if ((iter & 3) == 0) {
      uint8_t junk = rnd8();
      wire.inject(&junk, 1);
    }
    con.poll();
    wire.take(sink, sizeof sink);
  }
  CHECK(true);  // reaching here under the sanitizers is the test
}

int main() {
  test_tokenizer();
  test_tokenizer_fuzz();
  test_parse_uint();
  test_parse_int();
  test_parse_bool();
  test_parse_scaled();
  test_parse_hex_bytes();
  test_output();
  test_line_editor();
  test_console_basics();
  test_console_help();
  test_console_options();
  test_console_limits();
  test_console_one_command_per_poll();
  test_console_backpressure();
  test_console_drops_when_flooded();
  test_console_lying_stream();
  test_console_fuzz();

  printf("%d checks, %d failures\n", g_checks, g_failures);
  return g_failures ? 1 : 0;
}
