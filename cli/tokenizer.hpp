// tokenizer.hpp - split a command line into arguments, in place, no heap.
//
//   char line[] = "set name \"hello world\" 42";
//   char* argv[8 + 1];
//   size_t argc;
//   if (mcu::tokenize(line, argv, 8, argc) == mcu::TokenizeStatus::kOk) {
//     // argc == 4: "set", "name", "hello world", "42"
//   }
//
// The line is edited in place (separators become NULs, quotes and escapes are
// removed), so argv[] points into it and it must stay alive while you use them.
//
// Rules, close to a Unix shell:
//   * Space and tab separate arguments.
//   * "double quotes" group text. Inside them \" \\ \n \t \r are escapes and
//     any other backslash pair yields the second character.
//   * 'single quotes' group text literally, with no escapes.
//   * Outside quotes a backslash makes the next character literal: a\ b is one
//     argument. Quoted and unquoted pieces next to each other join: ab"c d"e
//     is one argument, abc de.
//   * "" gives an empty argument.
//
// Header-only, C++11, no heap.

#ifndef MCU_TOKENIZER_HPP
#define MCU_TOKENIZER_HPP

#include <stddef.h>
#include <stdint.h>

namespace mcu {

enum class TokenizeStatus : uint8_t {
  kOk,
  kTooManyArgs,        // more than max_args arguments
  kUnterminatedQuote   // a quote was opened and never closed
};

namespace detail {
inline char unescape_char(char c) {
  switch (c) {
    case 'n': return '\n';
    case 't': return '\t';
    case 'r': return '\r';
    default: return c;
  }
}
}  // namespace detail

// `argv` must have room for max_args + 1 pointers: on success argv[argc] is
// set to nullptr. argc is set even on failure (to what was found so far).
inline TokenizeStatus tokenize(char* line, char** argv, size_t max_args, size_t& argc) {
  char* r = line;  // read position
  char* w = line;  // write position; never ahead of r
  argc = 0;
  for (;;) {
    while (*r == ' ' || *r == '\t') ++r;
    if (*r == '\0') break;
    if (argc >= max_args) return TokenizeStatus::kTooManyArgs;
    argv[argc++] = w;
    while (*r != '\0' && *r != ' ' && *r != '\t') {
      if (*r == '"') {
        ++r;
        while (*r != '\0' && *r != '"') {
          if (*r == '\\' && r[1] != '\0') {
            ++r;
            *w++ = detail::unescape_char(*r++);
          } else {
            *w++ = *r++;
          }
        }
        if (*r == '\0') return TokenizeStatus::kUnterminatedQuote;
        ++r;
      } else if (*r == '\'') {
        ++r;
        while (*r != '\0' && *r != '\'') *w++ = *r++;
        if (*r == '\0') return TokenizeStatus::kUnterminatedQuote;
        ++r;
      } else if (*r == '\\' && r[1] != '\0') {
        ++r;
        *w++ = *r++;
      } else {
        *w++ = *r++;
      }
    }
    const bool at_end = (*r == '\0');
    if (!at_end) ++r;
    *w++ = '\0';
  }
  argv[argc] = nullptr;
  return TokenizeStatus::kOk;
}

}  // namespace mcu

#endif  // MCU_TOKENIZER_HPP
