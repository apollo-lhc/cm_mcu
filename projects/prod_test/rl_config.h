/*
 * Microrl library config files
 * Autor: Eugene Samoylov aka Helius (ghelius@gmail.com)
 */
#ifndef MICRORL_CONFIG_H_
#define MICRORL_CONFIG_H_

#define MICRORL_LIB_VER "1.5.1"

// Custom config
// PW HACK
#define MICRORL_PROMPT_DEFAULT "# "
#define MICRORL_PROMPT_LEN     2

#define MICRORL_USE_COMPLETE 0
#define MICRORL_USE_LIBC_STDIO 0

#define MICRORL_USE_CTRL_C 0
// end PW HACK

/******************************* CONFIG SECTION *******************************/
#ifdef MICRORL_INCLUDE_CONFIG_H
#include "microrl_custom_config.h"
#endif
/*
 * Command line length, define cmdline buffer size. Set max number of chars +
 * 1, because last byte of buffer need to contain '\0' - NULL terminator, and
 * not use for storing inputed char.  If user input chars more then it
 * parametrs-1, chars not added to command line.
 */
#ifndef MICRORL_COMMAND_LINE_LEN
#define MICRORL_COMMAND_LINE_LEN (1 + 100)
#endif

/*
 * Command token number, define max token it command line, if number of token
 * typed in command line exceed this value, then prints message about it and
 * command line not to be parced and 'execute' callback will not calls.
 * Token is word separate by white space, for example 3 token line:
 * "IRin> set mode test"
 */
#ifndef MICRORL_COMMAND_TOKEN_NMB
#define MICRORL_COMMAND_TOKEN_NMB 8
#endif

/*
 * Define you prompt string here. You can use colors escape code, for highlight
 * you prompt, for example this prompt will green color (if you terminal
 * supports color)
 */
//#define MICRORL_PROMPT_DEFAULT "\033[32mIRin >\033[0m "	// green color
#ifndef MICRORL_PROMPT_DEFAULT
#define MICRORL_PROMPT_DEFAULT "\033[32mIRin >\033[0m " // green color
#endif
//#define MICRORL_PROMPT_DEFAULT "IRin > "

/*
 * Define prompt text (without ESC sequence, only text) prompt length, it needs
 * because if you use ESC sequence, it's not possible detect only text length
 */
#ifndef MICRORL_PROMPT_LEN
#define MICRORL_PROMPT_LEN 7
#endif

/*
 * Define it, if you wanna use completion functional, also set completion
 * callback in you code, now if user press TAB calls 'copmlitetion' callback.
 * If you no need it, you can just set NULL to callback ptr and do not use it,
 * but for memory saving tune, if you are not going to use it - disable this
 * define.
 */
#ifndef MICRORL_USE_COMPLETE
#define MICRORL_USE_COMPLETE 1
#endif

/*
 * Define it, if you wanna use history. It s work's like bash history, and
 * set stored value to cmdline, if UP and DOWN key pressed. Using history add
 * memory consuming, depends from MICRORL_RING_HISTORY_LEN parametr
 */
#ifndef MICRORL_USE_HISTORY
#define MICRORL_USE_HISTORY 1
#endif

/*
 * History ring buffer length, define static buffer size.  For saving memory,
 * each entered cmdline store to history in ring buffer, so we can not say, how
 * many line we can store, it depends from cmdline len, but memory using more
 * effective. We not prefer dinamic memory allocation for small and embedded
 * devices. Overhead is 2 char on each saved line
 */
#ifndef MICRORL_RING_HISTORY_LEN
#define MICRORL_RING_HISTORY_LEN 128
#endif

/*
 * Enable Handling terminal ESC sequence. If disabling, then cursor arrow,
 * HOME, END will not work, use Ctrl+A(B,F,P,N,A,E,H,K,U,C) see README, but
 * decrease code memory.
 * */
#ifndef MICRORL_USE_ESC_SEQ
#define MICRORL_USE_ESC_SEQ 1
#endif

/*
 * Use snprintf from you standard complier library, but it gives some overhead.
 * If not defined, use my own u16int_to_str variant, it's save about 800 byte
 * of Code size on AVR (avr-gcc build).  Try to build with and without, and
 * compare Total code size for tune library.
 */
#ifndef MICRORL_USE_LIBC_STDIO
#define MICRORL_USE_LIBC_STDIO 1
#endif

/*
 * Enable 'interrupt signal' callback, if user press Ctrl+C
 */
#ifndef MICRORL_USE_CTRL_C
#define MICRORL_USE_CTRL_C 1
#endif

/*
 * Print prompt at 'microrl_init', if enable, prompt will print at startup,
 * otherwise first prompt will print after first press Enter in terminal NOTE!:
 * Enable it, if you call 'microrl_init' after your communication subsystem
 * already initialize and ready to print message
 */
#ifndef MICRORL_ENABLE_INIT_PROMPT
#define MICRORL_ENABLE_INIT_PROMPT 1
#endif

/*
 * New line symbol
 */
#if !defined(MICRORL_ENDL_CR) && !defined(MICRORL_ENDL_CRLF) && !defined(MICRORL_ENDL_LF) &&       \
    !defined(MICRORL_ENDL_LFCR)
#define MICRORL_ENDL_CR
#endif

#if defined(MICRORL_ENDL_CR)
#define MICRORL_ENDL "\r"
#elif defined(MICRORL_ENDL_CRLF)
#define MICRORL_ENDL "\r\n"
#elif defined(MICRORL_ENDL_LF)
#define MICRORL_ENDL "\n"
#elif defined(MICRORL_ENDL_LFCR)
#define MICRORL_ENDL "\n\r"
#else
#error "You must define new line symbol."
#endif

/********** END CONFIG SECTION ************/

#if MICRORL_RING_HISTORY_LEN > 256
#error                                                                                             \
    "This history implementation (ring buffer with 1 byte iterator) allow 256 byte buffer size maximum"
#endif

#endif // MICRORL_CONFIG_H_
