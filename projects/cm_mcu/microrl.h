#ifndef _MICRORL_H_
#define _MICRORL_H_

#include "config.h"

#include <stdbool.h>
#include <stdint.h>

#if MICRORL_USE_HISTORY
// history struct, contain internal variable
// history store in static ring buffer for memory saving
typedef struct {
	char ring_buf[MICRORL_RING_HISTORY_LEN];
	int begin;
	int end;
	int cur;
} ring_history_t;
#endif

struct microrl_config {
	const char *prompt_str; // pointer to prompt string
	uint8_t prompt_length;
	/**
	 * \brief Userspace cmdline execution callback (called when newline is
	 * received)
	 */
	int (*execute)(void *userdata, int argc, char **argv);
#if MICRORL_USE_COMPLETE
	/**
	 * \brief Userspace completion retrieval callback (called when <TAB> is
	 * received)
	 */
	const char **(*get_completion)(void *userdata, int argc,
								   const char *const *argv);
#endif
	/**
	 * \brief Userspace print callback
	 */
	void (*print)(const char *);
#if MICRORL_USE_CTRL_C
	/**
	 * \brief Userspace SIGINT handler
	 */
	void (*sigint)(void *userdata);
#endif
	/**
	 * \brief Userspace EOF handler
	 */
	void (*eof)(void *userdata);
	/**
	 * \brief User data
	 */
	void *userdata;
};

// microrl struct, contain internal library data
typedef struct {
#if MICRORL_USE_ESC_SEQ
	char escape_seq;
	char escape;
#endif
#if (defined(MICRORL_ENDL_CRLF) || defined(MICRORL_ENDL_LFCR))
	char tmpch;
#endif
#if MICRORL_USE_HISTORY
	ring_history_t ring_hist; // history object
#endif
#if MICRORL_USE_CTRL_C
	bool pending_execution;
#endif
	char cmdline[MICRORL_COMMAND_LINE_LEN]; // cmdline buffer
	int cmdlen;								// last position in command line
	int cursor;								// input cursor
	struct microrl_config config;
} microrl_t;

// init internal data, calls once at start up
void microrl_init(microrl_t *pThis, struct microrl_config *config);

// set echo mode (true/false), using for disabling echo for password input
// echo mode will enabled after user press Enter.
void microrl_set_echo(int);

void microrl_set_prompt(microrl_t *pThis, const char *prompt_str,
						uint8_t prompt_length);
#if MICRORL_USE_COMPLETE
// set pointer to callback complition func, that called when user press 'Tab'
// callback func description:
//   param: argc - argument count, argv - pointer array to token string
//   must return NULL-terminated string, contain complite variant splitted by
//   'Whitespace' If complite token found, it's must contain only one token to
//   be complitted Empty string if complite not found, and multiple string if
//   there are some token
void microrl_set_complete_callback(
	microrl_t *pThis,
	const char **(*get_completion)(void *, int, const char *const *));
#endif

// pointer to callback func, that called when user press 'Enter'
// execute func param: argc - argument count, argv - pointer array to token
// string
void microrl_set_execute_callback(microrl_t *pThis,
								  int (*execute)(void *, int, char **));

// set callback for Ctrl+C terminal signal
#if MICRORL_USE_CTRL_C
void microrl_set_sigint_callback(microrl_t *pThis,
								 void (*sigint)(void *userdata));
#endif

// insert char to cmdline (for example call in usart RX interrupt)
void microrl_insert_char(microrl_t *pThis, int ch);

#endif
