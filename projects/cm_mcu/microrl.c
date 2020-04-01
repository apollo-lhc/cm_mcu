/*
Author: Samoylov Eugene aka Helius (ghelius@gmail.com)
BUGS and TODO:
-- add echo_off feature
-- rewrite history for use more than 256 byte buffer
*/
#include "microrl.h"

#include <ctype.h>
#include <stdlib.h>
#include <string.h>
#if MICRORL_USE_LIBC_STDIO
#include <stdio.h>
#endif
#include <stdbool.h>

/*******************************************************************************
 * Key codes
 *******************************************************************************/
#define KEY_NUL 0  /**< ^@ Null character */
#define KEY_SOH 1  /**< ^A Start of heading, = console interrupt */
#define KEY_STX 2  /**< ^B Start of text, maintenance mode on HP console */
#define KEY_ETX 3  /**< ^C End of text */
#define KEY_EOT 4  /**< ^D End of transmission, not the same as ETB */
#define KEY_ENQ 5  /**< ^E Enquiry, goes with ACK; old HP flow control */
#define KEY_ACK 6  /**< ^F Acknowledge, clears ENQ logon hand */
#define KEY_BEL 7  /**< ^G Bell, rings the bell... */
#define KEY_BS 8   /**< ^H Backspace, works on HP terminals/computers */
#define KEY_HT 9   /**< ^I Horizontal tab, move to next tab stop */
#define KEY_LF 10  /**< ^J Line Feed */
#define KEY_VT 11  /**< ^K Vertical tab */
#define KEY_FF 12  /**< ^L Form Feed, page eject */
#define KEY_CR 13  /**< ^M Carriage Return*/
#define KEY_SO 14  /**< ^N Shift Out, alternate character set */
#define KEY_SI 15  /**< ^O Shift In, resume defaultn character set */
#define KEY_DLE 16 /**< ^P Data link escape */
#define KEY_DC1                                                                \
	17				/**< ^Q XON, with XOFF to pause listings; "okay to send".  \
					 */
#define KEY_DC2 18  /**< ^R Device control 2, block-mode flow control */
#define KEY_DC3 19  /**< ^S XOFF, with XON is TERM=18 flow control */
#define KEY_DC4 20  /**< ^T Device control 4 */
#define KEY_NAK 21  /**< ^U Negative acknowledge */
#define KEY_SYN 22  /**< ^V Synchronous idle */
#define KEY_ETB 23  /**< ^W End transmission block, not the same as EOT */
#define KEY_CAN 24  /**< ^X Cancel line, MPE echoes !!! */
#define KEY_EM 25   /**< ^Y End of medium, Control-Y interrupt */
#define KEY_SUB 26  /**< ^Z Substitute */
#define KEY_ESC 27  /**< ^[ Escape, next character is not echoed */
#define KEY_FS 28   /**< ^\ File separator */
#define KEY_GS 29   /**< ^] Group separator */
#define KEY_RS 30   /**< ^^ Record separator, block-mode terminator */
#define KEY_US 31   /**< ^_ Unit separator */
#define KEY_DEL 127 /**< Delete (not a real control character...) */
#define IS_CONTROL_CHAR(x) ((x) <= 31)
// direction of history navigation
#define _HIST_UP 0
#define _HIST_DOWN 1
// esc seq internal codes
#define _ESC_BRACKET 1
#define _ESC_HOME 2
#define _ESC_END 3
/******************************************************************************/

//#define DBG(...) fprintf(stderr,
//"\033[33m");fprintf(stderr,__VA_ARGS__);fprintf(stderr,"\033[0m");

#if MICRORL_USE_HISTORY

#ifdef _HISTORY_DEBUG
//*****************************************************************************
// print buffer content on screen
static void print_hist(ring_history_t *pThis) {
	printf("\n");
	for (int i = 0; i < MICRORL_RING_HISTORY_LEN; i++) {
		if (i == pThis->begin)
			printf("b");
		else
			printf(" ");
	}
	printf("\n");
	for (int i = 0; i < MICRORL_RING_HISTORY_LEN; i++) {
		if (isalpha(pThis->ring_buf[i]))
			printf("%c", pThis->ring_buf[i]);
		else
			printf("%d", pThis->ring_buf[i]);
	}
	printf("\n");
	for (int i = 0; i < MICRORL_RING_HISTORY_LEN; i++) {
		if (i == pThis->end)
			printf("e");
		else
			printf(" ");
	}
	printf("\n");
}
#endif

//*****************************************************************************
// remove older message from ring buffer
static void hist_erase_older(ring_history_t *pThis) {
	int new_pos = pThis->begin + pThis->ring_buf[pThis->begin] + 1;
	if (new_pos >= MICRORL_RING_HISTORY_LEN)
		new_pos = new_pos - MICRORL_RING_HISTORY_LEN;

	pThis->begin = new_pos;
}

//*****************************************************************************
// check space for new line, remove older while not space
static int hist_is_space_for_new(ring_history_t *pThis, int len) {
	if (pThis->ring_buf[pThis->begin] == 0)
		return true;
	if (pThis->end >= pThis->begin) {
		if (MICRORL_RING_HISTORY_LEN - pThis->end + pThis->begin - 1 > len)
			return true;
	} else {
		if (pThis->begin - pThis->end - 1 > len)
			return true;
	}
	return false;
}

//*****************************************************************************
// put line to ring buffer
static void hist_save_line(ring_history_t *pThis, char *line, int len) {
	if (len > MICRORL_RING_HISTORY_LEN - 2)
		return;
	while (!hist_is_space_for_new(pThis, len)) {
		hist_erase_older(pThis);
	}
	// if it's first line
	if (pThis->ring_buf[pThis->begin] == 0)
		pThis->ring_buf[pThis->begin] = len;

	// store line
	if (len < MICRORL_RING_HISTORY_LEN - pThis->end - 1)
		memcpy(pThis->ring_buf + pThis->end + 1, line, len);
	else {
		int part_len = MICRORL_RING_HISTORY_LEN - pThis->end - 1;
		memcpy(pThis->ring_buf + pThis->end + 1, line, part_len);
		memcpy(pThis->ring_buf, line + part_len, len - part_len);
	}
	pThis->ring_buf[pThis->end] = len;
	pThis->end = pThis->end + len + 1;
	if (pThis->end >= MICRORL_RING_HISTORY_LEN)
		pThis->end -= MICRORL_RING_HISTORY_LEN;
	pThis->ring_buf[pThis->end] = 0;
	pThis->cur = 0;
#ifdef _HISTORY_DEBUG
	print_hist(pThis);
#endif
}

//*****************************************************************************
// copy saved line to 'line' and return size of line
static int hist_restore_line(ring_history_t *pThis, char *line, int dir) {
	int cnt = 0;
	// count history record
	int header = pThis->begin;
	while (pThis->ring_buf[header] != 0) {
		header += pThis->ring_buf[header] + 1;
		if (header >= MICRORL_RING_HISTORY_LEN)
			header -= MICRORL_RING_HISTORY_LEN;
		cnt++;
	}

	if (dir == _HIST_UP) {
		if (cnt >= pThis->cur) {
			int header = pThis->begin;
			int j = 0;
			// found record for 'pThis->cur' index
			while ((pThis->ring_buf[header] != 0) &&
				   (cnt - j - 1 != pThis->cur)) {
				header += pThis->ring_buf[header] + 1;
				if (header >= MICRORL_RING_HISTORY_LEN)
					header -= MICRORL_RING_HISTORY_LEN;
				j++;
			}
			if (pThis->ring_buf[header]) {
				pThis->cur++;
				// obtain saved line
				if (pThis->ring_buf[header] + header <
					MICRORL_RING_HISTORY_LEN) {
					memset(line, 0, MICRORL_COMMAND_LINE_LEN);
					memcpy(line, pThis->ring_buf + header + 1,
						   pThis->ring_buf[header]);
				} else {
					int part0 = MICRORL_RING_HISTORY_LEN - header - 1;
					memset(line, 0, MICRORL_COMMAND_LINE_LEN);
					memcpy(line, pThis->ring_buf + header + 1, part0);
					memcpy(line + part0, pThis->ring_buf,
						   pThis->ring_buf[header] - part0);
				}
				return pThis->ring_buf[header];
			}
		}
	} else {
		if (pThis->cur > 0) {
			pThis->cur--;
			int header = pThis->begin;
			int j = 0;

			while ((pThis->ring_buf[header] != 0) && (cnt - j != pThis->cur)) {
				header += pThis->ring_buf[header] + 1;
				if (header >= MICRORL_RING_HISTORY_LEN)
					header -= MICRORL_RING_HISTORY_LEN;
				j++;
			}
			if (pThis->ring_buf[header] + header < MICRORL_RING_HISTORY_LEN) {
				memcpy(line, pThis->ring_buf + header + 1,
					   pThis->ring_buf[header]);
			} else {
				int part0 = MICRORL_RING_HISTORY_LEN - header - 1;
				memcpy(line, pThis->ring_buf + header + 1, part0);
				memcpy(line + part0, pThis->ring_buf,
					   pThis->ring_buf[header] - part0);
			}
			return pThis->ring_buf[header];
		} else {
			/* empty line */
			return 0;
		}
	}
	return -1;
}
#endif

//*****************************************************************************
// split cmdline to tkn array and return nmb of token
static int split(microrl_t *pThis, int limit, const char **tkn_arr) {
	int i = 0;
	int ind = 0;
	while (1) {
		// go to the first whitespace (zerro for us)
		while ((ind < limit) && (pThis->cmdline[ind] == '\0')) {
			ind++;
		}
		if (!(ind < limit))
			return i;
		tkn_arr[i++] = pThis->cmdline + ind;
		if (i >= MICRORL_COMMAND_TOKEN_NMB) {
			return -1;
		}
		// go to the first NOT whitespace (not zerro for us)
		while ((ind < limit) && (pThis->cmdline[ind] != '\0')  ) {
			ind++;
		}
		if (!(ind < limit))
			return i;
	}
	return i;
}

//*****************************************************************************
inline static void print_prompt(microrl_t *pThis) {
	pThis->config.print(pThis->config.prompt_str);
}

//*****************************************************************************
inline static void terminal_backspace(microrl_t *pThis) {
	pThis->config.print("\033[D \033[D");
}

//*****************************************************************************
inline static void terminal_newline(microrl_t *pThis) {
	pThis->config.print(MICRORL_ENDL);
}

#if !MICRORL_USE_LIBC_STDIO
//*****************************************************************************
// convert 16 bit value to string
// 0 value not supported!!! just make empty string
// Returns pointer to a buffer tail
static char *u16bit_to_str(unsigned int nmb, char *buf) {
	char tmp_str[6] = {
		0,
	};
	int i = 0, j;
	if (nmb <= 0xFFFF) {
		while (nmb > 0) {
			tmp_str[i++] = (nmb % 10) + '0';
			nmb /= 10;
		}
		for (j = 0; j < i; ++j)
			*(buf++) = tmp_str[i - j - 1];
	}
	*buf = '\0';
	return buf;
}
#endif

//*****************************************************************************
// set cursor at position from begin cmdline (after prompt) + offset
static void terminal_move_cursor(microrl_t *pThis, int offset) {
	char str[16] = {
		0,
	};
#if MICRORL_USE_LIBC_STDIO
	if (offset > 0) {
		snprintf(str, 16, "\033[%dC", offset);
	} else if (offset < 0) {
		snprintf(str, 16, "\033[%dD", -(offset));
	}
#else
	char *endstr;
	strcpy(str, "\033[");
	if (offset > 0) {
		endstr = u16bit_to_str(offset, str + 2);
		strcpy(endstr, "C");
	} else if (offset < 0) {
		endstr = u16bit_to_str(-(offset), str + 2);
		strcpy(endstr, "D");
	} else
		return;
#endif
	pThis->config.print(str);
}

//*****************************************************************************
static void terminal_reset_cursor(microrl_t *pThis) {
	char str[16];
#if MICRORL_USE_LIBC_STDIO
	snprintf(str, 16, "\033[%dD\033[%dC",
			 MICRORL_COMMAND_LINE_LEN + pThis->config.prompt_length + 2,
			 pThis->config.prompt_length);
#else
	char *endstr;
	strcpy(str, "\033[");
	endstr = u16bit_to_str(
		MICRORL_COMMAND_LINE_LEN + pThis->config.prompt_length + 2, str + 2);
	strcpy(endstr, "D\033[");
	endstr += 3;
	endstr = u16bit_to_str(pThis->config.prompt_length, endstr);
	strcpy(endstr, "C");
#endif
	pThis->config.print(str);
}

//*****************************************************************************
// print cmdline to screen, replace '\0' to wihitespace
static void terminal_print_line(microrl_t *pThis, int pos, int cursor) {
	pThis->config.print("\033[K"); // delete all from cursor to end

	char nch[] = {0, 0};
	int i;
	for (i = pos; i < pThis->cmdlen; i++) {
		nch[0] = pThis->cmdline[i];
		if (nch[0] == '\0')
			nch[0] = ' ';
		pThis->config.print(nch);
	}

	terminal_reset_cursor(pThis);
	terminal_move_cursor(pThis, cursor);
}

//*****************************************************************************
void microrl_init(microrl_t *pThis, struct microrl_config *config) {
	memset(pThis->cmdline, 0, MICRORL_COMMAND_LINE_LEN);
#if MICRORL_USE_HISTORY
	memset(pThis->ring_hist.ring_buf, 0, MICRORL_RING_HISTORY_LEN);
	pThis->ring_hist.begin = 0;
	pThis->ring_hist.end = 0;
	pThis->ring_hist.cur = 0;
#endif
#if MICRORL_USE_ESC_SEQ
	pThis->escape = 0;
#endif
	pThis->cmdlen = 0;
	pThis->cursor = 0;
	pThis->config = *config;
	if (!pThis->config.prompt_str) {
		pThis->config.prompt_str = MICRORL_PROMPT_DEFAULT;
		pThis->config.prompt_length = MICRORL_PROMPT_LEN;
	}
#if MICRORL_ENABLE_INIT_PROMPT
	print_prompt(pThis);
#endif
}

void microrl_set_prompt(microrl_t *pThis, const char *prompt_str,
						uint8_t prompt_length) {
	pThis->config.prompt_str = prompt_str;
	pThis->config.prompt_length = prompt_length;
}
#if MICRORL_USE_COMPLETE
//*****************************************************************************
void microrl_set_complete_callback(
	microrl_t *pThis,
	const char **(*get_completion)(void *userdata, int, const char *const *)) {
	pThis->config.get_completion = get_completion;
}
#endif
//*****************************************************************************
void microrl_set_execute_callback(microrl_t *pThis,
								  int (*execute)(void *, int, char **)) {
	pThis->config.execute = execute;
}
#if MICRORL_USE_CTRL_C
//*****************************************************************************
void microrl_set_sigint_callback(microrl_t *pThis,
								 void (*sigint)(void *userdata)) {
	pThis->config.sigint = sigint;
}
#endif

#if MICRORL_USE_HISTORY
static void hist_search(microrl_t *pThis, int dir) {
	int len = hist_restore_line(&pThis->ring_hist, pThis->cmdline, dir);
	if (len >= 0) {
		pThis->cmdline[len] = '\0';
		pThis->cursor = pThis->cmdlen = len;
		terminal_reset_cursor(pThis);
		terminal_print_line(pThis, 0, pThis->cursor);
	}
}
#endif

#if MICRORL_USE_ESC_SEQ
//*****************************************************************************
// handling escape sequences
static int escape_process(microrl_t *pThis, char ch) {
	if (ch == '[') {
		pThis->escape_seq = _ESC_BRACKET;
		return 0;
	} else if (pThis->escape_seq == _ESC_BRACKET) {
		if (ch == 'A') {
#if MICRORL_USE_HISTORY
			hist_search(pThis, _HIST_UP);
#endif
			return 1;
		} else if (ch == 'B') {
#if MICRORL_USE_HISTORY
			hist_search(pThis, _HIST_DOWN);
#endif
			return 1;
		} else if (ch == 'C') {
			if (pThis->cursor < pThis->cmdlen) {
				terminal_move_cursor(pThis, 1);
				pThis->cursor++;
			}
			return 1;
		} else if (ch == 'D') {
			if (pThis->cursor > 0) {
				terminal_move_cursor(pThis, -1);
				pThis->cursor--;
			}
			return 1;
		} else if (ch == '7') {
			pThis->escape_seq = _ESC_HOME;
			return 0;
		} else if (ch == '8') {
			pThis->escape_seq = _ESC_END;
			return 0;
		}
	} else if (ch == '~') {
		if (pThis->escape_seq == _ESC_HOME) {
			terminal_reset_cursor(pThis);
			pThis->cursor = 0;
			return 1;
		} else if (pThis->escape_seq == _ESC_END) {
			terminal_move_cursor(pThis, pThis->cmdlen - pThis->cursor);
			pThis->cursor = pThis->cmdlen;
			return 1;
		}
	}

	/* unknown escape sequence, stop */
	return 1;
}
#endif

//*****************************************************************************
// insert len char of text at cursor position
static int microrl_insert_text(microrl_t *pThis, const char *text, int len) {
	if (pThis->cmdlen + len < MICRORL_COMMAND_LINE_LEN) {
		memmove(pThis->cmdline + pThis->cursor + len,
				pThis->cmdline + pThis->cursor, pThis->cmdlen - pThis->cursor);
		for (int i = 0; i < len; i++) {
			pThis->cmdline[pThis->cursor + i] = text[i];
			if (pThis->cmdline[pThis->cursor + i] == ' ') {
				pThis->cmdline[pThis->cursor + i] = 0;
			}
		}
		pThis->cursor += len;
		pThis->cmdlen += len;
		pThis->cmdline[pThis->cmdlen] = '\0';
		return true;
	}
	return false;
}

//*****************************************************************************
// remove one char at cursor
static void microrl_backspace(microrl_t *pThis) {
	if (pThis->cursor > 0) {
		terminal_backspace(pThis);
		memmove(pThis->cmdline + pThis->cursor - 1,
				pThis->cmdline + pThis->cursor,
				pThis->cmdlen - pThis->cursor + 1);
		pThis->cursor--;
		pThis->cmdline[pThis->cmdlen] = '\0';
		pThis->cmdlen--;
	}
}

#if MICRORL_USE_COMPLETE

//*****************************************************************************
static int common_len(const char **arr) {
	int i;
	int j;
	const char *shortest = arr[0];
	int shortlen = strlen(shortest);

	for (i = 0; arr[i] != NULL; ++i)
		if (strlen(arr[i]) < shortlen) {
			shortest = arr[i];
			shortlen = strlen(shortest);
		}

	for (i = 0; i < shortlen; ++i)
		for (j = 0; arr[j] != 0; ++j)
			if (shortest[i] != arr[j][i])
				return i;

	return i;
}

//*****************************************************************************
static void microrl_get_complete(microrl_t *pThis) {
	char const *tkn_arr[MICRORL_COMMAND_TOKEN_NMB];
	const char **compl_token;

	if (pThis->config.get_completion == NULL) // callback was not set
		return;

	int status = split(pThis, pThis->cursor, tkn_arr);
	if (pThis->cmdline[pThis->cursor > 0 ? pThis->cursor - 1 : 0] == '\0')
		tkn_arr[status++] = "";
	compl_token =
		pThis->config.get_completion(pThis->config.userdata, status, tkn_arr);
	if (compl_token[0] != NULL) {
		int i = 0;
		int len;

		if (compl_token[1] == NULL) {
			len = strlen(compl_token[0]);
		} else {
			len = common_len(compl_token);
			terminal_newline(pThis);
			while (compl_token[i] != NULL) {
				pThis->config.print(compl_token[i]);
				if (compl_token[i + 1] != NULL) {
					pThis->config.print(" ");
				}
				i++;
			}
			terminal_newline(pThis);
			print_prompt(pThis);
		}

		if (len) {
			microrl_insert_text(pThis,
								compl_token[0] + strlen(tkn_arr[status - 1]),
								len - strlen(tkn_arr[status - 1]));
			if (compl_token[1] == NULL)
				microrl_insert_text(pThis, " ", 1);
		}
		terminal_reset_cursor(pThis);
		terminal_print_line(pThis, 0, pThis->cursor);
	}
}
#endif

/**
 * \brief Newline keys (Ctrl-C, endl, etc.) handler
 *
 * \param pThis handle of a microrl instance
 * \param execute whether the inserted implies an execution or not.
 * If true, the current input line will be parsed and executed (when valid).
 * If false, the current input line is just discarded.
 */
static void new_line_handler(microrl_t *pThis, bool execute) {
	char *tkn_arr[MICRORL_COMMAND_TOKEN_NMB];
	int status;

	terminal_newline(pThis);
	if (execute) {
#if MICRORL_USE_HISTORY
		if (pThis->cmdlen > 0)
			hist_save_line(&pThis->ring_hist, pThis->cmdline, pThis->cmdlen);
#endif
		status = split(pThis, pThis->cmdlen, (const char **)tkn_arr);
		if (status == -1) {
			//          pThis->print ("ERROR: Max token amount exceeded\n");
			pThis->config.print("ERROR:too many tokens");
			pThis->config.print(MICRORL_ENDL);
		}
		if ((status > 0) && (pThis->config.execute != NULL)) {
#if MICRORL_USE_CTRL_C
			pThis->pending_execution = true;
#endif // MICRORL_USE_CTRL_C
			pThis->config.execute(pThis->config.userdata, status, tkn_arr);
		}
	}
	print_prompt(pThis);
	pThis->cmdlen = 0;
	pThis->cursor = 0;
	memset(pThis->cmdline, 0, MICRORL_COMMAND_LINE_LEN);
#if MICRORL_USE_HISTORY
	pThis->ring_hist.cur = 0;
#endif
}

//*****************************************************************************
// GCC 9 complains about the line 'if (isalnum(pThis->cmdline[pThis->cursor - 1])) {'
// it is complaining about the cmdline array (not the cursor variable, that is an int)
// See e.g. https://stackoverflow.com/a/60696378
void microrl_insert_char(microrl_t *pThis, int ch) {
#if MICRORL_USE_ESC_SEQ
	if (pThis->escape) {
		if (escape_process(pThis, ch))
			pThis->escape = 0;
	} else {
#endif
		switch (ch) {
			//-----------------------------------------------------
#ifdef MICRORL_ENDL_CR
		case KEY_CR:
			new_line_handler(pThis, true);
			break;
		case KEY_LF:
			break;
#elif defined(MICRORL_ENDL_CRLF)
	case KEY_CR:
		pThis->tmpch = KEY_CR;
		break;
	case KEY_LF:
		if (pThis->tmpch == KEY_CR)
			new_line_handler(pThis, true);
		break;
#elif defined(MICRORL_ENDL_LFCR)
	case KEY_LF:
		pThis->tmpch = KEY_LF;
		break;
	case KEY_CR:
		if (pThis->tmpch == KEY_LF)
			new_line_handler(pThis, true);
		break;
#else
	case KEY_CR:
		break;
	case KEY_LF:
		new_line_handler(pThis, true);
		break;
#endif
			//-----------------------------------------------------
#if MICRORL_USE_COMPLETE
		case KEY_HT:
			microrl_get_complete(pThis);
			break;
#endif
		//-----------------------------------------------------
		case KEY_ESC:
#if MICRORL_USE_ESC_SEQ
			pThis->escape = 1;
#endif
			break;
		//-----------------------------------------------------
		case KEY_NAK: // ^U
			while (pThis->cursor > 0) {
				microrl_backspace(pThis);
			}
			terminal_print_line(pThis, 0, pThis->cursor);
			break;
		case KEY_ETB: // ^W (cut word)
		{
			bool hit_word = false;
			while (pThis->cursor > 0) {
				if (isalnum((unsigned char)pThis->cmdline[pThis->cursor - 1])) {
					hit_word = true;
				}
				if (pThis->cmdline[pThis->cursor - 1] == '\0' && hit_word) {
					break;
				}
				microrl_backspace(pThis);
			}
			terminal_print_line(pThis, pThis->cursor, pThis->cursor);
		} break;
		//-----------------------------------------------------
		case KEY_VT: // ^K
			pThis->config.print("\033[K");
			pThis->cmdlen = pThis->cursor;
			break;
		//-----------------------------------------------------
		case KEY_ENQ: // ^E
			terminal_move_cursor(pThis, pThis->cmdlen - pThis->cursor);
			pThis->cursor = pThis->cmdlen;
			break;
		//-----------------------------------------------------
		case KEY_SOH: // ^A
			terminal_reset_cursor(pThis);
			pThis->cursor = 0;
			break;
		//-----------------------------------------------------
		case KEY_ACK: // ^F
			if (pThis->cursor < pThis->cmdlen) {
				terminal_move_cursor(pThis, 1);
				pThis->cursor++;
			}
			break;
		//-----------------------------------------------------
		case KEY_STX: // ^B
			if (pThis->cursor) {
				terminal_move_cursor(pThis, -1);
				pThis->cursor--;
			}
			break;
		//-----------------------------------------------------
		case KEY_DLE: //^P
#if MICRORL_USE_HISTORY
			hist_search(pThis, _HIST_UP);
#endif
			break;
		//-----------------------------------------------------
		case KEY_SO: //^N
#if MICRORL_USE_HISTORY
			hist_search(pThis, _HIST_DOWN);
#endif
			break;
		//-----------------------------------------------------
		case KEY_DEL: // Backspace
		case KEY_BS:  // ^U
			microrl_backspace(pThis);
			terminal_print_line(pThis, pThis->cursor, pThis->cursor);
			break;
		//-----------------------------------------------------
		case KEY_DC2: // ^R
			terminal_newline(pThis);
			print_prompt(pThis);
			terminal_reset_cursor(pThis);
			terminal_print_line(pThis, 0, pThis->cursor);
			break;
		case KEY_FF: // ^L
			pThis->config.print("\033[2J\033[H");
			print_prompt(pThis);
			terminal_reset_cursor(pThis);
			terminal_print_line(pThis, 0, pThis->cursor);
			break;
		//-----------------------------------------------------
		case KEY_ETX:
#if MICRORL_USE_CTRL_C
			if (pThis->pending_execution) {
				if (pThis->config.sigint != NULL)
					pThis->config.sigint(pThis->config.userdata);
				pThis->pending_execution = false;
			} else {
				new_line_handler(pThis, false);
			}
#else
		new_line_handler(pThis, false);
#endif
			break;
		//-----------------------------------------------------
		case KEY_EOT:
			if (pThis->config.eof != NULL) {
				pThis->config.eof(pThis->config.userdata);
			}
			break;
		//-----------------------------------------------------
		default:
			if (((ch == ' ') && (pThis->cmdlen == 0)) || IS_CONTROL_CHAR(ch))
				break;
			if (microrl_insert_text(pThis, (char *)&ch, 1))
				terminal_print_line(pThis, pThis->cursor - 1, pThis->cursor);

			break;
		}
#if MICRORL_USE_ESC_SEQ
	}
#endif
}
