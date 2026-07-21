
#ifndef __ESC_SEQUENCE_H
#define __ESC_SEQUENCE_H

#ifdef __cplusplus
extern "C" {
#endif

/*! \name VT100 Common Commands
*/
//! @{
#define CLEARSCR          "\x1B[2J\x1B[;H"    //!< Clear screen.
#define CLEAREOL          "\x1B[K"            //!< Clear end of line.
#define CLEAREOS          "\x1B[J"            //!< Clear end of screen.
#define CLEARLCR          "\x1B[0K"           //!< Clear line cursor right.
#define CLEARLCL          "\x1B[1K"           //!< Clear line cursor left.
#define CLEARELN          "\x1B[2K"           //!< Clear entire line.
#define CLEARCDW          "\x1B[0J"           //!< Clear cursor down.
#define CLEARCUP          "\x1B[1J"           //!< Clear cursor up.
#define GOTOYX            "\x1B[%.2d;%.2dH"   //!< Set cursor to (y, x).
#define INSERTMOD         "\x1B[4h"           //!< Insert mode.
#define OVERWRITEMOD      "\x1B[4l"           //!< Overwrite mode.
#define DELAFCURSOR       "\x1B[K"            //!< Erase from cursor to end of line.
#define CRLF              "\r\n"              //!< Carriage Return + Line Feed.
//! @}

/*! \name VT100 Cursor Commands
*/
//! @{
#define CURSON            "\x1B[?25h"         //!< Show cursor.
#define CURSOFF           "\x1B[?25l"         //!< Hide cursor.
//! @}

/*! \name VT100 Character Commands
*/
//! @{
#define NORMAL            "\x1B[0m"           //!< Normal.
#define BOLD              "\x1B[1m"           //!< Bold.
#define UNDERLINE         "\x1B[4m"           //!< Underline.
#define BLINKING          "\x1B[5m"           //!< Blink.
#define INVVIDEO          "\x1B[7m"           //!< Inverse video.
//! @}

/*! \name VT100 Color Commands
*/
//! @{
#define CL_BLACK          "\x1B[22;30m"       //!< Black.
#define CL_RED            "\x1B[22;31m"       //!< Red.
#define CL_GREEN          "\x1B[22;32m"       //!< Green.
#define CL_BROWN          "\x1B[22;33m"       //!< Brown.
#define CL_BLUE           "\x1B[22;34m"       //!< Blue.
#define CL_MAGENTA        "\x1B[22;35m"       //!< Magenta.
#define CL_CYAN           "\x1B[22;36m"       //!< Cyan.
#define CL_GRAY           "\x1B[22;37m"       //!< Gray.
#define CL_DARKGRAY       "\x1B[01;30m"       //!< Dark gray.
#define CL_LIGHTRED       "\x1B[01;31m"       //!< Light red.
#define CL_LIGHTGREEN     "\x1B[01;32m"       //!< Light green.
#define CL_LIGHTYELLOW    "\x1B[01;33m"       //!< Yellow.
#define CL_LIGHTBLUE      "\x1B[01;34m"       //!< Light blue.
#define CL_LIGHTMAGENTA   "\x1B[01;35m"       //!< Light magenta.
#define CL_LIGHTCYAN      "\x1B[01;36m"       //!< Light cyan.
#define CL_WHITE          "\x1B[01;37m"       //!< White.
//! @}

#ifdef __cplusplus
}
#endif

#endif /* __ESC_SEQUENCE_H */
