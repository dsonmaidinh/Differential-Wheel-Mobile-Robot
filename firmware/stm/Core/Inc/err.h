/**
 * @file       err.h
 * @copyright  [Your Copyright]
 * @license    [Your License]
 * @version    1.0.0
 * @date       2026-03-02
 * @author     Dong Son
 *
 * @brief
 */
/* Define to prevent recursive inclusion ------------------------------ */
#ifndef __ERR_H
#define __ERR_H

/* Includes ----------------------------------------------------------- */
#include <assert.h> /* For debug-time checks */
#include <stdint.h>

/* Public enumerate/structure ----------------------------------------- */
typedef struct
{
	const char *file;
	uint32_t line;
} error_info_t;

/* Public variables --------------------------------------------------- */
extern error_info_t global_error;

/* Public function prototypes ----------------------------------------- */
void System_Error_Hook(void) __attribute__((weak));

/* Public defines ----------------------------------------------------- */
#define CHECK_ERR(expr, err_code) \
  do                              \
  {                               \
    if ((expr))                   \
    {                             \
      global_error.file = __FILE__; \
      global_error.line = __LINE__; \
      if (System_Error_Hook)      \
      {                           \
         System_Error_Hook();     \
      }                           \
      return (err_code);          \
    }                             \
  } while (0)

#define CHECK_PARAM(expr, err_code) \
  do                                \
  {                                 \
    if ((expr))                     \
    {                               \
      global_error.file = __FILE__; \
      global_error.line = __LINE__; \
      if (System_Error_Hook)        \
      {                             \
         System_Error_Hook();       \
      }                             \
      return (err_code);            \
    }                               \
  } while (0)

/* Public function prototypes ----------------------------------------- */
void System_Error_Hook(void) __attribute__((weak));

#endif /* INC_APP_H_ */

/* End of file -------------------------------------------------------- */
