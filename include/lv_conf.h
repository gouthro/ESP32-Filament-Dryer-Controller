/**
 * @file lv_conf.h
 * Configuration file for v9.4.0-dev
 */

#if 1

#ifndef LV_CONF_H
#define LV_CONF_H


/*====================
   COLOR SETTINGS
 *====================*/

#define LV_COLOR_DEPTH 16

/*====================
   LOG SETTINGS
 *====================*/

#define LV_USE_LOG 1
#if LV_USE_LOG
    #define LV_LOG_LEVEL LV_LOG_LEVEL_WARN
    #define LV_LOG_PRINTF 1
    #define LV_LOG_USE_TIMESTAMP 1
#endif

/*====================
   TEXT SETTINGS
 *====================*/

#define LV_TXT_ENC LV_TXT_ENC_UTF8
#define LV_USE_BIDI 1
#if LV_USE_BIDI
    #define LV_BIDI_BASE_DIR_DEF LV_BIDI_DIR_AUTO
#endif

/*====================
   WIDGET SETTINGS
 *====================*/

#define LV_USE_BUTTON 1
#define LV_USE_LABEL 1


#endif /*LV_CONF_H*/
#endif /*End of "Content enable"*/
