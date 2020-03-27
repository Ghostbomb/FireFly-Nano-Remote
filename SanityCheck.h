
#include <globals.h>


#if defined RFM_USA && defined RFM_EU
#error "Make sure you chose what region you are in. Comment out one of the following in user_config.h: RFM_USA or RFM_EU"
#endif

#if defined MAINPAGE_FULL && defined MAINPAGE_LITE
  #error "You can't have 2 different menus defined. Go to 'globals.h' and undefine one of the Menus. MAINPAGE_FULL or MAINPAGE_LITE."
#endif