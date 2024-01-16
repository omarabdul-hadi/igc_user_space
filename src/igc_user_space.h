
#ifndef _IGC_USER_SPACE_API_H_
#define _IGC_USER_SPACE_API_H_

#include <stdint.h>

bool is_igc_user_space_drv_supported();
void init();
void deinit();
void get_mac_addr(uint8_t* data);
void send_frame(uint8_t* data, int len);
uint32_t receive_frame(uint8_t* receive_pkt);

#endif /* _IGC_USER_SPACE_API_H_ */